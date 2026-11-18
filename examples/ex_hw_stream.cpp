/**
 * 位置跟随 / 流式目标（覆盖 forcerobot_sdk.h 流式全部接口）
 *
 *   sudo ./ex_hw_stream ../config eth0 0          # 默认 joint
 *   sudo ./ex_hw_stream ../config eth0 0 joint
 *   sudo ./ex_hw_stream ../config eth0 0 tcp
 *
 * joint: start_joint_stream → stream_update_joint ×N → finish_stream
 * tcp:   start_tcp_stream   → stream_update_tcp   ×N → finish_stream
 */
#include "hw_util.hpp"

#include <algorithm>
#include <cstring>

static const double kSoftMin[7] = {-180, -90, -180, 0, -180, -47, -47};
static const double kSoftMax[7] = {180, 90, 180, 130, 180, 47, 47};

static int run_joint_stream(frs_robot* robot, FrsTagWait* wait, int arm,
                            const frs_state& st) {
    frs_result acc{};
    int rc = frs_start_joint_stream(robot, arm, 8.0, 30.0, 300, &acc);
    std::printf("start_joint_stream rc=%d %s\n", rc, acc.message);
    if (rc != FRS_OK) return rc;

    double q[7];
    for (int j = 0; j < 7; ++j) q[j] = st.measured_q_deg[arm][j];
    for (int step = 0; step < 6; ++step) {
        q[5] = std::min(kSoftMax[5], std::max(kSoftMin[5], q[5] + ((step % 2) ? -2.0 : 2.0)));
        frs_print_q("stream_update_joint:", q);
        frs_stream_update_joint(robot, arm, q);
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
    }

    rc = frs_finish_stream(robot, arm, 310, &acc);
    std::printf("finish_stream rc=%d %s\n", rc, acc.message);
    if (rc == FRS_OK) std::printf("wait finish rc=%d\n", frs_wait_tag(wait, 310, 10000));
    return rc;
}

static int run_tcp_stream(frs_robot* robot, FrsTagWait* wait, int arm,
                          const frs_state& st) {
    frs_result acc{};
    // 线速度宜略低于“步长/周期”，否则目标跳变会抖
    const double lin_speed = 0.03;   // m/s
    const double step_m = 0.06;     // 0.005每步 5mm，同向累加
    const int interval_ms = 600;     // 给跟随留足时间
    int rc = frs_start_tcp_stream(robot, arm, lin_speed, 0.15, 400, &acc);
    std::printf("start_tcp_stream rc=%d %s\n", rc, acc.message);
    if (rc != FRS_OK) return rc;

    frs_pose pose{};
    pose.position_m[0] = st.tcp_pose[arm][0];
    pose.position_m[1] = st.tcp_pose[arm][1];
    pose.position_m[2] = st.tcp_pose[arm][2];
    pose.rpy_deg[0] = st.tcp_pose[arm][3];
    pose.rpy_deg[1] = st.tcp_pose[arm][4];
    pose.rpy_deg[2] = st.tcp_pose[arm][5];
    const double z0 = pose.position_m[2];
    std::printf("start tcp xyz=(%.4f,%.4f,%.4f)\n",
                pose.position_m[0], pose.position_m[1], pose.position_m[2]);
    std::printf("smooth Z: +4 steps then -4 steps, step=%.3fm interval=%dms\n",
                step_m, interval_ms);

    // 先同向抬高，再同向放下（不要每步反向，否则会抖）
    auto push = [&](double dz) {
        pose.position_m[2] += dz;
        std::printf("stream_update_tcp z=%.4f (dz=%+.3f)\n", pose.position_m[2], dz);
        frs_stream_update_tcp(robot, arm, &pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    };
    for (int i = 0; i < 4; ++i) push(step_m);
    for (int i = 0; i < 4; ++i) push(-step_m);
    pose.position_m[2] = z0;  // 回到起点附近
    frs_stream_update_tcp(robot, arm, &pose);
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));

    rc = frs_finish_stream(robot, arm, 410, &acc);
    std::printf("finish_stream rc=%d %s\n", rc, acc.message);
    if (rc == FRS_OK) std::printf("wait finish rc=%d\n", frs_wait_tag(wait, 410, 15000));
    return rc;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr,
                     "usage: %s <config> <ifname|fake> [arm] [joint|tcp]\n",
                     argv[0]);
        return 1;
    }
    const char* config = argv[1];
    const char* ifname = argv[2];
    int arm = (argc > 3) ? argv[3][0] - '0' : 0;
    const char* mode = (argc > 4) ? argv[4] : "joint";
    if (arm < 0 || arm > 1) {
        std::fprintf(stderr, "arm must be 0/1\n");
        return 1;
    }
    const int is_tcp = (std::strcmp(mode, "tcp") == 0 || std::strcmp(mode, "TCP") == 0);
    if (!is_tcp && std::strcmp(mode, "joint") != 0 && std::strcmp(mode, "JOINT") != 0 &&
        argc > 4) {
        std::fprintf(stderr, "mode must be joint or tcp\n");
        return 1;
    }
    const int require_bus = !frs_is_fake(ifname);

    std::printf("ex_hw_stream mode=%s arm=%d ifname=%s\n", is_tcp ? "tcp" : "joint",
                arm, ifname);
    std::fflush(stdout);

    frs_options opt{};
    frs_fill_options(&opt, ifname);
    FrsTagWait wait;
    frs_robot* robot = frs_create(config, &opt);
    if (!robot) {
        std::fprintf(stderr, "frs_create failed (quit other ex_/runtime? clear lock?)\n"
                             "  sudo rm -f /tmp/force_robot_sdk.lock\n");
        return 2;
    }
    frs_set_result_callback(robot, frs_tag_on_result, &wait);
    if (frs_start(robot) != FRS_OK) {
        std::fprintf(stderr, "frs_start failed\n");
        frs_destroy(robot);
        return 3;
    }
    frs_state st{};
    if (frs_wait_ready(robot, &st, 15000, require_bus) != FRS_OK) {
        std::fprintf(stderr, "wait ready failed\n");
        frs_stop(robot);
        frs_destroy(robot);
        return 4;
    }

    char prompt[64];
    std::snprintf(prompt, sizeof(prompt), "enable + %s_stream? type y: ",
                  is_tcp ? "tcp" : "joint");
    if (!frs_confirm_y(prompt)) {
        frs_stop(robot);
        frs_destroy(robot);
        return 0;
    }
    if (frs_enable_arm(robot, arm) != FRS_OK) {
        frs_stop(robot);
        frs_destroy(robot);
        return 5;
    }
    frs_get_state(robot, &st);

    int rc = is_tcp ? run_tcp_stream(robot, &wait, arm, st)
                    : run_joint_stream(robot, &wait, arm, st);
    if (rc != FRS_OK && is_tcp)
        std::fprintf(stderr, "hint: TCP stream 近奇异时先 joint 摆姿态再试\n");

    frs_disable(robot, arm, 320, nullptr);
    frs_stop(robot);
    frs_destroy(robot);
    return 0;
}
