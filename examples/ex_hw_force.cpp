/**
 * 力控 / 阻抗（需求：力控模式、关节阻抗、drag/spring、阻抗目标）
 *   sudo ./ex_hw_force ../config eth0 0
 *
 * 流程：enable → autozero(等完成) → set_cst_gains → start_drag → hold
 *       → start_spring → hold → move_impedance_joint → move_impedance_tcp → disable
 * 真机务必确认周围无障碍；拖动时手臂会跟随外力。
 * 注意：autozero 约需 5s，未完成前禁止下发其它策略，否则会被顶掉并拒绝力控。
 * TCP 阻抗近奇异时可能被拒，可先关节摆到肘弯姿态。
 */
#include "hw_util.hpp"

#include <algorithm>

static const double kSoftMin[7] = {-180, -90, -180, 0, -180, -47, -47};
static const double kSoftMax[7] = {180, 90, 180, 130, 180, 47, 47};

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s <config> <ifname|fake> [arm]\n", argv[0]);
        return 1;
    }
    const char* config = argv[1];
    const char* ifname = argv[2];
    int arm = (argc > 3) ? argv[3][0] - '0' : 0;
    const int require_bus = !frs_is_fake(ifname);

    frs_options opt{};
    frs_fill_options(&opt, ifname);
    FrsTagWait wait;
    frs_robot* robot = frs_create(config, &opt);
    if (!robot) return 2;
    frs_set_result_callback(robot, frs_tag_on_result, &wait);
    if (frs_start(robot) != FRS_OK) {
        frs_destroy(robot);
        return 3;
    }
    frs_state st{};
    if (frs_wait_ready(robot, &st, 15000, require_bus) != FRS_OK) {
        frs_stop(robot);
        frs_destroy(robot);
        return 4;
    }
    if (!frs_confirm_y("enable+autozero+drag/spring/impedance? type y: ")) {
        frs_stop(robot);
        frs_destroy(robot);
        return 0;
    }
    if (frs_enable_arm(robot, arm) != FRS_OK) {
        frs_stop(robot);
        frs_destroy(robot);
        return 5;
    }

    frs_result acc{};
    int     rc = frs_autozero(robot, arm, 20, &acc);
    std::printf("autozero rc=%d %s\n", rc, acc.message);
    if (rc != FRS_OK) {
        frs_disable(robot, arm, 90, nullptr);
        frs_stop(robot);
        frs_destroy(robot);
        return 6;
    }
    std::printf("waiting autozero (~5s), do not interrupt...\n");
    rc = frs_wait_tag(&wait, 20, 30000);  // 超时 30s，实测约 5s
    std::printf("wait autozero rc=%d\n", rc);
    if (rc != FRS_OK) {
        std::fprintf(stderr, "autozero 未完成，中止力控（勿在零力标定中发其它命令）\n");
        frs_hold(robot, arm, 91, nullptr);
        frs_disable(robot, arm, 92, nullptr);
        frs_stop(robot);
        frs_destroy(robot);
        return 7;
    }

    frs_get_state(robot, &st);
    std::printf("autozero_done=%d\n", st.autozero_done[arm]);
    if (!st.autozero_done[arm]) {
        std::fprintf(stderr, "state.autozero_done=0，拒绝继续\n");
        frs_disable(robot, arm, 93, nullptr);
        frs_stop(robot);
        frs_destroy(robot);
        return 8;
    }

    double kp[7], kd[7];
    for (int j = 0; j < 7; ++j) {
        kp[j] = 2.0;
        kd[j] = 0.05;
    }
    rc = frs_set_cst_gains(robot, arm, kp, kd, 21, &acc);
    std::printf("set_cst_gains rc=%d %s\n", rc, acc.message);
    double kp_r[7] = {}, kd_r[7] = {};
    if (frs_get_cst_gains(robot, arm, kp_r, kd_r) == FRS_OK)
        std::printf("get_cst_gains kp0=%.3f kd0=%.3f\n", kp_r[0], kd_r[0]);

    rc = frs_start_drag(robot, arm, 30, &acc);
    std::printf("start_drag rc=%d %s\n", rc, acc.message);
    if (rc != FRS_OK) {
        frs_hold(robot, arm, 94, nullptr);
        frs_disable(robot, arm, 95, nullptr);
        frs_stop(robot);
        frs_destroy(robot);
        return 9;
    }
    std::printf("drag 3s（可轻推关节）...\n");
    std::this_thread::sleep_for(std::chrono::seconds(3));
    frs_hold(robot, arm, 31, nullptr);
    frs_wait_tag(&wait, 31, 5000);

    rc = frs_start_spring(robot, arm, 40, &acc);
    std::printf("start_spring rc=%d %s\n", rc, acc.message);
    if (rc == FRS_OK) {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        frs_hold(robot, arm, 41, nullptr);
        frs_wait_tag(&wait, 41, 5000);
    }

    frs_get_state(robot, &st);
    double q[7];
    for (int j = 0; j < 7; ++j) q[j] = st.measured_q_deg[arm][j];
    q[5] = std::min(kSoftMax[5], std::max(kSoftMin[5], q[5] + 2.0));
    rc = frs_move_impedance_joint(robot, arm, q, 5.0, 50, &acc);
    std::printf("move_impedance_joint rc=%d %s\n", rc, acc.message);
    if (rc == FRS_OK) frs_wait_tag(&wait, 50, 20000);
    frs_hold(robot, arm, 51, nullptr);
    frs_wait_tag(&wait, 51, 3000);

    // TCP 阻抗：相对当前 TCP 沿 Z +5mm
    frs_get_state(robot, &st);
    frs_pose tgt{};
    tgt.position_m[0] = st.tcp_pose[arm][0];
    tgt.position_m[1] = st.tcp_pose[arm][1];
    tgt.position_m[2] = st.tcp_pose[arm][2] + 0.005;
    tgt.rpy_deg[0] = st.tcp_pose[arm][3];
    tgt.rpy_deg[1] = st.tcp_pose[arm][4];
    tgt.rpy_deg[2] = st.tcp_pose[arm][5];
    std::printf("impedance_tcp target z=%.4f ( +5mm )\n", tgt.position_m[2]);
    rc = frs_move_impedance_tcp(robot, arm, &tgt, 0.03, 0.15, 60, &acc);
    std::printf("move_impedance_tcp rc=%d %s\n", rc, acc.message);
    if (rc == FRS_OK) {
        std::printf("wait impedance_tcp rc=%d\n", frs_wait_tag(&wait, 60, 20000));
    } else {
        std::fprintf(stderr, "hint: 近奇异时阻抗 TCP 易被拒，可先 joint 摆姿态\n");
    }

    frs_hold(robot, arm, 61, nullptr);
    frs_disable(robot, arm, 62, nullptr);
    frs_stop(robot);
    frs_destroy(robot);
    return 0;
}
