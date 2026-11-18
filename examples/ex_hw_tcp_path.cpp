/**
 * 停顿式多点 TCP（frs_tcp_path）
 *
 *   sudo ./ex_hw_tcp_path ../config eth0 0
 *   sudo ./ex_hw_tcp_path ../config eth0 0 dz 0.01 0.02 0.03
 *
 * 相对当前 TCP 沿 Z 走 3 点，每点停留；近奇异位形可能被拒。
 */
#include "hw_util.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

static void fill_from_tcp(const frs_state& st, int arm, const double dz[3],
                          frs_path_point pts[3], int with_dwell) {
    for (int i = 0; i < 3; ++i) {
        for (int k = 0; k < 6; ++k) pts[i].values[k] = st.tcp_pose[arm][k];
        pts[i].values[6] = 0;
        pts[i].values[2] += dz[i];
        pts[i].speed = 0.05;           // m/s
        pts[i].rotation_speed = 0.2;   // rad/s
        pts[i].accel = 0.5;            // m/s²
        pts[i].dwell_s = with_dwell ? ((i == 2) ? 0.2 : 0.4) : 0.0;
        pts[i].blend = 0;
    }
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr,
                     "usage:\n"
                     "  %s <config> <ifname> [arm]\n"
                     "  %s <config> <ifname> <arm> dz <z0> <z1> <z2>\n",
                     argv[0], argv[0]);
        return 1;
    }
    const char* config = argv[1];
    const char* ifname = argv[2];
    int arm = (argc > 3) ? argv[3][0] - '0' : 0;
    if (arm < 0 || arm > 1) return 1;
    const int require_bus = !frs_is_fake(ifname);

    double dz[3] = {0.01, 0.02, 0.03};
    if (argc >= 8 && (std::strcmp(argv[4], "dz") == 0 || std::strcmp(argv[4], "DZ") == 0)) {
        dz[0] = std::atof(argv[5]);
        dz[1] = std::atof(argv[6]);
        dz[2] = std::atof(argv[7]);
    }

    frs_options opt{};
    frs_fill_options(&opt, ifname);
    FrsTagWait wait;
    frs_robot* robot = frs_create(config, &opt);
    if (!robot) {
        std::fprintf(stderr, "frs_create failed\n");
        return 2;
    }
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

    std::printf("current tcp xyz=(%.4f,%.4f,%.4f)\n",
                st.tcp_pose[arm][0], st.tcp_pose[arm][1], st.tcp_pose[arm][2]);
    std::printf("relative dz m: %.4f %.4f %.4f (dwell on)\n", dz[0], dz[1], dz[2]);

    frs_path_point pts[3] = {};
    fill_from_tcp(st, arm, dz, pts, 1);
    for (int i = 0; i < 3; ++i) {
        std::printf("pt%d xyz=(%.4f,%.4f,%.4f) dwell=%.2f\n", i,
                    pts[i].values[0], pts[i].values[1], pts[i].values[2],
                    pts[i].dwell_s);
    }

    if (!frs_confirm_y("enable + tcp_path? type y: ")) {
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
    const uint64_t tag = 220;
    int rc = frs_tcp_path(robot, arm, pts, 3, tag, &acc);
    std::printf("tcp_path rc=%d %s\n", rc, acc.message);
    if (rc == FRS_OK)
        std::printf("wait rc=%d\n", frs_wait_tag(&wait, tag, 60000));
    else
        std::fprintf(stderr, "hint: 近奇异时先 joint 摆到肘弯姿态再试小位移\n");

    frs_hold(robot, arm, 221, nullptr);
    frs_disable(robot, arm, 222, nullptr);
    frs_stop(robot);
    frs_destroy(robot);
    return 0;
}
