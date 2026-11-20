/**
 * 只读状态：create → start → get_state → stop（不使能）
 *
 * 用法：
 *   ./ex_hw_read ../config fake          # FakeBus，无硬件
 *   sudo ./ex_hw_read ../config eth0      # 真机，网口按现场改
 */
#include "hw_util.hpp"

#include <cstdio>

static void print_arm(const frs_state& st, int arm) {
    std::printf("arm%d enabled=%d autozero=%d mode=%d strategy=%d\n",
                arm, st.enabled_joints[arm], st.autozero_done[arm],
                st.mode[arm], st.strategy_status[arm]);
    std::printf("  measured q:");
    for (int j = 0; j < FRS_JOINTS; ++j)
        std::printf(" %.3f", st.measured_q_deg[arm][j]);
    std::printf("\n  tcp xyzrpy:");
    for (int k = 0; k < 6; ++k) std::printf(" %.4f", st.tcp_pose[arm][k]);
    std::printf("\n");
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s <config_dir> <ifname|fake>\n", argv[0]);
        return 1;
    }
    const char* config = argv[1];
    const char* ifname = argv[2];
    const int require_bus = !frs_is_fake(ifname);

    std::printf("SDK %s\n", frs_version());

    frs_options opt{};
    frs_fill_options(&opt, ifname);

    frs_robot* robot = frs_create(config, &opt);
    if (!robot) {
        std::fprintf(stderr, "frs_create failed (config/lock?)\n");
        return 2;
    }
    if (frs_start(robot) != FRS_OK) {
        std::fprintf(stderr, "frs_start failed\n");
        frs_destroy(robot);
        return 3;
    }

    frs_state st{};
    if (frs_wait_ready(robot, &st, 15000, require_bus) != FRS_OK) {
        frs_stop(robot);
        frs_destroy(robot);
        return 4;
    }

    print_arm(st, 0);
    print_arm(st, 1);

    frs_pose tcp{};
    if (frs_get_tcp(robot, 0, &tcp) == FRS_OK) {
        std::printf("get_tcp arm0: %.4f %.4f %.4f  rpy %.2f %.2f %.2f\n",
                    tcp.position_m[0], tcp.position_m[1], tcp.position_m[2],
                    tcp.rpy_deg[0], tcp.rpy_deg[1], tcp.rpy_deg[2]);
    }

    double mass = 0;
    double com[3] = {};
    if (frs_get_payload(robot, 0, &mass, com) == FRS_OK) {
        std::printf("payload arm0: mass=%.3f com=%.4f %.4f %.4f\n",
                    mass, com[0], com[1], com[2]);
    }

    frs_stop(robot);
    frs_destroy(robot);
    std::printf("ok\n");
    return 0;
}
