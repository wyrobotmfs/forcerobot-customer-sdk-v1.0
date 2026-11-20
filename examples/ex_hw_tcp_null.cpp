/**
 * TCP + 手推摆肘（tcp_null，对齐 1440 交付能力）
 *   sudo ./ex_hw_tcp_null ../config eth0 0
 *   sudo ./ex_hw_tcp_null ../config eth0 0 0 0 0.01
 *   ./ex_hw_tcp_null ../config fake 0 0.01 0 0
 *
 * 参数：config  ifname  [arm]  [dx dy dz]（米，相对当前 TCP；默认 0 0 0.01）
 * 未标定外力时仍可位置运动，只不叠加摆肘。
 */
#include "hw_util.hpp"

#include <cstdlib>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr,
                     "usage: %s <config> <ifname|fake> [arm] [dx dy dz]\n"
                     "  dx/dy/dz in meters, relative to current TCP (default 0 0 0.01)\n",
                     argv[0]);
        return 1;
    }
    const char* config = argv[1];
    const char* ifname = argv[2];
    int arm = (argc > 3) ? argv[3][0] - '0' : 0;
    double dx = 0.0, dy = 0.0, dz = 0.01;
    if (argc >= 7) {
        dx = std::atof(argv[4]);
        dy = std::atof(argv[5]);
        dz = std::atof(argv[6]);
    } else if (argc != 3 && argc != 4) {
        std::fprintf(stderr, "need all three of dx dy dz, or omit all for default +1cm Z\n");
        return 1;
    }
    if (arm < 0 || arm > 1) {
        std::fprintf(stderr, "arm must be 0 or 1\n");
        return 1;
    }
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

    frs_pose cur{};
    cur.position_m[0] = st.tcp_pose[arm][0];
    cur.position_m[1] = st.tcp_pose[arm][1];
    cur.position_m[2] = st.tcp_pose[arm][2];
    cur.rpy_deg[0] = st.tcp_pose[arm][3];
    cur.rpy_deg[1] = st.tcp_pose[arm][4];
    cur.rpy_deg[2] = st.tcp_pose[arm][5];
    std::printf("current tcp xyz=(%.4f,%.4f,%.4f) rpy=(%.2f,%.2f,%.2f)\n",
                cur.position_m[0], cur.position_m[1], cur.position_m[2],
                cur.rpy_deg[0], cur.rpy_deg[1], cur.rpy_deg[2]);

    frs_pose tgt = cur;
    tgt.position_m[0] += dx;
    tgt.position_m[1] += dy;
    tgt.position_m[2] += dz;
    std::printf("target xyz=(%.4f,%.4f,%.4f) via move_tcp_null\n",
                tgt.position_m[0], tgt.position_m[1], tgt.position_m[2]);

    char prompt[160];
    std::snprintf(prompt, sizeof(prompt),
                  "enable+move_tcp_null d=(%.3f,%.3f,%.3f)m? type y: ", dx, dy, dz);
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

    frs_result acc{};
    const uint64_t tag = 110;
    int rc = frs_move_tcp_null(robot, arm, &tgt, 0.05, 0.2, 0.5, 1.0, tag, &acc);
    std::printf("move_tcp_null rc=%d %s\n", rc, acc.message);
    if (rc == FRS_OK) {
        int wr = frs_wait_tag(&wait, tag, 20000);
        std::printf("wait tag rc=%d\n", wr);
    }

    frs_hold(robot, arm, 111, nullptr);
    frs_disable(robot, arm, 112, nullptr);
    frs_stop(robot);
    frs_destroy(robot);
    return 0;
}
