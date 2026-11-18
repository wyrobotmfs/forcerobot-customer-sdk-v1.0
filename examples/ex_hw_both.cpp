/**
 * 双臂近似同步（需求：双臂同步运动 —— SDK 不保证原子同启）
 *   sudo ./ex_hw_both ../config eth0
 *
 * 依次 enable 0/1，再几乎连续下发两臂 move_joint（小幅 j6）。
 */
#include "hw_util.hpp"

#include <algorithm>

static const double kSoftMin[7] = {-180, -90, -180, 0, -180, -47, -47};
static const double kSoftMax[7] = {180, 90, 180, 130, 180, 47, 47};

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s <config> <ifname|fake>\n", argv[0]);
        return 1;
    }
    const char* config = argv[1];
    const char* ifname = argv[2];
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
    if (!frs_confirm_y("enable both arms + move_joint? type y: ")) {
        frs_stop(robot);
        frs_destroy(robot);
        return 0;
    }
    if (frs_enable_arm(robot, 0) != FRS_OK || frs_enable_arm(robot, 1) != FRS_OK) {
        frs_stop(robot);
        frs_destroy(robot);
        return 5;
    }
    frs_get_state(robot, &st);

    double q0[7], q1[7];
    for (int j = 0; j < 7; ++j) {
        q0[j] = st.measured_q_deg[0][j];
        q1[j] = st.measured_q_deg[1][j];
    }
    q0[5] = std::min(kSoftMax[5], std::max(kSoftMin[5], q0[5] + 2.0));
    q1[5] = std::min(kSoftMax[5], std::max(kSoftMin[5], q1[5] + 2.0));

    frs_result a0{}, a1{};
    int r0 = frs_move_joint(robot, 0, q0, 8.0, 30.0, 100, &a0);
    int r1 = frs_move_joint(robot, 1, q1, 8.0, 30.0, 101, &a1);
    std::printf("move arm0 rc=%d %s\n", r0, a0.message);
    std::printf("move arm1 rc=%d %s\n", r1, a1.message);
    if (r0 == FRS_OK) frs_wait_tag(&wait, 100, 30000);
    if (r1 == FRS_OK) frs_wait_tag(&wait, 101, 30000);

    frs_hold(robot, 0, 110, nullptr);
    frs_hold(robot, 1, 111, nullptr);
    frs_disable(robot, 0, 120, nullptr);
    frs_disable(robot, 1, 121, nullptr);
    frs_stop(robot);
    frs_destroy(robot);
    return 0;
}
