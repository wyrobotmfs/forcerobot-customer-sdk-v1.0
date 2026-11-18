/**
 * 停顿式多点关节（需求：停顿式多点 / 多点运动）
 *   sudo ./ex_hw_path ../config eth0 0
 * 在当前关节角附近走两点（j6 ±3°），每点 dwell=0.5s。
 */
#include "hw_util.hpp"

#include <algorithm>

static const double kSoftMin[7] = {-180, -90, -180, 0, -180, -47, -47};
static const double kSoftMax[7] = {180, 90, 180, 130, 180, 47, 47};

static void clamp7(double q[7]) {
    for (int j = 0; j < 7; ++j)
        q[j] = std::min(kSoftMax[j], std::max(kSoftMin[j], q[j]));
}

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
    if (!frs_confirm_y("enable + joint_path 2pts? type y: ")) {
        frs_stop(robot);
        frs_destroy(robot);
        return 0;
    }
    if (frs_enable_arm(robot, arm) != FRS_OK) {
        frs_stop(robot);
        frs_destroy(robot);
        return 5;
    }
    if (frs_get_state(robot, &st) != FRS_OK) {
        frs_stop(robot);
        frs_destroy(robot);
        return 4;
    }

    double q0[7], q1[7];
    for (int j = 0; j < 7; ++j) {
        q0[j] = st.measured_q_deg[arm][j];
        q1[j] = q0[j];
    }
    q0[5] = std::min(kSoftMax[5], q0[5] + 3.0);
    q1[5] = std::max(kSoftMin[5], q1[5] - 3.0);
    clamp7(q0);
    clamp7(q1);

    frs_path_point pts[2] = {};
    for (int j = 0; j < 7; ++j) {
        pts[0].values[j] = q0[j];
        pts[1].values[j] = q1[j];
    }
    pts[0].speed = 8;
    pts[0].accel = 30;
    pts[0].dwell_s = 0.5;
    pts[0].blend = 0;
    pts[1].speed = 8;
    pts[1].accel = 30;
    pts[1].dwell_s = 0.3;
    pts[1].blend = 0;

    frs_print_q("pt0:", q0);
    frs_print_q("pt1:", q1);

    const uint64_t tag = 200;
    frs_result acc{};
    int rc = frs_joint_path(robot, arm, pts, 2, tag, &acc);
    std::printf("joint_path rc=%d %s\n", rc, acc.message);
    if (rc == FRS_OK) std::printf("wait rc=%d\n", frs_wait_tag(&wait, tag, 60000));

    frs_hold(robot, arm, 201, nullptr);
    frs_disable(robot, arm, 202, nullptr);
    frs_stop(robot);
    frs_destroy(robot);
    return 0;
}
