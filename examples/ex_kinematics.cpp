/**
 * 离线运动学示例：FK / IK / MOVL 规划（不连总线）。
 * 用法：./ex_kinematics ../config 0
 */
#include "forcerobot_sdk.h"

#include <cstdio>

int main(int argc, char** argv) {
    const char* config = (argc > 1) ? argv[1] : "../config";
    int arm = (argc > 2) ? argv[2][0] - '0' : 0;
    if (arm < 0 || arm > 1) arm = 0;

    frs_kin* kin = frs_kin_create(config, arm);
    if (!kin) {
        std::fprintf(stderr, "frs_kin_create failed\n");
        return 1;
    }

    double q[7] = {0, 0, 0, 0, 0, 0, 0};
    frs_pose pose{};
    if (frs_kin_fk(kin, q, &pose) != FRS_OK) {
        std::fprintf(stderr, "fk failed\n");
        frs_kin_destroy(kin);
        return 2;
    }
    std::printf("FK xyz=(%.4f,%.4f,%.4f) rpy=(%.2f,%.2f,%.2f)\n",
                pose.position_m[0], pose.position_m[1], pose.position_m[2],
                pose.rpy_deg[0], pose.rpy_deg[1], pose.rpy_deg[2]);

    double q_out[7] = {};
    if (frs_kin_ik(kin, &pose, q, q_out) == FRS_OK) {
        std::printf("IK ok q0=%.3f\n", q_out[0]);
    } else {
        std::printf("IK rejected (构型可能不可达/退化)\n");
    }

    frs_pose target = pose;
    target.position_m[2] += 0.02;  // 沿 Z 抬 2cm
    frs_traj* traj = nullptr;
    int rc = frs_kin_plan_line(kin, q, &target,
                               0.05, 0.2, 1.0, 0.0,
                               0.002, 20000, &traj);
    if (rc == FRS_OK && traj) {
        std::printf("plan_line samples=%zu\n", frs_traj_count(traj));
        frs_traj_destroy(traj);
    } else {
        std::printf("plan_line failed rc=%d\n", rc);
    }

    frs_kin_destroy(kin);
    return 0;
}
