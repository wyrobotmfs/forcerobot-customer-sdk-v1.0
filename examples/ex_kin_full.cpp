/**
 * 离线运动学全覆盖（不连总线）
 * 对应需求：正逆解 / 连续IK / 臂角 / MOVL / 多点MOVL
 *
 *   ./ex_kin_full ../config 0
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

    double q[7] = {0, 10, 0, 40, 0, 20, 0};
    frs_pose pose{};

    // 1) FK
    if (frs_kin_fk(kin, q, &pose) != FRS_OK) {
        std::fprintf(stderr, "fk failed\n");
        frs_kin_destroy(kin);
        return 2;
    }
    std::printf("FK xyz=(%.4f,%.4f,%.4f) rpy=(%.2f,%.2f,%.2f)\n",
                pose.position_m[0], pose.position_m[1], pose.position_m[2],
                pose.rpy_deg[0], pose.rpy_deg[1], pose.rpy_deg[2]);

    // 2) 参考角 IK
    double q_ik[7] = {};
    if (frs_kin_ik(kin, &pose, q, q_ik) == FRS_OK) {
        std::printf("IK near ok\n");
    } else {
        std::printf("IK near rejected\n");
    }

    // 3) 连续 IK：沿 Z 抬高若干步，以上一步解为参考
    frs_pose step = pose;
    double q_ref[7];
    for (int j = 0; j < 7; ++j) q_ref[j] = q[j];
    int ok_steps = 0;
    for (int i = 0; i < 5; ++i) {
        step.position_m[2] += 0.005;
        double q_out[7] = {};
        if (frs_kin_ik(kin, &step, q_ref, q_out) != FRS_OK) break;
        for (int j = 0; j < 7; ++j) q_ref[j] = q_out[j];
        ++ok_steps;
    }
    std::printf("continuous IK steps=%d\n", ok_steps);

    // 4) 臂角
    double phi = 0, amin = 0, amax = 0;
    if (frs_kin_measure_arm_angle(kin, q, &phi) == FRS_OK)
        std::printf("arm_angle=%.2f deg\n", phi);
    if (frs_kin_arm_angle_range(kin, q, &amin, &amax) == FRS_OK)
        std::printf("arm_angle range=[%.2f, %.2f]\n", amin, amax);
    double q_phi[7] = {};
    if (frs_kin_solve_arm_angle(kin, q, phi, q_phi) == FRS_OK)
        std::printf("solve_arm_angle ok\n");

    // 5) 直线 MOVL
    frs_pose tgt = pose;
    tgt.position_m[2] += 0.02;
    frs_traj* traj = nullptr;
    if (frs_kin_plan_line(kin, q, &tgt, 0.05, 0.2, 1.0, 0.0, 0.002, 30000, &traj) ==
        FRS_OK) {
        std::printf("plan_line samples=%zu\n", frs_traj_count(traj));
        frs_traj_destroy(traj);
    } else {
        std::printf("plan_line failed\n");
    }

    // 6) 多点 MOVL（末点 blend=0）
    frs_path_point pts[2] = {};
    pts[0].values[0] = pose.position_m[0];
    pts[0].values[1] = pose.position_m[1];
    pts[0].values[2] = pose.position_m[2] + 0.01;
    pts[0].values[3] = pose.rpy_deg[0];
    pts[0].values[4] = pose.rpy_deg[1];
    pts[0].values[5] = pose.rpy_deg[2];
    pts[0].speed = 0.05;
    pts[0].rotation_speed = 0.2;
    pts[0].accel = 1.0;
    pts[0].dwell_s = 0.0;
    pts[0].blend = 0.005;

    pts[1] = pts[0];
    pts[1].values[2] = pose.position_m[2] + 0.02;
    pts[1].blend = 0.0;  // 末点必须 0

    traj = nullptr;
    if (frs_kin_plan_tcp_path(kin, q, pts, 2, 0.002, 50000, &traj) == FRS_OK) {
        std::printf("plan_tcp_path samples=%zu\n", frs_traj_count(traj));
        frs_traj_sample s{};
        if (frs_traj_count(traj) > 0 &&
            frs_traj_copy(traj, frs_traj_count(traj) - 1, 1, &s, 1) == FRS_OK) {
            std::printf("last sample t=%.3f q0=%.3f\n", s.time_s, s.joint_deg[0]);
        }
        frs_traj_destroy(traj);
    } else {
        std::printf("plan_tcp_path failed\n");
    }

    // 工具 TCP（离线模型）
    frs_pose tcp{};
    tcp.position_m[2] = 0.05;
    if (frs_kin_set_tcp(kin, &tcp) == FRS_OK)
        std::printf("kin set_tcp ok\n");

    frs_kin_destroy(kin);
    std::printf("ok (tool dynamics identify: NOT in SDK)\n");
    return 0;
}
