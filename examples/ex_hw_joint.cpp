/**
 * 真机低速关节：等 bus_ready → enable → 在限位内改一小角 → ARRIVED → disable
 *
 * 用法（先 quit runtime_dual，急停在手）：
 *   sudo ./ex_hw_joint ../config eth0 0 5 3.0
 * 参数：config  ifname  arm  joint_index(0..6)  delta_deg
 *
 * 注意：j4 软限位下限为 0°。实测角可能因噪声略小于 0，直接当目标会被拒。
 * 本示例会对 7 轴目标做软限位钳位；ARRIVED 只认 move 的 tag=100。
 */
#include "forcerobot_sdk.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

static std::atomic<int> g_move_arrived{0};
static std::atomic<int> g_move_failed{0};

// 与 config/parameter.json 本包默认一致（双臂相同）
static const double kSoftMin[7] = {-180, -90, -180, 0, -180, -47, -47};
static const double kSoftMax[7] = {180, 90, 180, 130, 180, 47, 47};

static void on_result(const frs_result* r, void*) {
    if (!r) return;
    std::printf("[result] arm=%d kind=%d tag=%llu %s\n",
                r->arm, r->kind, (unsigned long long)r->tag, r->message);
    // 只跟踪本次 move_joint 的 tag=100，避免 enable_done(kind=ARRIVED) 误判
    if (r->tag != 100) return;
    if (r->kind == FRS_RESULT_ARRIVED) g_move_arrived.store(1);
    if (r->kind == FRS_RESULT_FAILED || r->kind == FRS_RESULT_REJECTED)
        g_move_failed.store(1);
}

static double clamp_joint(int j, double q) {
    return std::min(kSoftMax[j], std::max(kSoftMin[j], q));
}

static void clamp_all(double q[7]) {
    for (int j = 0; j < 7; ++j) q[j] = clamp_joint(j, q[j]);
}

static int wait_bus_ready(frs_robot* robot, frs_state* st, int timeout_ms) {
    const int step_ms = 100;
    std::printf("waiting bus_ready (timeout %d ms)...\n", timeout_ms);
    std::fflush(stdout);
    for (int waited = 0; waited <= timeout_ms; waited += step_ms) {
        if (frs_get_state(robot, st) == FRS_OK && st->bus_ready) {
            std::printf("bus_ready=1 after %d ms, cycle=%llu\n",
                        waited, (unsigned long long)st->cycle);
            return FRS_OK;
        }
        if (waited > 0 && (waited % 1000) == 0) {
            std::printf("  ... still waiting (%d ms), bus_ready=%d\n",
                        waited, st->bus_ready);
            std::fflush(stdout);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
    }
    std::fprintf(stderr, "timeout waiting bus_ready (last=%d)\n", st->bus_ready);
    return FRS_ERR_NOT_READY;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr,
                     "usage: %s <config_dir> <ifname> [arm] [joint] [delta_deg]\n",
                     argv[0]);
        return 1;
    }
    const char* config = argv[1];
    const char* ifname = argv[2];
    int arm = (argc > 3) ? std::atoi(argv[3]) : 0;
    int joint = (argc > 4) ? std::atoi(argv[4]) : 5;
    double delta = (argc > 5) ? std::atof(argv[5]) : 3.0;

    if (arm < 0 || arm > 1 || joint < 0 || joint > 6) {
        std::fprintf(stderr, "arm must be 0/1, joint 0..6\n");
        return 1;
    }
    frs_options opt{};
    opt.use_ethercat = 1;
    opt.interface_name = ifname;
    opt.log_every_cycles = 0;

    frs_robot* robot = frs_create(config, &opt);
    if (!robot) {
        std::fprintf(stderr, "frs_create failed (quit runtime_dual? clear lock?)\n");
        return 2;
    }
    frs_set_result_callback(robot, on_result, nullptr);
    if (frs_start(robot) != FRS_OK) {
        std::fprintf(stderr, "frs_start failed\n");
        frs_destroy(robot);
        return 3;
    }

    frs_state st{};
    if (wait_bus_ready(robot, &st, 15000) != FRS_OK) {
        frs_stop(robot);
        frs_destroy(robot);
        return 4;
    }

    frs_result acc{};
    int rc = frs_enable(robot, arm, 1, &acc);
    std::printf("enable rc=%d kind=%d %s\n", rc, acc.kind, acc.message);
    if (rc != FRS_OK) {
        frs_stop(robot);
        frs_destroy(robot);
        return 5;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(800));

    // 使能后再读：优先用命令参考，其次实测；再钳到软限位
    if (frs_get_state(robot, &st) != FRS_OK) {
        frs_disable(robot, arm, 99, nullptr);
        frs_stop(robot);
        frs_destroy(robot);
        return 4;
    }

    double q[7];
    for (int j = 0; j < 7; ++j) q[j] = st.measured_q_deg[arm][j];
    clamp_all(q);  // 消除噪声越界，例如 j4 实测 -0.005 < soft_min 0

    std::printf("base q (clamped):");
    for (int j = 0; j < 7; ++j) std::printf(" %.3f", q[j]);
    std::printf("\n");
    std::printf("soft j%d limit = [%.1f, %.1f]\n",
                joint + 1, kSoftMin[joint], kSoftMax[joint]);

    q[joint] = clamp_joint(joint, q[joint] + delta);

    std::printf("target q:");
    for (int j = 0; j < 7; ++j) std::printf(" %.3f", q[j]);
    std::printf("\n");
    std::printf("arm=%d joint=%d delta=%+.2f  type y then Enter to MOVE: ",
                arm, joint, delta);
    std::fflush(stdout);
    char line[32] = {};
    if (!std::fgets(line, sizeof(line), stdin) || (line[0] != 'y' && line[0] != 'Y')) {
        std::printf("aborted\n");
        frs_disable(robot, arm, 99, nullptr);
        frs_stop(robot);
        frs_destroy(robot);
        return 0;
    }

    g_move_arrived.store(0);
    g_move_failed.store(0);
    rc = frs_move_joint(robot, arm, q, 5.0, 20.0, 100, &acc);
    std::printf("move_joint rc=%d kind=%d %s\n", rc, acc.kind, acc.message);
    if (rc != FRS_OK) {
        std::fprintf(stderr, "move rejected, skip waiting ARRIVED\n");
        frs_hold(robot, arm, 101, nullptr);
        frs_disable(robot, arm, 102, nullptr);
        frs_stop(robot);
        frs_destroy(robot);
        return 6;
    }

    for (int i = 0; i < 100 && !g_move_arrived.load() && !g_move_failed.load(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (frs_get_state(robot, &st) == FRS_OK) {
        std::printf("after measured q:");
        for (int j = 0; j < 7; ++j)
            std::printf(" %.3f", st.measured_q_deg[arm][j]);
        std::printf("\n");
    }

    frs_hold(robot, arm, 101, nullptr);
    frs_disable(robot, arm, 102, nullptr);
    frs_stop(robot);
    frs_destroy(robot);

    if (g_move_failed.load()) return 6;
    if (!g_move_arrived.load()) {
        std::fprintf(stderr, "timeout waiting ARRIVED (check measured anyway)\n");
        return 7;
    }
    std::printf("ok\n");
    return 0;
}
