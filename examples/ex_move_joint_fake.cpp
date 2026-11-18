/**
 * FakeBus 下发关节运动示例（无真实机械运动）。
 * 真机请用 ex_hw_move_joint，并现场具备急停。
 *
 * 用法：./ex_move_joint_fake ../config
 */
#include "forcerobot_sdk.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>

static std::atomic<uint64_t> g_arrived{0};
static std::atomic<uint64_t> g_failed{0};

static void on_result(const frs_result* r, void*) {
    if (!r) return;
    std::printf("[result] arm=%d kind=%d tag=%llu %s\n",
                r->arm, r->kind, (unsigned long long)r->tag, r->message);
    if (r->kind == FRS_RESULT_ARRIVED) g_arrived.store(r->tag);
    if (r->kind == FRS_RESULT_FAILED || r->kind == FRS_RESULT_REJECTED)
        g_failed.store(r->tag);
}

static int wait_tag(uint64_t tag, int timeout_ms) {
    const int step = 50;
    for (int t = 0; t <= timeout_ms; t += step) {
        if (g_failed.load() == tag) return FRS_ERR_REJECTED;
        if (g_arrived.load() == tag) return FRS_OK;
        std::this_thread::sleep_for(std::chrono::milliseconds(step));
    }
    return FRS_ERR_TIMEOUT;
}

int main(int argc, char** argv) {
    const char* config = (argc > 1) ? argv[1] : "../config";

    frs_options opt{};
    opt.use_ethercat = 0;  // FakeBus
    opt.interface_name = nullptr;
    opt.log_every_cycles = 0;

    frs_robot* robot = frs_create(config, &opt);
    if (!robot) {
        std::fprintf(stderr, "create failed\n");
        return 1;
    }
    frs_set_result_callback(robot, on_result, nullptr);
    if (frs_start(robot) != FRS_OK) {
        frs_destroy(robot);
        return 2;
    }

    // 等状态可读（FakeBus 一般很快）
    frs_state st{};
    for (int i = 0; i < 50; ++i) {
        if (frs_get_state(robot, &st) == FRS_OK) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::printf("cycle=%llu bus_ready=%d\n",
                (unsigned long long)st.cycle, st.bus_ready);

    frs_result acc{};
    g_arrived.store(0);
    g_failed.store(0);
    int rc = frs_enable(robot, 0, 1, &acc);
    std::printf("enable rc=%d kind=%d %s\n", rc, acc.kind, acc.message);
    if (rc != FRS_OK) {
        frs_stop(robot);
        frs_destroy(robot);
        return 3;
    }
    rc = wait_tag(1, 5000);
    std::printf("wait enable rc=%d\n", rc);
    if (rc != FRS_OK) {
        frs_disable(robot, 0, 2, nullptr);
        frs_stop(robot);
        frs_destroy(robot);
        return 4;
    }

    // 目标必须在软限位内：j4(索引3) ≥ 0°；j6/j7 ∈ [-47,47]
    // 相对当前角小幅运动，避免写死不合理姿态
    if (frs_get_state(robot, &st) != FRS_OK) {
        frs_disable(robot, 0, 2, nullptr);
        frs_stop(robot);
        frs_destroy(robot);
        return 5;
    }
    double q[7];
    for (int j = 0; j < 7; ++j) q[j] = st.measured_q_deg[0][j];
    q[3] = (q[3] < 5.0) ? 10.0 : q[3];  // 保证 j4 不低于限位
    q[5] = q[5] + 2.0;
    if (q[5] > 47.0) q[5] = 45.0;
    if (q[5] < -47.0) q[5] = -45.0;

    std::printf("target q:");
    for (int j = 0; j < 7; ++j) std::printf(" %.3f", q[j]);
    std::printf("\n");

    g_arrived.store(0);
    g_failed.store(0);
    rc = frs_move_joint(robot, 0, q, 10.0, 50.0, 100, &acc);
    std::printf("move_joint rc=%d kind=%d %s\n", rc, acc.kind, acc.message);
    if (rc == FRS_OK) {
        rc = wait_tag(100, 10000);
        std::printf("wait move rc=%d\n", rc);
    }

    if (frs_get_state(robot, &st) == FRS_OK) {
        std::printf("cmd q0=%.2f meas q0=%.2f meas q3=%.2f q5=%.2f\n",
                    st.command_q_deg[0][0], st.measured_q_deg[0][0],
                    st.measured_q_deg[0][3], st.measured_q_deg[0][5]);
    }

    frs_disable(robot, 0, 2, nullptr);
    wait_tag(2, 3000);
    frs_stop(robot);
    frs_destroy(robot);
    return 0;
}
