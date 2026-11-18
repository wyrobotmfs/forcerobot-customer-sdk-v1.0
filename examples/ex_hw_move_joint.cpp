/**
 * 真机：单臂 / 双臂绝对关节运动
 *
 * 交互：
 *   sudo ./ex_hw_move_joint ../config eth0
 *     0  0 0 0 30 0 0 0              # 单臂
 *     1  0 0 0 20 0 0 0 10 50
 *     both <臂0的7角> <臂1的7角> [speed] [accel]
 *     both 0 0 0 30 0 0 0   0 0 0 20 0 0 0
 *     both 0 0 0 30 0 0 0   0 0 0 20 0 0 0  10 50
 *     q
 *
 * 说明：双臂为连续两次 move_joint，尽量同时启动，但不保证硬件级硬同步。
 */
#include "forcerobot_sdk.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

static std::atomic<uint64_t> g_arrived_mask{0};  // bit0=tagA, bit1=tagB 完成
static std::atomic<uint64_t> g_failed_mask{0};
static uint64_t g_tag_a = 0;
static uint64_t g_tag_b = 0;

static const double kSoftMin[7] = {-180, -90, -180, 0, -180, -47, -47};
static const double kSoftMax[7] = {180, 90, 180, 130, 180, 47, 47};

static void on_result(const frs_result* r, void*) {
    if (!r) return;
    std::printf("[result] arm=%d kind=%d tag=%llu %s\n",
                r->arm, r->kind, (unsigned long long)r->tag, r->message);
    uint64_t bit = 0;
    if (g_tag_a && r->tag == g_tag_a) bit = 1ull;
    if (g_tag_b && r->tag == g_tag_b) bit = 2ull;
    if (!bit) return;
    if (r->kind == FRS_RESULT_ARRIVED) g_arrived_mask.fetch_or(bit);
    if (r->kind == FRS_RESULT_FAILED || r->kind == FRS_RESULT_REJECTED)
        g_failed_mask.fetch_or(bit);
}

static int wait_bus_ready(frs_robot* robot, frs_state* st, int timeout_ms) {
    const int step_ms = 100;
    std::printf("waiting bus_ready...\n");
    for (int w = 0; w <= timeout_ms; w += step_ms) {
        if (frs_get_state(robot, st) == FRS_OK && st->bus_ready) {
            std::printf("bus_ready=1 after %d ms\n", w);
            return FRS_OK;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
    }
    std::fprintf(stderr, "timeout waiting bus_ready\n");
    return FRS_ERR_NOT_READY;
}

static void clamp_all(double q[7], int* clamped) {
    *clamped = 0;
    for (int j = 0; j < 7; ++j) {
        const double c = std::min(kSoftMax[j], std::max(kSoftMin[j], q[j]));
        if (c != q[j]) *clamped = 1;
        q[j] = c;
    }
}

static void print_q(const char* label, const double q[7]) {
    std::printf("%s", label);
    for (int j = 0; j < 7; ++j) std::printf(" %.3f", q[j]);
    std::printf("\n");
}

static int enable_arm(frs_robot* robot, int arm, int enabled[2]) {
    if (enabled[arm]) return FRS_OK;
    frs_result acc{};
    int rc = frs_enable(robot, arm, static_cast<uint64_t>(10 + arm), &acc);
    std::printf("enable arm%d rc=%d %s\n", arm, rc, acc.message);
    if (rc != FRS_OK) return rc;
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    enabled[arm] = 1;
    return FRS_OK;
}

static int enable_both(frs_robot* robot, int enabled[2]) {
    if (enable_arm(robot, 0, enabled) != FRS_OK) return FRS_ERR_REJECTED;
    if (enable_arm(robot, 1, enabled) != FRS_OK) return FRS_ERR_REJECTED;
    return FRS_OK;
}

static int submit_move(frs_robot* robot, int arm, double q[7],
                       double speed, double accel, uint64_t tag) {
    int clamped = 0;
    clamp_all(q, &clamped);
    if (clamped) {
        std::printf("warning: arm%d clamped\n", arm);
        print_q("  target:", q);
    } else {
        char label[32];
        std::snprintf(label, sizeof(label), "arm%d target:", arm);
        print_q(label, q);
    }
    frs_result acc{};
    int rc = frs_move_joint(robot, arm, q, speed, accel, tag, &acc);
    std::printf("move_joint arm%d rc=%d kind=%d %s\n", arm, rc, acc.kind, acc.message);
    return rc;
}

static int wait_tags(uint64_t need_mask, int timeout_ms) {
    const int step = 100;
    for (int w = 0; w <= timeout_ms; w += step) {
        const uint64_t failed = g_failed_mask.load();
        if (failed & need_mask) {
            std::fprintf(stderr, "move failed mask=0x%llx\n",
                         (unsigned long long)failed);
            return FRS_ERR_REJECTED;
        }
        if ((g_arrived_mask.load() & need_mask) == need_mask) {
            std::printf("ARRIVED ok mask=0x%llx\n", (unsigned long long)need_mask);
            return FRS_OK;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(step));
    }
    std::fprintf(stderr, "timeout waiting ARRIVED mask=0x%llx got=0x%llx\n",
                 (unsigned long long)need_mask,
                 (unsigned long long)g_arrived_mask.load());
    return FRS_ERR_TIMEOUT;
}

static int do_move_one(frs_robot* robot, int arm, double q[7],
                       double speed, double accel, uint64_t tag) {
    g_tag_a = tag;
    g_tag_b = 0;
    g_arrived_mask.store(0);
    g_failed_mask.store(0);
    if (submit_move(robot, arm, q, speed, accel, tag) != FRS_OK) return FRS_ERR_REJECTED;
    return wait_tags(1ull, 30000);
}

/** 双臂连续下发，尽量同时动（非硬同步） */
static int do_move_both(frs_robot* robot, double q0[7], double q1[7],
                        double speed, double accel, uint64_t tag0, uint64_t tag1) {
    g_tag_a = tag0;
    g_tag_b = tag1;
    g_arrived_mask.store(0);
    g_failed_mask.store(0);

    std::printf("dual move (best-effort sync, not hard-realtime)\n");
    const int rc0 = submit_move(robot, 0, q0, speed, accel, tag0);
    const int rc1 = submit_move(robot, 1, q1, speed, accel, tag1);
    if (rc0 != FRS_OK || rc1 != FRS_OK) return FRS_ERR_REJECTED;
    return wait_tags(1ull | 2ull, 30000);
}

/**
 * 返回：
 *  0 = quit
 *  1 = 单臂（*arm, q）
 *  2 = 双臂（q0, q1）
 * -1 = 解析错误
 */
static int parse_line(const char* line, int* arm, double q[7], double q1[7],
                      double* speed, double* accel) {
    if (!line) return -1;
    while (*line == ' ' || *line == '\t') ++line;
    if (*line == '\0' || *line == '\n') return -1;
    if (*line == 'q' || *line == 'Q') return 0;

    char buf[768];
    std::snprintf(buf, sizeof(buf), "%s", line);
    char* tok = std::strtok(buf, " \t\r\n");
    if (!tok) return -1;

    const int is_both =
        (std::strcmp(tok, "both") == 0 || std::strcmp(tok, "BOTH") == 0 ||
         std::strcmp(tok, "dual") == 0 || std::strcmp(tok, "DUAL") == 0);

    if (is_both) {
        for (int j = 0; j < 7; ++j) {
            tok = std::strtok(nullptr, " \t\r\n");
            if (!tok) return -1;
            q[j] = std::atof(tok);
        }
        for (int j = 0; j < 7; ++j) {
            tok = std::strtok(nullptr, " \t\r\n");
            if (!tok) return -1;
            q1[j] = std::atof(tok);
        }
        tok = std::strtok(nullptr, " \t\r\n");
        if (tok) *speed = std::atof(tok);
        tok = std::strtok(nullptr, " \t\r\n");
        if (tok) *accel = std::atof(tok);
        if (!(*speed > 0.0) || !(*accel > 0.0)) return -1;
        return 2;
    }

    *arm = std::atoi(tok);
    if (*arm < 0 || *arm > 1) return -1;
    for (int j = 0; j < 7; ++j) {
        tok = std::strtok(nullptr, " \t\r\n");
        if (!tok) return -1;
        q[j] = std::atof(tok);
    }
    tok = std::strtok(nullptr, " \t\r\n");
    if (tok) *speed = std::atof(tok);
    tok = std::strtok(nullptr, " \t\r\n");
    if (tok) *accel = std::atof(tok);
    if (!(*speed > 0.0) || !(*accel > 0.0)) return -1;
    return 1;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr,
                     "usage:\n"
                     "  %s <config> <ifname>\n"
                     "  %s <config> <ifname> <arm> <q1..q7> [speed] [accel]\n"
                     "  %s <config> <ifname> both <q0_1..7> <q1_1..7> [speed] [accel]\n",
                     argv[0], argv[0], argv[0]);
        return 1;
    }
    const char* config = argv[1];
    const char* ifname = argv[2];

    frs_options opt{};
    opt.use_ethercat = 1;
    opt.interface_name = ifname;
    opt.log_every_cycles = 0;

    frs_robot* robot = frs_create(config, &opt);
    if (!robot) {
        std::fprintf(stderr, "frs_create failed\n");
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

    int enabled[2] = {0, 0};
    uint64_t tag = 100;

    // 命令行 both：prog config ifname both q0x7 q1x7 [speed] [accel] => argc>=18
    if (argc >= 4 && (std::strcmp(argv[3], "both") == 0 || std::strcmp(argv[3], "dual") == 0)) {
        if (argc < 18) {
            std::fprintf(stderr, "both needs 14 joint values\n");
            frs_stop(robot);
            frs_destroy(robot);
            return 1;
        }
        double q0[7], q1[7];
        for (int j = 0; j < 7; ++j) q0[j] = std::atof(argv[4 + j]);
        for (int j = 0; j < 7; ++j) q1[j] = std::atof(argv[11 + j]);
        double speed = (argc >= 19) ? std::atof(argv[18]) : 10.0;
        double accel = (argc >= 20) ? std::atof(argv[19]) : 50.0;
        print_q("req arm0:", q0);
        print_q("req arm1:", q1);
        std::printf("type y to MOVE both: ");
        std::fflush(stdout);
        char line[32] = {};
        if (std::fgets(line, sizeof(line), stdin) && (line[0] == 'y' || line[0] == 'Y')) {
            if (enable_both(robot, enabled) == FRS_OK)
                do_move_both(robot, q0, q1, speed, accel, tag, tag + 1);
        } else {
            std::printf("aborted\n");
        }
    } else if (argc >= 11) {
        int arm = std::atoi(argv[3]);
        double q[7];
        for (int j = 0; j < 7; ++j) q[j] = std::atof(argv[4 + j]);
        double speed = (argc >= 12) ? std::atof(argv[11]) : 10.0;
        double accel = (argc >= 13) ? std::atof(argv[12]) : 50.0;
        print_q("request:", q);
        std::printf("arm=%d type y to MOVE: ", arm);
        std::fflush(stdout);
        char line[32] = {};
        if (std::fgets(line, sizeof(line), stdin) && (line[0] == 'y' || line[0] == 'Y')) {
            if (enable_arm(robot, arm, enabled) == FRS_OK)
                do_move_one(robot, arm, q, speed, accel, tag);
        } else {
            std::printf("aborted\n");
        }
    } else {
        std::printf("commands:\n");
        std::printf("  <arm> <q1..q7> [speed] [accel]\n");
        std::printf("  both <arm0 q1..q7> <arm1 q1..q7> [speed] [accel]\n");
        std::printf("  q\n");
        std::printf("note: dual is best-effort, not hard sync\n");
        char line[768];
        while (std::printf("> "), std::fflush(stdout), std::fgets(line, sizeof(line), stdin)) {
            int arm = 0;
            double q0[7] = {};
            double q1[7] = {};
            double speed = 10.0;
            double accel = 50.0;
            const int pr = parse_line(line, &arm, q0, q1, &speed, &accel);
            if (pr == 0) break;
            if (pr < 0) {
                std::printf("parse error\n");
                continue;
            }
            if (frs_get_state(robot, &st) == FRS_OK) {
                print_q("cur0:", st.measured_q_deg[0]);
                print_q("cur1:", st.measured_q_deg[1]);
            }
            tag += 2;
            if (pr == 2) {
                if (enable_both(robot, enabled) != FRS_OK) continue;
                do_move_both(robot, q0, q1, speed, accel, tag, tag + 1);
            } else {
                if (enable_arm(robot, arm, enabled) != FRS_OK) continue;
                do_move_one(robot, arm, q0, speed, accel, tag);
            }
        }
    }

    for (int a = 0; a < 2; ++a) {
        if (!enabled[a]) continue;
        frs_hold(robot, a, 200 + static_cast<uint64_t>(a), nullptr);
        frs_disable(robot, a, 210 + static_cast<uint64_t>(a), nullptr);
    }
    frs_stop(robot);
    frs_destroy(robot);
    std::printf("bye\n");
    return 0;
}
