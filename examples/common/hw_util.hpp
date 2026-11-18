#pragma once
/** 真机/FakeBus 示例共用工具 */
#include "forcerobot_sdk.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

inline int frs_is_fake(const char* ifname) {
    return !ifname || std::strcmp(ifname, "fake") == 0 ||
           std::strcmp(ifname, "FakeBus") == 0;
}

inline void frs_fill_options(frs_options* opt, const char* ifname) {
    std::memset(opt, 0, sizeof(*opt));
    if (frs_is_fake(ifname)) {
        opt->use_ethercat = 0;
        opt->interface_name = nullptr;
    } else {
        opt->use_ethercat = 1;
        opt->interface_name = ifname;
    }
    opt->log_every_cycles = 0;
}

/** require_bus=1：真机等 bus_ready；FakeBus 可读到状态即过 */
inline int frs_wait_ready(frs_robot* robot, frs_state* st, int timeout_ms, int require_bus) {
    const int step = 100;
    std::printf("waiting ready (require_bus=%d)...\n", require_bus);
    for (int w = 0; w <= timeout_ms; w += step) {
        if (frs_get_state(robot, st) == FRS_OK) {
            if (!require_bus || st->bus_ready) {
                std::printf("ready after %d ms bus_ready=%d cycle=%llu\n",
                            w, st->bus_ready, (unsigned long long)st->cycle);
                return FRS_OK;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(step));
    }
    std::fprintf(stderr, "timeout waiting ready\n");
    return FRS_ERR_NOT_READY;
}

struct FrsTagWait {
    std::atomic<uint64_t> arrived{0};
    std::atomic<uint64_t> failed{0};
    uint64_t watch{0};
};

inline void frs_tag_on_result(const frs_result* r, void* user) {
    auto* w = static_cast<FrsTagWait*>(user);
    if (!r || !w) return;
    std::printf("[result] arm=%d kind=%d tag=%llu %s\n",
                r->arm, r->kind, (unsigned long long)r->tag, r->message);
    if (w->watch && r->tag != w->watch) return;
    if (r->kind == FRS_RESULT_ARRIVED) w->arrived.store(r->tag);
    if (r->kind == FRS_RESULT_FAILED || r->kind == FRS_RESULT_REJECTED ||
        r->kind == FRS_RESULT_SUPERSEDED)
        w->failed.store(r->tag);
}

inline int frs_wait_tag(FrsTagWait* w, uint64_t tag, int timeout_ms) {
    w->watch = tag;
    w->arrived.store(0);
    w->failed.store(0);
    const int step = 100;
    for (int t = 0; t <= timeout_ms; t += step) {
        if (w->failed.load() == tag) return FRS_ERR_REJECTED;
        if (w->arrived.load() == tag) return FRS_OK;
        std::this_thread::sleep_for(std::chrono::milliseconds(step));
    }
    return FRS_ERR_TIMEOUT;
}

inline int frs_enable_arm(frs_robot* robot, int arm) {
    frs_result acc{};
    int rc = frs_enable(robot, arm, 1, &acc);
    std::printf("enable arm%d rc=%d %s\n", arm, rc, acc.message);
    if (rc != FRS_OK) return rc;
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    return FRS_OK;
}

inline void frs_print_q(const char* label, const double q[7]) {
    std::printf("%s", label);
    for (int j = 0; j < 7; ++j) std::printf(" %.3f", q[j]);
    std::printf("\n");
}

inline int frs_confirm_y(const char* prompt) {
    std::printf("%s", prompt);
    std::fflush(stdout);
    char line[32] = {};
    if (!std::fgets(line, sizeof(line), stdin)) return 0;
    return line[0] == 'y' || line[0] == 'Y';
}
