/**
 * 实时订阅 + CSV 采集（需求：实时反馈/订阅/数据采集保存）
 *   sudo ./ex_hw_subscribe_csv ../config eth0
 *   ./ex_hw_subscribe_csv ../config fake
 *
 * 订阅每 50 周期打印一次；同时写 CSV，约 3s 后停止。
 */
#include "hw_util.hpp"

#include <atomic>

struct SubCtx {
    std::atomic<int> count{0};
};

static void on_state(const frs_state* st, void* user) {
    auto* ctx = static_cast<SubCtx*>(user);
    if (!st || !ctx) return;
    int n = ctx->count.fetch_add(1);
    if (n % 10 != 0) return;  // 再降采样打印
    std::printf("[sub#%d] cycle=%llu q0=%.2f tcp_z=%.4f Fz=%.2f I0=%d\n",
                n,
                (unsigned long long)st->cycle,
                st->measured_q_deg[0][0],
                st->tcp_pose[0][2],
                st->force_6d[0][2],
                (int)st->current_ma[0][0]);
}

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
    frs_robot* robot = frs_create(config, &opt);
    if (!robot) return 2;
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

    SubCtx ctx;
    int rc = frs_subscribe_state(robot, 50, on_state, &ctx);
    std::printf("subscribe_state rc=%d\n", rc);

    rc = frs_csv_start(robot, "frs_gather.csv");
    std::printf("csv_start rc=%d\n", rc);
    char path[512] = {};
    if (frs_csv_get_path(robot, path, sizeof(path)) == FRS_OK)
        std::printf("csv path: %s\n", path);

    std::printf("gathering ~3s...\n");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    frs_csv_stop(robot);
    frs_unsubscribe_state(robot);
    std::printf("subscribe callbacks=%d\n", ctx.count.load());

    // 单次快照：位置/速度/电流/外力/TCP
    if (frs_get_state(robot, &st) == FRS_OK) {
        frs_print_q("q_deg:", st.measured_q_deg[0]);
        frs_print_q("qd:", st.measured_qd_deg_s[0]);
        std::printf("tcp xyz=(%.4f,%.4f,%.4f) rpy=(%.2f,%.2f,%.2f)\n",
                    st.tcp_pose[0][0], st.tcp_pose[0][1], st.tcp_pose[0][2],
                    st.tcp_pose[0][3], st.tcp_pose[0][4], st.tcp_pose[0][5]);
        std::printf("ext_tau0=%.3f force Fz=%.3f I0=%d\n",
                    st.external_torque_nm[0][0], st.force_6d[0][2],
                    (int)st.current_ma[0][0]);
    }

    frs_stop(robot);
    frs_destroy(robot);
    return 0;
}
