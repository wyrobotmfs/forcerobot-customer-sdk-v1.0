/**
 * 工具 TCP / 负载 / 工具系相对运动（需求：设置和移除工具、payload、tool 点到点）
 *   sudo ./ex_hw_tcp_tool ../config eth0 0
 * FakeBus：./ex_hw_tcp_tool ../config fake 0
 *
 * 流程：读 TCP → set_tcp(+5cm Z) → set_payload → move_tool 抬 Z 5mm → clear_tcp
 * 真机请确认工作空间后再输入 y。
 */
#include "hw_util.hpp"

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

    frs_pose tcp{};
    if (frs_get_tcp(robot, arm, &tcp) == FRS_OK) {
        std::printf("get_tcp xyz=(%.4f,%.4f,%.4f)\n",
                    tcp.position_m[0], tcp.position_m[1], tcp.position_m[2]);
    }

    frs_pose offset{};
    offset.position_m[2] = 0.05;
    frs_result acc{};
    int rc = frs_set_tcp(robot, arm, &offset, 10, &acc);
    std::printf("set_tcp rc=%d %s\n", rc, acc.message);

    double com[3] = {0, 0, 0.02};
    rc = frs_set_payload(robot, arm, 0.5, com, 11, &acc);
    std::printf("set_payload rc=%d %s\n", rc, acc.message);
    double mass = 0;
    double com_r[3] = {};
    if (frs_get_payload(robot, arm, &mass, com_r) == FRS_OK)
        std::printf("get_payload mass=%.3f com=(%.4f,%.4f,%.4f)\n",
                    mass, com_r[0], com_r[1], com_r[2]);

    if (!frs_confirm_y("enable + move_tool +5mm Z? type y: ")) {
        frs_clear_tcp(robot, arm, 12, nullptr);
        frs_stop(robot);
        frs_destroy(robot);
        return 0;
    }
    if (frs_enable_arm(robot, arm) != FRS_OK) {
        frs_stop(robot);
        frs_destroy(robot);
        return 5;
    }

    double dxyz[3] = {0, 0, 0.005};
    double drpy[3] = {0, 0, 0};
    const uint64_t tag = 100;
    rc = frs_move_tool(robot, arm, dxyz, drpy, 0.03, 0.2, 0.3, 1.0, tag, &acc);
    std::printf("move_tool rc=%d %s\n", rc, acc.message);
    if (rc == FRS_OK) std::printf("wait rc=%d\n", frs_wait_tag(&wait, tag, 20000));

    frs_hold(robot, arm, 101, nullptr);
    rc = frs_clear_tcp(robot, arm, 102, &acc);
    std::printf("clear_tcp rc=%d %s\n", rc, acc.message);
    frs_disable(robot, arm, 103, nullptr);
    frs_stop(robot);
    frs_destroy(robot);
    return 0;
}
