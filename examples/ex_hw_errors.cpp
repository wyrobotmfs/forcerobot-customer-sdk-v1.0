/**
 * 驱动错误码刷新与读取（需求：获取错误码）
 *   sudo ./ex_hw_errors ../config eth0 0
 *
 * 说明：hold/disable 非硬件急停；本示例只演示 603F 缓存回读。
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

    frs_result acc{};
    int rc = frs_refresh_drive_errors(robot, arm, 1, &acc);
    std::printf("refresh_drive_errors rc=%d %s\n", rc, acc.message);
    if (rc == FRS_OK) frs_wait_tag(&wait, 1, 5000);

    uint16_t codes[7] = {};
    int valid[7] = {};
    rc = frs_get_drive_error_codes(robot, arm, codes, valid);
    std::printf("get_drive_error_codes rc=%d\n", rc);
    for (int j = 0; j < 7; ++j) {
        std::printf("  j%d valid=%d code=0x%04X\n", j, valid[j], (unsigned)codes[j]);
    }

    frs_stop(robot);
    frs_destroy(robot);
    return 0;
}
