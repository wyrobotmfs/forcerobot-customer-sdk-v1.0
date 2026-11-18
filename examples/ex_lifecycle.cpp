/**
 * FakeBus 生命周期示例：不使能、不运动。
 * 用法（在构建目录）：
 *   ./ex_lifecycle ../config
 */
#include "forcerobot_sdk.h"

#include <cstdio>

int main(int argc, char** argv) {
    const char* config = (argc > 1) ? argv[1] : "../config";

    frs_options opt{};
    opt.use_ethercat = 0;  // FakeBus
    opt.interface_name = nullptr;
    opt.log_every_cycles = 0;

    std::printf("SDK version: %s\n", frs_version());

    frs_robot* robot = frs_create(config, &opt);
    if (!robot) {
        std::fprintf(stderr, "frs_create failed (config/lock?)\n");
        return 1;
    }

    if (frs_start(robot) != FRS_OK) {
        std::fprintf(stderr, "frs_start failed\n");
        frs_destroy(robot);
        return 2;
    }

    frs_state st{};
    if (frs_get_state(robot, &st) == FRS_OK) {
        std::printf("cycle=%llu bus_ready=%d arm0_enabled=%d\n",
                    (unsigned long long)st.cycle, st.bus_ready, st.enabled_joints[0]);
    }

    frs_stop(robot);
    frs_destroy(robot);
    std::printf("ok\n");
    return 0;
}
