# ForceRobot Customer SDK v1.0

面向客户二次开发的 **C/C++ 双臂七轴力控臂 SDK**（Linux aarch64）。

客户只需：

1. `#include "forcerobot_sdk.h"`
2. 链接 `libforcerobot_sdk`（依赖 `libforce_robot` / `libforce_robot_kinematics`）
3. 使用随包 `config/`（或现场标定配置）

## 文档导航（对外）

| 文档 | 内容 |
|------|------|
| [docs/QUICKSTART.txt](docs/QUICKSTART.txt) | 编译 / FakeBus / 真机起步 |
| [docs/CAPABILITY.txt](docs/CAPABILITY.txt) | **能力** |
| [docs/EXAMPLES_MAP.txt](docs/EXAMPLES_MAP.txt) | 示例 |
| [docs/API_EXAMPLES.txt](docs/API_EXAMPLES.txt) | API ↔ 示例 |
| `include/forcerobot_sdk.h` | 完整 API 与单位约定 |

## 目录结构

```text
forcerobot-customer-sdk-v1.0/
├── include/forcerobot_sdk.h     # 对外唯一头文件
├── src/forcerobot_sdk.cpp
├── vendor/                      # 底层 .so + 头（内部链接）
├── config/                      # 双臂配置
├── tools/                       # runtime_dual / gravity_identify
├── examples/                    # 示例（见 docs/EXAMPLES_MAP.txt）
├── docs/                        # 对外与联调文档
├── CMakeLists.txt
├── README.md
└── MANIFEST.txt
```

## 快速编译（aarch64 Linux）

```bash
mkdir build && cd build
cmake ..
cmake --build . -j

# FakeBus（无需硬件）
./ex_lifecycle ../config
./ex_kin_full ../config 0
./ex_move_joint_fake ../config

# 真机（网口按现场修改；运动前需输入 y）
# sudo rm -f /tmp/force_robot_sdk.lock
# sudo ./ex_hw_read ../config eth0
# sudo ./ex_hw_move_joint ../config eth0
```

可执行文件与 `.so` 同在 `build/`，已写 `RPATH=$ORIGIN`，一般无需再设 `LD_LIBRARY_PATH`。

## 最小示例

```cpp
#include "forcerobot_sdk.h"

int main() {
    frs_options opt{};
    opt.use_ethercat = 0; // FakeBus；真机改为 1 并填 interface_name
    frs_robot* r = frs_create("./config", &opt);
    if (!r) return 1;
    frs_start(r);
    frs_state st{};
    frs_get_state(r, &st);
    frs_stop(r);
    frs_destroy(r);
    return 0;
}
```

## 安全提示

- `hold` / `disable` / `stop` **不能替代硬件急停**
- 使能、autozero、运动、力控会产生机械动作，须现场批准
- 真机前先用 FakeBus 跑通生命周期与回调
- 单实例锁：`/tmp/force_robot_sdk.lock`（不可与 `runtime_dual` 并行抢总线）

## 本版不提供（详见 CAPABILITY.txt）

驱动清错/复位、温度与外编专用接口、软件急停、双臂原子同步、速度百分比接口等。  
EtherCAT 协议对象 ≠ 本 SDK 已开放 API。

## 许可注意

底层 EtherCAT 主站使用 **SOEM 1.4（GPLv2）**。对外再分发前请确认许可与源码提供义务。

## 版本

- Customer SDK：`1.0.0`（`frs_version()`）
- 底层配置集：`dual-shared-single-20261112`
