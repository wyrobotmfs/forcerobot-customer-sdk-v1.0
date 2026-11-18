#ifndef FORCE_ROBOT_KINEMATICS_SDK_H
#define FORCE_ROBOT_KINEMATICS_SDK_H

/**
 * @file
 * @brief 独立离线运动学 C API；同步计算，不连接设备、不使能、不执行运动。
 *
 * 绝对 TCP 位姿位于配置 base 变换后的公共坐标系，末端变换包含 tool * user_tcp。
 * 关节角采用模型关节坐标，顺序为 j1..j7；RPY 采用 Rz(yaw) * Ry(pitch) * Rx(roll)。
 * 所有数值输入须有限；所有入参及结构输入字段均须显式提供，无省略参数或零值默认规则。
 * 输入数组仅在调用期间读取，不转移所有权；普通输出缓冲区由调用方分配并持有。
 * 同一模型的只读计算可并发；set_tcp 须与该模型的其他调用互斥。
 * 句柄销毁须在使用它的全部调用结束后进行；轨迹只读访问可并发。
 */
#include <stddef.h>

#if defined(_WIN32)
# if defined(FR_KINEMATICS_BUILD)
#  define FR_KIN_API __declspec(dllexport)
# else
#  define FR_KIN_API __declspec(dllimport)
# endif
#else
# define FR_KIN_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 每臂关节数；关节数组按 j1..j7 排列。 */
#define FR_KIN_JOINTS 7
/** @brief 单次 TCP 路径的最大目标点数，不包含起始构型。 */
#define FR_KIN_PATH_MAX 32

/** @brief 固定一个配置臂的离线模型；由 create 分配、destroy 释放。 */
typedef struct fr_kin_model fr_kin_model;
/** @brief 持有完整采样结果的只读轨迹；独立于模型，由 trajectory_destroy 释放。 */
typedef struct fr_kin_trajectory fr_kin_trajectory;

/** @brief 返回码；各 int 接口均以此枚举值表示结果，无全局 last_error。 */
typedef enum fr_kin_result {
    FR_KIN_OK = 0,                  /**< 操作成功。 */
    FR_KIN_INVALID = -1,            /**< 指针、数值、索引、点数或入参组合无效。 */
    FR_KIN_CONFIG_ERROR = -2,       /**< 配置文件读取或内容校验失败。 */
    FR_KIN_REJECTED = -3,           /**< 求解、几何、软限位或轨迹推进条件不满足。 */
    FR_KIN_SAMPLE_LIMIT = -4,       /**< 样本预算耗尽且规划尚未完成；不返回部分轨迹。 */
    FR_KIN_BUFFER_TOO_SMALL = -5,   /**< 复制输出缓冲区的元素容量小于请求数量。 */
    FR_KIN_INTERNAL_ERROR = -6      /**< 内部异常，包括内存分配失败。 */
} fr_kin_result;

/** @brief 刚体位姿；用作绝对 TCP 位姿或 set_tcp 的 tool 局部偏置。 */
typedef struct fr_kin_pose {
    double position_m[3]; /**< [x,y,z]，m；绝对位姿在公共坐标系，TCP 偏置在 tool 系；有限，无统一位置范围。 */
    double rpy_deg[3];    /**< [roll,pitch,yaw]，deg；相对所属坐标系的姿态，Rz*Ry*Rx；输入有限、不限固定角度区间。 */
} fr_kin_pose;

/** @brief TCP 目标点及其入段推进参数；各字段均须显式赋值。 */
typedef struct fr_kin_tcp_point {
    fr_kin_pose target;               /**< 公共坐标系中的绝对 TCP 位姿；位置与姿态均参与求解，不是增量。 */
    double linear_speed_m_s;          /**< 平移速度上限，m/s，有限且 >0；与motion_limits.tcp_series.max_linear_velocity_m_s取小（发布配置0.3），无默认值。 */
    double rotation_speed_rad_s;      /**< 姿态速度上限，rad/s，有限且 >0；独立于平移推进，无公共分类上限、无默认值。 */
    double linear_acceleration_m_s2;  /**< 平移标量推进加速度，m/s²，有限且 >0；与motion_limits.tcp_series.max_linear_acceleration_m_s2取小（发布配置5），无默认值。 */
    double dwell_s;                   /**< 停点保持时间，s，有限且 >=0；0 仍保持至少一个采样周期，圆角点必须为0，无默认值。 */
    double blend_radius_m;            /**< 三维相切圆弧半径，m，有限且 >=0；0 为停点，末点必须为0；须适配相邻线段，不自动缩放。 */
} fr_kin_tcp_point;

/** @brief 离线轨迹样本，仅含时间与关节位置，不含速度、加速度或力矩。 */
typedef struct fr_kin_sample {
    double time_s;                    /**< 相对起点时间，s；样本索引乘规划的 sample_period_s，样本0为0。 */
    double joint_deg[FR_KIN_JOINTS];   /**< 模型关节坐标 j1..j7，deg；样本0原样保存起始角，所有样本满足配置软限位。 */
} fr_kin_sample;

/**
 * @brief 从四 JSON 配置创建指定臂的离线模型；不访问设备。
 * @param[in] config_dir 配置目录，非 NULL、非空；含devices/mdh/mass/parameter.json，无默认路径。
 * @param[in] config_arm_index 配置臂索引；single为0，dual为0或1，必须显式指定。
 * @param[out] out_model 模型句柄输出地址，非 NULL；成功返回新句柄，失败置NULL；用 fr_kin_destroy 释放。
 * @return FR_KIN_OK；参数或索引无效为INVALID，配置失败为CONFIG_ERROR，内部异常为INTERNAL_ERROR。
 */
FR_KIN_API int fr_kin_create(const char* config_dir, int config_arm_index,
                           fr_kin_model** out_model);

/**
 * @brief 释放离线模型，不影响已生成的轨迹。
 * @param[in] model 模型句柄；NULL无操作，释放后不得再使用。
 * @return 无返回值。
 */
FR_KIN_API void fr_kin_destroy(fr_kin_model* model);

/**
 * @brief 设置相对标定 tool 的 TCP 偏置，不修改控制 SDK 或 JSON。
 * @param[in] model 有效模型句柄，非 NULL；须与该模型的其他调用互斥。
 * @param[in] offset 偏置，非 NULL；tool系平移[m]及RPY[deg]均须有限，无固定范围、无默认值；全零置单位变换。
 * @return FR_KIN_OK；参数无效为INVALID，内部异常为INTERNAL_ERROR；失败不改变模型。
 */
FR_KIN_API int fr_kin_set_tcp(fr_kin_model* model, const fr_kin_pose* offset);

/**
 * @brief 由七轴关节角计算绝对 TCP 位姿，包含标定 tool 和 TCP 偏置。
 * @param[in] model 有效模型句柄，非 NULL。
 * @param[in] joint_deg j1..j7绝对关节角，非 NULL、长度7，单位deg；全部有限，无默认值；此函数不检查软限位。
 * @param[out] out_pose 位姿输出，非 NULL；公共坐标系xyz[m]及RPY[deg]；失败不修改。
 * @return FR_KIN_OK；参数无效为INVALID，结果非有限为REJECTED，内部异常为INTERNAL_ERROR。
 */
FR_KIN_API int fr_kin_fk(const fr_kin_model* model, const double joint_deg[7],
                       fr_kin_pose* out_pose);

/**
 * @brief 以参考关节角为初值求解完整 TCP 位姿，返回一个解。
 * @param[in] model 有效模型句柄，非 NULL。
 * @param[in] target 绝对TCP目标，非 NULL；公共坐标系xyz[m]和RPY[deg]全部有限且必须填写；无统一可达范围。
 * @param[in] reference_deg 求解初值，非 NULL、长度7，j1..j7，deg；全部有限，无默认值；不预检初值软限位。
 * @param[out] out_joint_deg 解输出，非 NULL、容量至少7，单位deg；成功解满足软限位，失败不修改。
 * @return FR_KIN_OK；参数无效为INVALID，无合格解为REJECTED，内部异常为INTERNAL_ERROR；不保证全局最近解。
 */
FR_KIN_API int fr_kin_solve_ik_near(const fr_kin_model* model,
    const fr_kin_pose* target, const double reference_deg[7], double out_joint_deg[7]);

/**
 * @brief 计算关节构型的几何臂角，不是某一关节角。
 * @param[in] model 有效模型句柄，非 NULL。
 * @param[in] joint_deg 参考构型，非 NULL、长度7，j1..j7，deg；全部有限，无默认值；不预检软限位。
 * @param[out] out_angle_deg 臂角输出，非 NULL，单位deg，范围[-180,180]；失败不修改。
 * @return FR_KIN_OK；参数无效为INVALID，几何退化或不适用为REJECTED，内部异常为INTERNAL_ERROR。
 */
FR_KIN_API int fr_kin_measure_arm_angle(const fr_kin_model* model,
    const double joint_deg[7], double* out_angle_deg);

/**
 * @brief 固定参考 TCP 位姿和解析分支，求包含当前臂角的连续可行区间。
 * @param[in] model 有效模型句柄，非 NULL。
 * @param[in] reference_deg 参考构型，非 NULL、长度7，j1..j7，deg；全部有限，其当前臂角对应解须可行，无默认值。
 * @param[out] out_min_deg 下界输出地址，非 NULL，deg；不得与out_max_deg相同；失败不修改。
 * @param[out] out_max_deg 上界输出地址，非 NULL，deg；边界可超出[-180,180]，不折回周期；失败不修改。
 * @return FR_KIN_OK；参数无效为INVALID，几何或当前构型不可行为REJECTED，内部异常为INTERNAL_ERROR。
 * @note 以1deg步长扫描至首个不可行点之前；可能遗漏窄缺口，不保证区间内每一点均可解。
 */
FR_KIN_API int fr_kin_get_arm_angle_range(const fr_kin_model* model,
    const double reference_deg[7], double* out_min_deg, double* out_max_deg);

/**
 * @brief 保持参考构型的完整 TCP 位姿和解析分支，求指定臂角的关节解。
 * @param[in] model 有效模型句柄，非 NULL。
 * @param[in] reference_deg 参考构型，非 NULL、长度7，j1..j7，deg；全部有限，携带位姿、分支及关节选圈参考，无默认值。
 * @param[in] angle_deg 目标臂角，deg；有限、无默认值，无固定输入角度区间；可行范围依构型和软限位。
 * @param[out] out_joint_deg 七轴关节解输出，非 NULL、容量至少7，deg；成功满足软限位，失败不修改。
 * @return FR_KIN_OK；参数无效为INVALID，无可行解为REJECTED，内部异常为INTERNAL_ERROR。
 */
FR_KIN_API int fr_kin_solve_arm_angle(const fr_kin_model* model,
    const double reference_deg[7], double angle_deg, double out_joint_deg[7]);

/**
 * @brief 规划单段 TCP 直线及姿态运动，返回完整 time+q 采样序列。
 * @param[in] model 有效模型句柄，非 NULL；使用当前TCP偏置、软限位及TCP分类上限。
 * @param[in] start_joint_deg 起始关节角，非 NULL、长度7，j1..j7，deg；有限且满足配置软限位，无默认值。
 * @param[in] point 单个目标点，非 NULL；位姿、速度、加速度及停留按结构填写，blend_radius_m必须0。
 * @param[in] sample_period_s 采样/推进周期，s，有限且>0，无默认值；如0.002表示2ms。
 * @param[in] max_samples 样本数量预算，含起点，>=1且(max_samples-1)×sample_period_s有限；无默认值，受可用内存限制。
 * @param[out] out_trajectory 轨迹输出地址，非 NULL；成功返回完整轨迹，用trajectory_destroy释放；失败置NULL，不返回半条轨迹。
 * @return FR_KIN_OK；参数无效为INVALID，路径/IK/软限位不满足为REJECTED，预算耗尽为SAMPLE_LIMIT，内部异常为INTERNAL_ERROR。
 * @note 起点样本时间为0，其后时间为索引×采样周期；平移/姿态独立推进，无碰撞检测，不限制逐轴关节速度；轨迹不依赖模型生命周期。
 */
FR_KIN_API int fr_kin_plan_line(const fr_kin_model* model,
    const double start_joint_deg[7], const fr_kin_tcp_point* point,
    double sample_period_s, size_t max_samples, fr_kin_trajectory** out_trajectory);

/**
 * @brief 规划多点 TCP 直线/相切圆弧路径，返回完整 time+q 采样序列。
 * @param[in] model 有效模型句柄，非 NULL；使用当前TCP偏置、软限位及TCP分类上限。
 * @param[in] start_joint_deg 起始关节角，非 NULL、长度7，j1..j7，deg；有限且满足配置软限位，无默认值。
 * @param[in] points 目标点数组，非 NULL、至少point_count项；各字段按tcp_point填写，半径不自动缩放，末点半径须0。
 * @param[in] point_count 目标数，范围1..32，不含起点，必须显式传入。
 * @param[in] sample_period_s 采样/推进周期，s，有限且>0，无默认值；如0.002表示2ms。
 * @param[in] max_samples 样本数量预算，含起点，>=1且(max_samples-1)×sample_period_s有限；无默认值，受可用内存限制。
 * @param[out] out_trajectory 轨迹输出地址，非 NULL；成功返回完整轨迹，用trajectory_destroy释放；失败置NULL，不返回半条轨迹。
 * @return FR_KIN_OK；参数无效为INVALID，路径/IK/软限位不满足为REJECTED，预算耗尽为SAMPLE_LIMIT，内部异常为INTERNAL_ERROR。
 * @note 起点样本时间为0，其后时间为索引×采样周期；平移/姿态独立推进，无碰撞检测，不限制逐轴关节速度；轨迹不依赖模型生命周期。
 */
FR_KIN_API int fr_kin_plan_tcp_path(const fr_kin_model* model,
    const double start_joint_deg[7], const fr_kin_tcp_point* points, size_t point_count,
    double sample_period_s, size_t max_samples, fr_kin_trajectory** out_trajectory);

/**
 * @brief 读取轨迹样本总数，包含起点和停留样本。
 * @param[in] trajectory 轨迹句柄；可为 NULL。
 * @return 样本数量；NULL句柄返回0。
 */
FR_KIN_API size_t fr_kin_trajectory_sample_count(const fr_kin_trajectory* trajectory);

/**
 * @brief 将连续轨迹样本复制到独立输出数组。
 * @param[in] trajectory 有效轨迹句柄，非 NULL。
 * @param[in] first_sample 首样本零起始索引，范围0..总样本数；等于总数时count须0，无默认值。
 * @param[in] count 复制数量，范围0..总样本数-first_sample；0为空操作，无默认值。
 * @param[out] out_samples 输出数组，count>0时非 NULL；容量至少capacity个sample对象，成功写前count项，失败不修改。
 * @param[in] capacity 输出元素容量，不是字节数；须>=count，无默认值。
 * @return FR_KIN_OK；参数/范围无效为INVALID，容量不足为BUFFER_TOO_SMALL。
 */
FR_KIN_API int fr_kin_trajectory_copy(const fr_kin_trajectory* trajectory,
    size_t first_sample, size_t count, fr_kin_sample* out_samples, size_t capacity);

/**
 * @brief 释放轨迹及内部样本，不影响已复制的样本。
 * @param[in] trajectory 轨迹句柄；NULL无操作，释放后不得再使用。
 * @return 无返回值。
 */
FR_KIN_API void fr_kin_trajectory_destroy(fr_kin_trajectory* trajectory);

#ifdef __cplusplus
}
#endif
#endif
