#pragma once

/**
 * @file
 * @brief 双臂机械臂控制 C API。
 *
 * 关节数组按 j1..j7 排列。绝对 TCP 位姿使用配置 base 变换后的公共坐标系，
 * 末端链为第7轴 × 标定 tool × TCP 偏置；RPY 为 Rz(yaw)×Ry(pitch)×Rx(roll)。
 * 角度为 deg，关节速度为 deg/s；姿态速度为 rad/s，不是 deg/s。
 * 命令同步等待受理、异步执行；ARRIVED 仅表示命令参考完成，不表示实测到位。
 * 运动不提供碰撞检测。hold、disable、stop 均不能替代机械急停。
 * start/stop/subscribe/unsubscribe 应串行调用；destroy 不得与其他调用并发。
 * 状态和结果回调在独立线程执行，回调内不得阻塞或销毁句柄。
 */


#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#  define FR_API __declspec(dllexport)
#else
#  define FR_API __attribute__((visibility("default")))
#endif

#define FR_STATE_VERSION 3u
#define FR_ROBOT_MAX_ARMS 2
#define FR_ROBOT_JOINTS 7

typedef struct fr_robot fr_robot;

typedef enum {
    FR_OK = 0, /**< 成功。 */
    FR_ERR_INVALID = -1, /**< 参数、指针、结构版本或输入格式无效。 */
    FR_ERR_NOT_READY = -2, /**< 实例未运行、总线未就绪或无可用快照。 */
    FR_ERR_BUSY = -3, /**< 生命周期、模式切换或其他互斥事务忙。 */
    FR_ERR_NOT_ENABLED = -4, /**< 所需关节尚未使能。 */
    FR_ERR_NOT_FOUND = -5, /**< 未找到所请求项目。 */
    FR_ERR_TIMEOUT = -6, /**< 等待受理或反馈超时；不保证请求未执行。 */
    FR_ERR_HARDWARE = -7, /**< 总线、线程、文件I/O或系统资源错误。 */
    FR_ERR_REJECTED = -8, /**< 运行层拒绝，如未autozero、限位或安全条件不满足。 */
    FR_ERR_ABORTED = -9 /**< 请求被其他操作中止。 */
} fr_status;

typedef enum {
    FR_MODE_HOLD = 0,
    FR_MODE_JOINT,
    FR_MODE_TCP,
    FR_MODE_ATTRACTOR3, /**< 保留编号，不可运行。 */
    FR_MODE_ATTRACTOR6,
    FR_MODE_DRAG,
    FR_MODE_SPRING,
    FR_MODE_CART3, /**< 保留编号，不可运行。 */
    FR_MODE_CART6,
    FR_MODE_NULL,
    FR_MODE_TOOL,
    FR_MODE_CART6NULL,
    FR_MODE_ARMANGLE,
    FR_MODE_ATTRACTOR6NULL,
    FR_MODE_CART6X,
    FR_MODE_CART6Y,
    FR_MODE_CART6Z,
    FR_MODE_ATTRACTOR6X,
    FR_MODE_ATTRACTOR6Y,
    FR_MODE_ATTRACTOR6Z,
    FR_MODE_IMP_JOINT,
    FR_MODE_IMP_TCP,
    FR_MODE_IMP_TOOL,
    FR_MODE_JOINT_PATH,
    FR_MODE_TCP_PATH,
    FR_MODE_IMP_JOINT_PATH,
    FR_MODE_IMP_TCP_PATH,
    FR_MODE_JOINT_STREAM,
    FR_MODE_TCP_STREAM,
    FR_MODE_IMP_JOINT_STREAM,
    FR_MODE_IMP_TCP_STREAM,
    FR_MODE_IMP_CART6,
    FR_MODE_IMP_CART6NULL,
    FR_MODE_IMP_CART6X,
    FR_MODE_IMP_CART6Y,
    FR_MODE_IMP_CART6Z,
    FR_MODE_IMP_ATTRACTOR6,
    FR_MODE_IMP_ATTRACTOR6NULL,
    FR_MODE_IMP_ATTRACTOR6X,
    FR_MODE_IMP_ATTRACTOR6Y,
    FR_MODE_IMP_ATTRACTOR6Z,
    FR_MODE_IMP_NULL,
    FR_MODE_HYBRID_FORCE_POSITION,
    FR_MODE_IMP_HYBRID_FORCE_POSITION,
    FR_MODE_CART6NULL_EXP,
    FR_MODE_ATTRACTOR6NULL_EXP,
    FR_MODE_TCP_NULL,
    FR_MODE_TOOL_NULL
} fr_mode;

typedef enum {
    FR_STRATEGY_IDLE = 0,
    FR_STRATEGY_RUNNING = 1,
    FR_STRATEGY_FINISHED = 2,
    FR_STRATEGY_FAILED = 3
} fr_strategy_status;

typedef enum {
    FR_DRIVE_OFFLINE = 0,
    FR_DRIVE_SWITCH_ON_DISABLED = 1,
    FR_DRIVE_READY_TO_SWITCH_ON = 2,
    FR_DRIVE_SWITCHED_ON = 3,
    FR_DRIVE_OPERATION_ENABLED = 4,
    FR_DRIVE_QUICK_STOP_ACTIVE = 5,
    FR_DRIVE_FAULT = 6,
    FR_DRIVE_UNKNOWN = 7
} fr_drive_state;

typedef struct {
    uint32_t struct_version; /**< 填写 FR_STATE_VERSION；建议先将整个结构清零。 */
    int      use_soem; /**< 0=FakeBus；1=EtherCAT；必须显式选择。 */
    const char* interface_name; /**< EtherCAT 网口名；NULL 使用 eth0；FakeBus 时忽略。 */
    int      log_every_cycles; /**< 日志采样间隔，单位控制拍；>0 异步写log/run.txt，0关闭，默认0；实时仅有界快照入队，队满丢日志计数，不等待磁盘。 */
} fr_options;

typedef struct {
    uint32_t struct_version; /**< 输出状态结构版本FR_STATE_VERSION。 */
    int64_t  timestamp_ns; /**< 状态采样时间戳，ns。 */
    uint64_t cycle; /**< 控制拍号；时间差可按拍差乘控制周期计算。 */
    int      bus_ready; /**< 1总线就绪，0未就绪。 */
    int      working_counter; /**< 最近一拍实际WKC；-1表示未收到帧。 */
    int      pending_command; /**< 1有待处理动作命令，0无。 */
    int      last_error; /**< 实例/起机错误码；0无此类错误，不表示最近运动结果。 */
    int      enabled_joints[FR_ROBOT_MAX_ARMS]; /**< 逐臂已使能关节数量，0..7。 */
    int      autozero_done[FR_ROBOT_MAX_ARMS]; /**< 逐臂零漂标定完成标志，0/1。 */
    int      enable_in_progress[FR_ROBOT_MAX_ARMS]; /**< 逐臂使能推进标志，0/1。 */
    int      disable_in_progress[FR_ROBOT_MAX_ARMS]; /**< 逐臂下使能推进标志，0/1。 */
    int      autozero_in_progress[FR_ROBOT_MAX_ARMS]; /**< 逐臂零漂标定推进标志，0/1。 */
    int      mode[FR_ROBOT_MAX_ARMS]; /**< 逐臂当前策略枚举mode，不是CSP/CST驱动模式。 */
    int      strategy_status[FR_ROBOT_MAX_ARMS]; /**< 逐臂策略状态枚举strategy_status。 */
    int      drive_state[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴CiA402状态，使用drive_state枚举。 */
    double   command_position_deg[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴命令参考位置，deg，不是实测位置。 */
    double   measured_position_deg[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴实测位置，deg。 */
    double   measured_velocity_deg_s[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴实测速度，deg/s，来自0x606C。 */
    double   estimated_acceleration_deg_s2[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 由速度窗口拟合的加速度估计，deg/s²；非驱动器直接测量。 */
    double   measured_torque_unfiltered_nm[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴未滤波力矩，Nm。 */
    double   measured_torque_filtered_nm[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴滤波后力矩，Nm，存在滤波延迟。 */
    double   target_torque_nm[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴命令力矩，Nm。 */
    double   gravity_hold_nm[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 纯重力保持模型力矩，Nm，不含摩擦或零漂；检查gravity_hold_valid，无效时0。 */
    double   external_torque_nm[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴外力矩估计，Nm；需有效估计链及autozero。 */
    double   torque_drift_nm[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< autozero标定的测量侧零漂，Nm；检查torque_drift_valid，无效时0。 */
    double   equivalent_force_3d_n[FR_ROBOT_MAX_ARMS][3]; /**< 三维等效力估计[x,y,z]，N。 */
    double   equivalent_force_6d[FR_ROBOT_MAX_ARMS][6]; /**< 六维等效力/力矩估计[Fx,Fy,Fz,Mx,My,Mz]，N/Nm。 */
    double   tool_pose[FR_ROBOT_MAX_ARMS][6]; /**< measured_tcp_pose的实测TCP别名，公共坐标系x/y/z[m]+RPY[deg]；不是命令位姿。 */
    int32_t  position_counts[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴原始位置计数。 */
    int32_t  velocity_counts[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴0x606C原始速度计数。 */
    int32_t  measured_torque_lsb[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴解环绕后的原始整数力矩LSB，不是Nm。 */
    int32_t  written_target_torque_lsb[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴写入驱动的原始力矩LSB。 */
    uint32_t written_target_position_counts[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴写入驱动的目标位置计数。 */
    double   command_velocity_deg_s[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴命令位置差分速度，deg/s。 */
    int      strategy_flags[FR_ROBOT_MAX_ARMS]; /**< 逐臂FR_FLAG_*策略标志位或。 */
    double   command_tcp_pose[FR_ROBOT_MAX_ARMS][6]; /**< 同拍command_position_deg经FK得到的TCP，公共系x/y/z[m]+RPY[deg]；不是最终运动目标。 */
    double   measured_tcp_pose[FR_ROBOT_MAX_ARMS][6]; /**< 同拍measured_position_deg经FK得到的TCP，公共系x/y/z[m]+RPY[deg]；含当前工具及用户TCP偏置。 */
    int      command_tcp_valid[FR_ROBOT_MAX_ARMS]; /**< 命令TCP有效时1，否则0；无需autozero。 */
    int      measured_tcp_valid[FR_ROBOT_MAX_ARMS]; /**< 实测关节反馈有效且实测TCP有限时1，否则0；无需autozero。 */
    double   modeled_friction_torque_nm[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 外力估计链扣除的有符号摩擦项，Nm；CSP/CST含义相同，不是CST摩擦前馈；无效时0。 */
    uint32_t gravity_hold_valid[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 重力保持模型发布有效时1，否则0；需有效autozero、使能与反馈。 */
    uint32_t friction_valid[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 摩擦估计发布有效时1，否则0；检查后再使用modeled_friction_torque_nm。 */
    uint32_t torque_drift_valid[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 标定零漂发布有效时1，否则0；需有效autozero、使能与反馈。 */
} fr_state;

/**
 * @brief 查询 SDK 产品版本；无需创建实例。
 * @return 静态只读版本字符串（如 "1.0.0"），无需释放；与状态结构版本无关。
 */
FR_API const char* fr_get_version(void);

#define FR_FEEDBACK_VERSION 1u
#define FR_FEEDBACK_POSITION_VALID       1u
#define FR_FEEDBACK_VELOCITY_VALID       2u
#define FR_FEEDBACK_TORQUE_VALID         4u
#define FR_FEEDBACK_CURRENT_VALID        8u
#define FR_FEEDBACK_ACCELERATION_VALID  16u
#define FR_FEEDBACK_EXTERNAL_TORQUE_VALID 32u
#define FR_FEEDBACK_PAYLOAD_VALID         1u
#define FR_FEEDBACK_FORCE_3D_VALID        2u
#define FR_FEEDBACK_FORCE_6D_VALID        4u
#define FR_FEEDBACK_TOOL_POSE_VALID      8u
#define FR_FEEDBACK_TCP_VALID           16u

typedef struct {
    uint32_t struct_size; /**< 调用前填 sizeof(本结构)。 */
    uint32_t struct_version; /**< 调用前填1。 */
    int64_t timestamp_ns; /**< 同拍时间戳，ns。 */
    uint64_t cycle; /**< 所属控制拍号；仅可与相同cycle状态配对。 */
    int32_t current_ma[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴0x6078电流反馈，有符号mA；检查CURRENT_VALID。 */
    uint32_t joint_valid_flags[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴FR_FEEDBACK_*_VALID位或；0表示各项均无效。 */
    uint32_t arm_valid_flags[FR_ROBOT_MAX_ARMS]; /**< 逐臂负载/力估计/TCP位姿有效标志位或。 */
    double payload_mass_kg[FR_ROBOT_MAX_ARMS]; /**< 当前负载质量，kg；0质量也可有效。 */
    double payload_com_m[FR_ROBOT_MAX_ARMS][3]; /**< 标定tool系负载质心[x,y,z]，m。 */
    double tcp_offset[FR_ROBOT_MAX_ARMS][6]; /**< 相对标定tool的TCP偏置，x/y/z[m]+RPY[deg]。 */
} fr_feedback;
/**
 * @brief 复制带有效位的最新反馈；仅在控制线程运行且已产生反馈时可用。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in,out] feedback 反馈输出，非 NULL；调用前填写 struct_size=sizeof(*feedback)、struct_version=FR_FEEDBACK_VERSION；有效位只适用于相同 cycle。
 * @return FR_OK 表示输出成功；无数据返回 FR_ERR_NOT_READY，参数无效返回 FR_ERR_INVALID。
 */
FR_API int fr_get_feedback(fr_robot* robot, fr_feedback* feedback);

/**
 * @brief 一次复制同一控制拍的状态及反馈有效位。
 * @note 状态版本3包含命令/实测TCP及估计链摩擦有效位；必须使用配套版本3头文件和库重新编译，不得混用旧结构。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[out] state 状态输出，非 NULL；提供一个 fr_state 对象，无须预填版本。
 * @param[in,out] feedback 反馈输出，非 NULL；调用前填写 struct_size=sizeof(*feedback)、struct_version=FR_FEEDBACK_VERSION；有效位只适用于相同 cycle。
 * @return FR_OK 表示输出成功；无数据返回 FR_ERR_NOT_READY，参数无效返回 FR_ERR_INVALID。
 */
FR_API int fr_get_state_with_feedback(fr_robot* robot, fr_state* state, fr_feedback* feedback);

#define FR_DRIVE_MODE_STATE_VERSION 2u
typedef enum { FR_DRIVE_MODE_UNKNOWN=0, FR_DRIVE_MODE_CSP=8, FR_DRIVE_MODE_CST=10 } fr_drive_mode;
typedef struct {
    uint32_t struct_size; /**< 调用前填 sizeof(本结构)。 */
    uint32_t struct_version; /**< 调用前填当前驱动模式状态版本。 */
    uint64_t cycle; /**< 所属控制拍。 */
    int current_strategy_mode[FR_ROBOT_MAX_ARMS]; /**< 同拍当前策略。 */
    int requested[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴请求驱动模式，8=CSP、10=CST、0未知。 */
    int display[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴PDO6061反馈模式，8=CSP、10=CST。 */
    int confirmed[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴有效反馈与请求模式一致时为1，否则0。 */
    int target_strategy_mode[FR_ROBOT_MAX_ARMS]; /**< 目标策略枚举mode，不是驱动模式值。 */
    int switch_stage[FR_ROBOT_MAX_ARMS]; /**< 0空闲、1等待静止、2邮箱切换、3已接管、4失败、5取消。 */
    uint32_t confirmed_mask[FR_ROBOT_MAX_ARMS]; /**< 逐轴确认位掩码；bit0..6对应j1..j7。 */
    uint64_t tag[FR_ROBOT_MAX_ARMS]; /**< 当前切模请求关联号。 */
    char error[FR_ROBOT_MAX_ARMS][96]; /**< 每臂切模原因文本，NUL结尾，容量96字节。 */
} fr_drive_mode_state;
/**
 * @brief 复制逐轴 CSP/CST 请求、实际模式及切换状态；不执行总线读取。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in,out] state 驱动模式输出，非 NULL；调用前填 struct_size=sizeof(*state)、struct_version=FR_DRIVE_MODE_STATE_VERSION。
 * @return FR_OK 表示输出成功；无数据返回 FR_ERR_NOT_READY，参数无效返回 FR_ERR_INVALID。
 */
FR_API int fr_get_drive_mode_state(fr_robot* robot, fr_drive_mode_state* state);

#ifndef FR_ARMANGLE_SNAPSHOT_DEFINED
#define FR_ARMANGLE_SNAPSHOT_DEFINED
typedef struct {
    uint64_t cycle; /**< 所属控制拍。 */
    int active, valid, trustworthy, range_ready, at_limit, reached;
    int branch_valid, shoulder, wrist, mirror;
    double phi_deg, phi_min_deg, phi_max_deg, target_phi_deg;
    double max_velocity_deg_s, elbow_radius_m; /**< 无效标量为 NaN。 */
} fr_armangle_snapshot;
#endif

/**
 * @brief 读取一条臂的同拍臂角策略诊断；不计算新的臂角。
 * @param[in] robot 运行中的实例，非 NULL。
 * @param[in] arm 逻辑臂号，0 或 1。
 * @param[out] snapshot 输出，非 NULL；无效标量为 NaN，分支须检查 branch_valid。
 * @return FR_OK 成功；参数错误返回 INVALID；无快照返回 NOT_READY。
 */
FR_API int fr_get_armangle_snapshot(fr_robot* robot, int arm, fr_armangle_snapshot* snapshot);

#define FR_FLAG_TCP_ESCAPING        1u
#define FR_FLAG_TCP_TARGET_REJECTED 2u
#define FR_FLAG_ARMANGLE_AT_LIMIT   4u
#define FR_FLAG_ARMANGLE_REACHED    8u

typedef enum {
    FR_EVENT_OTHER = 0,
    FR_EVENT_IK_FAILED,
    FR_EVENT_TARGET_REJECTED,
    FR_EVENT_SINGULAR_POSE_REFUSED,
    FR_EVENT_NULL_BLOCKED,
    FR_EVENT_NULL_RELEASED,
    FR_EVENT_NULL_LIMIT,
    FR_EVENT_ARMANGLE_LIMIT,
    FR_EVENT_ARRIVED,
    FR_EVENT_FAILED,
    FR_EVENT_BUS_READY,
    FR_EVENT_LINK_DOWN
} fr_event_code;

typedef enum {
    FR_RESULT_ACCEPTED = 0,
    FR_RESULT_REJECTED,
    FR_RESULT_ARRIVED,
    FR_RESULT_FAILED,
    FR_RESULT_SUPERSEDED,
    FR_RESULT_MODE_ERROR,
    FR_RESULT_STRATEGY_STARTED
} fr_result_kind;

typedef struct {
    int      code; /**< status返回码。 */
    int      arm; /**< 逻辑臂号0/1；系统事件为-1。 */
    int      kind; /**< result_kind枚举，区分受理、完成、失败等阶段。 */
    int      event_code; /**< event_code枚举，说明通知或失败原因。 */
    uint64_t tag; /**< 原请求关联号，原样返回。 */
    char     strategy[16]; /**< 策略名，NUL结尾，容量16字节。 */
    char     message[256]; /**< 原因说明，NUL结尾，容量256字节。 */
} fr_result;

typedef void (*fr_result_callback)(const fr_result* result, void* user);
/**
 * @brief 注册或替换异步结果回调；回调在独立分发线程执行。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] callback 结果回调函数；可为 NULL 以取消注册；result 指针仅回调期间有效，回调不得阻塞。
 * @param[in] user 回调上下文指针，可为 NULL；SDK 仅透传、不释放，须覆盖在途回调的生命周期。
 * @return FR_OK 表示操作成功；负值为 fr_status 错误码。
 * @note 结果队列容量64，满时丢最旧结果；不提供持久账本或无损送达保证。
 */
FR_API int fr_set_result_callback(fr_robot* robot, fr_result_callback callback,
                                  void* user);

/**
 * @brief 加载配置并创建控制实例；不启动总线、不使能。
 * @param[in] config_dir 四 JSON 配置目录，非 NULL、非空；无默认路径，必须匹配 single/dual 版型。
 * @param[in] options 创建选项，可为 NULL；NULL 默认 EtherCAT、eth0、不写周期日志；非 NULL 时按 options 字段填写。
 * @return 成功返回独占控制句柄；配置、锁、资源或初始化失败返回 NULL。
 */
FR_API fr_robot* fr_create(const char* config_dir, const fr_options* options);
/**
 * @brief 启动总线与控制线程，等待起机就绪；不使能关节。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @return FR_OK 表示操作成功；负值为 fr_status 错误码。
 * @note SDK 回调内调用返回 BUSY；生命周期操作须在外部串行化。
 */
FR_API int       fr_start(fr_robot* robot);
/**
 * @brief 请求保持、下使能并等待控制线程及后台采集收尾；停止后须销毁重建。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @return FR_OK 表示操作成功；负值为 fr_status 错误码。
 * @note SDK 回调内调用返回 BUSY；生命周期操作须在外部串行化。
 */
FR_API int       fr_stop(fr_robot* robot);
/**
 * @brief 查询控制线程是否仍在运行；不表示总线就绪或关节已使能。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @return 1 表示控制线程运行中；0 表示未启动、已结束或 robot 为 NULL。
 */
FR_API int       fr_is_running(fr_robot* robot);
/**
 * @brief 停止并释放实例；不得与其他接口并发调用。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @return 无返回值；robot 为 NULL 时无操作；在 SDK 回调内调用时不释放句柄。
 */
FR_API void      fr_destroy(fr_robot* robot);

/**
 * @brief 请求七轴使能；必要时先切换至 CSP，可能引起机械运动。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_enable(fr_robot* robot, int arm, uint64_t tag, fr_result* accepted);
/**
 * @brief 请求七轴下使能；可取消正在进行的模式切换或诊断事务。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_disable(fr_robot* robot, int arm, uint64_t tag, fr_result* accepted);
/**
 * @brief 交换两臂的七轴站址组；不交换标定参数、不修改配置文件。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note 要求两臂均已下使能、总线就绪且无冲突事务；重启恢复配置站址。
 */
FR_API int fr_reverse(fr_robot* robot, uint64_t tag, fr_result* accepted);

typedef struct {
    double position_m[3]; /**< 位置[x,y,z]，m；全部有限，无统一数值范围。 */
    double rpy_deg[3]; /**< 姿态[roll,pitch,yaw]，deg；R=Rz×Ry×Rx，全部有限，无固定输入角度区间。 */
} fr_pose;

/**
 * @brief 设置相对标定 tool 的 TCP 偏置，仅修改运行内存。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] offset TCP 偏置，非 NULL；标定 tool 系平移[m]及 RPY[deg]，全部有限；全零为单位变换，无默认值或统一数值上限。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note 仅允许整臂未使能，或全轴使能且处于 hold，并且无启停/切模/标定事务；成功后等待反馈同步，不写 JSON。
 */
FR_API int fr_set_tcp(fr_robot* robot, int arm, const fr_pose* offset,
                       uint64_t tag, fr_result* accepted);
/**
 * @brief 将 TCP 偏置置为单位变换；不是恢复 JSON 中的偏置。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note 仅允许整臂未使能，或全轴使能且处于 hold，并且无启停/切模/标定事务；成功后等待反馈同步，不写 JSON。
 */
FR_API int fr_clear_tcp(fr_robot* robot, int arm, uint64_t tag, fr_result* accepted);
/**
 * @brief 读取当前 TCP 偏置；启动前读取配置值，运行时读取反馈快照。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[out] offset TCP 偏置输出，非 NULL；位置为标定 tool 系[m]，RPY 为相对 tool 姿态[deg]。
 * @return FR_OK 表示输出成功；无数据返回 FR_ERR_NOT_READY，参数无效返回 FR_ERR_INVALID。
 * @note 启动前返回配置值；运行时返回当前快照，停止后返回 NOT_READY；不写配置文件。
 */
FR_API int fr_get_tcp(fr_robot* robot, int arm, fr_pose* offset);

/**
 * @brief 用运行实例的标定模型和当前 TCP 偏置计算正运动学，不发控制命令。
 * @param[in] robot 运行中的 SDK 实例，非 NULL。
 * @param[in] arm 逻辑臂号，0 或 1。
 * @param[in] q_deg 七轴关节角，单位 deg，非 NULL，所有值必须有限。
 * @param[out] pose 基座系 TCP 位姿，单位 m 和 RPY deg，非 NULL；仅成功时写入。
 * @return FR_OK 成功；参数无效或结果非有限返回 INVALID；实例未运行或暂无 TCP 快照返回 NOT_READY。
 */
FR_API int fr_compute_fk(fr_robot* robot, int arm, const double q_deg[FR_ROBOT_JOINTS],
    fr_pose* pose);

/**
 * @brief 读取当前工具负载质量与质心；启动前读取配置值。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[out] mass_kg 质量输出地址，非 NULL；单位 kg，0 表示无负载。
 * @param[out] com_m 质心输出数组，非 NULL、容量至少3；标定 tool 坐标系 x/y/z，单位 m，不随 TCP 偏置改变。
 * @return FR_OK 表示输出成功；无数据返回 FR_ERR_NOT_READY，参数无效返回 FR_ERR_INVALID。
 * @note 启动前返回配置值；运行时返回当前快照，停止后返回 NOT_READY；不写配置文件。
 */
FR_API int fr_get_payload(fr_robot* robot, int arm, double* mass_kg, double com_m[3]);

/**
 * @brief 启动力矩零漂标定；标定过程可能包含关节运动。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_autozero(fr_robot* robot, int arm, uint64_t tag, fr_result* accepted);
/**
 * @brief 冻结当前命令参考并清除目标；保持 CSP/CST 模式，不执行切模。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note 模式切换忙时拒绝；该操作不是机械急停。
 */
FR_API int fr_hold(fr_robot* robot, int arm, uint64_t tag, fr_result* accepted);

typedef enum {
    FR_MOTION_JOINT = 0, FR_MOTION_TCP, FR_MOTION_TOOL, FR_MOTION_ARM_ANGLE,
    FR_MOTION_IMP_JOINT, FR_MOTION_IMP_TCP, FR_MOTION_IMP_TOOL,
    FR_MOTION_TCP_NULL, FR_MOTION_TOOL_NULL
} fr_motion_kind;

typedef enum {
    FR_MOTION_SPEED = 1u << 0, FR_MOTION_ACCEL = 1u << 1,
    FR_MOTION_ROTATION_SPEED = 1u << 2, FR_MOTION_ROTATION_ACCEL = 1u << 3
} fr_motion_field;

typedef struct {
    uint32_t present;    /**< 按位组合 fr_motion_field；未标记的字段不读取。 */
    double speed;       /**< 有限正数；关节/臂角 deg/s，TCP/tool m/s。 */
    double accel;       /**< 有限正数；关节 deg/s²，TCP/tool m/s²；臂角不支持。 */
    double rotation_speed; /**< 有限正数，rad/s；仅 TCP/tool。 */
    double rotation_accel; /**< 有限正数，rad/s²；仅 TCP/tool。 */
} fr_motion_options;

/**
 * @brief 提交一次有限运动，未提供的运动参数在控制层受理时逐臂解析并锁存。
 * @param[in] robot 有效控制实例句柄，非 NULL。
 * @param[in] arm 配置逻辑臂号 0 或 1，不支持广播。
 * @param[in] kind 运动类型，取 fr_motion_kind 的成员。
 * @param[in] target 非 NULL；joint/imp_joint 为长度7的关节角[deg]，armangle 为长度1的臂角[deg]，
 *            TCP 为长度3的绝对位置[m]，tool 为长度3的工具系位置增量[m]；所有分量有限。
 * TCP_NULL / TOOL_NULL 分别与 TCP / TOOL 使用相同目标语义，并叠加手推零空间速度。
 * @param[in] rpy_deg 长度3的有限 RPY[deg]；TCP / TCP_NULL 为绝对姿态，NULL 时保持受理时命令姿态；
 *            tool / tool_null 为工具系姿态增量，NULL 表示零增量；joint/armangle 必须为 NULL。
 * @param[in] options 可为 NULL（全部参数省略）；显式值必须有限且大于零。省略项使用该臂策略默认；
 *            armangle 不限制加速度。运动分类和逐轴限制仍适用。
 * @param[in] tag 请求关联号，0 合法，结果原样返回。
 * @param[out] accepted 可为 NULL；非 NULL 时返回受理结果或具体拒绝原因。
 * @return FR_OK 表示已受理，不表示启动或完成；负值为 fr_status 错误码。
 */
FR_API int fr_move_motion(fr_robot* robot, int arm, fr_motion_kind kind,
                          const double* target, const double rpy_deg[3],
                          const fr_motion_options* options, uint64_t tag,
                          fr_result* accepted);

/**
 * @brief 以 CSP 执行七轴同步关节运动；所有轴使用共同路径进度。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] q_deg 七轴绝对关节角数组，非 NULL、长度7，顺序 j1..j7，单位 deg；全部有限且在各轴配置软限位内，无默认值。
 * @param[in] speed_deg_s 最快关节轴参考速度，单位 deg/s，有限且 >0，必须显式传入；与 motion_limits.joint_series.max_velocity_deg_per_s 取小（发布配置60）。
 * @param[in] accel_deg_s2 共同路径参考加速度，单位 deg/s²，有限且 >0，必须显式传入；与 motion_limits.joint_series.max_acceleration_deg_per_s2 取小（发布配置1000）。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_move_joint(fr_robot* robot, int arm,
                         const double q_deg[FR_ROBOT_JOINTS],
                         double speed_deg_s, double accel_deg_s2,
                         uint64_t tag, fr_result* accepted);
/**
 * @brief 以 CSP 执行绝对 TCP 直线运动；平移与姿态分别推进。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] target 绝对 TCP 位姿，非 NULL；公共坐标系位置[m]及 RPY[deg]均须填写且有限；无默认值、无统一可达范围。
 * @param[in] linear_speed_m_s TCP 平移参考速度，单位 m/s，有限且 >0，必须显式传入；与 motion_limits.tcp_series.max_linear_velocity_m_s 取小（发布配置0.3）。
 * @param[in] rotation_speed_rad_s 姿态参考角速度，单位 rad/s，有限且 >0，必须显式传入；无公共分类数值上限，0 不表示保持姿态。
 * @param[in] linear_accel_m_s2 TCP 平移参考加速度，单位 m/s²，有限且 >0，必须显式传入；与 motion_limits.tcp_series.max_linear_acceleration_m_s2 取小（发布配置5）。
 * @param[in] rotation_accel_rad_s2 姿态参考角加速度，单位 rad/s²，有限且 >0，必须显式传入；无公共分类数值上限。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_move_tcp(fr_robot* robot, int arm, const fr_pose* target,
                       double linear_speed_m_s, double rotation_speed_rad_s,
                       double linear_accel_m_s2, double rotation_accel_rad_s2,
                       uint64_t tag, fr_result* accepted);

/**
 * @brief 以 CSP 执行基座系绝对 TCP 平移，保持控制层受理时的命令 TCP 姿态。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] position_m 基座系绝对 TCP 位置，非 NULL、长度3，顺序 x/y/z，单位 m；全部有限，无默认值、无统一可达范围。
 * @param[in] speed TCP 平移参考速度，单位 m/s，有限且 >0；必填，与 motion_limits.tcp_series.max_linear_velocity_m_s 取小。
 * @param[in] rotation_speed 姿态参考角速度，单位 rad/s，有限且 >0；必填，无公共分类数值上限。
 * @param[in] acceleration TCP 平移参考加速度，单位 m/s²，有限且 >0；必填，与 motion_limits.tcp_series.max_linear_acceleration_m_s2 取小。
 * @param[in] rotation_acceleration 姿态参考角加速度，单位 rad/s²，有限且 >0；必填，无公共分类数值上限。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 fr_result 对象，入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_move_tcp_position(fr_robot* robot, int arm, const double position_m[3],
                                double speed, double rotation_speed,
                                double acceleration, double rotation_acceleration,
                                uint64_t tag, fr_result* accepted);

/**
 * @brief 以 CST 关节空间阻抗执行基座系绝对 TCP 平移，保持控制层受理时的命令 TCP 姿态。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] position_m 基座系绝对 TCP 位置，非 NULL、长度3，顺序 x/y/z，单位 m；全部有限，无默认值、无统一可达范围。
 * @param[in] speed TCP 平移参考速度，单位 m/s，有限且 >0；必填，与 motion_limits.tcp_series.max_linear_velocity_m_s 取小。
 * @param[in] rotation_speed 姿态参考角速度，单位 rad/s，有限且 >0；必填，无公共分类数值上限。
 * @param[in] acceleration TCP 平移参考加速度，单位 m/s²，有限且 >0；必填，与 motion_limits.tcp_series.max_linear_acceleration_m_s2 取小。
 * @param[in] rotation_acceleration 姿态参考角加速度，单位 rad/s²，有限且 >0；必填，无公共分类数值上限。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 fr_result 对象，入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note CST 要求已使能、autozero 完成及各轴正值力矩/位置误差限额；同策略新目标连续接续。
 */
FR_API int fr_move_impedance_tcp_position(fr_robot* robot, int arm, const double position_m[3],
                                          double speed, double rotation_speed,
                                          double acceleration, double rotation_acceleration,
                                          uint64_t tag, fr_result* accepted);

/**
 * @brief 以 CSP 执行命令 TCP 坐标系中的相对位姿运动。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] delta_position_m 平移增量数组，非 NULL、长度3，顺序 x/y/z，单位 m；沿受理时命令 TCP 坐标轴，全部有限，无统一距离上限、无默认值。
 * @param[in] delta_rpy_deg 右乘姿态增量数组，长度3，顺序 roll/pitch/yaw，单位 deg，Rz×Ry×Rx；全部有限，可为 NULL（等同三个0）。
 * @param[in] linear_speed_m_s TCP 平移参考速度，单位 m/s，有限且 >0，必须显式传入；与 motion_limits.tcp_series.max_linear_velocity_m_s 取小（发布配置0.3）。
 * @param[in] rotation_speed_rad_s 姿态参考角速度，单位 rad/s，有限且 >0，必须显式传入；无公共分类数值上限，0 不表示保持姿态。
 * @param[in] linear_accel_m_s2 TCP 平移参考加速度，单位 m/s²，有限且 >0，必须显式传入；与 motion_limits.tcp_series.max_linear_acceleration_m_s2 取小（发布配置5）。
 * @param[in] rotation_accel_rad_s2 姿态参考角加速度，单位 rad/s²，有限且 >0，必须显式传入；无公共分类数值上限。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note 目标=受理时命令TCP×增量；重复提交相同增量会再次相对移动。
 */
FR_API int fr_move_tool(fr_robot* robot, int arm,
                        const double delta_position_m[3],
                        const double delta_rpy_deg[3],
                        double linear_speed_m_s, double rotation_speed_rad_s,
                        double linear_accel_m_s2, double rotation_accel_rad_s2,
                        uint64_t tag, fr_result* accepted);

/**
 * @brief 以 CST 执行关节空间阻抗运动；加速度采用 imp_joint 当前默认值。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] q_deg 七轴绝对关节角数组，非 NULL、长度7，顺序 j1..j7，单位 deg；全部有限且在各轴配置软限位内，无默认值。
 * @param[in] speed_deg_s 最快关节轴参考速度，单位 deg/s，有限且 >0，必须显式传入；与 motion_limits.joint_series.max_velocity_deg_per_s 取小（发布配置60）。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note CST 要求已使能、autozero 完成及各轴正值力矩/位置误差限额；同策略新目标连续接续。
 */
FR_API int fr_move_impedance_joint(fr_robot* robot, int arm,
                                   const double q_deg[FR_ROBOT_JOINTS],
                                   double speed_deg_s,
                                   uint64_t tag, fr_result* accepted);
/**
 * @brief 以 CST 执行 TCP 目标运动；关节空间阻抗，加速度采用 imp_tcp 当前默认值。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] target 绝对 TCP 位姿，非 NULL；公共坐标系位置[m]及 RPY[deg]均须填写且有限；无默认值、无统一可达范围。
 * @param[in] linear_speed_m_s TCP 平移参考速度，单位 m/s，有限且 >0，必须显式传入；与 motion_limits.tcp_series.max_linear_velocity_m_s 取小（发布配置0.3）。
 * @param[in] rotation_speed_rad_s 姿态参考角速度，单位 rad/s，有限且 >0，必须显式传入；无公共分类数值上限，0 不表示保持姿态。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note CST 要求已使能、autozero 完成及各轴正值力矩/位置误差限额；同策略新目标连续接续。
 */
FR_API int fr_move_impedance_tcp(fr_robot* robot, int arm, const fr_pose* target,
                                 double linear_speed_m_s, double rotation_speed_rad_s,
                                 uint64_t tag, fr_result* accepted);
/**
 * @brief 以 CST 执行相对 TCP 运动；关节空间阻抗，加速度采用 imp_tool 当前默认值。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] delta_position_m 平移增量数组，非 NULL、长度3，顺序 x/y/z，单位 m；沿受理时命令 TCP 坐标轴，全部有限，无统一距离上限、无默认值。
 * @param[in] delta_rpy_deg 右乘姿态增量数组，长度3，顺序 roll/pitch/yaw，单位 deg，Rz×Ry×Rx；全部有限，可为 NULL（等同三个0）。
 * @param[in] linear_speed_m_s TCP 平移参考速度，单位 m/s，有限且 >0，必须显式传入；与 motion_limits.tcp_series.max_linear_velocity_m_s 取小（发布配置0.3）。
 * @param[in] rotation_speed_rad_s 姿态参考角速度，单位 rad/s，有限且 >0，必须显式传入；无公共分类数值上限，0 不表示保持姿态。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note CST 要求已使能、autozero 完成及各轴正值力矩/位置误差限额；同策略新目标连续接续。 目标=受理时命令TCP×增量；重复提交相同增量会再次相对移动。
 */
FR_API int fr_move_impedance_tool(fr_robot* robot, int arm,
                                  const double delta_position_m[3],
                                  const double delta_rpy_deg[3],
                                  double linear_speed_m_s, double rotation_speed_rad_s,
                                  uint64_t tag, fr_result* accepted);

#define FR_PATH_MAX_POINTS 32
typedef struct {
    double values[FR_ROBOT_JOINTS]; /**< Joint：j1..j7绝对角[deg]，须满足软限位；TCP：x/y/z[m]+RPY[deg]，第7项忽略；均须有限。 */
    double speed; /**< 入段参考速度，有限且>0；Joint deg/s，TCP m/s；受各自分类上限裁剪，无默认值。 */
    double rotation_speed; /**< TCP姿态速度，rad/s，有限且>0，无公共分类上限；Joint忽略。 */
    double accel; /**< 入段参考加速度，有限且>0；Joint deg/s²，TCP m/s²；受分类上限裁剪，无默认值。 */
    double dwell_s; /**< 参考到点后停留时间，s，有限且>=0；0仍至少保持一拍；正转接点须为0。 */
} fr_path_point;
/**
 * @brief 以 CSP执行关节多点停点路径；每点完成参考推进后停留。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] points 目标点数组，非 NULL、至少 count 项；values[0..6]为七轴绝对角度[deg]，须满足软限位；speed[deg/s]、accel[deg/s²]有限且>0，rotation_speed忽略。dwell_s[s]有限且>=0；各使用字段无省略默认。
 * @param[in] count 目标点数，整数 1..32，不包含当前起点，必须显式传入。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note 整表复制后受理；不保证全程 IK 可达或无碰撞。
 */
FR_API int fr_execute_joint_path(fr_robot* robot, int arm,
                                 const fr_path_point* points, int count,
                                 uint64_t tag, fr_result* accepted);
/**
 * @brief 以 CSP执行 TCP 多点停点路径；每点完成参考推进后停留。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] points 目标点数组，非 NULL、至少 count 项；values[0..5]=x/y/z[m]/roll/pitch/yaw[deg]（公共坐标系），values[6]忽略；speed[m/s]、rotation_speed[rad/s]、accel[m/s²]均有限且>0。dwell_s[s]有限且>=0；各使用字段无省略默认。
 * @param[in] count 目标点数，整数 1..32，不包含当前起点，必须显式传入。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note 整表复制后受理；不保证全程 IK 可达或无碰撞。
 */
FR_API int fr_execute_tcp_path(fr_robot* robot, int arm,
                               const fr_path_point* points, int count,
                               uint64_t tag, fr_result* accepted);
/**
 * @brief 以 CST 阻抗执行关节多点停点路径；每点完成参考推进后停留。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] points 目标点数组，非 NULL、至少 count 项；values[0..6]为七轴绝对角度[deg]，须满足软限位；speed[deg/s]、accel[deg/s²]有限且>0，rotation_speed忽略。dwell_s[s]有限且>=0；各使用字段无省略默认。
 * @param[in] count 目标点数，整数 1..32，不包含当前起点，必须显式传入。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note 整表复制后受理；不保证全程 IK 可达或无碰撞。 CST 要求已使能、autozero 完成及各轴正值力矩/位置误差限额；同策略新目标连续接续。
 */
FR_API int fr_execute_imp_joint_path(fr_robot* robot, int arm,
                                     const fr_path_point* points, int count,
                                     uint64_t tag, fr_result* accepted);
/**
 * @brief 以 CST 阻抗执行 TCP 多点停点路径；每点完成参考推进后停留。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] points 目标点数组，非 NULL、至少 count 项；values[0..5]=x/y/z[m]/roll/pitch/yaw[deg]（公共坐标系），values[6]忽略；speed[m/s]、rotation_speed[rad/s]、accel[m/s²]均有限且>0。dwell_s[s]有限且>=0；各使用字段无省略默认。
 * @param[in] count 目标点数，整数 1..32，不包含当前起点，必须显式传入。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note 整表复制后受理；不保证全程 IK 可达或无碰撞。 CST 要求已使能、autozero 完成及各轴正值力矩/位置误差限额；同策略新目标连续接续。
 */
FR_API int fr_execute_imp_tcp_path(fr_robot* robot, int arm,
                                   const fr_path_point* points, int count,
                                   uint64_t tag, fr_result* accepted);

/**
 * @brief 以 CSP执行关节连续过渡路径。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] points 目标点数组，非 NULL、至少 count 项；values[0..6]为七轴绝对角度[deg]，须满足软限位；speed[deg/s]、accel[deg/s²]有限且>0，rotation_speed忽略。dwell_s[s]有限且>=0；各使用字段无省略默认。
 * @param[in] radius 每点时间抛物线转接范围[deg，按最大轴距离]数组，非 NULL、至少 count 项；有限且>=0，0表示停点，末项必须0，正值点 dwell_s 必须0；超出相邻段可容纳范围则拒绝，不自动缩放。
 * @param[in] count 目标点数，整数 1..32，不包含当前起点，必须显式传入。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note 整表复制后受理；不保证全程 IK 可达或无碰撞。
 */
FR_API int fr_execute_joint_path_blend(fr_robot* robot, int arm,
                                      const fr_path_point* points, const double* radius,
                                      int count, uint64_t tag, fr_result* accepted);
/**
 * @brief 以 CSP执行 TCP 连续过渡路径。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] points 目标点数组，非 NULL、至少 count 项；values[0..5]=x/y/z[m]/roll/pitch/yaw[deg]（公共坐标系），values[6]忽略；speed[m/s]、rotation_speed[rad/s]、accel[m/s²]均有限且>0。dwell_s[s]有限且>=0；各使用字段无省略默认。
 * @param[in] radius 每点相切圆弧半径[m]数组，非 NULL、至少 count 项；有限且>=0，0表示停点，末项必须0，正值点 dwell_s 必须0；超出相邻段可容纳范围则拒绝，不自动缩放。
 * @param[in] count 目标点数，整数 1..32，不包含当前起点，必须显式传入。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note 整表复制后受理；不保证全程 IK 可达或无碰撞。
 */
FR_API int fr_execute_tcp_path_blend(fr_robot* robot, int arm,
                                    const fr_path_point* points, const double* radius,
                                    int count, uint64_t tag, fr_result* accepted);
/**
 * @brief 以 CST 阻抗执行关节连续过渡路径。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] points 目标点数组，非 NULL、至少 count 项；values[0..6]为七轴绝对角度[deg]，须满足软限位；speed[deg/s]、accel[deg/s²]有限且>0，rotation_speed忽略。dwell_s[s]有限且>=0；各使用字段无省略默认。
 * @param[in] radius 每点时间抛物线转接范围[deg，按最大轴距离]数组，非 NULL、至少 count 项；有限且>=0，0表示停点，末项必须0，正值点 dwell_s 必须0；超出相邻段可容纳范围则拒绝，不自动缩放。
 * @param[in] count 目标点数，整数 1..32，不包含当前起点，必须显式传入。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note 整表复制后受理；不保证全程 IK 可达或无碰撞。 CST 要求已使能、autozero 完成及各轴正值力矩/位置误差限额；同策略新目标连续接续。
 */
FR_API int fr_execute_imp_joint_path_blend(fr_robot* robot, int arm,
                                          const fr_path_point* points, const double* radius,
                                          int count, uint64_t tag, fr_result* accepted);
/**
 * @brief 以 CST 阻抗执行 TCP 连续过渡路径。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] points 目标点数组，非 NULL、至少 count 项；values[0..5]=x/y/z[m]/roll/pitch/yaw[deg]（公共坐标系），values[6]忽略；speed[m/s]、rotation_speed[rad/s]、accel[m/s²]均有限且>0。dwell_s[s]有限且>=0；各使用字段无省略默认。
 * @param[in] radius 每点相切圆弧半径[m]数组，非 NULL、至少 count 项；有限且>=0，0表示停点，末项必须0，正值点 dwell_s 必须0；超出相邻段可容纳范围则拒绝，不自动缩放。
 * @param[in] count 目标点数，整数 1..32，不包含当前起点，必须显式传入。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note 整表复制后受理；不保证全程 IK 可达或无碰撞。 CST 要求已使能、autozero 完成及各轴正值力矩/位置误差限额；同策略新目标连续接续。
 */
FR_API int fr_execute_imp_tcp_path_blend(fr_robot* robot, int arm,
                                        const fr_path_point* points, const double* radius,
                                        int count, uint64_t tag, fr_result* accepted);

/**
 * @brief 启动 CSP关节最新目标流；启动后通过 stream_update 提交目标。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] speed_deg_s 最快关节轴参考速度，单位 deg/s，有限且 >0，必须显式传入；与 motion_limits.joint_series.max_velocity_deg_per_s 取小（发布配置60）。
 * @param[in] accel_deg_s2 共同路径参考加速度，单位 deg/s²，有限且 >0，必须显式传入；与 motion_limits.joint_series.max_acceleration_deg_per_s2 取小（发布配置1000）。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note 未更新时继续执行最后目标并保持，不提供断流超时或自动停止。
 */
FR_API int fr_start_joint_stream(fr_robot* robot, int arm, double speed_deg_s,
                                 double accel_deg_s2,
                                 uint64_t tag, fr_result* accepted);
/**
 * @brief 启动 CSP TCP最新目标流；启动后通过 stream_update 提交目标。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] linear_speed_m_s TCP 平移参考速度，单位 m/s，有限且 >0，必须显式传入；与 motion_limits.tcp_series.max_linear_velocity_m_s 取小（发布配置0.3）。
 * @param[in] rotation_speed_rad_s 姿态参考角速度，单位 rad/s，有限且 >0，必须显式传入；无公共分类数值上限，0 不表示保持姿态。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note 未更新时继续执行最后目标并保持，不提供断流超时或自动停止。加速度使用 tcp 策略当前默认值。
 */
FR_API int fr_start_tcp_stream(fr_robot* robot, int arm, double linear_speed_m_s,
                               double rotation_speed_rad_s,
                               uint64_t tag, fr_result* accepted);
/**
 * @brief 启动 CST 阻抗关节最新目标流；启动后通过 stream_update 提交目标。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] speed_deg_s 最快关节轴参考速度，单位 deg/s，有限且 >0，必须显式传入；与 motion_limits.joint_series.max_velocity_deg_per_s 取小（发布配置60）。
 * @param[in] accel_deg_s2 共同路径参考加速度，单位 deg/s²，有限且 >0，必须显式传入；与 motion_limits.joint_series.max_acceleration_deg_per_s2 取小（发布配置1000）。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note 未更新时继续执行最后目标并保持，不提供断流超时或自动停止。 CST 要求已使能、autozero 完成及各轴正值力矩/位置误差限额；同策略新目标连续接续。
 */
FR_API int fr_start_imp_joint_stream(fr_robot* robot, int arm, double speed_deg_s,
                                     double accel_deg_s2,
                                     uint64_t tag, fr_result* accepted);
/**
 * @brief 启动 CST 阻抗 TCP最新目标流；启动后通过 stream_update 提交目标。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] linear_speed_m_s TCP 平移参考速度，单位 m/s，有限且 >0，必须显式传入；与 motion_limits.tcp_series.max_linear_velocity_m_s 取小（发布配置0.3）。
 * @param[in] rotation_speed_rad_s 姿态参考角速度，单位 rad/s，有限且 >0，必须显式传入；无公共分类数值上限，0 不表示保持姿态。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note 未更新时继续执行最后目标并保持，不提供断流超时或自动停止。加速度使用 imp_tcp 策略当前默认值。 CST 要求已使能、autozero 完成及各轴正值力矩/位置误差限额；同策略新目标连续接续。
 */
FR_API int fr_start_imp_tcp_stream(fr_robot* robot, int arm,
                                   double linear_speed_m_s,
                                   double rotation_speed_rad_s,
                                   uint64_t tag, fr_result* accepted);
/**
 * @brief 提交关节流的最新绝对目标；覆盖尚未消费的目标，不逐条排队。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] q_deg 七轴绝对关节角数组，非 NULL、长度7，顺序 j1..j7，单位 deg；全部有限且在各轴配置软限位内，无默认值。
 * @return FR_OK 仅表示目标已写入输入槽；类型错误返回 INVALID，流关闭返回 BUSY，未运行返回 NOT_READY；关节越限返回 REJECTED。
 */
FR_API int fr_stream_update_joint(fr_robot* robot, int arm,
                                  const double q_deg[FR_ROBOT_JOINTS]);
/**
 * @brief 提交 TCP 流的最新绝对目标；覆盖尚未消费的目标，不逐条排队。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] target 绝对 TCP 位姿，非 NULL；公共坐标系位置[m]及 RPY[deg]均须填写且有限；无默认值、无统一可达范围。
 * @return FR_OK 仅表示目标已写入输入槽；类型错误返回 INVALID，流关闭返回 BUSY，未运行返回 NOT_READY；关节越限返回 REJECTED。
 */
FR_API int fr_stream_update_tcp(fr_robot* robot, int arm, const fr_pose* target);
/**
 * @brief 关闭目标流并切换至 hold，冻结当前命令参考。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note 模式切换忙时拒绝；该操作不是机械急停。
 */
FR_API int fr_finish_stream(fr_robot* robot, int arm,
                            uint64_t tag, fr_result* accepted);

/**
 * @brief 保持当前 TCP 位姿调整几何臂角；不是单关节运动。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] phi_deg 目标几何臂角，单位 deg，有限、无默认值；无固定输入角度区间，可行范围由当前构型及软限位决定。
 * @param[in] speed_deg_s 臂角参考速度，单位 deg/s，有限且>0，必须显式传入；关节输出另受 armangle.joint_velocity_limit 限制。
 * @param[in] accel_deg_s2 兼容保留参数，单位 deg/s²；须有限且>0以通过校验，但不参与控制；无默认值，armangle不限制加速度。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_move_arm_angle(fr_robot* robot, int arm, double phi_deg,
                             double speed_deg_s, double accel_deg_s2,
                             uint64_t tag, fr_result* accepted);

/**
 * @brief 已移除的3D力源模式兼容入口；不执行任何策略。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] position_m 绝对 TCP 位置数组，非 NULL、长度3，公共坐标系 x/y/z，单位 m；全部有限，无统一可达范围、无默认值。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_ERR_REJECTED；仅保留链接兼容，不提供运行功能。
 */
FR_API int fr_move_attractor3(fr_robot* robot, int arm,
                              const double position_m[3],
                              uint64_t tag, fr_result* accepted);
/**
 * @brief 设置六维力估计吸引子的相对位置目标，保持姿态。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] delta_position_m 局部位移数组，非NULL、长度3，命令TCP系x/y/z，单位m；全部有限，无固定可达范围或默认值；目标=受理时命令位置+命令姿态×位移。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_move_attractor6(fr_robot* robot, int arm,
                              const double delta_position_m[3],
                              uint64_t tag, fr_result* accepted);
/**
 * @brief 设置六维吸引子位置目标，同时允许臂角零空间让位。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] delta_position_m 局部位移数组，非NULL、长度3，命令TCP系x/y/z，单位m；全部有限，无固定可达范围或默认值；目标=受理时命令位置+命令姿态×位移。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_move_attractor6null(fr_robot* robot, int arm,
                                  const double delta_position_m[3],
                                  uint64_t tag, fr_result* accepted);

/**
 * @brief 以接管时命令 TCP 的局部相对位移为目标，启动 TCP 优先的吸引子与臂角让位实验策略（CSP）。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] delta_position_m 局部位移数组，非NULL、长度3，命令TCP系x/y/z，单位m；全部有限，无固定可达范围或默认值；目标=受理时命令位置+命令姿态×位移。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_move_attractor6null_exp(fr_robot* robot, int arm,
                                      const double delta_position_m[3],
                                      uint64_t tag, fr_result* accepted);

/**
 * @name 策略启动参数
 * params=NULL 或对应 set_mask=0 时，使用该策略的当前默认值；置位字段仅覆盖本次启动，
 * 不改变 set_parameter 设置的默认。数组字段按 j1..j7，所用字段均须有限。
 * gain/deadband/lowpass_time 等未设置统一软件数值范围；数值合法不代表安全整定。
 * 当前发布配置初值（运行期可修改，使用 get_parameter 查询当前值）：
 * - drag：gain=[2,2,5,5,10,30,30]，deadband=[1.5,1.5,1.5,1.5,0.5,0.5,0.5]，
 *   lowpass_time=100ms，limit_taper_band=2deg，关节限速30deg/s、加速度500deg/s²。
 * - spring：gain=[2,2,5,5,20,30,30]，deadband=[1.5,2,1.5,2,0.5,0.5,0.5]，
 *   lowpass_time=100ms，limit_taper_band=4deg，pull_gain=1/s，pull_velocity_limit=60deg/s；
 *   最终关节限速30deg/s、加速度500deg/s²。
 * - cart/attractor各策略：gain=0.0075m/(s·N)，deadband=5N，lowpass_time=100ms，
 *   velocity_limit=0.1m/s，acceleration_limit=5m/s²，关节限速30deg/s。
 * - attractor各策略：attract_speed=0.01m/s，attract_tolerance=0.002m；轴向offset=0m。
 * - null：gain=3deg/(s·Nm)，deadband=1Nm，max_step=0.2deg/拍，
 *   velocity_limit内建30deg/s，关节限速30deg/s，无加速度限制。
 * - cart6null：null_gain=3deg/(s·Nm)；attractor6null：null_gain=10deg/(s·Nm)；
 *   二者null_deadband=1Nm，null_velocity_limit=60deg/s，null_min_elbow_radius=0.03m。
 */
#define FR_DRAG_SET_GAIN            (1u << 0)
#define FR_DRAG_SET_DEADBAND        (1u << 1)
#define FR_DRAG_SET_VELOCITY_LIMIT  (1u << 2) /**< 保留位，禁止置位；改用策略joint_velocity_limit。 */
#define FR_DRAG_SET_LOWPASS_TIME    (1u << 3)
#define FR_DRAG_SET_TAPER_BAND      (1u << 4)

typedef struct {
    uint32_t set_mask; /**< FR_DRAG_SET_*按位或；0使用策略当前默认；每一数组位控制整组7轴。 */
    double gain[FR_ROBOT_JOINTS]; /**< 外力矩到关节速度增益，7项，deg/(s·Nm)；置位时有限，无软件数值上下限。 */
    double deadband[FR_ROBOT_JOINTS]; /**< 力矩死区，7项，Nm；置位时有限，无软件数值上下限。 */
    double velocity_limit[FR_ROBOT_JOINTS]; /**< 保留字段，不使用；置 FR_DRAG_SET_VELOCITY_LIMIT 返回 INVALID。 */
    double lowpass_time[FR_ROBOT_JOINTS]; /**< 速度低通时间，7项，ms；0直通；置位时有限，无软件数值上下限。 */
    double limit_taper_band[FR_ROBOT_JOINTS]; /**< 接近限位的减速带宽，7项，deg；置位时有限，无软件数值上下限。 */
} fr_drag_params;

#define FR_SPRING_SET_PULL_GAIN           (1u << 0)
#define FR_SPRING_SET_PULL_VELOCITY_LIMIT (1u << 1)

typedef struct {
    fr_drag_params drag; /**< 弹簧策略的导纳覆盖；使用其独立set_mask，不继承drag策略默认。 */
    uint32_t set_mask; /**< FR_SPRING_SET_*按位或；0保留spring策略默认。 */
    double pull_gain; /**< 回靠速度/角度误差增益，1/s；有限，无软件数值上下限。 */
    double pull_velocity_limit; /**< 回靠速度上限，deg/s，有限且>0；最终还受spring关节限速限制。 */
} fr_spring_params;

#define FR_CART_SET_GAIN              (1u << 0)
#define FR_CART_SET_VELOCITY_LIMIT    (1u << 1)
#define FR_CART_SET_DEADBAND          (1u << 2)
#define FR_CART_SET_LOWPASS_TIME      (1u << 3)
#define FR_CART_SET_ACCELERATION      (1u << 4)

typedef struct {
    uint32_t set_mask; /**< FR_CART_SET_*按位或；0使用所选策略当前默认。 */
    double gain; /**< 平移导纳增益，m/(s·N)；有限，无软件数值上下限。 */
    double velocity_limit; /**< 平移导纳速度上限，m/s，有限且>0；不保证最终合成速度范数。 */
    double deadband; /**< 力死区，N；有限，无软件数值上下限。 */
    double lowpass_time; /**< 平移导纳速度低通时间，ms；0直通；有限，无软件数值上下限。 */
    double acceleration_limit; /**< 平移导纳加速度上限，m/s²，有限且>0；不保证最终合成加速度范数。 */
} fr_cart_params;

#define FR_ATTRACT_SET_SPEED          (1u << 0)
#define FR_ATTRACT_SET_TOLERANCE      (1u << 1)

typedef struct {
    fr_cart_params cart; /**< 所选吸引子策略的导纳参数覆盖；使用其独立set_mask。 */
    uint32_t set_mask; /**< FR_ATTRACT_SET_*按位或；0保留所选策略默认。 */
    double attract_speed; /**< 向目标位置吸引的速度，m/s，有限且>0；无固定数值上限。 */
    double attract_tolerance; /**< 吸引目标位置容差，m；有限，无统一软件数值上下限。 */
} fr_attractor_params;

#define FR_NULL_SET_GAIN              (1u << 0)
#define FR_NULL_SET_DEADBAND          (1u << 1)
#define FR_NULL_SET_VELOCITY_LIMIT    (1u << 2)
#define FR_NULL_SET_MAX_STEP          (1u << 3)

typedef struct {
    uint32_t set_mask; /**< FR_NULL_SET_*按位或；0使用null策略当前默认。 */
    double gain; /**< 外力矩到臂角速度增益，deg/(s·Nm)；有限，无软件数值上下限。 */
    double deadband; /**< 臂角驱动力矩死区，Nm；有限，无软件数值上下限。 */
    double velocity_limit; /**< 臂角速度上限，deg/s，有限且>0；关节输出另受null.joint_velocity_limit限制。 */
    double max_step; /**< 回靠锁定关节参考的单轴最大步长，deg/拍；有限，无统一软件数值上下限。 */
} fr_null_params;

#define FR_CART6NULL_NULL_SET_GAIN            (1u << 0)
#define FR_CART6NULL_NULL_SET_DEADBAND        (1u << 1)
#define FR_CART6NULL_NULL_SET_VELOCITY_LIMIT  (1u << 2)
#define FR_CART6NULL_NULL_SET_MIN_ELBOW       (1u << 3)

typedef struct {
    fr_cart_params cart; /**< cart6null策略的导纳参数覆盖。 */
    uint32_t null_set_mask; /**< FR_CART6NULL_NULL_SET_*按位或；0保留cart6null的臂角参数默认。 */
    double null_gain; /**< 臂角让位增益，deg/(s·Nm)，有限，无软件数值上下限。 */
    double null_deadband; /**< 臂角驱动力矩死区，Nm，有限，无软件数值上下限。 */
    double null_velocity_limit; /**< 臂角速度上限，deg/s，有限且>0。 */
    double null_min_elbow_radius; /**< 允许臂角让位的最小肘圆半径，m；有限，负数按0处理，无软件上限。 */
} fr_cart6null_params;

#define FR_ATTRACTOR6NULL_NULL_SET_GAIN            (1u << 0)
#define FR_ATTRACTOR6NULL_NULL_SET_DEADBAND        (1u << 1)
#define FR_ATTRACTOR6NULL_NULL_SET_VELOCITY_LIMIT  (1u << 2)
#define FR_ATTRACTOR6NULL_NULL_SET_MIN_ELBOW       (1u << 3)

typedef struct {
    fr_attractor_params attractor; /**< attractor6null策略的吸引/导纳参数覆盖。 */
    uint32_t null_set_mask; /**< FR_ATTRACTOR6NULL_NULL_SET_*按位或；0保留本策略臂角参数默认。 */
    double null_gain; /**< 臂角让位增益，deg/(s·Nm)，有限，无软件数值上下限。 */
    double null_deadband; /**< 臂角驱动力矩死区，Nm，有限，无软件数值上下限。 */
    double null_velocity_limit; /**< 臂角速度上限，deg/s，有限且>0。 */
    double null_min_elbow_radius; /**< 允许让位的最小肘圆半径，m；有限，负数按0处理，无软件上限。 */
} fr_attractor6null_params;

#define FR_ATTRACTOR_AXIS_SET_OFFSET (1u << 0)

typedef struct {
    fr_attractor_params attractor; /**< 所选轴向吸引子策略的吸引/导纳参数覆盖。 */
    uint32_t set_mask; /**< FR_ATTRACTOR_AXIS_SET_OFFSET位控制offset；0保留所选策略默认。 */
    double offset; /**< 目标相对启动命令TCP的位置偏移，m；沿axis指定轴，有限，无固定范围；0为启动位置。 */
} fr_attractor_axis_params;

/**
 * @brief 启动 CST 关节拖动；外力矩经导纳转换为关节参考速度。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] params 本次启动的参数覆盖，可为 NULL（使用该策略当前默认值）；非 NULL 时仅 set_mask 指定字段生效，字段范围和单位见对应结构。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note CST 要求已使能、autozero 完成及各轴正值力矩/位置误差限额；同策略新目标连续接续。
 */
FR_API int fr_start_drag(fr_robot* robot, int arm, const fr_drag_params* params,
                         uint64_t tag, fr_result* accepted);
/**
 * @brief 启动 CST 弹簧策略；拖动释放后回靠启动时锁定的关节参考。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] params 本次启动的参数覆盖，可为 NULL（使用该策略当前默认值）；非 NULL 时仅 set_mask 指定字段生效，字段范围和单位见对应结构。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note CST 要求已使能、autozero 完成及各轴正值力矩/位置误差限额；同策略新目标连续接续。
 */
FR_API int fr_start_spring(fr_robot* robot, int arm, const fr_spring_params* params,
                           uint64_t tag, fr_result* accepted);
/**
 * @brief 已移除的3D力源模式兼容入口；不执行任何策略。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] params 本次启动的参数覆盖，可为 NULL（使用该策略当前默认值）；非 NULL 时仅 set_mask 指定字段生效，字段范围和单位见对应结构。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_ERR_REJECTED；仅保留链接兼容，不提供运行功能。
 */
FR_API int fr_start_cart3(fr_robot* robot, int arm, const fr_cart_params* params,
                          uint64_t tag, fr_result* accepted);
/**
 * @brief 启动六维力估计笛卡尔导纳，保持姿态，使用 CSP。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] params 本次启动的参数覆盖，可为 NULL（使用该策略当前默认值）；非 NULL 时仅 set_mask 指定字段生效，字段范围和单位见对应结构。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_start_cart6(fr_robot* robot, int arm, const fr_cart_params* params,
                          uint64_t tag, fr_result* accepted);
/**
 * @brief 已移除的3D力源模式兼容入口；不执行任何策略。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] params 本次启动的参数覆盖，可为 NULL（使用该策略当前默认值）；非 NULL 时仅 set_mask 指定字段生效，字段范围和单位见对应结构。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_ERR_REJECTED；仅保留链接兼容，不提供运行功能。
 */
FR_API int fr_start_attractor3(fr_robot* robot, int arm,
                               const fr_attractor_params* params,
                               uint64_t tag, fr_result* accepted);
/**
 * @brief 以当前命令 TCP 位置为吸引目标启动六维吸引子。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] params 本次启动的参数覆盖，可为 NULL（使用该策略当前默认值）；非 NULL 时仅 set_mask 指定字段生效，字段范围和单位见对应结构。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_start_attractor6(fr_robot* robot, int arm,
                               const fr_attractor_params* params,
                               uint64_t tag, fr_result* accepted);
/**
 * @brief 启动零空间让位；保持 TCP 位姿并根据外力矩调整臂角。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] params 本次启动的参数覆盖，可为 NULL（使用该策略当前默认值）；非 NULL 时仅 set_mask 指定字段生效，字段范围和单位见对应结构。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_start_null(fr_robot* robot, int arm, const fr_null_params* params,
                         uint64_t tag, fr_result* accepted);
/**
 * @brief 启动六维笛卡尔导纳及臂角零空间让位；使用 CSP。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] params 本次启动的参数覆盖，可为 NULL（使用该策略当前默认值）；非 NULL 时仅 set_mask 指定字段生效，字段范围和单位见对应结构。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_start_cart6null(fr_robot* robot, int arm,
                              const fr_cart6null_params* params,
                              uint64_t tag, fr_result* accepted);

/**
 * @brief 启动 TCP 优先的 XYZ 导纳与臂角让位实验策略（CSP）。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] params 本次启动的参数覆盖，可为 NULL（使用该策略当前默认值）；非 NULL 时仅各掩码选中的字段生效，字段范围和单位见对应结构。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_start_cart6null_exp(fr_robot* robot, int arm,
                                  const fr_cart6null_params* params,
                                  uint64_t tag, fr_result* accepted);
/**
 * @brief 以当前命令位置为吸引目标，启动六维吸引子及零空间让位。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] params 本次启动的参数覆盖，可为 NULL（使用该策略当前默认值）；非 NULL 时仅 set_mask 指定字段生效，字段范围和单位见对应结构。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_start_attractor6null(fr_robot* robot, int arm,
                                   const fr_attractor6null_params* params,
                                   uint64_t tag, fr_result* accepted);

/**
 * @brief 以接管时命令位置为吸引目标，启动 TCP 优先的吸引子与臂角让位实验策略（CSP）。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] params 本次启动的参数覆盖，可为 NULL（使用该策略当前默认值）；非 NULL 时仅各掩码选中的字段生效，字段范围和单位见对应结构。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_start_attractor6null_exp(fr_robot* robot, int arm,
                                       const fr_attractor6null_params* params,
                                       uint64_t tag, fr_result* accepted);

/**
 * @brief 启动沿指定命令 TCP 轴方向的六维笛卡尔导纳。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] axis 启动时命令 TCP 坐标轴：0=x、1=y、2=z；必须显式传入。
 * @param[in] params 本次启动的参数覆盖，可为 NULL（使用该策略当前默认值）；非 NULL 时仅 set_mask 指定字段生效，字段范围和单位见对应结构。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_start_cart6_axis(fr_robot* robot, int arm, int axis,
                               const fr_cart_params* params,
                               uint64_t tag, fr_result* accepted);
/**
 * @brief 启动指定命令 TCP 轴方向的吸引子，目标由启动位置加轴向偏置得到。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] axis 启动时命令 TCP 坐标轴：0=x、1=y、2=z；必须显式传入。
 * @param[in] params 本次启动的参数覆盖，可为 NULL（使用该策略当前默认值）；非 NULL 时仅 set_mask 指定字段生效，字段范围和单位见对应结构。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_start_attractor6_axis(fr_robot* robot, int arm, int axis,
                                    const fr_attractor_axis_params* params,
                                    uint64_t tag, fr_result* accepted);

/* Shared wrench measurement ABI. Either public control header can be included independently. */
#ifndef FR_WRENCH_TYPES_DEFINED
#define FR_WRENCH_TYPES_DEFINED

typedef enum {
    FR_WRENCH_REFERENCE = 0, /**< 锁存当前有效帧为参考。 */
    FR_WRENCH_READ = 1,      /**< 读取最新有效帧相对于参考的变化。 */
    FR_WRENCH_START = 2,     /**< 启动有限次异步采样。 */
    FR_WRENCH_STOP = 3,      /**< 停止采样并等待在途回调退出。 */
    FR_WRENCH_CLEAR_REFERENCE = 4 /**< 清除本臂参考，不停止采样。 */
} fr_wrench_operation;

typedef struct {
    uint64_t cycle;       /**< 控制拍号。 */
    uint64_t timestamp_ns; /**< 同拍系统时钟 Unix 时间戳，ns；采样调度不依赖此时间戳。 */
    int32_t torque_lsb[7]; /**< 解环绕后的驱动器原始力矩，LSB；仅供诊断，不参与六维换算。 */
    double torque_nm[7];   /**< 同拍发布的滤波后实测关节力矩，Nm。 */
    double wrench[6];      /**< 在 TCP 处、以基座坐标表示的 Fx/Fy/Fz/Mx/My/Mz，N/Nm。 */
} fr_wrench_frame;

typedef struct {
    fr_wrench_frame reference;   /**< 参考帧。 */
    fr_wrench_frame measurement; /**< 测量帧；REFERENCE 和 valid=0 时不提供。 */
    double wrench[6];            /**< READ/采样的测量减参考，N/Nm。 */
    int valid;                   /**< 1 表示本次参考或差值有效，0 表示仅状态/统计。 */
    uint64_t sampled;            /**< 本次异步采样已交付的有效样本数。 */
    uint64_t missed;             /**< 本次异步采样已错过或无效的采样机会数。 */
    int active;                  /**< 1 表示采样中或样本通知，0 表示空闲/结束。 */
} fr_wrench_result;

/** 回调在非实时线程执行；result 只在调用期间有效。 */
typedef void (*fr_wrench_callback)(const fr_wrench_result* result, void* user);

typedef struct {
    fr_wrench_operation operation;
    uint64_t sample_count;      /**< START 必填，有效交付数，至少 1；其他操作忽略。 */
    double sample_hz;           /**< START 必填，有限正数且不超过控制拍率，Hz；其他操作忽略。 */
    fr_wrench_callback callback; /**< START 必填非 NULL；其他操作忽略。 */
    void* user;                  /**< START 传给回调的自定义指针，可为 NULL。 */
} fr_wrench_request;

#endif /* FR_WRENCH_TYPES_DEFINED */

/**
 * @brief 保存或清除参考、读取滤波力矩换算的六维差值，或启停每臂独立的定次异步采样。
 * @note 两帧分别使用各自的同拍实测关节角和 TCP 解算，输入为负的实测关节力矩；
 * 使用现有滤波后实测力矩（含读侧已应用的零漂校正），不读取控制外力估计，
 * 不另做重力、载荷或摩擦补偿，不要求使能/autozero。
 * 未使能或滤波窗口为 0 时，现有滤波通道直通；不对实测关节角作滤波延迟对齐。
 * START 按控制拍调度；无效反馈、发布缺帧或回调来不及接收均计 missed，不补交旧帧，
 * 达到有效样本数后自动停止。每个样本 valid=1、active=1；另发结束通知 valid=0、active=0。
 * STOP 幂等，取消未开始的回调并等待已在途回调返回，不额外发结束通知；
 * 显式 STOP 或实例停止/销毁完成后不再调用该回调。同实例回调内不得调用本入口或生命周期入口。
 * CLEAR_REFERENCE 幂等，不要求运行或有效反馈，不改变采样计数；
 * 本臂正在采样或回调尚未返回时，REFERENCE 和 CLEAR_REFERENCE 返回 FR_ERR_BUSY，不改变参考。
 * @param[in] robot 控制实例句柄，非 NULL，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1，不支持广播。
 * @param[in] request 请求结构，非 NULL；字段和单位见 fr_wrench_request。
 * @param[out] result 输出；REFERENCE/READ 必须非 NULL，其他操作可为 NULL，无需预填。
 * CLEAR_REFERENCE 成功时仅返回状态/统计，参考帧清零，valid=0。
 * @return FR_OK 表示成功（START 仅受理）；FR_ERR_INVALID 参数无效；
 * FR_ERR_NOT_FOUND 读或启动时无参考；FR_ERR_NOT_READY 设置参考、读或启动时未运行或无有效反馈；
 * FR_ERR_BUSY 正在采样、回调未退出或在同实例回调内调用；FR_ERR_HARDWARE 后台线程启动失败。
 */
FR_API fr_status fr_measure_wrench(fr_robot* robot, int arm,
    const fr_wrench_request* request, fr_wrench_result* result);

/**
 * @brief 复制最新控制状态；停止后仍可能返回最后一帧，不代表实时有效。
 * @note 状态版本3包含命令/实测TCP及估计链摩擦有效位；必须使用配套版本3头文件和库重新编译，不得混用旧结构。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[out] state 状态输出，非 NULL；提供一个 fr_state 对象，无须预填版本。
 * @return FR_OK 表示输出成功；无数据返回 FR_ERR_NOT_READY，参数无效返回 FR_ERR_INVALID。
 */
FR_API int fr_get_state(fr_robot* robot, fr_state* state);

/**
 * @brief 复制最近若干状态，按时间升序排列；不消费历史，重复读取可重叠。
 * @note 状态版本3包含命令/实测TCP及估计链摩擦有效位；必须使用配套版本3头文件和库重新编译，不得混用旧结构。
 * @note 同拍状态由非实时线程发布；若发布队列溢出，历史完整性锁存失效，返回HARDWARE，不返回伪连续历史。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[out] buffer 状态输出数组，非 NULL，至少 max_count 个 state 对象。
 * @param[in] max_count 输出数组可容纳的状态帧数，整数 >0；返回帧数不超过此值，无默认值。
 * @param[out] first_sequence 首个返回帧的全局历史序号输出，可为 NULL；不是控制 cycle，可用于去重和缺口检查。
 * @return 实际复制帧数（>0），不超过 max_count；无状态时返回 NOT_READY，参数错误返回 INVALID。
 */
FR_API int fr_read_state_history(fr_robot* robot, fr_state* buffer,
                                 int max_count, uint64_t* first_sequence);

typedef void (*fr_state_callback)(const fr_state* state, void* user);
/**
 * @brief 注册唯一状态订阅，按控制拍抽样；慢回调会丢弃旧帧。
 * @note 状态版本3包含命令/实测TCP及估计链摩擦有效位；必须使用配套版本3头文件和库重新编译，不得混用旧结构。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] every_n_cycles 回调抽样间隔，整数 >=1，单位为控制拍，无默认值；500Hz 时10拍对应20ms。
 * @param[in] callback 状态回调函数，非 NULL；state 指针仅回调期间有效，回调不得阻塞。
 * @param[in] user 回调上下文指针，可为 NULL；SDK 仅透传、不释放，须覆盖在途回调的生命周期。
 * @return FR_OK 表示操作成功；负值为 fr_status 错误码。
 * @note SDK 回调内调用返回 BUSY；生命周期操作须在外部串行化。
 */
FR_API int fr_subscribe_state(fr_robot* robot, int every_n_cycles,
                              fr_state_callback callback, void* user);
/**
 * @brief 取消状态订阅并等待正在执行的状态回调退出。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @return FR_OK 表示操作成功；负值为 fr_status 错误码。
 * @note SDK 回调内调用返回 BUSY；生命周期操作须在外部串行化。
 */
FR_API int fr_unsubscribe_state(fr_robot* robot);

/**
 * @brief 修改运行期参数；校验通过后生效，不写入 JSON。
 * @note servo/cst.max_lag_deg仅保留运行期兼容读写，当前不参与CST冻结且不从JSON加载；joint_index必须-1。旧位置/速度误差兼容字段不作关断门。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] strategy 策略名，非 NULL 字符串，如 "joint"、"tcp"、"drag"、"servo"、"motion_limits"；必须与参数所属策略匹配。
 * @param[in] name 参数名，非 NULL 字符串；例如 joint 的 "rate"、drag 的 "gain"、servo 的 "cst.kp"；无默认名称。
 * @param[in] value 参数值，须有限；速度/加速度类须 >0，其他范围取决于具体参数，无统一默认值或数值上限；servo参数另按各字段约束校验。
 * @param[in] joint_index 关节索引 0..6；-1 表示标量或全部关节；读取逐轴参数时 -1 返回 j1 的值。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_set_parameter(fr_robot* robot, int arm, const char* strategy,
                            const char* name, double value, int joint_index,
                            uint64_t tag, fr_result* accepted);

/**
 * @brief 原子设置运行期工具负载质量和质心，不修改 mass.json。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] mass_kg 工具负载质量，单位 kg，有限且 >=0；0 表示无负载，必须显式传入，无软件数值上限。
 * @param[in] com_m 负载质心数组，非 NULL、长度3，标定 tool 坐标系 x/y/z，单位 m；全部有限，质量为0仍须传入，无统一数值上限。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_set_payload(fr_robot* robot, int arm, double mass_kg,
                          const double com_m[3], uint64_t tag,
                          fr_result* accepted);
/**
 * @brief 读取策略当前默认参数，包含配置预设和运行期覆盖值。
 * @note servo/cst.max_lag_deg返回本臂兼容值[deg]，不参与CST冻结；joint_index必须-1。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] strategy 策略名，非 NULL 字符串，如 "joint"、"tcp"、"drag"、"servo"、"motion_limits"；必须与参数所属策略匹配。
 * @param[in] name 参数名，非 NULL 字符串；例如 joint 的 "rate"、drag 的 "gain"、servo 的 "cst.kp"；无默认名称。
 * @param[in] joint_index 关节索引 0..6；-1 表示标量或全部关节；读取逐轴参数时 -1 返回 j1 的值。
 * @param[out] value 参数值输出，非 NULL；单位和范围由 strategy/name 决定，失败时不得使用该输出。
 * @return FR_OK 表示查询成功；负值为状态错误码，包括无效参数、未就绪、拒绝或查询超时。
 */
FR_API int fr_get_parameter(fr_robot* robot, int arm, const char* strategy,
                            const char* name, int joint_index, double* value);

/**
 * @brief 读取参数，并在运行层拒绝时回传具体原因。
 * @param[in] robot 控制实例，非 NULL。
 * @param[in] arm 逻辑臂号 0 或 1。
 * @param[in] strategy 参数所属策略名，非 NULL。
 * @param[in] name 参数名，非 NULL。
 * @param[in] joint_index 关节索引 0..6，-1 表示标量或按 j1 读取逐轴值。
 * @param[out] value 参数值，非 NULL，仅成功时有效。
 * @param[out] reason 原因文本缓冲区；容量为 0 时可传 NULL。
 * @param[in] reason_size 缓冲区字节数，包括末尾 NUL；0 表示不接收文本。
 * @return FR_OK 成功；无值或被拒绝返回 NOT_FOUND；其他错误同 fr_get_parameter。
 */
FR_API int fr_get_parameter_detail(fr_robot* robot, int arm, const char* strategy,
    const char* name, int joint_index, double* value, char* reason, size_t reason_size);

/**
 * @brief 原子设置七轴 CST 比例及阻尼增益；任一值无效则整组拒绝。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] kp 七轴比例增益数组，非 NULL、长度7，j1..j7，单位 Nm/deg；全部有限且 >=0，无软件数值上限；必须显式传入。
 * @param[in] kd 七轴阻尼增益数组，非 NULL、长度7，j1..j7，单位 Nm/(deg/s)；全部有限且 >=0，无软件数值上限；必须显式传入。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_set_cst_gains(fr_robot* robot, int arm,
                            const double kp[FR_ROBOT_JOINTS],
                            const double kd[FR_ROBOT_JOINTS],
                            uint64_t tag, fr_result* accepted);

/**
 * @brief 读取同一控制拍的七轴 CST 增益快照。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[out] kp 增益输出数组，非 NULL、容量至少7，顺序j1..j7；单位Nm/deg。
 * @param[out] kd 增益输出数组，非 NULL、容量至少7，顺序j1..j7；单位Nm/(deg/s)。
 * @return FR_OK 表示查询成功；负值为状态错误码，包括无效参数、未就绪、拒绝或查询超时。
 */
FR_API int fr_get_cst_gains(fr_robot* robot, int arm,
                            double kp[FR_ROBOT_JOINTS],
                            double kd[FR_ROBOT_JOINTS]);

/**
 * @brief 以 CST 执行关节空间阻抗运动，并显式指定参考加速度。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] q_deg 七轴绝对关节角数组，非 NULL、长度7，顺序 j1..j7，单位 deg；全部有限且在各轴配置软限位内，无默认值。
 * @param[in] speed_deg_s 最快关节轴参考速度，单位 deg/s，有限且 >0，必须显式传入；与 motion_limits.joint_series.max_velocity_deg_per_s 取小（发布配置60）。
 * @param[in] accel_deg_s2 共同路径参考加速度，单位 deg/s²，有限且 >0，必须显式传入；与 motion_limits.joint_series.max_acceleration_deg_per_s2 取小（发布配置1000）。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note CST 要求已使能、autozero 完成及各轴正值力矩/位置误差限额；同策略新目标连续接续。
 */
FR_API int fr_move_impedance_joint_with_acceleration(fr_robot* robot, int arm,
                                   const double q_deg[FR_ROBOT_JOINTS],
                                   double speed_deg_s,
                                   double accel_deg_s2,
                                   uint64_t tag, fr_result* accepted);
/**
 * @brief 以 CST 执行绝对 TCP 目标运动，并显式指定参考加速度。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] target 绝对 TCP 位姿，非 NULL；公共坐标系位置[m]及 RPY[deg]均须填写且有限；无默认值、无统一可达范围。
 * @param[in] linear_speed_m_s TCP 平移参考速度，单位 m/s，有限且 >0，必须显式传入；与 motion_limits.tcp_series.max_linear_velocity_m_s 取小（发布配置0.3）。
 * @param[in] rotation_speed_rad_s 姿态参考角速度，单位 rad/s，有限且 >0，必须显式传入；无公共分类数值上限，0 不表示保持姿态。
 * @param[in] linear_accel_m_s2 TCP 平移参考加速度，单位 m/s²，有限且 >0，必须显式传入；与 motion_limits.tcp_series.max_linear_acceleration_m_s2 取小（发布配置5）。
 * @param[in] rotation_accel_rad_s2 姿态参考角加速度，单位 rad/s²，有限且 >0，必须显式传入；无公共分类数值上限。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note CST 要求已使能、autozero 完成及各轴正值力矩/位置误差限额；同策略新目标连续接续。
 */
FR_API int fr_move_impedance_tcp_with_acceleration(fr_robot* robot, int arm, const fr_pose* target,
                                 double linear_speed_m_s, double rotation_speed_rad_s,
                                 double linear_accel_m_s2, double rotation_accel_rad_s2,
                                   uint64_t tag, fr_result* accepted);
/**
 * @brief 以 CST 执行相对 TCP 运动，并显式指定参考加速度。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] delta_position_m 平移增量数组，非 NULL、长度3，顺序 x/y/z，单位 m；沿受理时命令 TCP 坐标轴，全部有限，无统一距离上限、无默认值。
 * @param[in] delta_rpy_deg 右乘姿态增量数组，长度3，顺序 roll/pitch/yaw，单位 deg，Rz×Ry×Rx；全部有限，可为 NULL（等同三个0）。
 * @param[in] linear_speed_m_s TCP 平移参考速度，单位 m/s，有限且 >0，必须显式传入；与 motion_limits.tcp_series.max_linear_velocity_m_s 取小（发布配置0.3）。
 * @param[in] rotation_speed_rad_s 姿态参考角速度，单位 rad/s，有限且 >0，必须显式传入；无公共分类数值上限，0 不表示保持姿态。
 * @param[in] linear_accel_m_s2 TCP 平移参考加速度，单位 m/s²，有限且 >0，必须显式传入；与 motion_limits.tcp_series.max_linear_acceleration_m_s2 取小（发布配置5）。
 * @param[in] rotation_accel_rad_s2 姿态参考角加速度，单位 rad/s²，有限且 >0，必须显式传入；无公共分类数值上限。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note CST 要求已使能、autozero 完成及各轴正值力矩/位置误差限额；同策略新目标连续接续。 目标=受理时命令TCP×增量；重复提交相同增量会再次相对移动。
 */
FR_API int fr_move_impedance_tool_with_acceleration(fr_robot* robot, int arm,
                                  const double delta_position_m[3],
                                  const double delta_rpy_deg[3],
                                  double linear_speed_m_s, double rotation_speed_rad_s,
                                  double linear_accel_m_s2, double rotation_accel_rad_s2,
                                   uint64_t tag, fr_result* accepted);

typedef struct fr_csv_capture_status {
    uint32_t struct_size; /**< 调用前填 sizeof(本结构)。 */
    uint32_t struct_version; /**< 调用前填1。 */
    int active; /**< 1正在接收采集帧；0未采集。 */
    int worker_running; /**< 1后台写入线程仍在运行；0已停止。 */
    uint64_t accepted_frames; /**< 成功进入队列的累计控制帧数。 */
    uint64_t written_frames; /**< 成功写入完整帧的累计数量。 */
    uint64_t write_failed_frames; /**< 写入失败的累计帧数。 */
    uint64_t dropped_full; /**< 队列满导致丢弃的累计帧数。 */
    uint64_t dropped_busy; /**< 实时侧队列忙导致丢弃的累计帧数。 */
    uint64_t rejected_inactive; /**< 未开启采集时拒收的累计帧数。 */
    uint64_t rejected_invalid; /**< 无效输入导致拒收的累计帧数。 */
    uint64_t io_errors; /**< 文件I/O累计错误次数。 */
    uint64_t pending_frames; /**< 尚未写入完成的帧数。 */
    int last_errno; /**< 最近一次系统I/O错误号；0表示未记录错误。 */
} fr_csv_capture_status;
/**
 * @brief 开始逐拍 CSV 后台采集；需要 SDK 已启动，不要求使能。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] path 文件路径；可为 NULL 或空字符串以生成时间唯一名称；显式路径会覆盖同名普通文件，父目录须已存在。
 * @return FR_OK 表示操作成功；负值为 fr_status 错误码。
 * @note 采集位置/命令位置[deg]、速度[deg/s]、raw整数LSB、滤波力矩[Nm]及cycle/arm；有界队列可能丢帧，应检查状态计数。回调内返回 BUSY。
 */
FR_API int fr_csv_capture_start(fr_robot* robot, const char* path);
/**
 * @brief 停止接收采集帧，排空队列并关闭文件。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @return FR_OK 表示操作成功；负值为 fr_status 错误码。
 * @note SDK 回调内返回 BUSY；排空可能阻塞，写盘失败返回 HARDWARE。
 */
FR_API int fr_csv_capture_stop(fr_robot* robot);

/**
 * @brief 查询实际采集文件的绝对路径；停止采集后路径保留。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[out] buffer 路径输出缓冲，非 NULL，容量为 buffer_size 字节；成功写入含结尾 NUL 的绝对路径。
 * @param[in] buffer_size 输出路径缓冲的字节容量，包含结尾 NUL，须足够容纳完整路径，无默认值。
 * @return FR_OK；无历史采集路径返回 NOT_READY；缓冲不足返回 INVALID 且不修改 buffer。
 */
FR_API int fr_csv_capture_get_path(fr_robot* robot, char* buffer, size_t buffer_size);
/**
 * @brief 查询采集状态及生命周期累计计数；计数单位为控制拍，不是 CSV 行。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in,out] out 输出结构，非 NULL；调用前填 struct_size=sizeof(*out)、struct_version=1；各计数按控制拍累计，运行中为近似快照，stop 后可核账。
 * @return FR_OK 表示输出成功；无数据返回 FR_ERR_NOT_READY，参数无效返回 FR_ERR_INVALID。
 */
FR_API int fr_csv_capture_get_status(fr_robot* robot, fr_csv_capture_status* out);

#define FR_JOINT_DIAGNOSTICS_VERSION 3u
typedef struct {
    uint32_t struct_size; /**< 调用前填 sizeof(本结构)。 */
    uint32_t struct_version; /**< 调用前填FR_JOINT_DIAGNOSTICS_VERSION，当前3。 */
    uint64_t timestamp_ns; /**< 快照时间戳，ns。 */
    uint64_t cycle; /**< 摩擦估计所属控制拍号。 */
    double modeled_friction_torque_nm[FR_ROBOT_MAX_ARMS][7]; /**< 模型实际扣除的摩擦力矩估计，Nm；不是fc参数或传感器实测。 */
    uint32_t friction_valid[FR_ROBOT_MAX_ARMS][7]; /**< 逐轴摩擦估计有效位，0/1；未使能/未autozero/输入失效时0。 */
    uint16_t drive_error_code[FR_ROBOT_MAX_ARMS][7]; /**< 逐轴0x603F:00缓存值，16bit；0可表示成功读到无错误。 */
    uint32_t drive_error_valid[FR_ROBOT_MAX_ARMS][7]; /**< 逐轴错误码有效位，0未读取或最近读取失败，1读取成功；失败时数值0但不代表无错误。 */
    uint64_t drive_error_cycle[FR_ROBOT_MAX_ARMS][7]; /**< 逐轴错误码采样拍；不保证等于快照cycle。 */
    int transaction_busy[FR_ROBOT_MAX_ARMS]; /**< 逐臂诊断事务忙标志，0/1。 */
    double inverter_temperature_c[FR_ROBOT_MAX_ARMS][7]; /**< 0x2020:02逆变器温度，摄氏度；按固定原始UINT16值/256.0换算，保留小数；无效时0，须检查有效位。 */
    double motor_temperature_c[FR_ROBOT_MAX_ARMS][7]; /**< 0x2020:03电机温度，摄氏度；按固定原始UINT16值/256.0换算，保留小数；无效时0，须检查有效位。 */
    uint32_t inverter_temperature_valid[FR_ROBOT_MAX_ARMS][7]; /**< 逐轴逆变器温度读取有效位0/1；读取失败或尚未读取为0。 */
    uint32_t motor_temperature_valid[FR_ROBOT_MAX_ARMS][7]; /**< 逐轴电机温度读取有效位0/1；读取失败或尚未读取为0。 */
} fr_joint_diagnostics;

typedef enum {
    FR_DIAGNOSTICS_DRIVE_ERRORS = 0,
    FR_DIAGNOSTICS_TEMPERATURES = 1,
    FR_DIAGNOSTICS_ALL = 2
} fr_diagnostics_kind;

typedef void (*fr_diagnostics_callback)(const fr_result* result,
                                        const fr_joint_diagnostics* snapshot, void* user);

/**
 * @brief 请求一次诊断刷新，在 SDK 后台交付完成结果及其绑定快照。
 * @param[in] robot 控制实例句柄，非 NULL。
 * @param[in] arm 配置逻辑臂号，不支持广播。
 * @param[in] kind 刷新范围，取 fr_diagnostics_kind 中的值。
 * @param[in] tag 用户关联号，允许重复，原样进入结果。
 * @param[in] callback 非 NULL；实际受理后调用一次，不重发到普通结果回调。失败或取消时快照为 NULL；两个回调参数指针仅在回调内有效。
 * @param[in] user 透传给 callback；回调可能早于本函数返回，须保持有效直至回调结束或实例停止。
 * @param[out] accepted 可为 NULL；接收提交回执，不是完成结果。
 * @return FR_OK 表示受理，运行层拒绝时返回错误，FR_ERR_TIMEOUT 不保证未执行；随后实际受理仍会回调。
 */
FR_API int fr_request_diagnostics(fr_robot* robot, int arm, int kind, uint64_t tag,
                                  fr_diagnostics_callback callback, void* user, fr_result* accepted);

#ifndef FR_DRIVE_CLEAR_REPORT_DEFINED
#define FR_DRIVE_CLEAR_REPORT_DEFINED
/** @brief 一次清错请求的逐轴报告，数组按 j1..j7 排列。 */
typedef struct {
    uint64_t cycle; /**< 运行层产生终态的控制拍号。 */
    uint16_t before_code[7]; /**< 前读的 0x603F:00 原始错误码，仅 before_valid 为 1 时有效。 */
    uint16_t after_code[7]; /**< 复读的 0x603F:00 原始错误码，仅 after_valid 为 1 时有效。 */
    uint32_t before_valid[7]; /**< 1 表示前读成功，0 表示未读或读取失败。 */
    uint32_t after_valid[7]; /**< 1 表示复读成功，0 表示未读或读取失败。 */
    uint32_t write_attempted[7]; /**< 1 表示尝试将清错控制字写入 PDO，0 表示未尝试。 */
    uint32_t write_staged[7]; /**< 1 表示已写入待发送的 PDO，不表示驱动已清错。 */
    uint32_t fault_after[7]; /**< 1 表示终态时驱动报告 Fault，仅 fault_after_valid 为 1 时有效。 */
    uint32_t fault_after_valid[7]; /**< 1 表示终态时驱动状态可信，0 表示未知。 */
} fr_drive_clear_report;
#endif /* FR_DRIVE_CLEAR_REPORT_DEFINED */

typedef void (*fr_drive_clear_callback)(const fr_result* result,
                                        const fr_drive_clear_report* report, void* user);

/**
 * @brief 请求指定臂执行一次前读、清错及复读；仅在安全条件满足时受理，不重试、不自动使能。
 * @param[in] robot 控制实例句柄，非 NULL。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不支持广播，两臂需分别请求。
 * @param[in] tag 用户关联号，允许 0 和重复值，原样进入结果。
 * @param[in] callback 非 NULL；实际受理后由 SDK 后台调用一次，不重发到普通结果回调。失败或取消也可带部分报告；运行层退出且未交付终态时报告为 NULL。结果和报告指针仅在回调期间有效。
 * @param[in] user 透传给 callback，可为 NULL；回调可能早于本函数返回，须保持有效直至回调结束或实例停止。
 * @param[out] accepted 可为 NULL；接收受理回执，不是清错结果。入口校验失败时不保证写入。
 * @return FR_OK 表示受理；负值为 fr_status 错误码。FR_ERR_TIMEOUT 不保证未执行，随后实际受理仍会回调。
 */
FR_API int fr_clear_drive_errors(fr_robot* robot, int arm, uint64_t tag,
                                 fr_drive_clear_callback callback, void* user, fr_result* accepted);

/**
 * @brief 读取当拍摩擦估计与最近一次按需SDO诊断缓存；本函数不触发SDO。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in,out] out 输出结构，非 NULL；调用前填 struct_size=sizeof(*out)、struct_version=FR_JOINT_DIAGNOSTICS_VERSION；分别检查摩擦、错误码和两路温度的有效位；不兼容旧版诊断结构。
 * @return FR_OK 表示输出成功；无数据返回 FR_ERR_NOT_READY，参数无效返回 FR_ERR_INVALID。
 */
FR_API int fr_get_joint_diagnostics(fr_robot* robot, fr_joint_diagnostics* out);

/**
 * @brief 异步刷新七轴 0x603F:00 错误码；仅读取，不执行故障复位。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 * @note 与模式切换邮箱互斥；disable 可取消。受理时清有效位，成功零码与未读取通过 valid 区分。
 */
FR_API int fr_refresh_drive_errors(fr_robot* robot, int arm, uint64_t tag, fr_result* accepted);

/**
 * @brief 异步按需读取指定臂七轴两路温度（2020:02/03）一次；不启用周期查询、不使能。
 * @param[in] robot 有效控制实例，非NULL。
 * @param[in] arm 逻辑臂号0或1，不是从站地址，不支持广播。
 * @param[in] tag 请求关联号，0..UINT64_MAX；调用方负责关联结果。
 * @param[out] accepted 受理结果，可NULL；不表示读取完成。
 * @return FR_OK表示受理，参数非法或事务忙时返回负错误码；按tag回调报告完成/失败。
 * @note getter只读缓存。切模优先，已发请求收尾后让路；单请求1秒，失败项无效但不下使能，disable/quit可取消。
 */
FR_API int fr_refresh_temperatures(fr_robot* robot, int arm, uint64_t tag, fr_result* accepted);

/**
 * @brief 异步按需读取指定臂七轴两路温度及603F错误码一次；不启用周期查询、不使能。
 * @param[in] robot 有效控制实例，非NULL。
 * @param[in] arm 逻辑臂号0或1，不是从站地址，不支持广播。
 * @param[in] tag 请求关联号，0..UINT64_MAX；调用方负责关联结果。
 * @param[out] accepted 受理结果，可NULL；不表示读取完成。
 * @return FR_OK表示受理，参数非法或事务忙时返回负错误码；按tag回调报告完成/失败。
 * @note getter只读缓存。切模优先，已发请求收尾后让路；单请求1秒，失败项无效但不下使能，disable/quit可取消。
 */
FR_API int fr_refresh_joint_diagnostics(fr_robot* robot, int arm, uint64_t tag, fr_result* accepted);


/**
 * @brief 启动CST笛卡尔导纳。
 * @param[in] robot 有效控制实例，非NULL。
 * @param[in] arm 配置逻辑臂号0或1，不是站址。
 * @param[in] params 本次参数覆盖，可为NULL使用本策略独立默认；字段单位/范围见结构，set_mask为0不覆盖。
 * @param[in] tag 请求关联号，0..UINT64_MAX，必须显式传入。
 * @param[out] accepted 受理结果，可为NULL；不表示执行完成。
 * @return FR_OK表示受理；负值为状态错误码。
 * @note 要求使能、autozero及CST限额；共用本臂CST控制律和kp/kd；max_lag不参与冻结。
 */
FR_API int fr_start_imp_cart6(fr_robot* robot, int arm, const fr_cart_params* params, uint64_t tag, fr_result* accepted);

/**
 * @brief 启动CST笛卡尔导纳及零空间让位。
 * @param[in] robot 有效控制实例，非NULL。
 * @param[in] arm 配置逻辑臂号0或1，不是站址。
 * @param[in] params 本次参数覆盖，可为NULL使用本策略独立默认；字段单位/范围见结构，set_mask为0不覆盖。
 * @param[in] tag 请求关联号，0..UINT64_MAX，必须显式传入。
 * @param[out] accepted 受理结果，可为NULL；不表示执行完成。
 * @return FR_OK表示受理；负值为状态错误码。
 * @note 要求使能、autozero及CST限额；共用本臂CST控制律和kp/kd；max_lag不参与冻结。
 */
FR_API int fr_start_imp_cart6null(fr_robot* robot, int arm, const fr_cart6null_params* params, uint64_t tag, fr_result* accepted);

/**
 * @brief 启动沿指定TCP轴的CST导纳。
 * @param[in] robot 有效控制实例，非NULL。
 * @param[in] arm 配置逻辑臂号0或1，不是站址。
 * @param[in] axis 启动命令TCP轴，0=x、1=y、2=z，无默认值。
 * @param[in] params 本次参数覆盖，可为NULL使用本策略独立默认；字段单位/范围见结构，set_mask为0不覆盖。
 * @param[in] tag 请求关联号，0..UINT64_MAX，必须显式传入。
 * @param[out] accepted 受理结果，可为NULL；不表示执行完成。
 * @return FR_OK表示受理；负值为状态错误码。
 * @note 要求使能、autozero及CST限额；共用本臂CST控制律和kp/kd；max_lag不参与冻结。
 */
FR_API int fr_start_imp_cart6_axis(fr_robot* robot, int arm, int axis, const fr_cart_params* params, uint64_t tag, fr_result* accepted);

/**
 * @brief 锁定当前命令位置启动CST吸引子。
 * @param[in] robot 有效控制实例，非NULL。
 * @param[in] arm 配置逻辑臂号0或1，不是站址。
 * @param[in] params 本次参数覆盖，可为NULL使用本策略独立默认；字段单位/范围见结构，set_mask为0不覆盖。
 * @param[in] tag 请求关联号，0..UINT64_MAX，必须显式传入。
 * @param[out] accepted 受理结果，可为NULL；不表示执行完成。
 * @return FR_OK表示受理；负值为状态错误码。
 * @note 要求使能、autozero及CST限额；共用本臂CST控制律和kp/kd；max_lag不参与冻结。
 */
FR_API int fr_start_imp_attractor6(fr_robot* robot, int arm, const fr_attractor_params* params, uint64_t tag, fr_result* accepted);

/**
 * @brief 启动CST吸引子及零空间让位。
 * @param[in] robot 有效控制实例，非NULL。
 * @param[in] arm 配置逻辑臂号0或1，不是站址。
 * @param[in] params 本次参数覆盖，可为NULL使用本策略独立默认；字段单位/范围见结构，set_mask为0不覆盖。
 * @param[in] tag 请求关联号，0..UINT64_MAX，必须显式传入。
 * @param[out] accepted 受理结果，可为NULL；不表示执行完成。
 * @return FR_OK表示受理；负值为状态错误码。
 * @note 要求使能、autozero及CST限额；共用本臂CST控制律和kp/kd；max_lag不参与冻结。
 */
FR_API int fr_start_imp_attractor6null(fr_robot* robot, int arm, const fr_attractor6null_params* params, uint64_t tag, fr_result* accepted);

/**
 * @brief 启动带局部轴偏移的CST吸引子。
 * @param[in] robot 有效控制实例，非NULL。
 * @param[in] arm 配置逻辑臂号0或1，不是站址。
 * @param[in] axis 启动命令TCP轴，0=x、1=y、2=z，无默认值。
 * @param[in] params 本次参数覆盖，可为NULL使用本策略独立默认；字段单位/范围见结构，set_mask为0不覆盖。
 * @param[in] tag 请求关联号，0..UINT64_MAX，必须显式传入。
 * @param[out] accepted 受理结果，可为NULL；不表示执行完成。
 * @return FR_OK表示受理；负值为状态错误码。
 * @note 要求使能、autozero及CST限额；共用本臂CST控制律和kp/kd；max_lag不参与冻结。
 */
FR_API int fr_start_imp_attractor6_axis(fr_robot* robot, int arm, int axis, const fr_attractor_axis_params* params, uint64_t tag, fr_result* accepted);

/**
 * @brief 启动CST零空间让位。
 * @param[in] robot 有效控制实例，非NULL。
 * @param[in] arm 配置逻辑臂号0或1，不是站址。
 * @param[in] params 本次参数覆盖，可为NULL使用本策略独立默认；字段单位/范围见结构，set_mask为0不覆盖。
 * @param[in] tag 请求关联号，0..UINT64_MAX，必须显式传入。
 * @param[out] accepted 受理结果，可为NULL；不表示执行完成。
 * @return FR_OK表示受理；负值为状态错误码。
 * @note 要求使能、autozero及CST限额；共用本臂CST控制律和kp/kd；max_lag不参与冻结。
 */
FR_API int fr_start_imp_null(fr_robot* robot, int arm, const fr_null_params* params, uint64_t tag, fr_result* accepted);

/**
 * @brief 提交CST吸引子局部相对目标。
 * @param[in] robot 有效控制实例，非NULL。
 * @param[in] arm 配置逻辑臂号0或1，不是站址。
 * @param[in] delta_position_m 局部位移xyz[m]，非NULL、长度3，有限，无默认值或固定可达范围；沿本次受理时命令TCP轴。
 * @param[in] tag 请求关联号，0..UINT64_MAX，必须显式传入。
 * @param[out] accepted 受理结果，可为NULL；不表示执行完成。
 * @return FR_OK表示受理；负值为状态错误码。
 * @note 要求使能、autozero及CST限额；共用本臂CST控制律和kp/kd；max_lag不参与冻结。
 */
FR_API int fr_move_imp_attractor6(fr_robot* robot, int arm, const double delta_position_m[3], uint64_t tag, fr_result* accepted);

/**
 * @brief 提交带零空间让位的CST吸引子局部相对目标。
 * @param[in] robot 有效控制实例，非NULL。
 * @param[in] arm 配置逻辑臂号0或1，不是站址。
 * @param[in] delta_position_m 局部位移xyz[m]，非NULL、长度3，有限，无默认值或固定可达范围；沿本次受理时命令TCP轴。
 * @param[in] tag 请求关联号，0..UINT64_MAX，必须显式传入。
 * @param[out] accepted 受理结果，可为NULL；不表示执行完成。
 * @return FR_OK表示受理；负值为状态错误码。
 * @note 要求使能、autozero及CST限额；共用本臂CST控制律和kp/kd；max_lag不参与冻结。
 */
FR_API int fr_move_imp_attractor6null(fr_robot* robot, int arm, const double delta_position_m[3], uint64_t tag, fr_result* accepted);



/** @brief 仅用于力位混合启动接口，选择逻辑臂0和1。 */
#define FR_HYBRID_BOTH_ARMS (-1)

/**
 * @brief 启动CSP位置执行的力位混合控制：锁定工具XY速度运动，Z压力带导纳。
 * @param[in] robot 有效控制句柄，非NULL。
 * @param[in] arm 逻辑臂号0或1（不是站址），或FR_HYBRID_BOTH_ARMS选择双臂。
 * @param[in] pressure_min_n 最小挤压力，N，有限且>=0；必须显式传入，且小于pressure_max_n。
 * @param[in] pressure_max_n 最大允许挤压力，N，有限且>pressure_min_n；必须显式传入，无软件固定上限，不是硬过力急停值。
 * @param[in] velocity_x_m_s 接管时锁定命令TCP的X向速度，m/s，有限，可正/负/0，无默认值；持续执行至停止。
 * @param[in] velocity_y_m_s 同坐标系Y向速度，m/s，有限，可正/负/0，无默认值；XY速度为两者合成，不分别归一化。
 * @param[in] xy_force_threshold_n 指向本次XY外力阈值，N，有限且>0；NULL使用各目标臂本策略当前默认值。仅XY外力幅值严格超过该值时检查对抗角。
 * @param[in] opposition_angle_deg 指向本次对抗角阈值，deg，有限且90<值<180；NULL使用各目标臂本策略当前默认值。
 * @param[in] tag 关联号，0..UINT64_MAX；双臂共用该号。
 * @param[out] accepted 可为NULL；单臂指向1项，双臂指向按臂0、1排列的2项。输出受理/拒绝，不表示已启动或完成。
 * @return 全部受理返回FR_OK；否则返回首个错误码；双臂逐臂结果见accepted。
 * @note 沿工具+Z压入，挤压力P=-Fz；低于下界前进，高于上界退让，带内Z目标速度0；XY不因接触丢失暂停。
 * @note gain默认0.005m/(s·N)，Z速度上限0.05m/s、加速度上限5m/s²、关节速度上限30deg/s，均为独立策略参数。
 * @note 两个可选值可独立省略；显式覆盖只用于本次。arm可为FR_HYBRID_BOTH_ARMS：全部参数先解析校验，失败则均不提交；通过后逐臂提交，非原子且不回滚。
 * @note XY合力未超过启动阈值时仅跳过夹角判断，不暂停XY运动或Z压力控制；XY指令速度为0时不判夹角。
 * @note 要求使能及autozero；超对抗角冻结参考，解除后继续；不是碰撞安全保证。
 */
FR_API int fr_start_hybrid_force_position(fr_robot* robot, int arm, double pressure_min_n, double pressure_max_n, double velocity_x_m_s, double velocity_y_m_s, const double* xy_force_threshold_n, const double* opposition_angle_deg, uint64_t tag, fr_result* accepted);

/**
 * @brief 启动CST阻抗执行的力位混合控制：锁定工具XY速度运动，Z压力带导纳。
 * @param[in] robot 有效控制句柄，非NULL。
 * @param[in] arm 逻辑臂号0或1（不是站址），或FR_HYBRID_BOTH_ARMS选择双臂。
 * @param[in] pressure_min_n 最小挤压力，N，有限且>=0；必须显式传入，且小于pressure_max_n。
 * @param[in] pressure_max_n 最大允许挤压力，N，有限且>pressure_min_n；必须显式传入，无软件固定上限，不是硬过力急停值。
 * @param[in] velocity_x_m_s 接管时锁定命令TCP的X向速度，m/s，有限，可正/负/0，无默认值；持续执行至停止。
 * @param[in] velocity_y_m_s 同坐标系Y向速度，m/s，有限，可正/负/0，无默认值；XY速度为两者合成，不分别归一化。
 * @param[in] xy_force_threshold_n 指向本次XY外力阈值，N，有限且>0；NULL使用各目标臂本策略当前默认值。仅XY外力幅值严格超过该值时检查对抗角。
 * @param[in] opposition_angle_deg 指向本次对抗角阈值，deg，有限且90<值<180；NULL使用各目标臂本策略当前默认值。
 * @param[in] tag 关联号，0..UINT64_MAX；双臂共用该号。
 * @param[out] accepted 可为NULL；单臂指向1项，双臂指向按臂0、1排列的2项。输出受理/拒绝，不表示已启动或完成。
 * @return 全部受理返回FR_OK；否则返回首个错误码；双臂逐臂结果见accepted。
 * @note 沿工具+Z压入，挤压力P=-Fz；低于下界前进，高于上界退让，带内Z目标速度0；XY不因接触丢失暂停。
 * @note gain默认0.005m/(s·N)，Z速度上限0.05m/s、加速度上限5m/s²、关节速度上限30deg/s，均为独立策略参数。
 * @note 两个可选值可独立省略；显式覆盖只用于本次。arm可为FR_HYBRID_BOTH_ARMS：全部参数先解析校验，失败则均不提交；通过后逐臂提交，非原子且不回滚。
 * @note XY合力未超过启动阈值时仅跳过夹角判断，不暂停XY运动或Z压力控制；XY指令速度为0时不判夹角。
 * @note 要求使能及autozero；超对抗角冻结参考，解除后继续；不是碰撞安全保证。使用每臂关节阻抗kp/kd，CST不接入max_lag冻结。
 */
FR_API int fr_start_imp_hybrid_force_position(fr_robot* robot, int arm, double pressure_min_n, double pressure_max_n, double velocity_x_m_s, double velocity_y_m_s, const double* xy_force_threshold_n, const double* opposition_angle_deg, uint64_t tag, fr_result* accepted);



#define FR_IMPEDANCE_STATE_VERSION 4u
/** @brief CST阻抗执行层同拍快照；调用前填写size/version。 */
typedef struct {
    uint32_t struct_size; /**< sizeof(本结构)。 */
    uint32_t struct_version; /**< FR_IMPEDANCE_STATE_VERSION，当前4；不兼容旧版结构。 */
    uint64_t cycle; /**< 控制拍号。 */
    int output_valid[2]; /**< 本拍完整CST力矩有效时1；CSP/未使能为0。 */
    double spring_nm[2][7]; /**< 关节弹簧回复力矩，Nm，不含重力/阻尼/摩擦。 */
    double damping_nm[2][7]; /**< 关节阻尼力矩，Nm。 */
    double friction_nm[2][7]; /**< 驱动侧摩擦前馈，Nm；不是估计链扣除的摩擦反馈。 */
    double gravity_nm[2][7]; /**< 当前实测构型动态重力前馈，Nm。 */
    double target_torque_nm[2][7]; /**< 弹簧、阻尼、摩擦和重力合成的未限幅目标力矩，Nm。 */
    double output_torque_nm[2][7]; /**< 最终限幅后的命令力矩，Nm，驱动编码前值；不是实测力矩。 */
    uint32_t saturated[2][7]; /**< 逐轴最终限幅标志0/1。 */
    double stiffness_matrix_nm_rad[2][7][7]; /**< 本拍完整关节刚度矩阵，Nm/rad；关节阻抗下为对角矩阵。 */
    double damping_matrix_nm_s_rad[2][7][7]; /**< 本拍完整关节阻尼矩阵，Nm·s/rad；关节阻抗下为对角矩阵。 */
} fr_impedance_state;

/**
 * @brief 查询本拍关节刚度/阻尼矩阵及各力矩分量，不触发总线读取。
 * @param[in] robot 有效控制实例，非NULL。
 * @param[in,out] state 输出结构，非NULL；预填struct_size=sizeof(*state)、struct_version=FR_IMPEDANCE_STATE_VERSION。
 * @return FR_OK表示复制成功；未运行或无快照返回FR_ERR_NOT_READY，结构版本/指针无效返回FR_ERR_INVALID。
 */
FR_API int fr_get_impedance_state(fr_robot* robot, fr_impedance_state* state);

#ifdef __cplusplus
}
#endif
