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

#define FR_STATE_VERSION 2u
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
    FR_MODE_ATTRACTOR3,
    FR_MODE_ATTRACTOR6,
    FR_MODE_DRAG,
    FR_MODE_SPRING,
    FR_MODE_CART3,
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
    FR_MODE_IMP_TCP_STREAM
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
    int      log_every_cycles; /**< 日志采样间隔，单位控制拍；>0 写 log/run.txt，0关闭；默认0。 */
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
    double   gravity_hold_nm[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴重力模型力矩，Nm。 */
    double   external_torque_nm[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴外力矩估计，Nm；需有效估计链及autozero。 */
    double   torque_drift_nm[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴标定测量侧零漂，Nm。 */
    double   equivalent_force_3d_n[FR_ROBOT_MAX_ARMS][3]; /**< 三维等效力估计[x,y,z]，N。 */
    double   equivalent_force_6d[FR_ROBOT_MAX_ARMS][6]; /**< 六维等效力/力矩估计[Fx,Fy,Fz,Mx,My,Mz]，N/Nm。 */
    double   tool_pose[FR_ROBOT_MAX_ARMS][6]; /**< 当前TCP位姿，公共坐标系x/y/z[m]+RPY[deg]。 */
    int32_t  position_counts[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴原始位置计数。 */
    int32_t  velocity_counts[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴0x606C原始速度计数。 */
    int32_t  measured_torque_lsb[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴解环绕后的原始整数力矩LSB，不是Nm。 */
    int32_t  written_target_torque_lsb[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴写入驱动的原始力矩LSB。 */
    uint32_t written_target_position_counts[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴写入驱动的目标位置计数。 */
    double   command_velocity_deg_s[FR_ROBOT_MAX_ARMS][FR_ROBOT_JOINTS]; /**< 逐轴命令位置差分速度，deg/s。 */
    int      strategy_flags[FR_ROBOT_MAX_ARMS]; /**< 逐臂FR_FLAG_*策略标志位或。 */
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
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[out] state 状态输出，非 NULL；提供一个 fr_state 对象，无须预填版本。
 * @param[in,out] feedback 反馈输出，非 NULL；调用前填写 struct_size=sizeof(*feedback)、struct_version=FR_FEEDBACK_VERSION；有效位只适用于相同 cycle。
 * @return FR_OK 表示输出成功；无数据返回 FR_ERR_NOT_READY，参数无效返回 FR_ERR_INVALID。
 */
FR_API int fr_get_state_with_feedback(fr_robot* robot, fr_state* state, fr_feedback* feedback);

#define FR_DRIVE_MODE_STATE_VERSION 1u
typedef enum { FR_DRIVE_MODE_UNKNOWN=0, FR_DRIVE_MODE_CSP=8, FR_DRIVE_MODE_CST=10 } fr_drive_mode;
typedef struct {
    uint32_t struct_size; /**< 调用前填 sizeof(本结构)。 */
    uint32_t struct_version; /**< 调用前填1。 */
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
 * @brief 设置三维力估计吸引子的绝对位置目标，保持姿态。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] position_m 绝对 TCP 位置数组，非 NULL、长度3，公共坐标系 x/y/z，单位 m；全部有限，无统一可达范围、无默认值。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_move_attractor3(fr_robot* robot, int arm,
                              const double position_m[3],
                              uint64_t tag, fr_result* accepted);
/**
 * @brief 设置六维力估计吸引子的绝对位置目标，保持姿态。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] position_m 绝对 TCP 位置数组，非 NULL、长度3，公共坐标系 x/y/z，单位 m；全部有限，无统一可达范围、无默认值。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_move_attractor6(fr_robot* robot, int arm,
                              const double position_m[3],
                              uint64_t tag, fr_result* accepted);
/**
 * @brief 设置六维吸引子位置目标，同时允许臂角零空间让位。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] position_m 绝对 TCP 位置数组，非 NULL、长度3，公共坐标系 x/y/z，单位 m；全部有限，无统一可达范围、无默认值。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
 */
FR_API int fr_move_attractor6null(fr_robot* robot, int arm,
                                  const double position_m[3],
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
 * @brief 启动三维力估计笛卡尔导纳，保持姿态，使用 CSP。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] params 本次启动的参数覆盖，可为 NULL（使用该策略当前默认值）；非 NULL 时仅 set_mask 指定字段生效，字段范围和单位见对应结构。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
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
 * @brief 以当前命令 TCP 位置为吸引目标启动三维吸引子。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in] arm 配置逻辑臂号，取 0 或 1；不表示 EtherCAT 站号，不支持 -1 广播。
 * @param[in] params 本次启动的参数覆盖，可为 NULL（使用该策略当前默认值）；非 NULL 时仅 set_mask 指定字段生效，字段范围和单位见对应结构。
 * @param[in] tag 请求关联号，范围 0..UINT64_MAX，必须显式传入；0 合法，SDK 不分配唯一号。
 * @param[out] accepted 受理结果输出，可为 NULL；非 NULL 时提供一个 result 对象；入口校验失败时不保证写入。
 * @return FR_OK 表示请求已受理，不表示执行完成；负值为 fr_status 错误码。执行结果通过结果回调按 tag 返回。
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

/**
 * @brief 复制最新控制状态；停止后仍可能返回最后一帧，不代表实时有效。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[out] state 状态输出，非 NULL；提供一个 fr_state 对象，无须预填版本。
 * @return FR_OK 表示输出成功；无数据返回 FR_ERR_NOT_READY，参数无效返回 FR_ERR_INVALID。
 */
FR_API int fr_get_state(fr_robot* robot, fr_state* state);

/**
 * @brief 复制最近若干状态，按时间升序排列；不消费历史，重复读取可重叠。
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
 * @brief 修改策略运行期默认参数；校验通过后生效，不写入 JSON。
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

typedef struct {
    uint32_t struct_size; /**< 调用前填 sizeof(本结构)。 */
    uint32_t struct_version; /**< 调用前填1。 */
    uint64_t timestamp_ns; /**< 快照时间戳，ns。 */
    uint64_t cycle; /**< 摩擦估计所属控制拍号。 */
    double modeled_friction_torque_nm[FR_ROBOT_MAX_ARMS][7]; /**< 模型实际扣除的摩擦力矩估计，Nm；不是fc参数或传感器实测。 */
    uint32_t friction_valid[FR_ROBOT_MAX_ARMS][7]; /**< 逐轴摩擦估计有效位，0/1；未使能/未autozero/输入失效时0。 */
    uint16_t drive_error_code[FR_ROBOT_MAX_ARMS][7]; /**< 逐轴0x603F:00缓存值，16bit；0可表示成功读到无错误。 */
    uint32_t drive_error_valid[FR_ROBOT_MAX_ARMS][7]; /**< 逐轴错误码有效位，0未读取或读失败，1读取成功。 */
    uint64_t drive_error_cycle[FR_ROBOT_MAX_ARMS][7]; /**< 逐轴错误码采样拍；不保证等于快照cycle。 */
    int transaction_busy[FR_ROBOT_MAX_ARMS]; /**< 逐臂诊断事务忙标志，0/1。 */
} fr_joint_diagnostics;
/**
 * @brief 读取当拍摩擦估计及按需缓存的驱动错误码；不触发 SDO。
 * @param[in] robot 控制实例句柄，非 NULL；由 create 返回，调用期间须保持有效。
 * @param[in,out] out 输出结构，非 NULL；调用前填 struct_size=sizeof(*out)、struct_version=1；通过 friction_valid 和 drive_error_valid 判断各轴数据是否有效。
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
#ifdef __cplusplus
}
#endif
