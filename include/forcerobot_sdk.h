/**
 * @file forcerobot_sdk.h
 * @brief ForceRobot 对外 C SDK（双臂 7 轴，基于可交付能力封装）
 *
 * 目标平台：Linux aarch64（glibc ≥ 2.33）
 * 客户只需包含本头文件并链接 libforcerobot_sdk。
 *
 * 单位约定：
 *   关节角 deg；关节速度 deg/s；关节加速度 deg/s²
 *   位置 m；姿态 RPY deg（Rz×Ry×Rx）
 *   姿态角速度 rad/s；姿态角加速度 rad/s²
 *   力矩 Nm；力 N；电流 mA
 *
 * 重要语义：
 *   - 运动接口返回 OK 仅表示受理，完成靠结果回调（tag）
 *   - ARRIVED = 指令参考完成，不等于实测到位
 *   - hold/disable/stop 不能替代硬件急停
 *   - 双臂可分别控制，不保证原子同步启动
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#  ifdef FORCEROBOT_SDK_BUILD
#    define FRS_API __declspec(dllexport)
#  else
#    define FRS_API __declspec(dllimport)
#  endif
#else
#  define FRS_API __attribute__((visibility("default")))
#endif

#define FRS_VERSION_STR "1.0.1"
#define FRS_ARMS 2
#define FRS_JOINTS 7
#define FRS_PATH_MAX 32

typedef struct frs_robot frs_robot;
typedef struct frs_kin frs_kin;
typedef struct frs_traj frs_traj;

typedef enum {
    FRS_OK = 0,
    FRS_ERR_INVALID = -1,
    FRS_ERR_NOT_READY = -2,
    FRS_ERR_BUSY = -3,
    FRS_ERR_NOT_ENABLED = -4,
    FRS_ERR_NOT_FOUND = -5,
    FRS_ERR_TIMEOUT = -6,
    FRS_ERR_HARDWARE = -7,
    FRS_ERR_REJECTED = -8,
    FRS_ERR_ABORTED = -9,
    FRS_ERR_INTERNAL = -100
} frs_status;

typedef enum {
    FRS_RESULT_ACCEPTED = 0,
    FRS_RESULT_REJECTED = 1,
    FRS_RESULT_ARRIVED = 2,       /**< 指令参考完成，非实测到位 */
    FRS_RESULT_FAILED = 3,
    FRS_RESULT_SUPERSEDED = 4,
    FRS_RESULT_MODE_ERROR = 5,
    FRS_RESULT_STRATEGY_STARTED = 6
} frs_result_kind;

typedef struct {
    int code;
    int arm;              /**< 0/1；系统事件可能为 -1 */
    int kind;             /**< frs_result_kind */
    int event_code;
    uint64_t tag;
    char strategy[16];
    char message[256];
} frs_result;

typedef void (*frs_result_callback)(const frs_result* result, void* user);

typedef struct {
    double position_m[3]; /**< x y z，m */
    double rpy_deg[3];    /**< roll pitch yaw，deg */
} frs_pose;

typedef struct {
    int use_ethercat;           /**< 0=FakeBus；1=EtherCAT */
    const char* interface_name; /**< 网口名；FakeBus 可 NULL */
    int log_every_cycles;       /**< >0 写周期日志；0 关闭 */
} frs_options;

/** 精简状态（每臂） */
typedef struct {
    int bus_ready;
    uint64_t cycle;
    int enabled_joints[FRS_ARMS];
    int autozero_done[FRS_ARMS];
    int mode[FRS_ARMS];
    int strategy_status[FRS_ARMS];
    double measured_q_deg[FRS_ARMS][FRS_JOINTS];
    double command_q_deg[FRS_ARMS][FRS_JOINTS];
    double measured_qd_deg_s[FRS_ARMS][FRS_JOINTS];
    double tcp_pose[FRS_ARMS][6]; /**< xyz[m] + rpy[deg] */
    double external_torque_nm[FRS_ARMS][FRS_JOINTS];
    double force_6d[FRS_ARMS][6]; /**< Fx Fy Fz Mx My Mz */
    int32_t current_ma[FRS_ARMS][FRS_JOINTS];
} frs_state;

typedef void (*frs_state_callback)(const frs_state* state, void* user);

typedef struct {
    double values[FRS_JOINTS]; /**< joint: q[7] deg；tcp: xyz+rpy，第7项忽略 */
    double speed;              /**< joint: deg/s；tcp: m/s */
    double rotation_speed;     /**< tcp: rad/s；joint 忽略 */
    double accel;              /**< joint: deg/s²；tcp: m/s² */
    double dwell_s;            /**< 到参考点后停留，s */
    double blend;              /**< joint: transition_range_deg；tcp: blend_radius_m；末点必须 0 */
} frs_path_point;

typedef struct {
    double time_s;
    double joint_deg[FRS_JOINTS];
} frs_traj_sample;

/* ======================== 版本 / 生命周期 ======================== */

FRS_API const char* frs_version(void);

/**
 * @brief 创建控制实例（不启动总线）。
 * @param config_dir 含 devices/mdh/mass/parameter.json 的目录
 */
FRS_API frs_robot* frs_create(const char* config_dir, const frs_options* options);
FRS_API int frs_start(frs_robot* robot);
FRS_API int frs_stop(frs_robot* robot);
FRS_API int frs_is_running(frs_robot* robot);
FRS_API void frs_destroy(frs_robot* robot);

FRS_API int frs_set_result_callback(frs_robot* robot, frs_result_callback cb, void* user);

/* ======================== 使能 / 标定 ======================== */

FRS_API int frs_enable(frs_robot* robot, int arm, uint64_t tag, frs_result* accepted);
FRS_API int frs_disable(frs_robot* robot, int arm, uint64_t tag, frs_result* accepted);
FRS_API int frs_hold(frs_robot* robot, int arm, uint64_t tag, frs_result* accepted);
FRS_API int frs_autozero(frs_robot* robot, int arm, uint64_t tag, frs_result* accepted);

/* ======================== 工具 / 负载 ======================== */

FRS_API int frs_set_tcp(frs_robot* robot, int arm, const frs_pose* offset,
                        uint64_t tag, frs_result* accepted);
FRS_API int frs_clear_tcp(frs_robot* robot, int arm, uint64_t tag, frs_result* accepted);
FRS_API int frs_get_tcp(frs_robot* robot, int arm, frs_pose* offset);
FRS_API int frs_set_payload(frs_robot* robot, int arm, double mass_kg, const double com_m[3],
                            uint64_t tag, frs_result* accepted);
FRS_API int frs_get_payload(frs_robot* robot, int arm, double* mass_kg, double com_m[3]);

/* ======================== 位置运动（CSP） ======================== */

FRS_API int frs_move_joint(frs_robot* robot, int arm, const double q_deg[7],
                           double speed_deg_s, double accel_deg_s2,
                           uint64_t tag, frs_result* accepted);

FRS_API int frs_move_tcp(frs_robot* robot, int arm, const frs_pose* target,
                         double linear_speed_m_s, double rotation_speed_rad_s,
                         double linear_accel_m_s2, double rotation_accel_rad_s2,
                         uint64_t tag, frs_result* accepted);

FRS_API int frs_move_tool(frs_robot* robot, int arm,
                          const double delta_xyz_m[3], const double delta_rpy_deg[3],
                          double linear_speed_m_s, double rotation_speed_rad_s,
                          double linear_accel_m_s2, double rotation_accel_rad_s2,
                          uint64_t tag, frs_result* accepted);

/**
 * CSP 基座系绝对 TCP 运动，并叠加手推摆肘（tcp_null）。
 * 参数语义与 frs_move_tcp 相同；未标定外力时仍可做位置运动，只不叠加摆肘。
 */
FRS_API int frs_move_tcp_null(frs_robot* robot, int arm, const frs_pose* target,
                              double linear_speed_m_s, double rotation_speed_rad_s,
                              double linear_accel_m_s2, double rotation_accel_rad_s2,
                              uint64_t tag, frs_result* accepted);

/**
 * CSP 工具系相对运动，并叠加手推摆肘（tool_null）。
 * 参数语义与 frs_move_tool 相同。
 */
FRS_API int frs_move_tool_null(frs_robot* robot, int arm,
                               const double delta_xyz_m[3], const double delta_rpy_deg[3],
                               double linear_speed_m_s, double rotation_speed_rad_s,
                               double linear_accel_m_s2, double rotation_accel_rad_s2,
                               uint64_t tag, frs_result* accepted);

/* ======================== 多点路径 ======================== */

/** 停顿式多点（关节） */
FRS_API int frs_joint_path(frs_robot* robot, int arm,
                           const frs_path_point* points, int count,
                           uint64_t tag, frs_result* accepted);

/** 停顿式多点（TCP） */
FRS_API int frs_tcp_path(frs_robot* robot, int arm,
                         const frs_path_point* points, int count,
                         uint64_t tag, frs_result* accepted);

/**
 * 连续过渡多点（关节）。points[i].blend = transition_range_deg；末点 blend 必须为 0。
 */
FRS_API int frs_joint_path_blend(frs_robot* robot, int arm,
                                 const frs_path_point* points, int count,
                                 uint64_t tag, frs_result* accepted);

/**
 * 连续过渡多点（TCP）。points[i].blend = blend_radius_m；末点 blend 必须为 0。
 */
FRS_API int frs_tcp_path_blend(frs_robot* robot, int arm,
                               const frs_path_point* points, int count,
                               uint64_t tag, frs_result* accepted);

/* ======================== 流式跟随 ======================== */

FRS_API int frs_start_joint_stream(frs_robot* robot, int arm,
                                   double speed_deg_s, double accel_deg_s2,
                                   uint64_t tag, frs_result* accepted);
FRS_API int frs_start_tcp_stream(frs_robot* robot, int arm,
                                 double linear_speed_m_s, double rotation_speed_rad_s,
                                 uint64_t tag, frs_result* accepted);
FRS_API int frs_stream_update_joint(frs_robot* robot, int arm, const double q_deg[7]);
FRS_API int frs_stream_update_tcp(frs_robot* robot, int arm, const frs_pose* target);
FRS_API int frs_finish_stream(frs_robot* robot, int arm, uint64_t tag, frs_result* accepted);

/* ======================== 力控 / 阻抗（CST，需 autozero） ======================== */

FRS_API int frs_start_drag(frs_robot* robot, int arm, uint64_t tag, frs_result* accepted);
FRS_API int frs_start_spring(frs_robot* robot, int arm, uint64_t tag, frs_result* accepted);

FRS_API int frs_move_impedance_joint(frs_robot* robot, int arm, const double q_deg[7],
                                     double speed_deg_s, uint64_t tag, frs_result* accepted);
FRS_API int frs_move_impedance_tcp(frs_robot* robot, int arm, const frs_pose* target,
                                   double linear_speed_m_s, double rotation_speed_rad_s,
                                   uint64_t tag, frs_result* accepted);

/** CST 增益：kp Nm/deg，kd Nm/(deg/s)，各 7 轴 */
FRS_API int frs_set_cst_gains(frs_robot* robot, int arm,
                              const double kp[7], const double kd[7],
                              uint64_t tag, frs_result* accepted);
FRS_API int frs_get_cst_gains(frs_robot* robot, int arm, double kp[7], double kd[7]);

/* ======================== 状态 / 订阅 / 诊断 / CSV ======================== */

FRS_API int frs_get_state(frs_robot* robot, frs_state* state);
FRS_API int frs_subscribe_state(frs_robot* robot, int every_n_cycles,
                                frs_state_callback cb, void* user);
FRS_API int frs_unsubscribe_state(frs_robot* robot);

FRS_API int frs_refresh_drive_errors(frs_robot* robot, int arm,
                                     uint64_t tag, frs_result* accepted);
/** 读缓存的 603F 错误码；先 refresh。out_codes[7]，out_valid[7] */
FRS_API int frs_get_drive_error_codes(frs_robot* robot, int arm,
                                      uint16_t out_codes[7], int out_valid[7]);

/** 一次清错请求的逐轴报告（j1..j7） */
typedef struct {
    uint64_t cycle;
    uint16_t before_code[7];
    uint16_t after_code[7];
    uint32_t before_valid[7];
    uint32_t after_valid[7];
    uint32_t write_attempted[7];
    uint32_t write_staged[7];
    uint32_t fault_after[7];
    uint32_t fault_after_valid[7];
} frs_drive_clear_report;

typedef void (*frs_drive_clear_callback)(const frs_result* result,
                                         const frs_drive_clear_report* report,
                                         void* user);

/**
 * 请求清错（仅目标臂已可信下使能且无冲突事务时受理）。
 * 结果走专用 callback，不进入普通 result_callback；report 指针仅回调期间有效。
 * 装入 PDO 不等于驱动已清；不能替代硬件急停。
 */
FRS_API int frs_clear_drive_errors(frs_robot* robot, int arm, uint64_t tag,
                                   frs_drive_clear_callback cb, void* user,
                                   frs_result* accepted);

FRS_API int frs_csv_start(frs_robot* robot, const char* path_or_null);
FRS_API int frs_csv_stop(frs_robot* robot);
FRS_API int frs_csv_get_path(frs_robot* robot, char* buffer, size_t size);

/* ======================== 离线运动学 ======================== */

FRS_API frs_kin* frs_kin_create(const char* config_dir, int arm_index);
FRS_API void frs_kin_destroy(frs_kin* kin);
FRS_API int frs_kin_set_tcp(frs_kin* kin, const frs_pose* offset);
FRS_API int frs_kin_fk(frs_kin* kin, const double q_deg[7], frs_pose* out_pose);
FRS_API int frs_kin_ik(frs_kin* kin, const frs_pose* target,
                       const double q_ref_deg[7], double out_q_deg[7]);
FRS_API int frs_kin_measure_arm_angle(frs_kin* kin, const double q_deg[7],
                                      double* out_deg);
FRS_API int frs_kin_arm_angle_range(frs_kin* kin, const double q_ref_deg[7],
                                    double* out_min_deg, double* out_max_deg);
FRS_API int frs_kin_solve_arm_angle(frs_kin* kin, const double q_ref_deg[7],
                                    double angle_deg, double out_q_deg[7]);

/**
 * 直线 MOVL 规划。point 使用 tcp 语义：values 忽略，请用单独参数；
 * 为简洁提供专用参数接口。
 */
FRS_API int frs_kin_plan_line(frs_kin* kin, const double q_start_deg[7],
                              const frs_pose* target,
                              double linear_speed_m_s, double rotation_speed_rad_s,
                              double linear_accel_m_s2, double dwell_s,
                              double sample_period_s, size_t max_samples,
                              frs_traj** out_traj);

FRS_API int frs_kin_plan_tcp_path(frs_kin* kin, const double q_start_deg[7],
                                  const frs_path_point* points, size_t count,
                                  double sample_period_s, size_t max_samples,
                                  frs_traj** out_traj);

FRS_API size_t frs_traj_count(const frs_traj* traj);
FRS_API int frs_traj_copy(const frs_traj* traj, size_t first, size_t count,
                          frs_traj_sample* out, size_t capacity);
FRS_API void frs_traj_destroy(frs_traj* traj);

#ifdef __cplusplus
}
#endif
