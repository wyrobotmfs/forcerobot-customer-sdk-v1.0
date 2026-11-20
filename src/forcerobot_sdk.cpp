#include "forcerobot_sdk.h"

#include "force_robot.h"
#include "kinematics_sdk.h"

#include <cstring>
#include <new>

struct frs_robot {
    fr_robot* raw = nullptr;
    frs_result_callback user_result_cb = nullptr;
    void* user_result_ctx = nullptr;
    frs_state_callback user_state_cb = nullptr;
    void* user_state_ctx = nullptr;
    frs_drive_clear_callback user_clear_cb = nullptr;
    void* user_clear_ctx = nullptr;
};

struct frs_kin {
    fr_kin_model* model = nullptr;
};

struct frs_traj {
    fr_kin_trajectory* raw = nullptr;
};

namespace {

int map_status(int s) {
    return s; /* fr_status 与 frs_status 数值对齐 0..-9 */
}

void copy_result(const fr_result* in, frs_result* out) {
    if (!out) return;
    out->code = in->code;
    out->arm = in->arm;
    out->kind = in->kind;
    out->event_code = in->event_code;
    out->tag = in->tag;
    std::memcpy(out->strategy, in->strategy, sizeof(out->strategy));
    std::memcpy(out->message, in->message, sizeof(out->message));
}

fr_pose to_fr_pose(const frs_pose* p) {
    fr_pose o{};
    if (!p) return o;
    for (int i = 0; i < 3; ++i) {
        o.position_m[i] = p->position_m[i];
        o.rpy_deg[i] = p->rpy_deg[i];
    }
    return o;
}

void from_fr_pose(const fr_pose& in, frs_pose* out) {
    if (!out) return;
    for (int i = 0; i < 3; ++i) {
        out->position_m[i] = in.position_m[i];
        out->rpy_deg[i] = in.rpy_deg[i];
    }
}

void fill_path_point(const frs_path_point& in, fr_path_point* out) {
    for (int i = 0; i < FRS_JOINTS; ++i) out->values[i] = in.values[i];
    out->speed = in.speed;
    out->rotation_speed = in.rotation_speed;
    out->accel = in.accel;
    out->dwell_s = in.dwell_s;
}

int convert_path(const frs_path_point* points, int count, fr_path_point* out) {
    if (!points || count < 1 || count > FRS_PATH_MAX) return FRS_ERR_INVALID;
    for (int i = 0; i < count; ++i) fill_path_point(points[i], &out[i]);
    return FRS_OK;
}

void fill_state(const fr_state& st, const fr_feedback* fb, frs_state* out) {
    std::memset(out, 0, sizeof(*out));
    out->bus_ready = st.bus_ready;
    out->cycle = st.cycle;
    for (int a = 0; a < FRS_ARMS; ++a) {
        out->enabled_joints[a] = st.enabled_joints[a];
        out->autozero_done[a] = st.autozero_done[a];
        out->mode[a] = st.mode[a];
        out->strategy_status[a] = st.strategy_status[a];
        for (int j = 0; j < FRS_JOINTS; ++j) {
            out->measured_q_deg[a][j] = st.measured_position_deg[a][j];
            out->command_q_deg[a][j] = st.command_position_deg[a][j];
            out->measured_qd_deg_s[a][j] = st.measured_velocity_deg_s[a][j];
            out->external_torque_nm[a][j] = st.external_torque_nm[a][j];
            if (fb) out->current_ma[a][j] = fb->current_ma[a][j];
        }
        for (int k = 0; k < 6; ++k) {
            out->tcp_pose[a][k] = st.tool_pose[a][k];
            out->force_6d[a][k] = st.equivalent_force_6d[a][k];
        }
    }
}

void on_result(const fr_result* result, void* user) {
    auto* self = static_cast<frs_robot*>(user);
    if (!self || !self->user_result_cb || !result) return;
    frs_result out{};
    copy_result(result, &out);
    self->user_result_cb(&out, self->user_result_ctx);
}

void on_state(const fr_state* state, void* user) {
    auto* self = static_cast<frs_robot*>(user);
    if (!self || !self->user_state_cb || !state) return;
    frs_state out{};
    fill_state(*state, nullptr, &out);
    self->user_state_cb(&out, self->user_state_ctx);
}

void fill_motion_options(double linear_speed_m_s, double rotation_speed_rad_s,
                         double linear_accel_m_s2, double rotation_accel_rad_s2,
                         fr_motion_options* out) {
    out->present = FR_MOTION_SPEED | FR_MOTION_ACCEL | FR_MOTION_ROTATION_SPEED |
                   FR_MOTION_ROTATION_ACCEL;
    out->speed = linear_speed_m_s;
    out->accel = linear_accel_m_s2;
    out->rotation_speed = rotation_speed_rad_s;
    out->rotation_accel = rotation_accel_rad_s2;
}

void on_drive_clear(const fr_result* result, const fr_drive_clear_report* report,
                    void* user) {
    auto* self = static_cast<frs_robot*>(user);
    if (!self || !self->user_clear_cb) return;
    frs_result out{};
    if (result) copy_result(result, &out);
    frs_drive_clear_report rep{};
    const frs_drive_clear_report* rep_ptr = nullptr;
    if (report) {
        rep.cycle = report->cycle;
        for (int j = 0; j < FRS_JOINTS; ++j) {
            rep.before_code[j] = report->before_code[j];
            rep.after_code[j] = report->after_code[j];
            rep.before_valid[j] = report->before_valid[j];
            rep.after_valid[j] = report->after_valid[j];
            rep.write_attempted[j] = report->write_attempted[j];
            rep.write_staged[j] = report->write_staged[j];
            rep.fault_after[j] = report->fault_after[j];
            rep.fault_after_valid[j] = report->fault_after_valid[j];
        }
        rep_ptr = &rep;
    }
    self->user_clear_cb(result ? &out : nullptr, rep_ptr, self->user_clear_ctx);
}

}  // namespace

extern "C" {

const char* frs_version(void) { return FRS_VERSION_STR; }

frs_robot* frs_create(const char* config_dir, const frs_options* options) {
    if (!config_dir || !config_dir[0]) return nullptr;
    fr_options opt{};
    opt.struct_version = FR_STATE_VERSION;
    if (options) {
        opt.use_soem = options->use_ethercat ? 1 : 0;
        opt.interface_name = options->interface_name;
        opt.log_every_cycles = options->log_every_cycles;
    } else {
        opt.use_soem = 1;
        opt.interface_name = "eth0";
        opt.log_every_cycles = 0;
    }
    fr_robot* raw = fr_create(config_dir, &opt);
    if (!raw) return nullptr;
    auto* self = new (std::nothrow) frs_robot();
    if (!self) {
        fr_destroy(raw);
        return nullptr;
    }
    self->raw = raw;
    return self;
}

int frs_start(frs_robot* robot) {
    if (!robot || !robot->raw) return FRS_ERR_INVALID;
    return map_status(fr_start(robot->raw));
}

int frs_stop(frs_robot* robot) {
    if (!robot || !robot->raw) return FRS_ERR_INVALID;
    return map_status(fr_stop(robot->raw));
}

int frs_is_running(frs_robot* robot) {
    if (!robot || !robot->raw) return 0;
    return fr_is_running(robot->raw);
}

void frs_destroy(frs_robot* robot) {
    if (!robot) return;
    if (robot->raw) {
        if (fr_is_running(robot->raw)) fr_stop(robot->raw);
        fr_destroy(robot->raw);
        robot->raw = nullptr;
    }
    delete robot;
}

int frs_set_result_callback(frs_robot* robot, frs_result_callback cb, void* user) {
    if (!robot || !robot->raw) return FRS_ERR_INVALID;
    robot->user_result_cb = cb;
    robot->user_result_ctx = user;
    if (!cb) return map_status(fr_set_result_callback(robot->raw, nullptr, nullptr));
    return map_status(fr_set_result_callback(robot->raw, on_result, robot));
}

int frs_enable(frs_robot* robot, int arm, uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw) return FRS_ERR_INVALID;
    fr_result acc{};
    int rc = fr_enable(robot->raw, arm, tag, accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_disable(frs_robot* robot, int arm, uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw) return FRS_ERR_INVALID;
    fr_result acc{};
    int rc = fr_disable(robot->raw, arm, tag, accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_hold(frs_robot* robot, int arm, uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw) return FRS_ERR_INVALID;
    fr_result acc{};
    int rc = fr_hold(robot->raw, arm, tag, accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_autozero(frs_robot* robot, int arm, uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw) return FRS_ERR_INVALID;
    fr_result acc{};
    int rc = fr_autozero(robot->raw, arm, tag, accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_set_tcp(frs_robot* robot, int arm, const frs_pose* offset,
                uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw || !offset) return FRS_ERR_INVALID;
    fr_pose p = to_fr_pose(offset);
    fr_result acc{};
    int rc = fr_set_tcp(robot->raw, arm, &p, tag, accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_clear_tcp(frs_robot* robot, int arm, uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw) return FRS_ERR_INVALID;
    fr_result acc{};
    int rc = fr_clear_tcp(robot->raw, arm, tag, accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_get_tcp(frs_robot* robot, int arm, frs_pose* offset) {
    if (!robot || !robot->raw || !offset) return FRS_ERR_INVALID;
    fr_pose p{};
    int rc = fr_get_tcp(robot->raw, arm, &p);
    if (rc == FR_OK) from_fr_pose(p, offset);
    return map_status(rc);
}

int frs_set_payload(frs_robot* robot, int arm, double mass_kg, const double com_m[3],
                    uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw || !com_m) return FRS_ERR_INVALID;
    fr_result acc{};
    int rc = fr_set_payload(robot->raw, arm, mass_kg, com_m, tag,
                            accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_get_payload(frs_robot* robot, int arm, double* mass_kg, double com_m[3]) {
    if (!robot || !robot->raw || !mass_kg || !com_m) return FRS_ERR_INVALID;
    return map_status(fr_get_payload(robot->raw, arm, mass_kg, com_m));
}

int frs_move_joint(frs_robot* robot, int arm, const double q_deg[7],
                   double speed_deg_s, double accel_deg_s2,
                   uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw || !q_deg) return FRS_ERR_INVALID;
    fr_result acc{};
    int rc = fr_move_joint(robot->raw, arm, q_deg, speed_deg_s, accel_deg_s2,
                           tag, accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_move_tcp(frs_robot* robot, int arm, const frs_pose* target,
                 double linear_speed_m_s, double rotation_speed_rad_s,
                 double linear_accel_m_s2, double rotation_accel_rad_s2,
                 uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw || !target) return FRS_ERR_INVALID;
    fr_pose p = to_fr_pose(target);
    fr_result acc{};
    int rc = fr_move_tcp(robot->raw, arm, &p, linear_speed_m_s, rotation_speed_rad_s,
                         linear_accel_m_s2, rotation_accel_rad_s2,
                         tag, accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_move_tool(frs_robot* robot, int arm,
                  const double delta_xyz_m[3], const double delta_rpy_deg[3],
                  double linear_speed_m_s, double rotation_speed_rad_s,
                  double linear_accel_m_s2, double rotation_accel_rad_s2,
                  uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw || !delta_xyz_m) return FRS_ERR_INVALID;
    fr_result acc{};
    int rc = fr_move_tool(robot->raw, arm, delta_xyz_m, delta_rpy_deg,
                          linear_speed_m_s, rotation_speed_rad_s,
                          linear_accel_m_s2, rotation_accel_rad_s2,
                          tag, accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_move_tcp_null(frs_robot* robot, int arm, const frs_pose* target,
                      double linear_speed_m_s, double rotation_speed_rad_s,
                      double linear_accel_m_s2, double rotation_accel_rad_s2,
                      uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw || !target) return FRS_ERR_INVALID;
    fr_motion_options opt{};
    fill_motion_options(linear_speed_m_s, rotation_speed_rad_s, linear_accel_m_s2,
                        rotation_accel_rad_s2, &opt);
    double xyz[3] = {target->position_m[0], target->position_m[1], target->position_m[2]};
    double rpy[3] = {target->rpy_deg[0], target->rpy_deg[1], target->rpy_deg[2]};
    fr_result acc{};
    int rc = fr_move_motion(robot->raw, arm, FR_MOTION_TCP_NULL, xyz, rpy, &opt, tag,
                            accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_move_tool_null(frs_robot* robot, int arm,
                       const double delta_xyz_m[3], const double delta_rpy_deg[3],
                       double linear_speed_m_s, double rotation_speed_rad_s,
                       double linear_accel_m_s2, double rotation_accel_rad_s2,
                       uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw || !delta_xyz_m) return FRS_ERR_INVALID;
    fr_motion_options opt{};
    fill_motion_options(linear_speed_m_s, rotation_speed_rad_s, linear_accel_m_s2,
                        rotation_accel_rad_s2, &opt);
    fr_result acc{};
    int rc = fr_move_motion(robot->raw, arm, FR_MOTION_TOOL_NULL, delta_xyz_m,
                            delta_rpy_deg, &opt, tag, accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_joint_path(frs_robot* robot, int arm,
                   const frs_path_point* points, int count,
                   uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw) return FRS_ERR_INVALID;
    fr_path_point buf[FRS_PATH_MAX];
    int c = convert_path(points, count, buf);
    if (c != FRS_OK) return c;
    fr_result acc{};
    int rc = fr_execute_joint_path(robot->raw, arm, buf, count, tag,
                                   accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_tcp_path(frs_robot* robot, int arm,
                 const frs_path_point* points, int count,
                 uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw) return FRS_ERR_INVALID;
    fr_path_point buf[FRS_PATH_MAX];
    int c = convert_path(points, count, buf);
    if (c != FRS_OK) return c;
    fr_result acc{};
    int rc = fr_execute_tcp_path(robot->raw, arm, buf, count, tag,
                                 accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_joint_path_blend(frs_robot* robot, int arm,
                         const frs_path_point* points, int count,
                         uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw) return FRS_ERR_INVALID;
    fr_path_point buf[FRS_PATH_MAX];
    double blend[FRS_PATH_MAX];
    int c = convert_path(points, count, buf);
    if (c != FRS_OK) return c;
    for (int i = 0; i < count; ++i) blend[i] = points[i].blend;
    fr_result acc{};
    int rc = fr_execute_joint_path_blend(robot->raw, arm, buf, blend, count, tag,
                                         accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_tcp_path_blend(frs_robot* robot, int arm,
                       const frs_path_point* points, int count,
                       uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw) return FRS_ERR_INVALID;
    fr_path_point buf[FRS_PATH_MAX];
    double blend[FRS_PATH_MAX];
    int c = convert_path(points, count, buf);
    if (c != FRS_OK) return c;
    for (int i = 0; i < count; ++i) blend[i] = points[i].blend;
    fr_result acc{};
    int rc = fr_execute_tcp_path_blend(robot->raw, arm, buf, blend, count, tag,
                                       accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_start_joint_stream(frs_robot* robot, int arm,
                           double speed_deg_s, double accel_deg_s2,
                           uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw) return FRS_ERR_INVALID;
    fr_result acc{};
    int rc = fr_start_joint_stream(robot->raw, arm, speed_deg_s, accel_deg_s2,
                                   tag, accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_start_tcp_stream(frs_robot* robot, int arm,
                         double linear_speed_m_s, double rotation_speed_rad_s,
                         uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw) return FRS_ERR_INVALID;
    fr_result acc{};
    int rc = fr_start_tcp_stream(robot->raw, arm, linear_speed_m_s,
                                 rotation_speed_rad_s, tag,
                                 accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_stream_update_joint(frs_robot* robot, int arm, const double q_deg[7]) {
    if (!robot || !robot->raw || !q_deg) return FRS_ERR_INVALID;
    return map_status(fr_stream_update_joint(robot->raw, arm, q_deg));
}

int frs_stream_update_tcp(frs_robot* robot, int arm, const frs_pose* target) {
    if (!robot || !robot->raw || !target) return FRS_ERR_INVALID;
    fr_pose p = to_fr_pose(target);
    return map_status(fr_stream_update_tcp(robot->raw, arm, &p));
}

int frs_finish_stream(frs_robot* robot, int arm, uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw) return FRS_ERR_INVALID;
    fr_result acc{};
    int rc = fr_finish_stream(robot->raw, arm, tag, accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_start_drag(frs_robot* robot, int arm, uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw) return FRS_ERR_INVALID;
    fr_result acc{};
    int rc = fr_start_drag(robot->raw, arm, nullptr, tag, accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_start_spring(frs_robot* robot, int arm, uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw) return FRS_ERR_INVALID;
    fr_result acc{};
    int rc = fr_start_spring(robot->raw, arm, nullptr, tag, accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_move_impedance_joint(frs_robot* robot, int arm, const double q_deg[7],
                             double speed_deg_s, uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw || !q_deg) return FRS_ERR_INVALID;
    fr_result acc{};
    int rc = fr_move_impedance_joint(robot->raw, arm, q_deg, speed_deg_s, tag,
                                     accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_move_impedance_tcp(frs_robot* robot, int arm, const frs_pose* target,
                           double linear_speed_m_s, double rotation_speed_rad_s,
                           uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw || !target) return FRS_ERR_INVALID;
    fr_pose p = to_fr_pose(target);
    fr_result acc{};
    int rc = fr_move_impedance_tcp(robot->raw, arm, &p, linear_speed_m_s,
                                   rotation_speed_rad_s, tag,
                                   accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_set_cst_gains(frs_robot* robot, int arm,
                      const double kp[7], const double kd[7],
                      uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw || !kp || !kd) return FRS_ERR_INVALID;
    fr_result acc{};
    int rc = fr_set_cst_gains(robot->raw, arm, kp, kd, tag,
                              accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_get_cst_gains(frs_robot* robot, int arm, double kp[7], double kd[7]) {
    if (!robot || !robot->raw || !kp || !kd) return FRS_ERR_INVALID;
    return map_status(fr_get_cst_gains(robot->raw, arm, kp, kd));
}

int frs_get_state(frs_robot* robot, frs_state* state) {
    if (!robot || !robot->raw || !state) return FRS_ERR_INVALID;
    fr_state st{};
    fr_feedback fb{};
    fb.struct_size = sizeof(fb);
    fb.struct_version = FR_FEEDBACK_VERSION;
    int rc = fr_get_state_with_feedback(robot->raw, &st, &fb);
    if (rc != FR_OK) {
        rc = fr_get_state(robot->raw, &st);
        if (rc != FR_OK) return map_status(rc);
        fill_state(st, nullptr, state);
        return FRS_OK;
    }
    fill_state(st, &fb, state);
    return FRS_OK;
}

int frs_subscribe_state(frs_robot* robot, int every_n_cycles,
                        frs_state_callback cb, void* user) {
    if (!robot || !robot->raw || !cb || every_n_cycles < 1) return FRS_ERR_INVALID;
    robot->user_state_cb = cb;
    robot->user_state_ctx = user;
    return map_status(fr_subscribe_state(robot->raw, every_n_cycles, on_state, robot));
}

int frs_unsubscribe_state(frs_robot* robot) {
    if (!robot || !robot->raw) return FRS_ERR_INVALID;
    robot->user_state_cb = nullptr;
    robot->user_state_ctx = nullptr;
    return map_status(fr_unsubscribe_state(robot->raw));
}

int frs_refresh_drive_errors(frs_robot* robot, int arm,
                             uint64_t tag, frs_result* accepted) {
    if (!robot || !robot->raw) return FRS_ERR_INVALID;
    fr_result acc{};
    int rc = fr_refresh_drive_errors(robot->raw, arm, tag, accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_get_drive_error_codes(frs_robot* robot, int arm,
                              uint16_t out_codes[7], int out_valid[7]) {
    if (!robot || !robot->raw || !out_codes || !out_valid) return FRS_ERR_INVALID;
    if (arm < 0 || arm >= FRS_ARMS) return FRS_ERR_INVALID;
    fr_joint_diagnostics d{};
    d.struct_size = sizeof(d);
    d.struct_version = FR_JOINT_DIAGNOSTICS_VERSION;
    int rc = fr_get_joint_diagnostics(robot->raw, &d);
    if (rc != FR_OK) return map_status(rc);
    for (int j = 0; j < FRS_JOINTS; ++j) {
        out_codes[j] = static_cast<uint16_t>(d.drive_error_code[arm][j]);
        out_valid[j] = d.drive_error_valid[arm][j] ? 1 : 0;
    }
    return FRS_OK;
}

int frs_clear_drive_errors(frs_robot* robot, int arm, uint64_t tag,
                           frs_drive_clear_callback cb, void* user,
                           frs_result* accepted) {
    if (!robot || !robot->raw || !cb) return FRS_ERR_INVALID;
    robot->user_clear_cb = cb;
    robot->user_clear_ctx = user;
    fr_result acc{};
    int rc = fr_clear_drive_errors(robot->raw, arm, tag, on_drive_clear, robot,
                                   accepted ? &acc : nullptr);
    if (accepted) copy_result(&acc, accepted);
    return map_status(rc);
}

int frs_csv_start(frs_robot* robot, const char* path_or_null) {
    if (!robot || !robot->raw) return FRS_ERR_INVALID;
    return map_status(fr_csv_capture_start(robot->raw, path_or_null));
}

int frs_csv_stop(frs_robot* robot) {
    if (!robot || !robot->raw) return FRS_ERR_INVALID;
    return map_status(fr_csv_capture_stop(robot->raw));
}

int frs_csv_get_path(frs_robot* robot, char* buffer, size_t size) {
    if (!robot || !robot->raw || !buffer || size == 0) return FRS_ERR_INVALID;
    return map_status(fr_csv_capture_get_path(robot->raw, buffer, size));
}

/* -------------------- kinematics -------------------- */

frs_kin* frs_kin_create(const char* config_dir, int arm_index) {
    if (!config_dir || !config_dir[0]) return nullptr;
    fr_kin_model* model = nullptr;
    if (fr_kin_create(config_dir, arm_index, &model) != FR_KIN_OK || !model)
        return nullptr;
    auto* kin = new (std::nothrow) frs_kin();
    if (!kin) {
        fr_kin_destroy(model);
        return nullptr;
    }
    kin->model = model;
    return kin;
}

void frs_kin_destroy(frs_kin* kin) {
    if (!kin) return;
    if (kin->model) fr_kin_destroy(kin->model);
    delete kin;
}

int frs_kin_set_tcp(frs_kin* kin, const frs_pose* offset) {
    if (!kin || !kin->model || !offset) return FRS_ERR_INVALID;
    fr_kin_pose p{};
    for (int i = 0; i < 3; ++i) {
        p.position_m[i] = offset->position_m[i];
        p.rpy_deg[i] = offset->rpy_deg[i];
    }
    int rc = fr_kin_set_tcp(kin->model, &p);
    return rc == FR_KIN_OK ? FRS_OK : FRS_ERR_REJECTED;
}

int frs_kin_fk(frs_kin* kin, const double q_deg[7], frs_pose* out_pose) {
    if (!kin || !kin->model || !q_deg || !out_pose) return FRS_ERR_INVALID;
    fr_kin_pose p{};
    int rc = fr_kin_fk(kin->model, q_deg, &p);
    if (rc != FR_KIN_OK) return FRS_ERR_REJECTED;
    for (int i = 0; i < 3; ++i) {
        out_pose->position_m[i] = p.position_m[i];
        out_pose->rpy_deg[i] = p.rpy_deg[i];
    }
    return FRS_OK;
}

int frs_kin_ik(frs_kin* kin, const frs_pose* target,
               const double q_ref_deg[7], double out_q_deg[7]) {
    if (!kin || !kin->model || !target || !q_ref_deg || !out_q_deg) return FRS_ERR_INVALID;
    fr_kin_pose t{};
    for (int i = 0; i < 3; ++i) {
        t.position_m[i] = target->position_m[i];
        t.rpy_deg[i] = target->rpy_deg[i];
    }
    int rc = fr_kin_solve_ik_near(kin->model, &t, q_ref_deg, out_q_deg);
    return rc == FR_KIN_OK ? FRS_OK : FRS_ERR_REJECTED;
}

int frs_kin_measure_arm_angle(frs_kin* kin, const double q_deg[7], double* out_deg) {
    if (!kin || !kin->model || !q_deg || !out_deg) return FRS_ERR_INVALID;
    return fr_kin_measure_arm_angle(kin->model, q_deg, out_deg) == FR_KIN_OK
               ? FRS_OK
               : FRS_ERR_REJECTED;
}

int frs_kin_arm_angle_range(frs_kin* kin, const double q_ref_deg[7],
                            double* out_min_deg, double* out_max_deg) {
    if (!kin || !kin->model || !q_ref_deg || !out_min_deg || !out_max_deg)
        return FRS_ERR_INVALID;
    return fr_kin_get_arm_angle_range(kin->model, q_ref_deg, out_min_deg, out_max_deg) ==
                   FR_KIN_OK
               ? FRS_OK
               : FRS_ERR_REJECTED;
}

int frs_kin_solve_arm_angle(frs_kin* kin, const double q_ref_deg[7],
                            double angle_deg, double out_q_deg[7]) {
    if (!kin || !kin->model || !q_ref_deg || !out_q_deg) return FRS_ERR_INVALID;
    return fr_kin_solve_arm_angle(kin->model, q_ref_deg, angle_deg, out_q_deg) ==
                   FR_KIN_OK
               ? FRS_OK
               : FRS_ERR_REJECTED;
}

int frs_kin_plan_line(frs_kin* kin, const double q_start_deg[7],
                      const frs_pose* target,
                      double linear_speed_m_s, double rotation_speed_rad_s,
                      double linear_accel_m_s2, double dwell_s,
                      double sample_period_s, size_t max_samples,
                      frs_traj** out_traj) {
    if (!kin || !kin->model || !q_start_deg || !target || !out_traj) return FRS_ERR_INVALID;
    *out_traj = nullptr;
    fr_kin_tcp_point pt{};
    for (int i = 0; i < 3; ++i) {
        pt.target.position_m[i] = target->position_m[i];
        pt.target.rpy_deg[i] = target->rpy_deg[i];
    }
    pt.linear_speed_m_s = linear_speed_m_s;
    pt.rotation_speed_rad_s = rotation_speed_rad_s;
    pt.linear_acceleration_m_s2 = linear_accel_m_s2;
    pt.dwell_s = dwell_s;
    pt.blend_radius_m = 0.0;
    fr_kin_trajectory* raw = nullptr;
    int rc = fr_kin_plan_line(kin->model, q_start_deg, &pt, sample_period_s,
                              max_samples, &raw);
    if (rc != FR_KIN_OK || !raw) {
        if (rc == FR_KIN_SAMPLE_LIMIT) return FRS_ERR_REJECTED;
        if (rc == FR_KIN_INVALID) return FRS_ERR_INVALID;
        return FRS_ERR_REJECTED;
    }
    auto* t = new (std::nothrow) frs_traj();
    if (!t) {
        fr_kin_trajectory_destroy(raw);
        return FRS_ERR_INTERNAL;
    }
    t->raw = raw;
    *out_traj = t;
    return FRS_OK;
}

int frs_kin_plan_tcp_path(frs_kin* kin, const double q_start_deg[7],
                          const frs_path_point* points, size_t count,
                          double sample_period_s, size_t max_samples,
                          frs_traj** out_traj) {
    if (!kin || !kin->model || !q_start_deg || !points || !out_traj) return FRS_ERR_INVALID;
    if (count < 1 || count > FRS_PATH_MAX) return FRS_ERR_INVALID;
    *out_traj = nullptr;
    fr_kin_tcp_point pts[FRS_PATH_MAX];
    for (size_t i = 0; i < count; ++i) {
        pts[i].target.position_m[0] = points[i].values[0];
        pts[i].target.position_m[1] = points[i].values[1];
        pts[i].target.position_m[2] = points[i].values[2];
        pts[i].target.rpy_deg[0] = points[i].values[3];
        pts[i].target.rpy_deg[1] = points[i].values[4];
        pts[i].target.rpy_deg[2] = points[i].values[5];
        pts[i].linear_speed_m_s = points[i].speed;
        pts[i].rotation_speed_rad_s = points[i].rotation_speed;
        pts[i].linear_acceleration_m_s2 = points[i].accel;
        pts[i].dwell_s = points[i].dwell_s;
        pts[i].blend_radius_m = points[i].blend;
    }
    fr_kin_trajectory* raw = nullptr;
    int rc = fr_kin_plan_tcp_path(kin->model, q_start_deg, pts, count,
                                  sample_period_s, max_samples, &raw);
    if (rc != FR_KIN_OK || !raw) return FRS_ERR_REJECTED;
    auto* t = new (std::nothrow) frs_traj();
    if (!t) {
        fr_kin_trajectory_destroy(raw);
        return FRS_ERR_INTERNAL;
    }
    t->raw = raw;
    *out_traj = t;
    return FRS_OK;
}

size_t frs_traj_count(const frs_traj* traj) {
    if (!traj || !traj->raw) return 0;
    return fr_kin_trajectory_sample_count(traj->raw);
}

int frs_traj_copy(const frs_traj* traj, size_t first, size_t count,
                  frs_traj_sample* out, size_t capacity) {
    if (!traj || !traj->raw || (!out && count > 0)) return FRS_ERR_INVALID;
    if (count > capacity) return FRS_ERR_INVALID;
    if (count == 0) return FRS_OK;
    // 使用临时缓冲（堆）以避免超大栈
    auto* tmp = new (std::nothrow) fr_kin_sample[count];
    if (!tmp) return FRS_ERR_INTERNAL;
    int rc = fr_kin_trajectory_copy(traj->raw, first, count, tmp, count);
    if (rc == FR_KIN_OK) {
        for (size_t i = 0; i < count; ++i) {
            out[i].time_s = tmp[i].time_s;
            for (int j = 0; j < FRS_JOINTS; ++j) out[i].joint_deg[j] = tmp[i].joint_deg[j];
        }
    }
    delete[] tmp;
    if (rc == FR_KIN_BUFFER_TOO_SMALL) return FRS_ERR_INVALID;
    return rc == FR_KIN_OK ? FRS_OK : FRS_ERR_INVALID;
}

void frs_traj_destroy(frs_traj* traj) {
    if (!traj) return;
    if (traj->raw) fr_kin_trajectory_destroy(traj->raw);
    delete traj;
}

}  // extern "C"
