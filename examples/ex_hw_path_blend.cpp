/**
 * 连续多点 + 过渡（关节 path_blend）
 *
 * 推荐用法（相对当前姿态，只改 j6，最简单）：
 *   sudo ./ex_hw_path_blend ../config eth0 0 j6 10 20 30
 *   sudo ./ex_hw_path_blend ../config eth0 0 j6 10 20 30 8 150
 *
 * 交互：
 *   sudo ./ex_hw_path_blend ../config eth0 0
 *     d              默认：相对当前 j6 同向 +8/+16/+24
 *     j6 a b c       其余轴保持当前，只设三点 j6
 *     三行 7 角      完整自定义（会自动抬 accel 以过折返）
 *     q              退出
 *
 * 说明：路径从「当前角」接到第 1 点。若当前→pt0 与 pt0→pt1 反向，
 * 过渡约需 accel>=V^2/r；本示例会自动把 accel 提到够用。
 */
#include "hw_util.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

static const double kSoftMin[7] = {-180, -90, -180, 0, -180, -47, -47};
static const double kSoftMax[7] = {180, 90, 180, 130, 180, 47, 47};

static void clamp7(double q[7], int* clamped) {
    *clamped = 0;
    for (int j = 0; j < 7; ++j) {
        const double c = std::min(kSoftMax[j], std::max(kSoftMin[j], q[j]));
        if (c != q[j]) *clamped = 1;
        q[j] = c;
    }
}

static void apply_speed_accel_blend(frs_path_point pts[3], double speed, double accel,
                                    double blend01) {
    for (int i = 0; i < 3; ++i) {
        pts[i].speed = speed;
        pts[i].accel = accel;
        pts[i].dwell_s = 0;
        pts[i].rotation_speed = 0.1;
        pts[i].blend = (i == 2) ? 0.0 : blend01;
    }
}

/** 其余轴=当前，只改 j6 三个目标 */
static int fill_j6_only(const double base[7], double j6[3], frs_path_point pts[3],
                        double speed, double accel, double blend01) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 7; ++j) pts[i].values[j] = base[j];
        pts[i].values[5] = j6[i];
        int c = 0;
        clamp7(pts[i].values, &c);
        if (c) {
            char lab[32];
            std::snprintf(lab, sizeof(lab), "pt%d clamped:", i);
            frs_print_q(lab, pts[i].values);
        }
    }
    apply_speed_accel_blend(pts, speed, accel, blend01);
    return 1;
}

static void fill_default(const double base[7], frs_path_point pts[3]) {
    double j6[3];
    const double step = 8.0;
    j6[0] = base[5] + step;
    j6[1] = base[5] + 2.0 * step;
    j6[2] = base[5] + 3.0 * step;
    if (j6[2] > kSoftMax[5]) {
        j6[0] = base[5] - step;
        j6[1] = base[5] - 2.0 * step;
        j6[2] = base[5] - 3.0 * step;
    }
    fill_j6_only(base, j6, pts, 8.0, 50.0, 0.5);
}

/** 折返时自动抬高 accel，返回调整后的 accel */
static double auto_fix_accel(const double q_start[7], frs_path_point pts[3]) {
    double need = 0.0;
    const double* prev = q_start;
    for (int i = 0; i < 2; ++i) {
        int sign_prev = 0, sign_next = 0;
        for (int j = 0; j < 7; ++j) {
            const double d0 = pts[i].values[j] - prev[j];
            const double d1 = pts[i + 1].values[j] - pts[i].values[j];
            if (std::fabs(d0) > 1e-6) sign_prev += (d0 > 0) ? 1 : -1;
            if (std::fabs(d1) > 1e-6) sign_next += (d1 > 0) ? 1 : -1;
        }
        const double b = pts[i].blend;
        const double v = pts[i].speed;
        if (b > 1e-9 && sign_prev * sign_next < 0) {
            const double n = (v * v) / b;
            if (n > need) need = n;
        }
        prev = pts[i].values;
    }
    double accel = pts[0].accel;
    if (need > accel) {
        accel = need * 1.05;
        if (accel > 1000.0) accel = 1000.0;
        std::printf("auto: 检测到折返，accel %.1f -> %.1f (需约 V^2/r)\n",
                    pts[0].accel, accel);
        for (int i = 0; i < 3; ++i) pts[i].accel = accel;
    }
    return accel;
}

static int parse_point_line(const char* line, double q[7], double* blend) {
    char buf[512];
    std::snprintf(buf, sizeof(buf), "%s", line);
    char* tok = std::strtok(buf, " \t\r\n");
    for (int j = 0; j < 7; ++j) {
        if (!tok) return 0;
        q[j] = std::atof(tok);
        tok = std::strtok(nullptr, " \t\r\n");
    }
    *blend = 0.5;
    if (tok) *blend = std::atof(tok);
    return 1;
}

static int read_interactive(const double base[7], frs_path_point pts[3], double* speed,
                            double* accel) {
    frs_print_q("current q:", base);
    std::printf(
        "input:\n"
        "  d                 默认同向 j6\n"
        "  j6 a b c          只改第6轴三点（推荐）\n"
        "  三行: q0..q6 [blend]\n"
        "  q                 退出\n"
        "> ");
    std::fflush(stdout);

    char line[512] = {};
    if (!std::fgets(line, sizeof(line), stdin)) return -1;
    char* p = line;
    while (*p == ' ' || *p == '\t') ++p;
    if (*p == 'q' || *p == 'Q') return 0;
    if (*p == 'd' || *p == 'D' || *p == '\n' || *p == '\0') {
        fill_default(base, pts);
        *speed = pts[0].speed;
        *accel = pts[0].accel;
        return 1;
    }

    // j6 a b c
    if ((p[0] == 'j' || p[0] == 'J') && p[1] == '6') {
        double j6[3] = {};
        if (std::sscanf(p + 2, "%lf %lf %lf", &j6[0], &j6[1], &j6[2]) != 3) {
            std::fprintf(stderr, "usage: j6 a b c\n");
            return -1;
        }
        fill_j6_only(base, j6, pts, *speed > 0 ? *speed : 8.0,
                     *accel > 0 ? *accel : 50.0, 0.5);
        return 1;
    }

    double q[7], blends[3] = {0.5, 0.5, 0.0};
    if (!parse_point_line(p, q, &blends[0])) {
        std::fprintf(stderr, "bad point 0（也可用: j6 10 20 30）\n");
        return -1;
    }
    int clamped = 0;
    clamp7(q, &clamped);
    for (int j = 0; j < 7; ++j) pts[0].values[j] = q[j];

    for (int i = 1; i < 3; ++i) {
        std::printf("pt%d> ", i);
        std::fflush(stdout);
        if (!std::fgets(line, sizeof(line), stdin)) return -1;
        if (!parse_point_line(line, q, &blends[i])) {
            std::fprintf(stderr, "bad point %d\n", i);
            return -1;
        }
        clamp7(q, &clamped);
        for (int j = 0; j < 7; ++j) pts[i].values[j] = q[j];
    }
    blends[2] = 0.0;
    const double sp = *speed > 0 ? *speed : 8.0;
    const double ac = *accel > 0 ? *accel : 50.0;
    for (int i = 0; i < 3; ++i) {
        pts[i].speed = sp;
        pts[i].accel = ac;
        pts[i].dwell_s = 0;
        pts[i].rotation_speed = 0.1;
        pts[i].blend = blends[i];
    }
    return 1;
}

/** CLI: j6 a b c [speed accel] */
static int parse_cli_j6(int argc, char** argv, int arm_idx, const double base[7],
                        frs_path_point pts[3], double* speed, double* accel) {
    // argv: prog config ifname arm j6 a b c [speed accel]
    if (argc < arm_idx + 5) return 0;
    if (std::strcmp(argv[arm_idx + 1], "j6") != 0 &&
        std::strcmp(argv[arm_idx + 1], "J6") != 0)
        return 0;
    double j6[3] = {std::atof(argv[arm_idx + 2]), std::atof(argv[arm_idx + 3]),
                    std::atof(argv[arm_idx + 4])};
    *speed = 8.0;
    *accel = 50.0;
    if (argc > arm_idx + 5) *speed = std::atof(argv[arm_idx + 5]);
    if (argc > arm_idx + 6) *accel = std::atof(argv[arm_idx + 6]);
    if (!(*speed > 0) || !(*accel > 0)) return -1;
    fill_j6_only(base, j6, pts, *speed, *accel, 0.5);
    return 1;
}

static int parse_cli_full(int argc, char** argv, int start, frs_path_point pts[3],
                          double* speed, double* accel) {
    if (argc - start < 21) return 0;
    *speed = 8.0;
    *accel = 50.0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 7; ++j)
            pts[i].values[j] = std::atof(argv[start + i * 7 + j]);
        int c = 0;
        clamp7(pts[i].values, &c);
    }
    const int extra = start + 21;
    if (argc > extra) *speed = std::atof(argv[extra]);
    if (argc > extra + 1) *accel = std::atof(argv[extra + 1]);
    if (!(*speed > 0) || !(*accel > 0)) return -1;
    apply_speed_accel_blend(pts, *speed, *accel, 0.5);
    return 1;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr,
                     "usage:\n"
                     "  %s <config> <ifname> [arm]\n"
                     "  %s <config> <ifname> <arm> j6 <a> <b> <c> [speed accel]\n"
                     "  %s <config> <ifname> <arm> <21 angles> [speed accel]\n",
                     argv[0], argv[0], argv[0]);
        return 1;
    }
    const char* config = argv[1];
    const char* ifname = argv[2];
    int arm = (argc > 3) ? argv[3][0] - '0' : 0;
    if (arm < 0 || arm > 1) {
        std::fprintf(stderr, "arm must be 0/1\n");
        return 1;
    }
    const int require_bus = !frs_is_fake(ifname);

    frs_options opt{};
    frs_fill_options(&opt, ifname);
    FrsTagWait wait;
    frs_robot* robot = frs_create(config, &opt);
    if (!robot) {
        std::fprintf(stderr, "frs_create failed (clear lock?)\n");
        return 2;
    }
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

    double speed = 8.0, accel = 50.0;
    frs_path_point pts[3] = {};
    int have = 0;

    if (argc > 4) {
        have = parse_cli_j6(argc, argv, 3, st.measured_q_deg[arm], pts, &speed, &accel);
        if (have == 0)
            have = parse_cli_full(argc, argv, 4, pts, &speed, &accel);
        if (have < 0) {
            std::fprintf(stderr, "bad CLI args\n");
            frs_stop(robot);
            frs_destroy(robot);
            return 1;
        }
    }
    if (!have) {
        int r = read_interactive(st.measured_q_deg[arm], pts, &speed, &accel);
        if (r <= 0) {
            frs_stop(robot);
            frs_destroy(robot);
            return (r == 0) ? 0 : 1;
        }
    }

    accel = auto_fix_accel(st.measured_q_deg[arm], pts);
    speed = pts[0].speed;

    std::printf("from current -> 3 pts (j6: %.2f -> %.2f -> %.2f -> %.2f)\n",
                st.measured_q_deg[arm][5], pts[0].values[5], pts[1].values[5],
                pts[2].values[5]);
    for (int i = 0; i < 3; ++i) {
        char lab[32];
        std::snprintf(lab, sizeof(lab), "pt%d:", i);
        frs_print_q(lab, pts[i].values);
        std::printf("  speed=%.1f accel=%.1f blend=%.2f\n", pts[i].speed, pts[i].accel,
                    pts[i].blend);
    }

    if (!frs_confirm_y("enable + joint_path_blend? type y: ")) {
        frs_stop(robot);
        frs_destroy(robot);
        return 0;
    }
    if (frs_enable_arm(robot, arm) != FRS_OK) {
        frs_stop(robot);
        frs_destroy(robot);
        return 5;
    }

    const uint64_t tag = 210;
    frs_result acc{};
    int rc = frs_joint_path_blend(robot, arm, pts, 3, tag, &acc);
    std::printf("joint_path_blend rc=%d %s\n", rc, acc.message);
    if (rc == FRS_OK)
        std::printf("wait rc=%d\n", frs_wait_tag(&wait, tag, 60000));
    else
        std::fprintf(stderr, "仍失败时：减小 blend、加大点间距，或先 move 到靠近 pt0\n");

    frs_hold(robot, arm, 211, nullptr);
    frs_disable(robot, arm, 212, nullptr);
    frs_stop(robot);
    frs_destroy(robot);
    return 0;
}
