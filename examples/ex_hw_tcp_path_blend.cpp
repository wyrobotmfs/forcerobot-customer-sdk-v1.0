/**
 * 连续过渡多点 TCP（frs_tcp_path_blend）
 *
 *   sudo ./ex_hw_tcp_path_blend ../config eth0 0
 *   sudo ./ex_hw_tcp_path_blend ../config eth0 0 corner
 *   sudo ./ex_hw_tcp_path_blend ../config eth0 0 dz 0.01 0.02 0.03 0.002
 *
 * 要点：
 *   - blend>0 的点是「路过点」，不应满停；末点 blend 必须为 0，本来就会停。
 *   - TCP 过渡是空间相切圆弧。三点共线（纯 dz）几乎没有拐角，看不出圆弧，
 *     每段仍会加减速，肉眼像“顿一下”——这不是 stop path 的 dwell。
 *   - 默认用带拐角的路径，中间路过更明显。
 */
#include "hw_util.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

static void set_pt(frs_path_point* p, const double tcp[6], double dx, double dy,
                   double dz, double speed, double accel, double blend_m) {
    for (int k = 0; k < 6; ++k) p->values[k] = tcp[k];
    p->values[0] += dx;
    p->values[1] += dy;
    p->values[2] += dz;
    p->values[6] = 0;
    p->speed = speed;
    p->rotation_speed = 0.2;
    p->accel = accel;
    p->dwell_s = 0;  // 正 blend 点必须 0
    p->blend = blend_m;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr,
                     "usage:\n"
                     "  %s <config> <ifname> [arm]              # 默认拐角路径\n"
                     "  %s <config> <ifname> <arm> corner [blend_m]\n"
                     "  %s <config> <ifname> <arm> dz z0 z1 z2 [blend_m]\n",
                     argv[0], argv[0], argv[0]);
        return 1;
    }
    const char* config = argv[1];
    const char* ifname = argv[2];
    int arm = (argc > 3) ? argv[3][0] - '0' : 0;
    if (arm < 0 || arm > 1) return 1;
    const int require_bus = !frs_is_fake(ifname);

    enum { MODE_CORNER, MODE_DZ } mode = MODE_CORNER;
    double dz[3] = {0.01, 0.02, 0.03};
    double blend_m = 0.008;  // 8mm，配约 3cm 段长
    if (argc >= 5) {
        if (std::strcmp(argv[4], "dz") == 0 || std::strcmp(argv[4], "DZ") == 0) {
            if (argc < 8) {
                std::fprintf(stderr, "dz needs z0 z1 z2\n");
                return 1;
            }
            mode = MODE_DZ;
            dz[0] = std::atof(argv[5]);
            dz[1] = std::atof(argv[6]);
            dz[2] = std::atof(argv[7]);
            blend_m = (argc > 8) ? std::atof(argv[8]) : 0.002;
        } else if (std::strcmp(argv[4], "corner") == 0 ||
                   std::strcmp(argv[4], "CORNER") == 0) {
            mode = MODE_CORNER;
            if (argc > 5) blend_m = std::atof(argv[5]);
        }
    }

    frs_options opt{};
    frs_fill_options(&opt, ifname);
    FrsTagWait wait;
    frs_robot* robot = frs_create(config, &opt);
    if (!robot) {
        std::fprintf(stderr, "frs_create failed\n");
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

    const double* tcp = st.tcp_pose[arm];
    std::printf("current tcp xyz=(%.4f,%.4f,%.4f)\n", tcp[0], tcp[1], tcp[2]);

    frs_path_point pts[3] = {};
    const double speed = 0.05;
    const double accel = 0.5;
    if (mode == MODE_CORNER) {
        // 拐角：先 +Z，再 +X（路过），再 +Z（终点停）
        std::printf("mode=corner  relative: (0,0,0.03)->(0.03,0,0.03)->(0.03,0,0.06)\n");
        std::printf("blend_m=%.4f (pt0/pt1)，末点 0；中间应路过不停满\n", blend_m);
        set_pt(&pts[0], tcp, 0.00, 0.00, 0.03, speed, accel, blend_m);
        set_pt(&pts[1], tcp, 0.03, 0.00, 0.03, speed, accel, blend_m);
        set_pt(&pts[2], tcp, 0.03, 0.00, 0.06, speed, accel, 0.0);
    } else {
        std::printf("mode=dz  relative z: %.4f %.4f %.4f  blend_m=%.4f\n",
                    dz[0], dz[1], dz[2], blend_m);
        std::printf("note: 共线路径几乎无拐角，肉眼仍可能像顿一下（不是 dwell）\n");
        set_pt(&pts[0], tcp, 0, 0, dz[0], speed, accel, blend_m);
        set_pt(&pts[1], tcp, 0, 0, dz[1], speed, accel, blend_m);
        set_pt(&pts[2], tcp, 0, 0, dz[2], speed, accel, 0.0);
    }

    for (int i = 0; i < 3; ++i) {
        std::printf("pt%d xyz=(%.4f,%.4f,%.4f) blend=%.4f dwell=%.2f %s\n", i,
                    pts[i].values[0], pts[i].values[1], pts[i].values[2],
                    pts[i].blend, pts[i].dwell_s,
                    pts[i].blend > 0 ? "(路过)" : "(停点)");
    }

    if (!frs_confirm_y("enable + tcp_path_blend? type y: ")) {
        frs_stop(robot);
        frs_destroy(robot);
        return 0;
    }
    if (frs_enable_arm(robot, arm) != FRS_OK) {
        frs_stop(robot);
        frs_destroy(robot);
        return 5;
    }

    frs_result acc{};
    const uint64_t tag = 230;
    int rc = frs_tcp_path_blend(robot, arm, pts, 3, tag, &acc);
    std::printf("tcp_path_blend rc=%d %s\n", rc, acc.message);
    if (rc == FRS_OK)
        std::printf("wait rc=%d\n", frs_wait_tag(&wait, tag, 60000));
    else
        std::fprintf(stderr, "hint: 减小 blend / 缩短位移；近奇异先 joint 摆姿态\n");

    frs_hold(robot, arm, 231, nullptr);
    frs_disable(robot, arm, 232, nullptr);
    frs_stop(robot);
    frs_destroy(robot);
    return 0;
}
