//! 姿态口径的**强制点**：同一候选点，按 `next`（候选姿态）判过、按 `next_committed`（提交姿态）
//! 越界 —— `find_path` 的候选过滤必须按后者判（`robot.rs` 的 `position_check_committed` /
//! `standable_committed` / `walk_candidates` / `bfs`）。
//!
//! 背景（Lead 实测）：`data/csv/map.csv` + seed 42 的基线上，独立校验器
//! （`trapla-lab`，不复用 core 谓词）在 62 点里报出 16 步越界，而 core 自己的
//! `satisfy_spacing` / `Foot::standable` 对同样的 62 步 61/61 判通过 —— 差额全部来自
//! 「判过的姿态」与「执行的姿态」之间的半个转角漂移。
//!
//! 撤销本修复（把候选过滤改回 `position_check`）会让 `divergence_is_real` 里的
//! 「旧口径接受 / 新口径拒绝」断言不再成立 —— 这条测试就是那次回归的门禁。

use std::f64::consts::PI;

use trapla_core::config::{FootShape, RobotConfig};
use trapla_core::constraints;
use trapla_core::foot::Foot;
use trapla_core::geom::Dot;
use trapla_core::robot::{Robot, WhichFoot};

/// 候选点与两个口径判定结果的分歧样本。
struct Divergence {
    robot: Robot,
    candidate: Dot,
}

fn degrees(value: f64) -> f64 {
    value * PI / 180.0
}

/// 镜像 `Robot::walk_candidates` 的格点定义（`robot.rs`：步距分辨率 2.0、角度分辨率 π/12、
/// 距离从 `active_distance` 到 `max_stride`、角度相对**支撑脚** `rz`）。
///
/// 这不是第二份实现，只是给测试一个「旧口径的候选集」参照；`lattice_mirror_matches_walk_candidates`
/// 会钉住它与真实 `walk_candidates` 的格点一致，防止两处定义漂移。
fn lattice(robot: &Robot) -> Vec<Dot> {
    let cfg = robot.config();
    let swing = robot.swing_foot();
    let support = robot.support_foot();
    let mut points = Vec::new();

    let mut distance = cfg.active_distance;
    while distance <= cfg.max_stride + 1e-9 {
        let mut angle_offset = -cfg.max_turn;
        while angle_offset <= cfg.max_turn + 1e-9 {
            let angle = support.rz + angle_offset;
            points.push(Dot::new(
                swing.position.x() + distance * angle.cos(),
                swing.position.y() + distance * angle.sin(),
            ));
            angle_offset += PI / 12.0;
        }
        distance += 2.0;
    }
    points
}

/// 扫一批「摆动脚 / 支撑脚朝向 × 双足间距」组合，找一个两个口径判定不同的候选点。
///
/// 返回 `None` 表示这批组合里没有分歧 —— 那时 `divergence_is_real` 会失败（空转的测试
/// 比没有测试更危险），必须换组合重扫，不能把断言删掉。
fn find_divergence() -> Option<Divergence> {
    let cfg = RobotConfig::default();
    let shape = FootShape::biped();
    let angles = [-75.0, -45.0, -15.0, 0.0, 15.0, 45.0, 75.0];

    for swing_rz_deg in angles {
        for support_rz_deg in angles {
            for separation in [3.0, 5.2, 8.0] {
                let swing = Foot::new(Dot::new(0.0, 0.0), degrees(swing_rz_deg), shape);
                let support = Foot::new(Dot::new(0.0, separation), degrees(support_rz_deg), shape);
                let mut robot = Robot::new(cfg, Dot::new(0.0, 0.0));
                robot.set_feet(swing, support, WhichFoot::Left);

                for candidate in lattice(&robot) {
                    if robot.position_check(candidate) && !robot.position_check_committed(candidate)
                    {
                        return Some(Divergence { robot, candidate });
                    }
                }
            }
        }
    }
    None
}

/// 负向对照：旧口径（`next` 姿态）通过、新口径（`next_committed` 姿态）越界的候选确实存在，
/// 且规划器现在的候选集不再包含它。
#[test]
fn divergence_is_real() {
    let Divergence { robot, candidate } =
        find_divergence().expect("没扫到分歧候选：本测试会空转，必须换组合重扫而不是删断言");

    // ① 旧口径（候选姿态 `next`）：通过 —— 撤销修复后候选过滤走的就是它。
    assert!(
        robot.position_check(candidate),
        "旧口径本应接受这个候选（否则这条负向对照不成立）"
    );

    // ② 新口径（提交姿态 `next_committed`）：拒绝。
    assert!(
        !robot.position_check_committed(candidate),
        "新口径必须拒绝这个候选"
    );

    // ③ 分歧落在「间距」这条上，且裕度可量化：证明两处姿态确实不同名不同实。
    let raw_pose = robot.next(candidate);
    let committed_pose = robot.next_committed(candidate);
    assert!(
        (raw_pose.rz - committed_pose.rz).abs() > 1e-9,
        "两个姿态在 rz 上必须不同，否则这条分歧不该出现"
    );

    let support = robot.support_foot();
    let raw_spacing = constraints::spacing(&raw_pose, support, robot.config());
    let committed_spacing = constraints::spacing(&committed_pose, support, robot.config());
    assert!(
        raw_spacing.satisfied && !committed_spacing.satisfied,
        "间距口径应给出「候选姿态判过、提交姿态越界」：raw={raw_spacing:?} committed={committed_spacing:?}"
    );
    assert!(
        committed_spacing.margin < 0.0,
        "提交姿态的间距裕度应为负：{committed_spacing:?}"
    );

    println!(
        "分歧样本: swing=({:.6},{:.6}) rz={:.6} | support=({:.6},{:.6}) rz={:.6} | candidate=({:.6},{:.6})\n\
         next.rz={:.6} next_committed.rz={:.6} Δrz={:.6}\n\
         间距裕度: next={:.6}（满足={}） vs next_committed={:.6}（满足={}）",
        robot.swing_foot().position.x(),
        robot.swing_foot().position.y(),
        robot.swing_foot().rz,
        support.position.x(),
        support.position.y(),
        support.rz,
        candidate.x(),
        candidate.y(),
        raw_pose.rz,
        committed_pose.rz,
        raw_pose.rz - committed_pose.rz,
        raw_spacing.margin,
        raw_spacing.satisfied,
        committed_spacing.margin,
        committed_spacing.satisfied,
    );

    // ④ 规划器实际用的候选集：旧口径会把它放进去，新口径必须把它排除。
    let old_candidates: Vec<Dot> = lattice(&robot)
        .into_iter()
        .filter(|point| robot.position_check(*point))
        .collect();
    assert!(
        old_candidates.contains(&candidate),
        "旧口径的候选集本应包含这个点（否则它不在格点上，负向对照不成立）"
    );
    assert!(
        !robot.walk_candidates().contains(&candidate),
        "新口径的候选集必须排除这个点"
    );

    // ⑤ 新候选集里没有任何点漏过提交口径（无绕过入口）。
    for point in robot.walk_candidates() {
        assert!(
            robot.position_check_committed(point),
            "候选集里出现了过不了提交口径的点：{point:?}"
        );
    }
}

/// 格点镜像与真实 `walk_candidates` 同源：两者都非空，且新候选集是镜像格点的子集。
///
/// 这条守卫让 `lattice` 里抄的结构常量（2.0 / π/12 / active_distance）一旦与
/// `robot.rs` 漂移就变红。
#[test]
fn lattice_mirror_matches_walk_candidates() {
    let Divergence { robot, .. } = find_divergence().expect("需要同一批组合里的样本");
    let mirrored = lattice(&robot);
    let actual = robot.walk_candidates();

    assert!(!mirrored.is_empty() && !actual.is_empty());
    for point in &actual {
        assert!(
            mirrored.contains(point),
            "walk_candidates 里有不在镜像格点上的点：{point:?}"
        );
    }
}

/// 初始足位下的候选集里每个点都过提交口径（把「不许绕过该模块的入口」钉在候选生成点上）。
#[test]
fn initial_pose_candidates_all_pass_committed_gate() {
    let robot = Robot::new(RobotConfig::default(), Dot::new(0.0, 0.0));
    let candidates = robot.walk_candidates();
    assert!(!candidates.is_empty(), "初始足位应有候选");
    for point in candidates {
        assert!(robot.position_check_committed(point));
    }
}
