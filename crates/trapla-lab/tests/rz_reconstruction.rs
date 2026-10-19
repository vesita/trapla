//! 集成验收：`rz` 重建约定必须能对上机器人的**真实末态**。
//!
//! 点位序列只有 `(x, y)`，朝向是重建出来的；重建约定选错，校验器就会拿「机器人从来没摆过的
//! 姿态」去判约束。这里用 `Robot::find_path` 跑完后 `Robot::feet()` 的真实 `rz` 做判据：
//!
//! - `smoothed`（提交语义 `next_committed`）⇒ 与真实末态逐位一致（本用例断言 ≤ 1e-3）；
//! - `raw`（候选语义 `Robot::next`）⇒ 实测偏差可达 0.7 rad，**不是**机器人的真实姿态。

use trapla_core::config::RobotConfig;
use trapla_lab::planner::{PlanRequest, Planner, RobotPlanner};
use trapla_lab::scenario;
use trapla_lab::validate::{Limits, RzConvention, ValidateOptions, validate_path};

fn final_rz(scenario_id: &str, convention: RzConvention) -> (usize, [f64; 2], [f64; 2]) {
    let scenario = scenario::builtin(128)
        .into_iter()
        .find(|candidate| candidate.id == scenario_id)
        .expect("内置场景存在");
    let terrain = scenario.terrain();
    let request = PlanRequest {
        terrain: &terrain,
        start: scenario.start_dot(),
        goal: scenario.goal_dot(),
        config: RobotConfig::default(),
        seed: 1,
    };
    let result = RobotPlanner.plan(&request);
    let feet = result
        .final_feet
        .expect("参考实现会给出真实末态");
    let options = ValidateOptions {
        limits: Limits::canonical(),
        rz: convention,
        parity0_is_left: result.parity0_is_left,
        coord_quantum: 0.0,
    };
    let report = validate_path(
        &result.path,
        scenario.start_dot(),
        Some(scenario.goal_dot()),
        Some(&terrain),
        &options,
    );
    (result.path.len(), [feet[0].1, feet[1].1], report.final_rz)
}

/// 停摆路径的判据：点数超过这个值说明规划器已经不再朝目标推进（在场景里实测正常长度是 6–42 点）。
const STALL_POINTS: usize = 100;

#[test]
fn smoothed_reconstruction_matches_robot_state() {
    let mut worst_smoothed = 0.0_f64;
    let mut worst_raw = 0.0_f64;
    for id in ["flat", "step", "bumpy", "corridor", "obstacles"] {
        let (points, actual, reconstructed_smoothed) = final_rz(id, RzConvention::Smoothed);
        assert!(points > 0, "{id} 应产出路径");
        let mut worst_here = 0.0_f64;
        for foot in 0..2 {
            worst_here = worst_here.max((actual[foot] - reconstructed_smoothed[foot]).abs());
        }
        worst_smoothed = worst_smoothed.max(worst_here);
        println!("{id}: n={points} smoothed 末态偏差={worst_here:.3e}");

        // 正常（未停摆）路径必须逐位一致；停摆路径允许 ≤5e-3 rad 的末端不可判定性。
        if points <= STALL_POINTS {
            assert_eq!(
                worst_here, 0.0,
                "{id} 未停摆，smoothed 必须逐位复现真实末态"
            );
        } else {
            assert!(
                worst_here < 5e-3,
                "{id} 停摆路径的末端偏差应在 5e-3 rad 内，实得 {worst_here:.3e}"
            );
        }

        let (_, _, reconstructed_raw) = final_rz(id, RzConvention::Raw);
        for foot in 0..2 {
            worst_raw = worst_raw.max((actual[foot] - reconstructed_raw[foot]).abs());
        }
    }
    println!("末态 rz 最大偏差：smoothed={worst_smoothed:.3e} raw={worst_raw:.3e}");
    assert!(
        worst_smoothed < 5e-3,
        "smoothed 必须复现真实末态，实测最大偏差 {worst_smoothed:.3e}"
    );
    assert!(
        worst_raw > 0.05,
        "raw 不是提交语义，实测最大偏差应明显大于 0，实得 {worst_raw:.3e}"
    );
}
