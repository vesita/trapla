//! 指标面板的数据来源：宽松读取 `trapla plan --metrics` 的 JSON。
//!
//! 字段口径见 `task-11`：`steps` / `total_length` / `goal_error` / `proper_crossings` /
//! `same_foot_turn` / `lateral_separation`。**任何**缺失、类型不符、文件不存在都降级为
//! 「未提供」，绝不 panic、绝不因此改变退出码。

use std::path::Path;

use serde_json::Value;

/// 面板上一项的值：要么有值，要么明写「未提供」。
#[derive(Debug, Clone, PartialEq)]
pub enum Metric {
    /// 已取得的值。
    Value(f64),
    /// 没有提供（文件缺失或字段缺失）。
    Absent,
}

impl Metric {
    fn from_json(v: &Value, key: &str) -> Self {
        v.get(key)
            .and_then(Value::as_f64)
            .filter(|x| x.is_finite())
            .map_or(Self::Absent, Self::Value)
    }

    /// 显示文本：整数不带小数点，其余保留三位。
    pub fn display(&self) -> String {
        match self {
            Self::Absent => "未提供".to_string(),
            Self::Value(x) => {
                if x.fract() == 0.0 && x.abs() < 1e15 {
                    format!("{}", *x as i64)
                } else {
                    format!("{x:.3}")
                }
            }
        }
    }
}

/// 面板数据。
#[derive(Debug, Clone, PartialEq)]
pub struct MetricsPanel {
    /// 指标来源说明（文件名或「未提供」的原因）。
    pub source: String,
    /// 步数。
    pub steps: Metric,
    /// 路径总长。
    pub total_length: Metric,
    /// 终点误差。
    pub goal_error: Metric,
    /// 严格跨立次数。
    pub proper_crossings: Metric,
    /// 同足相邻步转角的中位数。
    pub same_foot_turn_median: Metric,
    /// 横向间距最小值。
    pub lateral_min: Metric,
    /// 横向间距中位数。
    pub lateral_median: Metric,
}

impl MetricsPanel {
    /// 没有 metrics 时的面板（全部「未提供」）。
    pub fn absent(reason: impl Into<String>) -> Self {
        Self {
            source: reason.into(),
            steps: Metric::Absent,
            total_length: Metric::Absent,
            goal_error: Metric::Absent,
            proper_crossings: Metric::Absent,
            same_foot_turn_median: Metric::Absent,
            lateral_min: Metric::Absent,
            lateral_median: Metric::Absent,
        }
    }

    /// 从 JSON 路径加载；`None` 表示调用方没给 `--metrics`。
    ///
    /// 读失败 / 解析失败不返回 `Err`：面板显示原因，其余功能照常。
    pub fn load(path: Option<&Path>) -> Self {
        let Some(path) = path else {
            return Self::absent("未提供（未给 --metrics）");
        };
        let text = match std::fs::read_to_string(path) {
            Ok(t) => t,
            Err(e) => return Self::absent(format!("未提供（读取 {} 失败: {e}）", path.display())),
        };
        let value: Value = match serde_json::from_str(&text) {
            Ok(v) => v,
            Err(e) => {
                return Self::absent(format!("未提供（解析 {} 失败: {e}）", path.display()));
            }
        };
        let Some(num) = value.get("same_foot_turn").and_then(Value::as_array) else {
            // same_foot_turn 缺失时该行「未提供」，其它字段仍可能可用。
            return Self {
                source: format!("{}（same_foot_turn 未提供）", path.display()),
                steps: Metric::from_json(&value, "steps"),
                total_length: Metric::from_json(&value, "total_length"),
                goal_error: Metric::from_json(&value, "goal_error"),
                proper_crossings: Metric::from_json(&value, "proper_crossings"),
                same_foot_turn_median: Metric::Absent,
                lateral_min: Self::from_json_vec(&value, "lateral_separation").0,
                lateral_median: Self::from_json_vec(&value, "lateral_separation").1,
            };
        };
        let same_foot: Vec<f64> = num.iter().filter_map(Value::as_f64).collect();
        let (lateral_min, lateral_median) = Self::from_json_vec(&value, "lateral_separation");
        Self {
            source: path.display().to_string(),
            steps: Metric::from_json(&value, "steps"),
            total_length: Metric::from_json(&value, "total_length"),
            goal_error: Metric::from_json(&value, "goal_error"),
            proper_crossings: Metric::from_json(&value, "proper_crossings"),
            same_foot_turn_median: median(&same_foot).map_or(Metric::Absent, Metric::Value),
            lateral_min,
            lateral_median,
        }
    }

    fn from_json_vec(v: &Value, key: &str) -> (Metric, Metric) {
        let Some(arr) = v.get(key).and_then(Value::as_array) else {
            return (Metric::Absent, Metric::Absent);
        };
        let xs: Vec<f64> = arr.iter().filter_map(Value::as_f64).collect();
        (
            xs.iter()
                .copied()
                .reduce(f64::min)
                .map_or(Metric::Absent, Metric::Value),
            median(&xs).map_or(Metric::Absent, Metric::Value),
        )
    }

    /// 面板文本（多行，供 Bevy UI `Text` 直接显示）。
    pub fn lines(&self) -> String {
        format!(
            "指标（来源: {src}）\n\
             steps: {steps}\n\
             total_length: {len}\n\
             goal_error: {err}\n\
             proper_crossings: {cross}\n\
             same_foot_turn 中位数: {turn}\n\
             lateral_separation 最小 / 中位: {lmin} / {lmed}",
            src = self.source,
            steps = self.steps.display(),
            len = self.total_length.display(),
            err = self.goal_error.display(),
            cross = self.proper_crossings.display(),
            turn = self.same_foot_turn_median.display(),
            lmin = self.lateral_min.display(),
            lmed = self.lateral_median.display(),
        )
    }

    /// 供自检打印的单行摘要。
    pub fn summary(&self) -> String {
        format!(
            "steps={} total_length={} goal_error={} proper_crossings={} same_foot_turn中位={} lateral最小/中位={}/{}",
            self.steps.display(),
            self.total_length.display(),
            self.goal_error.display(),
            self.proper_crossings.display(),
            self.same_foot_turn_median.display(),
            self.lateral_min.display(),
            self.lateral_median.display(),
        )
    }
}

/// 中位数；空序列返回 `None`。
pub fn median(xs: &[f64]) -> Option<f64> {
    if xs.is_empty() {
        return None;
    }
    let mut v: Vec<f64> = xs.to_vec();
    v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mid = v.len() / 2;
    if v.len() % 2 == 1 {
        Some(v[mid])
    } else {
        Some((v[mid - 1] + v[mid]) / 2.0)
    }
}
