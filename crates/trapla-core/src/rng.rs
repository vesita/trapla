//! 确定性随机源。
//!
//! 这是硬要求：原 C++ 在 8 处使用 `std::random_device + std::mt19937 + shuffle`
//! （如 `src/utils/geometry.cpp:287-293`、`315-327`），同一输入两次跑出不同结果
//! （实测 68 点 / 91 点）。重写后：
//!
//! - 种子只能来自显式传入的 `seed`（CLI `--seed`），禁止 `random_device` / 系统时间；
//! - 同一个 seed 在任何平台、任何次运行都必须产出同一条随机序列（算法固定、位宽固定）；
//! - `shuffle` 必须是 Fisher–Yates，随机数全部来自 `self` 的确定性序列。
//!
//! 涉及「顺序」的地方用 `BTreeMap` 或显式排序，禁止依赖 `HashMap` 迭代顺序。
//!
//! ## 钉死的算法（task-16 决策记录）
//!
//! 初始化用 **SplitMix64**（`state = splitmix64(seed)`，`seed` 先加黄金比例常数
//! `0x9E3779B97F4A7C15` 再混合），之后每次取值用 **xorshift64\***（`src`/`mult`：
//! `x ^= x >> 12; x ^= x << 25; x ^= x >> 27; x.wrapping_mul(0x2545F4914F6CDD1D)`）。
//! `state == 0` 会让 xorshift64\* 退化成恒 0 序列，因此初始化时若混合结果是 0 就抬到黄金比例
//! 常数（`from_seed(0)` 本身是良定义的：SplitMix64 会先把 0 加常数）。
//! 纯整数运算、无浮点、无平台相关类型；`next_below` 用拒绝采样（阈值法），不用取模直接出结果。

/// 确定性 PRNG。内部算法由实现方选定（`splitmix64` / `xorshift64*` 等），
/// 但必须满足：可 seed、可 clone、跨平台稳定。
#[derive(Debug, Clone)]
pub struct Rng {
    /// 内部状态。实现方自由选择算法，但必须是纯整数运算（禁浮点、禁平台相关类型）。
    state: u64,
}

/// SplitMix64 的黄金比例增量（`state += gamma` 后再混合）。
const GOLDEN_GAMMA: u64 = 0x9E37_79B9_7F4A_7C15;
/// SplitMix64 的两次乘法常数。
const SPLITMIX_MUL_1: u64 = 0xBF58_476D_1CE4_E5B9;
const SPLITMIX_MUL_2: u64 = 0x94D0_49BB_1331_11EB;
/// xorshift64\* 的输出乘数。
const XORSHIFT_MUL: u64 = 0x2545_F491_4F6C_DD1D;

/// SplitMix64 的一步混合：输入上一状态，输出混合结果（也被当作新状态）。
const fn splitmix64(state: u64) -> u64 {
    let mut z = state.wrapping_add(GOLDEN_GAMMA);
    z = (z ^ (z >> 30)).wrapping_mul(SPLITMIX_MUL_1);
    z = (z ^ (z >> 27)).wrapping_mul(SPLITMIX_MUL_2);
    z ^ (z >> 31)
}

impl Rng {
    /// 用给定种子构造。相同 seed ⇒ 相同序列。
    pub fn from_seed(seed: u64) -> Self {
        let state = splitmix64(seed);
        // xorshift64* 在 state == 0 时恒输出 0（退化）；SplitMix64 极少给出 0，这里显式兜住。
        let state = if state == 0 { GOLDEN_GAMMA } else { state };
        Self { state }
    }

    /// xorshift64\* 一步：返回本步输出，状态前移到新值。
    ///
    /// 也被 `next_below` 与 [`Rng::shuffle`] 使用 —— 序列只有这一处产生。
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(XORSHIFT_MUL)
    }

    /// 无偏的 `[0, n)` 随机整数（拒绝采样 / 阈值法，`n > 0`）。
    ///
    /// `threshold = (2^64 - n) % n = n.wrapping_neg() % n`：把 `[0, threshold)` 这一段
    /// 丢弃，剩下的 `2^64 - threshold` 个值恰能被 `n` 均分。`n == 1` 时 `threshold == 0`，
    /// 恒返回 0。**不许**改成 `next_u64() % n`（会有取模偏差，破坏跨版本序列的可解释性）。
    pub fn next_below(&mut self, n: u64) -> u64 {
        assert!(n > 0, "next_below 的 n 必须大于 0");
        let threshold = n.wrapping_neg() % n;
        loop {
            let r = self.next_u64();
            if r >= threshold {
                return r % n;
            }
        }
    }

    /// 原地 Fisher–Yates 洗牌；顺序完全由 `self` 的确定性序列决定。
    pub fn shuffle<T>(&mut self, s: &mut [T]) {
        for i in (1..s.len()).rev() {
            let j = self.next_below(i as u64 + 1) as usize;
            s.swap(i, j);
        }
    }
}
