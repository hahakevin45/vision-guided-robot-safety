// =============================================================================
// safe_apf.hpp — SafeApfFilter 的純 C++17 移植
// -----------------------------------------------------------------------------
// 對應的 Python 參考實作：safety_sim/filters/safe_apf.py（SafeApfFilter 類別）
//
// 移植原則：逐行忠實轉寫（包括 CBF 速度治理段與盲走漂移膨脹安全距離），
// 不是「重新設計」。數學函式刻意使用與 CPython math 模組相同的 libm
// （std::cos/std::sin/std::atan2/std::hypot/std::fmod），因此兩邊結果
// 位元級可比（差分測試在 1e-15 容差下仍全數通過）。
// 唯一的常數是 pi：kPi 的值與 Python math.pi 是同一個 double。
//
// 與 Python API 的刻意差異（C++ 用最小 struct，省略用不到的欄位）：
//   - Observation.pose 是「數值 + has_pose 旗標」（Python 是 Pose | None）
//   - Observation 用 wheel_l/wheel_r 取代 tuple wheel_feedback
//   - obstacles 整個省略：參考實作從來沒有讀過它
//   - StaticInfo 省略 params（DiffDriveParams）：濾波器不會讀它
//   - SafetyDecision 只帶 cmd + mode；debug dict 省略（不在差分測試契約內）
//
// 零 ROS、零 colcon、零外部依賴。純 g++ -std=c++17 即可編譯。
// =============================================================================

#ifndef CPP_SAFE_APF_SAFE_APF_HPP_
#define CPP_SAFE_APF_SAFE_APF_HPP_

#include <algorithm>   // std::min / std::max
#include <cmath>       // std::cos / std::sin / std::atan2 / std::hypot / std::fmod / std::abs
#include <cstddef>     // std::size_t
#include <string>      // std::string（SafetyDecision::mode）
#include <utility>     // std::pair（geofence 頂點）
#include <vector>      // std::vector

namespace safe_apf {

// -----------------------------------------------------------------------------
// 基本資料型別（對應 safety_sim/types.py）
// -----------------------------------------------------------------------------

/// 二維速度指令（對應 Python Twist）。
/// v = 線速度 [m/s]，omega = 角速度 [rad/s]（正 = 逆時針）。
struct Twist {
    double v = 0.0;        // m/s
    double omega = 0.0;    // rad/s

    /// 靜止指令：v=0、omega=0。
    static Twist stop() { return Twist{0.0, 0.0}; }
};

/// 平面位姿（對應 Python Pose）。
/// theta 是世界座標系的朝向角 [rad]。
struct Pose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

/// 濾波器每 tick 看到的感測器觀測（對應 Python Observation）。
/// 注意：這是「信念」不是真值 —— 盲走時 pose 來自 odom 推算。
struct Observation {
    bool has_pose = false;   // Python 的 pose is None 用這個旗標表示
    Pose pose{0.0, 0.0, 0.0};
    double pose_age_s = 0.0; // 位姿新鮮度：超過上限 → 直接 STOP
    double wheel_l = 0.0;    // 左輪速度回授 [m/s]（Python: wheel_feedback[0]）
    double wheel_r = 0.0;    // 右輪速度回授 [m/s]（Python: wheel_feedback[1]）
    double link_age_s = 0.0; // 底盤/輪速回授新鮮度：超過上限 → 直接 STOP
    double pose_drift_m = 0.0; // 定位不確定度上界 [m]（盲走時 = 0.10 + 0.30×盲走里程）
};

/// 靜態地圖資訊，reset() 時給一次（對應 Python StaticInfo）。
struct StaticInfo {
    double robot_radius_m = 0.2;                    // 車體外接圓半徑 [m]
    std::vector<std::pair<double, double>> geofence; // 安全圍欄多邊形頂點（世界座標）
    double max_v_mps = 0.5;                         // 線速度上限 [m/s]
    double max_omega_rad_s = 3.0;                   // 角速度上限 [rad/s]
};

/// 濾波器決策（對應 Python SafetyDecision）。
struct SafetyDecision {
    Twist cmd;              // 實際送出的指令（可能被修改）
    std::string mode;       // "PASS" = 原樣放行 | "MODIFIED" = 已修改 | "STOP" = 停車
};

/// 牆面幾何（`_wall_distances()` 的產出，僅在 filter() 內部使用）。
struct Wall {
    double distance;    // 到牆線的帶號距離 [m]（正 = 在圍欄內側）
    double nx, ny;      // 單位內法向量（指向圍欄內）
};

// -----------------------------------------------------------------------------
// SafeApfFilter — 安全人工勢場濾波器
// -----------------------------------------------------------------------------
// 論文出處：Szczepański, "Safe Artificial Potential Field", IEEE RA-L 2023。
// 本實作是「安全過濾器」而非論文原本的「局部規劃器」：
//   吸引力 ← 上游 desired 指令（不是全域目標）
//   排斥力 ← geofence 牆面的內法向量
// 另外實作了論文沒有的兩個機制（Python 版就有，移植必須保留）：
//   1. CBF 式速度治理（filter() 內）：接近牆時把允許速度壓到 alpha*h/closing
//   2. 漂移感知安全距離：d_safe = robot_radius + 0.05 + drift（盲走越久越保守）
// -----------------------------------------------------------------------------
class SafeApfFilter {
public:
    // 建構子預設值與 Python 版完全一致（safe_apf.py 的 __init__ 參數）：
    //   extra_safe_m=0.05     基本安全餘裕 [m]
    //   drift_cap_m=0.30      漂移上限 [m]（防止無限膨脹）
    //   influence_m=0.45      斥力影響圈半徑 [m]（圈外完全不影響）
    //   alpha=1.0             CBF 速度治理的 class-K 增益
    //   k_theta=2.0           朝向誤差 → 角速度增益
    //   theta_error_max=π/2   朝向誤差超過這個值 → v 歸零（原地轉）
    //   pose_age_limit=0.5s   位姿超過這個歲數 → STOP
    //   link_age_limit=0.5s   輪速回授超過這個歲數 → STOP
    explicit SafeApfFilter(double extra_safe_m = 0.05, double drift_cap_m = 0.30,
                           double influence_m = 0.45, double alpha = 1.0,
                           double k_theta = 2.0, double theta_error_max_rad = kPi / 2.0,
                           double pose_age_limit_s = 0.5, double link_age_limit_s = 0.5)
        : extra_safe_(extra_safe_m),
          drift_cap_(drift_cap_m),
          influence_(influence_m),
          alpha_(alpha),
          k_theta_(k_theta),
          theta_error_max_(theta_error_max_rad),
          pose_age_limit_(pose_age_limit_s),
          link_age_limit_(link_age_limit_s) {}

    /// 注入靜態地圖資訊（geofence、車體半徑、速度上限）。
    /// 對應 Python 的 reset(static_info)。
    void reset(const StaticInfo& static_info) { static_ = static_info; }

    /// 每 tick 的決策入口（對應 Python filter()）。
    /// 輸入上游想要的指令，輸出安全層決策（PASS/MODIFIED/STOP）。
    /// t 和 dt 在參考實作中沒有被用到（保留參數以維持介面形狀）。
    SafetyDecision filter(const Twist& desired_in, const Observation& obs,
                          double /*t*/, double /*dt*/) {
        // --- 階段 0：資料新鮮度門檻 ----------------------------------------
        // 位姿不存在、太舊、或輪速回授太舊 → 不知道車在哪 → 停車。
        // 這是「寧可瞎不要錯」的第一道防線。
        if (!obs.has_pose || obs.pose_age_s > pose_age_limit_ ||
            obs.link_age_s > link_age_limit_) {
            return {Twist::stop(), "STOP"};
        }

        // --- 階段 1：把 desired 鉗制到車體上限 ------------------------------
        // 先記住原始值，之後判斷「有沒有被我們改過」→ 決定 PASS 還是 MODIFIED。
        const Twist raw_desired = desired_in;
        const Twist desired = clamp_twist(desired_in);
        const bool clamped = !(isclose(desired.v, raw_desired.v) &&
                               isclose(desired.omega, raw_desired.omega));

        // --- 階段 2：算牆面距離；沒牆或靜止就放行 --------------------------
        // 沒有牆（geofence < 3 點）→ 沒有需要閃避的東西 → 直接放行。
        // desired.v ≈ 0 → 車沒在動 → 也直接放行（原地轉不需要 APF）。
        std::vector<Wall> walls = wall_distances(obs.pose);

        if (walls.empty() || isclose(desired.v, 0.0, 1e-9, 1e-12)) {
            return {desired, clamped ? "MODIFIED" : "PASS"};
        }

        // --- 階段 3：漂移感知的安全距離 ------------------------------------
        // 位姿不確定度等量放大安全距離：盲走越遠、離牆越遠就該停。
        //   d_safe = robot_radius + 0.05 + min(max(drift, 0), 0.30)
        // drift 來自上游 pose_fusion（= 0.10 + 0.30×盲走里程）。
        const double drift = std::min(std::max(obs.pose_drift_m, 0.0), drift_cap_);
        const double d_safe = static_.robot_radius_m + extra_safe_ + drift;

        // 行進方向：倒車時以車尾為「前方」（θ+π 鏡射），同一套公式直接生效。
        const int signed_speed = desired.v >= 0.0 ? 1 : -1;
        const double travel_theta =
            signed_speed > 0 ? obs.pose.theta : obs.pose.theta + kPi;
        const double ux = std::cos(travel_theta);
        const double uy = std::sin(travel_theta);

        // --- 階段 4：CBF 式速度治理（論文沒有，是我們加的）-----------------
        // 對每面牆建立 barrier h = distance - d_safe，離散化條件 ḣ ≥ -α·h：
        //   正在逼近牆（closing > 0）時，允許速度 ≤ α·h/closing
        // 效果：越靠近牆，允許的逼近速度線性趨零 → 漸進煞停而非硬停。
        bool needs_apf = false;
        double speed_limit = std::abs(desired.v);
        for (const Wall& w : walls) {
            const double h = w.distance - d_safe;
            const double closing = -(ux * w.nx + uy * w.ny);  // >0 = 正在逼近
            if (w.distance <= d_safe || w.distance < influence_) {
                needs_apf = true;   // 已侵入或已進影響圈 → 需要勢場介入
            }
            if (closing > 1e-9) {
                const double allowed = std::max(0.0, alpha_ * h / closing);
                if (std::abs(desired.v) > allowed + 1e-9) {
                    speed_limit = std::min(speed_limit, allowed);
                    needs_apf = true;
                }
            }
        }

        // 所有牆都在影響圈外且速度沒超限 → 不需要介入 → 放行。
        if (!needs_apf) {
            return {desired, clamped ? "MODIFIED" : "PASS"};
        }

        // --- 階段 5：APF 力場重導向 ----------------------------------------
        const Twist cmd =
            apf_command(desired, obs.pose, walls, d_safe, signed_speed, speed_limit);
        // Python 版的 `if cmd is None: STOP` 是死分支（_apf_command 永不回傳
        // None），C++ 版不移植；力場相消時 apf_command 內部已回傳 Twist::stop()。
        const bool modified =
            clamped || !(isclose(cmd.v, desired.v) && isclose(cmd.omega, desired.omega));
        return {cmd, modified ? "MODIFIED" : "PASS"};
    }

private:
    /// pi 常數 —— 刻意與 CPython math.pi 相同的 double 值，
    /// 讓三角函數結果可位元級比對。
    static constexpr double kPi = 3.141592653589793238462643383279502884;

    // 建構子參數（見建構子註解）
    double extra_safe_;
    double drift_cap_;
    double influence_;
    double alpha_;
    double k_theta_;
    double theta_error_max_;
    double pose_age_limit_;
    double link_age_limit_;
    StaticInfo static_;   // reset() 注入的靜態地圖

    // -------------------------------------------------------------------------
    // 工具函式
    // -------------------------------------------------------------------------

    /// 重現 Python math.isclose(a, b, *, rel_tol, abs_tol)：
    ///   |a-b| <= max(rel_tol * max(|a|, |b|), abs_tol)
    /// 用於「命令有沒有被修改過」的判斷，語意與 Python 一致。
    static bool isclose(double a, double b, double rel_tol = 1e-9,
                        double abs_tol = 0.0) {
        return std::abs(a - b) <=
               std::max(rel_tol * std::max(std::abs(a), std::abs(b)), abs_tol);
    }

    /// 對稱鉗制：把 value 限制在 [-limit, +limit]。
    static double clamp(double value, double limit) {
        return std::max(-limit, std::min(limit, value));
    }

    /// 整筆 Twist 鉗制到車體速度上限。
    Twist clamp_twist(const Twist& twist) const {
        return {clamp(twist.v, static_.max_v_mps),
                clamp(twist.omega, static_.max_omega_rad_s)};
    }

    /// 角度歸一化到 (-π, +π]。
    /// 對應 Python 的 (angle + pi) % (2*pi) - pi。
    /// 注意：Python 的 % 是「向下取整」模運算（結果恆 ≥ 0），
    /// 而 C 的 std::fmod 是「截斷」模運算（結果與被除數同號）——
    /// 所以這裡要手動修正負數，才能位元級一致。
    static double wrap(double angle) {
        double r = std::fmod(angle + kPi, 2.0 * kPi);
        if (r < 0.0) r += 2.0 * kPi;
        return r - kPi;
    }

    /// 多邊形帶號面積（鞋帶公式）÷ 2。
    /// 正 = 頂點逆時針（CCW）、負 = 順時針（CW）。
    /// 用來決定每面牆的內法向量方向。
    double signed_area() const {
        const std::vector<std::pair<double, double>>& poly = static_.geofence;
        double area = 0.0;
        const std::size_t n = poly.size();
        for (std::size_t i = 0; i < n; ++i) {
            const double x1 = poly[i].first, y1 = poly[i].second;
            const double x2 = poly[(i + 1) % n].first, y2 = poly[(i + 1) % n].second;
            area += x1 * y2 - x2 * y1;
        }
        return area / 2.0;
    }

    /// 對 geofence 每條邊計算「到無限直線的帶號距離」+ 內法向量。
    /// 距離 = n·(p - v_i)；正 = 在圍欄內側。
    /// 假設多邊形是凸的（凹多邊形此式不成立）。
    /// 邊長 ≤ 1e-12（退化邊）跳過。
    std::vector<Wall> wall_distances(const Pose& pose) const {
        const std::vector<std::pair<double, double>>& fence = static_.geofence;
        if (fence.size() < 3) return {};   // 不足三點 → 視為沒有牆
        const bool ccw = signed_area() >= 0.0;
        std::vector<Wall> walls;
        const std::size_t n = fence.size();
        for (std::size_t i = 0; i < n; ++i) {
            const double x1 = fence[i].first, y1 = fence[i].second;
            const double x2 = fence[(i + 1) % n].first, y2 = fence[(i + 1) % n].second;
            const double ex = x2 - x1, ey = y2 - y1;      // 邊向量
            const double length = std::hypot(ex, ey);
            if (length <= 1e-12) continue;
            // 內法向量：CCW 邊取左法向，CW 邊取右法向
            double nx, ny;
            if (ccw) {
                nx = -ey / length;
                ny = ex / length;
            } else {
                nx = ey / length;
                ny = -ex / length;
            }
            // 帶號距離：機器人在圍欄內 → 正
            const double distance = nx * (pose.x - x1) + ny * (pose.y - y1);
            walls.push_back({distance, nx, ny});
        }
        return walls;
    }

    /// APF 力場核心（對應論文式 (9)(10)(11)，見 filter() 的階段 5）。
    ///
    /// 力場合成：
    ///   吸引力 = |desired.v| 沿行進方向（不是朝全域目標 —— 我們是 filter）
    ///   排斥力 = 對影響圈內的每面牆：
    ///       distance ≤ d_safe     → strength = max_v（飽和，滿格推離）
    ///       d_safe < distance     → strength = max_v · rel²
    ///                                 其中 rel = (influence-d)/(influence-d_safe)
    ///                                （二次平滑上升，非論文的 1/d 漸近）
    /// 飽和設計的意義：滿斥力 ≥ 任何吸引力 → 侵入 d_safe 後淨力必然向外，
    /// 且輸出永遠有界（真車馬達做得到）。
    ///
    /// 力場 → 差速車指令（論文式 9/10/11）：
    ///   θ* = atan2(Fy, Fx)，ω = clamp(k_θ · wrap(θ* − θ), max_omega)
    ///   |θ_err| ≥ θ_max → v = 0（先原地轉）
    ///   否則 v = min(|desired.v|, speed_limit, ‖F‖, max_v) · (θ_max−|θ_err|)/θ_max
    ///   （CBF 上限與力場大小同時約束速度；朝向誤差越大速度越低）
    ///
    /// 力場相消（‖F‖ < 1e-9，APF 經典 local minimum）→ 回傳停車，
    /// 這是「卡住就停，不硬闖」的刻意取捨。
    Twist apf_command(const Twist& desired, const Pose& pose,
                      const std::vector<Wall>& walls, double d_safe,
                      int signed_speed, double speed_limit) const {
        // 行進方向（倒車時鏡射到車尾向）
        const double travel_theta =
            signed_speed > 0 ? pose.theta : pose.theta + kPi;
        // 吸引力：大小 = |desired.v|，方向 = 行進方向
        double fx = std::abs(desired.v) * std::cos(travel_theta);
        double fy = std::abs(desired.v) * std::sin(travel_theta);

        // 影響圈至少要有 d_safe 大（避免除零/負 rel）
        const double influence = std::max(influence_, d_safe + 1e-6);
        for (const Wall& w : walls) {
            if (w.distance >= influence) continue;   // 圈外 → 完全不管
            double strength;
            if (w.distance <= d_safe) {
                strength = static_.max_v_mps;        // 飽和斥力
            } else {
                const double rel = (influence - w.distance) / (influence - d_safe);
                strength = static_.max_v_mps * rel * rel;   // 二次衰減
            }
            fx += strength * w.nx;                   // 沿內法向量推離
            fy += strength * w.ny;
        }

        const double norm = std::hypot(fx, fy);
        if (norm < 1e-9) return Twist::stop();       // 力場相消 → 停車

        const double target_theta = std::atan2(fy, fx);
        const double theta_error = wrap(target_theta - travel_theta);
        const double omega = clamp(k_theta_ * theta_error, static_.max_omega_rad_s);

        // 朝向誤差折扣：誤差 ≥ 90° 先原地轉不前進
        const double abs_error = std::abs(theta_error);
        double v_mag = 0.0;
        if (abs_error < theta_error_max_) {
            const double scale = (theta_error_max_ - abs_error) / theta_error_max_;
            const double capped = std::min(
                std::abs(desired.v),                     // 不超過上游要求
                std::min(std::max(0.0, speed_limit),     // 不超過 CBF 上限
                         std::min(norm, static_.max_v_mps)));  // 不超過力場/車體上限
            v_mag = capped * scale;
        }

        // 倒車時速度要還原成負號
        return clamp_twist({signed_speed * v_mag, omega});
    }
};

}  // namespace safe_apf

#endif  // CPP_SAFE_APF_SAFE_APF_HPP_
