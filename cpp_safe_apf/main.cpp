// =============================================================================
// main.cpp — C++ SafeApfFilter 移植的差分測試 harness
// -----------------------------------------------------------------------------
// 流程：
//   1. 讀取 golden.json（格式見 gen_golden.py 檔頭說明）
//   2. 逐 case 重播 cpp_safe_apf::SafeApfFilter
//   3. 與「真實 Python 參考實作」產出的期望值比對：
//        - mode：字串完全相符（"PASS" | "MODIFIED" | "STOP"）
//        - cmd.v / cmd.omega：絕對容差 1e-9
//   4. 全部通過才回傳 0；任一失敗回傳 1；檔案/解析錯誤回傳 2
//
// 純 C++17、零外部依賴（JSON 解析器是下方內嵌的極簡遞迴下降實作，
// 只支援 golden.json 會用到的型別，不支援 unicode surrogate pair 等）。
// =============================================================================

#include "safe_apf.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

// ---------------------------------------------------------------------------
// 極簡 JSON 解析器（只為 golden.json 而寫）
// ---------------------------------------------------------------------------

namespace jparse {

/// 通用 JSON 值節點：任何型別只存一種內容（union 的簡化版）。
struct Value {
    enum class Type { Null, Bool, Number, String, Array, Object };
    Type type = Type::Null;
    bool boolean = false;
    double number = 0.0;
    std::string string;
    std::vector<Value> array;
    std::vector<std::pair<std::string, Value>> object;

    /// 在物件中依 key 查詢；不存在或本身不是物件 → nullptr。
    const Value* find(const std::string& key) const {
        if (type != Type::Object) return nullptr;
        for (const auto& kv : object) {
            if (kv.first == key) return &kv.second;
        }
        return nullptr;
    }
};

/// 解析錯誤（丟出後由 main 捕捉並印出訊息）。
struct ParseError {
    std::string msg;
};

/// 遞迴下降解析器：value → object/array/string/number/literal 五種。
class Parser {
public:
    explicit Parser(const std::string& text) : s_(text) {}

    /// 解析整個文件；結尾有多餘字元也算錯誤。
    Value parse() {
        Value v = parse_value();
        ws();
        if (i_ != s_.size()) throw ParseError{"trailing characters"};
        return v;
    }

private:
    const std::string& s_;   // 輸入文字（不複製）
    std::size_t i_ = 0;      // 目前讀取位置

    /// 跳過空白（空格 / tab / 換行 / 歸位）。
    void ws() {
        while (i_ < s_.size() &&
               (s_[i_] == ' ' || s_[i_] == '\t' || s_[i_] == '\n' || s_[i_] == '\r')) {
            ++i_;
        }
    }

    /// 檢查接下來的字元是否等於指定字面值（true/false/null 用）。
    void expect(const char* lit) {
        const std::size_t n = std::strlen(lit);
        if (s_.compare(i_, n, lit) != 0) throw ParseError{std::string("expected ") + lit};
        i_ += n;
    }

    /// 依下一個字元分派到各型別解析器。
    Value parse_value() {
        ws();
        if (i_ >= s_.size()) throw ParseError{"unexpected end of input"};
        const char c = s_[i_];
        if (c == '{') return parse_object();
        if (c == '[') return parse_array();
        if (c == '"') {
            Value v;
            v.type = Value::Type::String;
            v.string = parse_string();
            return v;
        }
        if (c == 't') {
            expect("true");
            Value v;
            v.type = Value::Type::Bool;
            v.boolean = true;
            return v;
        }
        if (c == 'f') {
            expect("false");
            Value v;
            v.type = Value::Type::Bool;
            v.boolean = false;
            return v;
        }
        if (c == 'n') {
            expect("null");
            return Value{};
        }
        return parse_number();
    }

    /// 物件：{ "key": value, ... }
    Value parse_object() {
        Value v;
        v.type = Value::Type::Object;
        ++i_;  // 吃掉 '{'
        ws();
        if (i_ < s_.size() && s_[i_] == '}') {   // 空物件 {}
            ++i_;
            return v;
        }
        for (;;) {
            ws();
            if (i_ >= s_.size() || s_[i_] != '"') throw ParseError{"expected string key"};
            std::string key = parse_string();
            ws();
            if (i_ >= s_.size() || s_[i_] != ':') throw ParseError{"expected ':'"};
            ++i_;
            v.object.emplace_back(std::move(key), parse_value());
            ws();
            if (i_ >= s_.size()) throw ParseError{"unterminated object"};
            if (s_[i_] == ',') {
                ++i_;
                continue;      // 還有下一個 key-value
            }
            if (s_[i_] == '}') {
                ++i_;
                return v;      // 物件結束
            }
            throw ParseError{"expected ',' or '}'"};
        }
    }

    /// 陣列：[ value, ... ]
    Value parse_array() {
        Value v;
        v.type = Value::Type::Array;
        ++i_;  // 吃掉 '['
        ws();
        if (i_ < s_.size() && s_[i_] == ']') {   // 空陣列 []
            ++i_;
            return v;
        }
        for (;;) {
            v.array.push_back(parse_value());
            ws();
            if (i_ >= s_.size()) throw ParseError{"unterminated array"};
            if (s_[i_] == ',') {
                ++i_;
                continue;
            }
            if (s_[i_] == ']') {
                ++i_;
                return v;
            }
            throw ParseError{"expected ',' or ']'"};
        }
    }

    /// 字串："..."，支援 \uXXXX（轉成 UTF-8 位元組）與常見跳脫。
    std::string parse_string() {
        ++i_;  // 吃掉 '"'
        std::string out;
        for (;;) {
            if (i_ >= s_.size()) throw ParseError{"unterminated string"};
            const char c = s_[i_++];
            if (c == '"') return out;        // 字串結束
            if (c != '\\') {
                out += c;
                continue;
            }
            if (i_ >= s_.size()) throw ParseError{"bad escape"};
            const char e = s_[i_++];
            switch (e) {
                case '"': out += '"'; break;
                case '\\': out += '\\'; break;
                case '/': out += '/'; break;
                case 'b': out += '\b'; break;
                case 'f': out += '\f'; break;
                case 'n': out += '\n'; break;
                case 'r': out += '\r'; break;
                case 't': out += '\t'; break;
                case 'u': {   // \uXXXX：4 個 hex 數字 → UTF-8（最多 3 bytes）
                    unsigned cp = 0;
                    for (int k = 0; k < 4; ++k) {
                        if (i_ >= s_.size()) throw ParseError{"bad \\u escape"};
                        const char h = s_[i_++];
                        cp <<= 4;
                        if (h >= '0' && h <= '9') cp |= static_cast<unsigned>(h - '0');
                        else if (h >= 'a' && h <= 'f') cp |= static_cast<unsigned>(h - 'a' + 10);
                        else if (h >= 'A' && h <= 'F') cp |= static_cast<unsigned>(h - 'A' + 10);
                        else throw ParseError{"bad \\u hex digit"};
                    }
                    if (cp < 0x80) {
                        out += static_cast<char>(cp);
                    } else if (cp < 0x800) {
                        out += static_cast<char>(0xC0 | (cp >> 6));
                        out += static_cast<char>(0x80 | (cp & 0x3F));
                    } else {
                        out += static_cast<char>(0xE0 | (cp >> 12));
                        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
                        out += static_cast<char>(0x80 | (cp & 0x3F));
                    }
                    break;
                }
                default: throw ParseError{"bad escape character"};
            }
        }
    }

    /// 數字：接受小數與指數記法（strtod 直接吃掉）。
    /// golden.json 的數值用最短表示法輸出，strtod 保證還原成同一 double。
    Value parse_number() {
        const std::size_t start = i_;
        if (i_ < s_.size() && (s_[i_] == '-' || s_[i_] == '+')) ++i_;
        while (i_ < s_.size() &&
               (std::isdigit(static_cast<unsigned char>(s_[i_])) || s_[i_] == '.' ||
                s_[i_] == 'e' || s_[i_] == 'E' || s_[i_] == '-' || s_[i_] == '+')) {
            ++i_;
        }
        if (i_ == start) throw ParseError{"bad number"};
        const std::string tok = s_.substr(start, i_ - start);
        char* end = nullptr;
        const double val = std::strtod(tok.c_str(), &end);
        if (end == tok.c_str() || *end != '\0') {
            throw ParseError{"bad number: " + tok};
        }
        Value v;
        v.type = Value::Type::Number;
        v.number = val;
        return v;
    }
};

}  // namespace jparse

// ---------------------------------------------------------------------------
// Golden fixture 載入
// ---------------------------------------------------------------------------

/// 讀整個檔案成字串（binary 模式，避免換行轉換）。
std::string read_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot open " + path);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

/// 從物件取 key；缺 key 直接丟例外（harness 資料壞掉要立刻知道）。
const jparse::Value& req(const jparse::Value& j, const char* key) {
    const jparse::Value* v = j.find(key);
    if (!v) throw std::runtime_error(std::string("missing JSON key: ") + key);
    return *v;
}

/// JSON → StaticInfo（geofence 是 [x,y] 陣列的陣列）。
safe_apf::StaticInfo static_from_json(const jparse::Value& j) {
    safe_apf::StaticInfo si;
    si.robot_radius_m = req(j, "robot_radius_m").number;
    si.max_v_mps = req(j, "max_v_mps").number;
    si.max_omega_rad_s = req(j, "max_omega_rad_s").number;
    const jparse::Value& g = req(j, "geofence");
    for (const jparse::Value& pt : g.array) {
        si.geofence.push_back({pt.array.at(0).number, pt.array.at(1).number});
    }
    return si;
}

/// 單一測試 case：輸入（desired + obs）+ 期望輸出（mode/v/omega）。
/// 可選 static override：S1–S7 情境各自有不同 StaticInfo，case 級覆寫。
struct TestCase {
    std::string name;
    safe_apf::Twist desired{0.0, 0.0};
    safe_apf::Observation obs;
    double t = 0.0;
    double dt = 0.05;
    std::string exp_mode;
    double exp_v = 0.0;
    double exp_omega = 0.0;
    safe_apf::StaticInfo static_override;
    bool has_override = false;
};

/// JSON → TestCase。
/// obs 陣列欄位順序（與 gen_golden.py 約定一致）：
///   [x, y, theta, pose_age_s, link_age_s, pose_drift_m, wheel_l, wheel_r]
/// 沒有 pose 的 case 用 "no_pose": true 表示。
TestCase case_from_json(const jparse::Value& j) {
    TestCase tc;
    tc.name = req(j, "name").string;
    const jparse::Value& d = req(j, "desired");
    tc.desired = {d.array.at(0).number, d.array.at(1).number};
    const jparse::Value& o = req(j, "obs");
    if (const jparse::Value* np = j.find("no_pose")) {
        tc.obs.has_pose = !np->boolean;
    } else {
        tc.obs.has_pose = true;
    }
    tc.obs.pose = {o.array.at(0).number, o.array.at(1).number, o.array.at(2).number};
    tc.obs.pose_age_s = o.array.at(3).number;
    tc.obs.link_age_s = o.array.at(4).number;
    tc.obs.pose_drift_m = o.array.at(5).number;
    tc.obs.wheel_l = o.array.at(6).number;
    tc.obs.wheel_r = o.array.at(7).number;
    tc.t = req(j, "t").number;
    tc.dt = req(j, "dt").number;
    const jparse::Value& e = req(j, "expected");
    tc.exp_mode = req(e, "mode").string;
    tc.exp_v = req(e, "v").number;
    tc.exp_omega = req(e, "omega").number;
    if (const jparse::Value* so = j.find("static")) {
        tc.static_override = static_from_json(*so);
        tc.has_override = true;
    }
    return tc;
}

}  // namespace

// ---------------------------------------------------------------------------
// 主程式：重播 golden cases 並比對
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    const std::string path = argc > 1 ? argv[1] : "golden.json";

    // 讀檔
    std::string text;
    try {
        text = read_file(path);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 2;
    }

    // 解析 JSON
    jparse::Value root;
    try {
        jparse::Parser parser(text);
        root = parser.parse();
    } catch (const jparse::ParseError& e) {
        std::fprintf(stderr, "error: JSON parse failed: %s\n", e.msg.c_str());
        return 2;
    }

    // 逐 case 重播 + 比對
    std::size_t passed = 0, failed = 0;
    try {
        const safe_apf::StaticInfo base_static = static_from_json(req(root, "static"));
        const jparse::Value& cases = req(root, "cases");
        safe_apf::SafeApfFilter filter;
        for (const jparse::Value& cj : cases.array) {
            const TestCase tc = case_from_json(cj);
            const safe_apf::StaticInfo& si = tc.has_override ? tc.static_override : base_static;
            filter.reset(si);
            const safe_apf::SafetyDecision dec = filter.filter(tc.desired, tc.obs, tc.t, tc.dt);
            // mode 精確字串比對；v/omega 絕對容差 1e-9
            const bool ok = dec.mode == tc.exp_mode &&
                            std::abs(dec.cmd.v - tc.exp_v) <= 1e-9 &&
                            std::abs(dec.cmd.omega - tc.exp_omega) <= 1e-9;
            if (ok) {
                ++passed;
                std::printf("PASS  %s\n", tc.name.c_str());
            } else {
                ++failed;
                std::printf("FAIL  %s\n", tc.name.c_str());
                std::printf("      mode:   got %s expected %s\n",
                            dec.mode.c_str(), tc.exp_mode.c_str());
                std::printf("      v:      got %.17g expected %.17g\n",
                            dec.cmd.v, tc.exp_v);
                std::printf("      omega: got %.17g expected %.17g\n",
                            dec.cmd.omega, tc.exp_omega);
            }
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 2;
    }

    // 全部通過 → 0；任一失敗 → 1
    std::printf("%zu/%zu cases passed\n", passed, passed + failed);
    return failed == 0 ? 0 : 1;
}
