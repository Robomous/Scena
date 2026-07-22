// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#include "support/trace_recorder.h"

#include <bit>
#include <cmath>
#include <fstream>
#include <optional>

namespace scena::testsupport {

std::string hex_bits(double value) {
    const auto bits = std::bit_cast<std::uint64_t>(value);
    static constexpr char kDigits[] = "0123456789abcdef";
    std::string out(16, '0');
    for (int i = 0; i < 16; ++i) {
        out[static_cast<std::size_t>(i)] = kDigits[(bits >> ((15 - i) * 4)) & 0xF];
    }
    return out;
}

namespace {

std::string int_to_string(int value) {
    // A tiny, locale-immune int formatter (std::to_string is locale-immune for
    // integers, but keeping every field hand-built keeps the format audit
    // trivial and platform-independent).
    if (value == 0) {
        return "0";
    }
    const bool negative = value < 0;
    // Accumulate into a wider type so negating INT_MIN is well-defined.
    long long v = negative ? -static_cast<long long>(value) : value;
    std::string digits;
    while (v > 0) {
        digits.push_back(static_cast<char>('0' + (v % 10)));
        v /= 10;
    }
    if (negative) {
        digits.push_back('-');
    }
    std::string out(digits.rbegin(), digits.rend());
    return out;
}

} // namespace

void TraceRecorder::header(const std::string& fixture_name) {
    text_ += "# scena-trace v1 fixture=";
    text_ += fixture_name;
    text_ += '\n';
}

void TraceRecorder::note(const std::string& line) {
    text_ += "# ";
    text_ += line;
    text_ += '\n';
}

void TraceRecorder::record_step(int step, const Engine& engine,
                                const std::vector<std::string>& entity_ids) {
    const std::string step_str = int_to_string(step);
    text_ += "t ";
    text_ += step_str;
    text_ += ' ';
    text_ += hex_bits(engine.time());
    text_ += '\n';

    for (const std::string& id : entity_ids) {
        text_ += "e ";
        text_ += step_str;
        text_ += ' ';
        text_ += id;
        const std::optional<EntityState> state = engine.state(id);
        if (!state.has_value()) {
            text_ += " MISSING\n";
            continue;
        }
        text_ += ' ';
        text_ += hex_bits(state->x);
        text_ += ' ';
        text_ += hex_bits(state->y);
        text_ += ' ';
        text_ += hex_bits(state->z);
        text_ += ' ';
        text_ += hex_bits(state->heading);
        text_ += ' ';
        text_ += hex_bits(state->speed);
        text_ += '\n';
    }
}

void TraceRecorder::record_value(const std::string& tag, double input, double output) {
    text_ += "m ";
    text_ += tag;
    text_ += ' ';
    text_ += hex_bits(input);
    text_ += ' ';
    // NaN bits are out of contract (payload may differ across platforms), so a
    // NaN output is normalized to the literal `nan` rather than its bit image.
    text_ += std::isnan(output) ? "nan" : hex_bits(output);
    text_ += '\n';
}

bool TraceRecorder::write_file(const std::filesystem::path& path) const {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        return false;
    }
    out.write(text_.data(), static_cast<std::streamsize>(text_.size()));
    return static_cast<bool>(out);
}

} // namespace scena::testsupport
