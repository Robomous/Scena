// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
//
// scena-trace-runner: emits a "scena-trace v1" image of a fixed detmath +
// engine workload to a file. Two runs on different platforms must produce
// byte-identical output iff the runtime is bit-identical — that comparison is
// the determinism-cross CI job. This is a plain executable (not a gtest, not
// registered via scn_add_test) so gtest_discover_tests never picks it up.
//
// Usage: scena-trace-runner <output-file>
//   argc != 2               -> usage on stderr, exit 2
//   any non-Ok engine status or write failure -> message on stderr, exit 1

#include <iostream>
#include <string>
#include <vector>

#include "scena/engine.h"
#include "scena/runtime/detmath.h"
#include "scena/status.h"
#include "support/detmath_probe.h"
#include "support/fixtures.h"
#include "support/trace_recorder.h"

namespace {

using scena::Engine;
using scena::Status;
using scena::testsupport::make_determinism_scenario;
using scena::testsupport::TraceRecorder;

const std::vector<std::string>& entity_ids() {
    static const std::vector<std::string> kIds = {"ego", "lead"};
    return kIds;
}

// (1) detmath table: sin and cos of every probe input.
void record_detmath_table(TraceRecorder& trace) {
    trace.note("detmath sin/cos over the probe set");
    for (const double x : scena::testsupport::detmath_probe_inputs()) {
        trace.record_value("sin", x, scena::runtime::det_sin(x));
        trace.record_value("cos", x, scena::runtime::det_cos(x));
    }
}

// (2) fixture run A: 1000 steps alternating dt 0.01/0.02, every step recorded.
bool record_fixture_short(TraceRecorder& trace) {
    Engine engine;
    if (engine.init(make_determinism_scenario()) != Status::Ok) {
        return false;
    }
    trace.note("fixture run A: 1000 steps, dt alternating 0.01/0.02");
    for (int i = 0; i < 1000; ++i) {
        const double dt = (i % 2 == 0) ? 0.01 : 0.02;
        if (engine.step(dt) != Status::Ok) {
            return false;
        }
        trace.record_step(i, engine, entity_ids());
    }
    return true;
}

// (3) fixture run B: fresh engine, 120k varying-dt steps, every 1000th + final.
bool record_fixture_long(TraceRecorder& trace) {
    Engine engine;
    if (engine.init(make_determinism_scenario()) != Status::Ok) {
        return false;
    }
    trace.note("fixture run B: 120000 steps, dt = 0.001 + 0.0005*(i%7)");
    constexpr int kSteps = 120000;
    for (int i = 0; i < kSteps; ++i) {
        const double dt = 0.001 + 0.0005 * (i % 7);
        if (engine.step(dt) != Status::Ok) {
            return false;
        }
        if (i % 1000 == 0 || i == kSteps - 1) {
            trace.record_step(i, engine, entity_ids());
        }
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: scena-trace-runner <output-file>\n";
        return 2;
    }

    TraceRecorder trace;
    trace.header("determinism-test");
    record_detmath_table(trace);
    if (!record_fixture_short(trace) || !record_fixture_long(trace)) {
        std::cerr << "scena-trace-runner: engine returned a non-Ok status\n";
        return 1;
    }

    if (!trace.write_file(argv[1])) {
        std::cerr << "scena-trace-runner: failed to write " << argv[1] << '\n';
        return 1;
    }
    return 0;
}
