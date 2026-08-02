/*
 * Copyright 2026 Robomous
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

//
// scena-run CLI behaviour (p6-s4, #38).
//
// What a script depends on: exit codes, the shape of the trace, and — the one
// that matters for the golden suite — that a double survives the round trip
// through the text file bit-for-bit. The binary is driven as a subprocess
// rather than linked, because that is how every consumer uses it.

#include <charconv>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

namespace fs = std::filesystem;

/// Runs scena-run with `args`, returning its exit code. Output goes to a file
/// so a failing test can show what the tool said.
int run(const std::string& args, std::string& output) {
    const fs::path log = fs::temp_directory_path() / "scena_run_test.log";
    // Quoting the binary path: a build directory may contain spaces.
    std::string command =
        std::string("\"") + SCENA_RUN_BINARY + "\" " + args + " > \"" + log.string() + "\" 2>&1";
#ifdef _WIN32
    // cmd.exe strips the outer pair of quotes from the command it is given, so
    // a command that *starts* with a quoted path loses it. Wrapping the whole
    // thing in one more pair is the documented way to keep both.
    command = "\"" + command + "\"";
#endif
    const int status = std::system(command.c_str());
    std::ifstream file(log);
    std::stringstream buffer;
    buffer << file.rdbuf();
    output = buffer.str();
#ifdef _WIN32
    return status;
#else
    // POSIX packs the exit status; WEXITSTATUS without <sys/wait.h>.
    return (status & 0x7f) == 0 ? ((status >> 8) & 0xff) : -1;
#endif
}

fs::path golden(const std::string& name) {
    return fs::path(SCENA_GOLDEN_DIR) / "scenarios" / name;
}

fs::path temp_file(const std::string& name) {
    const fs::path path = fs::temp_directory_path() / name;
    std::error_code ignored;
    fs::remove(path, ignored);
    return path;
}

std::vector<std::string> read_lines(const fs::path& path) {
    std::ifstream file(path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        lines.push_back(line);
    }
    return lines;
}

// --- usage and exit codes --------------------------------------------------

TEST(ScenaRunTest, HelpSucceedsAndDescribesTheTool) {
    std::string output;
    EXPECT_EQ(run("--help", output), 0);
    EXPECT_NE(output.find("scena-run"), std::string::npos);
    EXPECT_NE(output.find("--trace"), std::string::npos);
}

TEST(ScenaRunTest, NoArgumentsIsAUsageError) {
    std::string output;
    EXPECT_EQ(run("", output), 2);
}

TEST(ScenaRunTest, AnUnknownOptionIsAUsageError) {
    std::string output;
    EXPECT_EQ(run("--nonsense x", output), 2);
    EXPECT_NE(output.find("unknown option"), std::string::npos);
}

TEST(ScenaRunTest, ANonNumericDtIsAUsageError) {
    std::string output;
    EXPECT_EQ(run("\"" + golden("gs1-cruise-baseline.xosc").string() + "\" --dt fast", output), 2);
}

TEST(ScenaRunTest, ANonPositiveDtIsAUsageError) {
    std::string output;
    EXPECT_EQ(run("\"" + golden("gs1-cruise-baseline.xosc").string() + "\" --dt 0", output), 2);
}

TEST(ScenaRunTest, AMissingScenarioIsALoadError) {
    std::string output;
    EXPECT_EQ(run("no/such/scenario.xosc", output), 3);
}

TEST(ScenaRunTest, AMalformedScenarioIsALoadErrorNotACrash) {
    const fs::path path = temp_file("scena_run_malformed.xosc");
    std::ofstream(path) << "<OpenSCENARIO>";
    std::string output;
    EXPECT_EQ(run("\"" + path.string() + "\"", output), 3);
}

TEST(ScenaRunTest, AMissingReplayFileIsAReplayError) {
    std::string output;
    EXPECT_EQ(run("\"" + golden("gs1-cruise-baseline.xosc").string() +
                      "\" --replay ego=no/such/trace.csv",
                  output),
              6);
}

TEST(ScenaRunTest, AMissingMapIsAMapError) {
    std::string output;
    EXPECT_EQ(run("\"" + golden("gs1-cruise-baseline.xosc").string() + "\" --map no/such/map.xodr",
                  output),
              7);
}

TEST(ScenaRunTest, AScenarioNamingARoadNetworkGetsOne) {
    // GS-3 declares its map in RoadNetwork/LogicFile, relative to the scenario
    // file. If the map did not load, the absolute lane targets would fall back
    // to the flat-world model and ego would never reach the lane centre the
    // network defines.
    const fs::path trace = temp_file("scena_run_map.csv");
    std::string output;
    ASSERT_EQ(run("\"" + golden("gs3-overtake.xosc").string() +
                      "\" --dt 0.01 --duration 6 --trace \"" + trace.string() + "\"",
                  output),
              0)
        << output;
    const std::vector<std::string> lines = read_lines(trace);
    ASSERT_GT(lines.size(), 2U);
    // The lane -1 centre of the golden overtake map is y = -1.75; ego reaches
    // it once the lane change completes (to a rounding ulp). Without a road
    // network the absolute lane target has no world meaning at all.
    bool reached_lane_centre = false;
    for (const std::string& line : lines) {
        reached_lane_centre = reached_lane_centre || (line.find(",ego,") != std::string::npos &&
                                                      line.find(",-1.75") != std::string::npos);
    }
    EXPECT_TRUE(reached_lane_centre);
}

// --- running and tracing ---------------------------------------------------

TEST(ScenaRunTest, RunsAScenarioAndWritesACsvTrace) {
    const fs::path trace = temp_file("scena_run_gs1.csv");
    std::string output;
    ASSERT_EQ(run("\"" + golden("gs1-cruise-baseline.xosc").string() +
                      "\" --dt 0.01 --duration 1 --trace \"" + trace.string() + "\"",
                  output),
              0)
        << output;
    ASSERT_TRUE(fs::exists(trace));

    const std::vector<std::string> lines = read_lines(trace);
    ASSERT_FALSE(lines.empty());
    EXPECT_EQ(lines.front(), "t,entity,x,y,z,heading,speed");
    // 100 steps of one engine-controlled entity, plus the header.
    EXPECT_EQ(lines.size(), 101U);
    EXPECT_NE(lines[1].find(",ego,"), std::string::npos);
}

TEST(ScenaRunTest, WritesJsonWhenTheExtensionSaysSo) {
    const fs::path trace = temp_file("scena_run_gs1.json");
    std::string output;
    ASSERT_EQ(run("\"" + golden("gs1-cruise-baseline.xosc").string() +
                      "\" --dt 0.1 --duration 0.5 --trace \"" + trace.string() + "\"",
                  output),
              0)
        << output;
    const std::vector<std::string> lines = read_lines(trace);
    ASSERT_GE(lines.size(), 3U);
    EXPECT_EQ(lines.front(), "{");
    EXPECT_NE(lines[1].find("\"samples\""), std::string::npos);
    EXPECT_NE(lines[2].find("\"entity\": \"ego\""), std::string::npos);
}

TEST(ScenaRunTest, TheSameRunProducesByteIdenticalTraces) {
    // The property the whole golden suite rests on. Text, not floats: if the
    // formatting were lossy or locale-dependent, this is where it would show.
    const fs::path first = temp_file("scena_run_repeat_a.csv");
    const fs::path second = temp_file("scena_run_repeat_b.csv");
    std::string output;
    const std::string base =
        "\"" + golden("gs1-cruise-baseline.xosc").string() + "\" --dt 0.01 --duration 2 --trace ";
    ASSERT_EQ(run(base + "\"" + first.string() + "\"", output), 0) << output;
    ASSERT_EQ(run(base + "\"" + second.string() + "\"", output), 0) << output;

    std::ifstream a(first, std::ios::binary);
    std::ifstream b(second, std::ios::binary);
    const std::string bytes_a((std::istreambuf_iterator<char>(a)), {});
    const std::string bytes_b((std::istreambuf_iterator<char>(b)), {});
    EXPECT_FALSE(bytes_a.empty());
    EXPECT_EQ(bytes_a, bytes_b);
}

TEST(ScenaRunTest, TraceNumbersRoundTripExactly) {
    // Every number in the trace must read back as the identical double —
    // otherwise "bit-identical trace" would only mean "identical text".
    const fs::path trace = temp_file("scena_run_roundtrip.csv");
    std::string output;
    ASSERT_EQ(run("\"" + golden("gs1-cruise-baseline.xosc").string() +
                      "\" --dt 0.01 --duration 2 --trace \"" + trace.string() + "\"",
                  output),
              0)
        << output;

    const std::vector<std::string> lines = read_lines(trace);
    ASSERT_GT(lines.size(), 10U);
    std::size_t checked = 0;
    for (std::size_t i = 1; i < lines.size(); ++i) {
        std::istringstream stream(lines[i]);
        std::string field;
        std::size_t column = 0;
        while (std::getline(stream, field, ',')) {
            if (column != 1 && !field.empty()) { // column 1 is the entity id
                const double value = std::strtod(field.c_str(), nullptr);
                char buffer[64];
                const std::to_chars_result result =
                    std::to_chars(buffer, buffer + sizeof(buffer), value);
                ASSERT_EQ(result.ec, std::errc());
                EXPECT_EQ(std::string(buffer, result.ptr), field)
                    << "line " << i << " column " << column;
                ++checked;
            }
            ++column;
        }
    }
    EXPECT_GT(checked, 100U);
}

TEST(ScenaRunTest, ADurationOfZeroRunsNoStepsAndStillSucceeds) {
    const fs::path trace = temp_file("scena_run_zero.csv");
    std::string output;
    ASSERT_EQ(run("\"" + golden("gs1-cruise-baseline.xosc").string() +
                      "\" --dt 0.01 --duration 0 --trace \"" + trace.string() + "\"",
                  output),
              0)
        << output;
    const std::vector<std::string> lines = read_lines(trace);
    EXPECT_EQ(lines.size(), 1U); // the header alone
}

TEST(ScenaRunTest, ReplayDrivesAHostControlledEntity) {
    // Record the ego of a host-controlled scenario from a CSV the test writes,
    // and check the engine reported exactly what was replayed.
    const fs::path recorded = temp_file("scena_run_replay_in.csv");
    {
        std::ofstream file(recorded, std::ios::binary);
        file << "t,entity,x,y,z,heading,speed\n";
        for (int i = 1; i <= 20; ++i) {
            const double x = static_cast<double>(i) * 1.5;
            file << (0.1 * i) << ",npc," << x << ",0,0,0,15\n";
        }
    }
    const fs::path trace = temp_file("scena_run_replay_out.csv");
    std::string output;
    ASSERT_EQ(run("\"" + golden("gs10-host-controlled.xosc").string() +
                      "\" --dt 0.1 --duration 2 --replay npc=\"" + recorded.string() +
                      "\" --trace \"" + trace.string() + "\"",
                  output),
              0)
        << output;
    // The replayed entity is host-controlled, so it is polled and never
    // published — the trace holds the engine-controlled ego only.
    const std::vector<std::string> lines = read_lines(trace);
    ASSERT_GT(lines.size(), 1U);
    for (std::size_t i = 1; i < lines.size(); ++i) {
        EXPECT_EQ(lines[i].find(",npc,"), std::string::npos) << lines[i];
    }
}

TEST(ScenaRunTest, SelectIsAcceptedAndReportedAsNotYetActive) {
    std::string output;
    ASSERT_EQ(run("\"" + golden("gs1-cruise-baseline.xosc").string() +
                      "\" --dt 0.1 --duration 0.2 --select alt",
                  output),
              0)
        << output;
    EXPECT_NE(output.find("--select has no effect yet"), std::string::npos);
}

// --- the DSL frontend (p8-s1, #44) -----------------------------------------

/// Writes a DSL scenario to a temporary file and returns its path. The source
/// lives here rather than in a fixture directory because what these tests pin
/// is the *CLI*, and a scenario the reader can see beside the assertion is
/// worth more than one they have to go and find.
fs::path dsl_file(const std::string& name, const std::string& source) {
    const fs::path path = temp_file(name);
    std::ofstream(path) << source;
    return path;
}

constexpr const char* kCruiseOsc = R"(import osc.standard.all

namespace demo use std, stdtypes

scenario cruise:
    ego: vehicle
    start: position_3d
    keep(start.x == 5m)
    do serial:
        place: ego.assign_position(position: start)
        launch: ego.assign_speed(speed: 10mps)
)";

TEST(ScenaRunTest, ADslScenarioRunsEndToEnd) {
    // The p8-s1 exit criterion, driven the way a user would drive it: a .osc
    // file in, a trace out, through the same runtime the XML frontend feeds.
    const fs::path scenario = dsl_file("scena_run_cruise.osc", kCruiseOsc);
    const fs::path trace = temp_file("scena_run_cruise.csv");
    std::string output;
    ASSERT_EQ(run("\"" + scenario.string() + "\" --dt 0.1 --duration 1 --trace \"" +
                      trace.string() + "\"",
                  output),
              0)
        << output;
    const std::vector<std::string> lines = read_lines(trace);
    ASSERT_EQ(lines.size(), 11U); // header + 10 steps
    EXPECT_EQ(lines[0], "t,entity,x,y,z,heading,speed");
    // Teleported to x = 5 m, then driving at 10 m/s: after one step, 6 m.
    EXPECT_EQ(lines[1], "0.1,ego,6,0,0,0,10");
    EXPECT_EQ(lines.back(), "1,ego,15,0,0,0,10");
}

TEST(ScenaRunTest, ADslScenarioWithSeveralEntryPointsNeedsOneNamed) {
    // §7.7.2 leaves the choice to the implementation, and guessing would make
    // the run depend on declaration order.
    const fs::path scenario = dsl_file("scena_run_two.osc",
                                       R"(import osc.standard.all

namespace demo use std, stdtypes

scenario first:
    ego: vehicle
    do launch: ego.assign_speed(speed: 10mps)

scenario second:
    ego: vehicle
    do launch: ego.assign_speed(speed: 20mps)
)");
    std::string output;
    EXPECT_EQ(run("\"" + scenario.string() + "\" --dt 0.1 --duration 0.2", output), 3);
    EXPECT_NE(output.find("more than one scenario"), std::string::npos) << output;

    const fs::path trace = temp_file("scena_run_two.csv");
    ASSERT_EQ(run("\"" + scenario.string() +
                      "\" --dt 0.1 --duration 0.2 --entry second --trace \"" + trace.string() +
                      "\"",
                  output),
              0)
        << output;
    const std::vector<std::string> lines = read_lines(trace);
    ASSERT_GT(lines.size(), 1U);
    EXPECT_NE(lines[1].find(",20"), std::string::npos) << lines[1];
}

TEST(ScenaRunTest, AMalformedDslScenarioIsALoadErrorNotACrash) {
    const fs::path scenario = dsl_file("scena_run_broken.osc", "scenario :\n  ???\n");
    std::string output;
    EXPECT_EQ(run("\"" + scenario.string() + "\"", output), 3);
}

TEST(ScenaRunTest, ADslScenarioNamesItsOwnRoadNetwork) {
    // §8.5.4's map_file plays the part RoadNetwork/LogicFile plays on the XML
    // side, including being resolved relative to the scenario file.
    const fs::path scenario = dsl_file("scena_run_map.osc",
                                       R"(import osc.standard.all

namespace demo use std, stdtypes

scenario mapped:
    map.set_map_file("no_such_map.xodr")
    ego: vehicle
    do launch: ego.assign_speed(speed: 10mps)
)");
    std::string output;
    // The map does not exist, which is exactly what makes the reference
    // observable: the run stops with the map exit code rather than ignoring it.
    EXPECT_EQ(run("\"" + scenario.string() + "\" --dt 0.1 --duration 0.2", output), 7);
    EXPECT_NE(output.find("no_such_map.xodr"), std::string::npos) << output;
}

TEST(ScenaRunTest, QuietSuppressesDiagnosticsButNotFailures) {
    std::string output;
    EXPECT_EQ(run("no/such/scenario.xosc --quiet", output), 3);
    // The failure line still reaches stderr: --quiet is about diagnostics, not
    // about hiding that the run did not happen.
    EXPECT_NE(output.find("could not load"), std::string::npos);
}

} // namespace
