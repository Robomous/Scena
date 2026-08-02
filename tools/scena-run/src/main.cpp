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
// scena-run — execute a scenario headlessly and emit a state trace.
//
// The validation vehicle for the release gate, and deliberately a *thin*
// consumer: it uses only the public API (the loader, Engine, and the gateway
// interface) and contains no scenario semantics of its own. If scena-run needs
// to know something about a scenario to run it, that knowledge belongs in the
// library, not here.
//
// No visualization, by design (ADR-0001, library-first).

#include <charconv>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "scena/diagnostic.h"
#include "scena/dsl/load.h"
#include "scena/dsl/lower.h"
#include "scena/dsl/types.h"
#include "scena/engine.h"
#include "scena/gateway/simulator_gateway.h"
#include "scena/ir/entity.h"
#include "scena/opendrive/reader.h"
#include "scena/opendrive/road_query.h"
#include "scena/xml/loader.h"

namespace {

/// Exit codes. 0 is a completed run; everything else is a distinct failure a
/// script can branch on, which is why they are not all 1.
enum ExitCode : int {
    kOk = 0,
    kUsage = 2,        ///< bad command line
    kLoadFailed = 3,   ///< the scenario did not parse or did not validate
    kRunFailed = 4,    ///< a step returned a non-Ok status
    kOutputFailed = 5, ///< the trace could not be written
    kReplayFailed = 6, ///< a --replay file was unreadable or malformed
    kMapFailed = 7,    ///< the road network did not load
};

struct Options {
    std::filesystem::path scenario;
    double dt = 0.01;
    double duration = 10.0;
    std::filesystem::path trace;
    std::string trace_format; ///< "csv", "json", or empty ⇒ infer from the extension
    /// entity id -> replay file, from --replay <entity>=<csv>.
    std::map<std::string, std::filesystem::path> replay;
    std::string select; ///< --select <alt>, the one_of alternative (p8-s2)
    /// --entry <scenario>, the DSL entry point (§7.7.2). Empty means the only
    /// scenario the file declares; a file with several is reported, not guessed.
    std::string entry;
    /// -I/--search-path, where a DSL module reference is looked up (§7.7.5.1.2).
    std::vector<std::filesystem::path> search_paths;
    /// --map <file.xodr>, overriding the scenario's own RoadNetwork/LogicFile.
    std::filesystem::path map;
    bool quiet = false;
};

void print_usage() {
    std::cout << R"(scena-run — execute a scenario and emit a state trace

usage: scena-run <scenario> [options]

  <scenario>                 an OpenSCENARIO XML file (.xosc) or DSL file (.osc)

options:
  --dt <seconds>             fixed step size (default 0.01)
  --duration <seconds>       how long to run (default 10)
  --trace <file>             write the trace to <file> (.csv or .json)
  --trace-format <csv|json>  override the format inferred from the extension
  --map <file.xodr>          road network, overriding the scenario's LogicFile
  --replay <entity>=<file>   drive a host-controlled entity from a trace file
  --select <alternative>     choose a one_of alternative by label (.osc only)
  --entry <scenario>         the DSL scenario to run (.osc only)
  -I, --search-path <dir>    where DSL imports are looked up (repeatable)
  --quiet                    do not print diagnostics to stderr
  -h, --help                 this text

exit codes:
  0 ok   2 usage   3 load/validation   4 run   5 output   6 replay   7 map
)";
}

/// Parses a double the same way the rest of Scena does: std::from_chars, never
/// strtod — the locale must not decide what "1.5" means.
bool parse_double(std::string_view text, double& out) {
    const char* const begin = text.data();
    const char* const end = begin + text.size();
    const std::from_chars_result parsed = std::from_chars(begin, end, out);
    return parsed.ec == std::errc() && parsed.ptr == end;
}

/// Formats a double so that reading it back yields the identical bit pattern.
///
/// The golden suite compares traces as text, so "bit-identical" has to survive
/// the round trip through the file. std::to_chars' shortest round-trip form is
/// exactly that guarantee, and — unlike printf — it is locale-independent, so
/// the same run writes the same bytes on every machine.
std::string format_double(double value) {
    char buffer[64];
    const std::to_chars_result result = std::to_chars(buffer, buffer + sizeof(buffer), value);
    if (result.ec != std::errc()) {
        return "nan";
    }
    return std::string(buffer, result.ptr);
}

/// One recorded sample.
struct Sample {
    double t = 0.0;
    std::string entity;
    scena::EntityState state;
};

/// Replays recorded states into host-controlled entities, and records every
/// entity's state each step.
///
/// Both directions live in one gateway because both are the same seam: this is
/// what an embedder writes, and scena-run is meant to be readable as an example
/// of one.
class TraceGateway final : public scena::gateway::ISimulatorGateway {
public:
    std::vector<Sample> samples;

    void set_replay(std::string entity, std::vector<Sample> rows) {
        replay_[std::move(entity)] = {std::move(rows), 0};
    }

    void set_time(double t) { now_ = t; }

    /// Takes ownership of the road network the engine will query. Set before
    /// init; the engine holds the pointer for the whole run.
    void set_road_query(std::unique_ptr<scena::opendrive::OpenDriveRoadQuery> road) {
        road_ = std::move(road);
    }

    void publish_state(const std::string& entity_id, const scena::EntityState& state) override {
        samples.push_back(Sample{now_, entity_id, state});
    }

    bool poll_state(const std::string& entity_id, scena::EntityState& out) override {
        const auto it = replay_.find(entity_id);
        if (it == replay_.end()) {
            return false;
        }
        Replay& replay = it->second;
        if (replay.cursor >= replay.rows.size()) {
            return false; // ran out of recorded states: hold the last one
        }
        out = replay.rows[replay.cursor].state;
        ++replay.cursor;
        return true;
    }

    scena::gateway::IRoadQuery* road_query() override { return road_.get(); }

private:
    struct Replay {
        std::vector<Sample> rows;
        std::size_t cursor = 0;
    };
    std::map<std::string, Replay> replay_;
    std::unique_ptr<scena::opendrive::OpenDriveRoadQuery> road_;
    double now_ = 0.0;
};

const char* severity_name(scena::Severity severity) {
    switch (severity) {
    case scena::Severity::Info:
        return "note";
    case scena::Severity::Warning:
        return "warning";
    case scena::Severity::Error:
        return "error";
    }
    return "note";
}

void print_diagnostics(const std::vector<scena::Diagnostic>& diagnostics, bool quiet) {
    if (quiet) {
        return;
    }
    for (const scena::Diagnostic& diagnostic : diagnostics) {
        std::cerr << severity_name(diagnostic.severity) << ": ";
        if (!diagnostic.location.file.empty()) {
            std::cerr << diagnostic.location.file;
            if (diagnostic.location.line > 0) {
                std::cerr << ':' << diagnostic.location.line << ':' << diagnostic.location.column;
            }
            std::cerr << ": ";
        }
        if (!diagnostic.path.empty()) {
            std::cerr << diagnostic.path << ": ";
        }
        std::cerr << diagnostic.message;
        if (!diagnostic.rule_id.empty()) {
            std::cerr << " [" << diagnostic.rule_id << ']';
        }
        std::cerr << '\n';
    }
}

/// Reads a trace CSV back into samples, for --replay. Accepts the format
/// scena-run writes: a header line, then t,entity,x,y,z,heading,speed.
bool read_trace_csv(const std::filesystem::path& path, std::vector<Sample>& out,
                    const std::string& entity) {
    std::ifstream file(path);
    if (!file) {
        return false;
    }
    std::string line;
    bool first = true;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back(); // a CRLF file written on Windows, read anywhere
        }
        if (line.empty()) {
            continue;
        }
        if (first) {
            first = false;
            if (line.rfind("t,", 0) == 0) {
                continue; // header
            }
        }
        std::vector<std::string> fields;
        std::istringstream stream(line);
        std::string field;
        while (std::getline(stream, field, ',')) {
            fields.push_back(field);
        }
        if (fields.size() < 7) {
            return false;
        }
        if (fields[1] != entity) {
            continue; // a multi-entity trace: take only this entity's rows
        }
        Sample sample;
        if (!parse_double(fields[0], sample.t) || !parse_double(fields[2], sample.state.x) ||
            !parse_double(fields[3], sample.state.y) || !parse_double(fields[4], sample.state.z) ||
            !parse_double(fields[5], sample.state.heading) ||
            !parse_double(fields[6], sample.state.speed)) {
            return false;
        }
        sample.entity = entity;
        out.push_back(sample);
    }
    return true;
}

bool write_csv(const std::filesystem::path& path, const std::vector<Sample>& samples) {
    std::ofstream file(path, std::ios::binary); // binary: LF everywhere, so the
                                                // bytes match across platforms
    if (!file) {
        return false;
    }
    file << "t,entity,x,y,z,heading,speed\n";
    for (const Sample& sample : samples) {
        file << format_double(sample.t) << ',' << sample.entity << ','
             << format_double(sample.state.x) << ',' << format_double(sample.state.y) << ','
             << format_double(sample.state.z) << ',' << format_double(sample.state.heading) << ','
             << format_double(sample.state.speed) << '\n';
    }
    return file.good();
}

bool write_json(const std::filesystem::path& path, const std::vector<Sample>& samples) {
    std::ofstream file(path, std::ios::binary);
    if (!file) {
        return false;
    }
    file << "{\n  \"samples\": [\n";
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const Sample& sample = samples[i];
        file << "    {\"t\": " << format_double(sample.t) << ", \"entity\": \"" << sample.entity
             << "\", \"x\": " << format_double(sample.state.x)
             << ", \"y\": " << format_double(sample.state.y)
             << ", \"z\": " << format_double(sample.state.z)
             << ", \"heading\": " << format_double(sample.state.heading)
             << ", \"speed\": " << format_double(sample.state.speed) << "}"
             << (i + 1 < samples.size() ? "," : "") << '\n';
    }
    file << "  ]\n}\n";
    return file.good();
}

/// Parses the command line. Returns std::nullopt and prints why on a bad one.
std::optional<Options> parse_options(int argc, char** argv, int& exit_code) {
    Options options;
    exit_code = kOk;
    std::vector<std::string_view> args(argv + 1, argv + argc);

    const auto value_of = [&args](std::size_t& i, std::string_view name) -> std::string_view {
        if (i + 1 >= args.size()) {
            std::cerr << "scena-run: " << name << " needs a value\n";
            return {};
        }
        return args[++i];
    };

    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string_view arg = args[i];
        if (arg == "-h" || arg == "--help") {
            print_usage();
            exit_code = kOk;
            return std::nullopt;
        }
        if (arg == "--quiet") {
            options.quiet = true;
        } else if (arg == "--dt" || arg == "--duration") {
            const std::string_view text = value_of(i, arg);
            double parsed = 0.0;
            if (text.empty() || !parse_double(text, parsed)) {
                std::cerr << "scena-run: " << arg << " needs a number\n";
                exit_code = kUsage;
                return std::nullopt;
            }
            (arg == "--dt" ? options.dt : options.duration) = parsed;
        } else if (arg == "--trace") {
            const std::string_view text = value_of(i, arg);
            if (text.empty()) {
                exit_code = kUsage;
                return std::nullopt;
            }
            options.trace = std::filesystem::path(std::string(text));
        } else if (arg == "--trace-format") {
            const std::string_view text = value_of(i, arg);
            if (text != "csv" && text != "json") {
                std::cerr << "scena-run: --trace-format must be csv or json\n";
                exit_code = kUsage;
                return std::nullopt;
            }
            options.trace_format = std::string(text);
        } else if (arg == "--map") {
            const std::string_view text = value_of(i, arg);
            if (text.empty()) {
                exit_code = kUsage;
                return std::nullopt;
            }
            options.map = std::filesystem::path(std::string(text));
        } else if (arg == "--entry") {
            const std::string_view text = value_of(i, arg);
            if (text.empty()) {
                exit_code = kUsage;
                return std::nullopt;
            }
            options.entry = std::string(text);
        } else if (arg == "-I" || arg == "--search-path") {
            const std::string_view text = value_of(i, arg);
            if (text.empty()) {
                exit_code = kUsage;
                return std::nullopt;
            }
            options.search_paths.emplace_back(std::string(text));
        } else if (arg == "--select") {
            const std::string_view text = value_of(i, arg);
            if (text.empty()) {
                exit_code = kUsage;
                return std::nullopt;
            }
            options.select = std::string(text);
        } else if (arg == "--replay") {
            const std::string_view text = value_of(i, arg);
            const std::size_t equals = text.find('=');
            if (equals == std::string_view::npos || equals == 0) {
                std::cerr << "scena-run: --replay needs <entity>=<file>\n";
                exit_code = kUsage;
                return std::nullopt;
            }
            options.replay[std::string(text.substr(0, equals))] =
                std::filesystem::path(std::string(text.substr(equals + 1)));
        } else if (!arg.empty() && arg.front() == '-') {
            std::cerr << "scena-run: unknown option '" << arg << "'\n";
            exit_code = kUsage;
            return std::nullopt;
        } else if (options.scenario.empty()) {
            options.scenario = std::filesystem::path(std::string(arg));
        } else {
            std::cerr << "scena-run: more than one scenario given\n";
            exit_code = kUsage;
            return std::nullopt;
        }
    }

    if (options.scenario.empty()) {
        print_usage();
        exit_code = kUsage;
        return std::nullopt;
    }
    if (!(options.dt > 0.0)) {
        std::cerr << "scena-run: --dt must be positive\n";
        exit_code = kUsage;
        return std::nullopt;
    }
    if (!(options.duration >= 0.0)) {
        std::cerr << "scena-run: --duration must not be negative\n";
        exit_code = kUsage;
        return std::nullopt;
    }
    return options;
}

} // namespace

int main(int argc, char** argv) {
    int exit_code = kOk;
    const std::optional<Options> parsed = parse_options(argc, argv, exit_code);
    if (!parsed.has_value()) {
        return exit_code;
    }
    const Options& options = *parsed;

    if (!options.select.empty() && options.scenario.extension() != ".osc") {
        // Accepted and reported rather than silently ignored: `one_of` is a DSL
        // construct, so the option means nothing to an XML scenario.
        if (!options.quiet) {
            std::cerr << "warning: --select names a one_of alternative, which only an "
                         "OpenSCENARIO DSL scenario has\n";
        }
    }

    // Two frontends, one runtime: the extension picks which one compiles the
    // file, and everything past this point sees only the Scenario IR. The
    // road-network reference travels beside it in both cases, because a file
    // path is a host input rather than kernel state (ADR-0003).
    scena::DiagnosticSink load_sink;
    scena::ir::Scenario scenario;
    std::string road_network;
    if (options.scenario.extension() == ".osc") {
        scena::dsl::LoadOptions load_options;
        load_options.search_paths = options.search_paths;
        scena::dsl::LoadResult files;
        scena::dsl::Program program;
        scena::Status loaded =
            scena::dsl::check_file(options.scenario, load_options, files, program, load_sink);
        if (loaded == scena::Status::Ok) {
            scena::dsl::LowerOptions lower_options;
            lower_options.entry_point = options.entry;
            lower_options.alternative = options.select;
            scena::dsl::LowerResult lowered;
            loaded = scena::dsl::lower(program, files, lower_options, lowered, load_sink);
            scenario = std::move(lowered.scenario);
            road_network = std::move(lowered.map_file);
        }
        print_diagnostics(load_sink.diagnostics(), options.quiet);
        if (loaded != scena::Status::Ok) {
            std::cerr << "scena-run: could not load " << options.scenario.string() << '\n';
            return kLoadFailed;
        }
    } else {
        scena::xml::Document document;
        const scena::Status loaded = scena::xml::load_file(options.scenario, document, load_sink);
        print_diagnostics(load_sink.diagnostics(), options.quiet);
        if (loaded != scena::Status::Ok) {
            std::cerr << "scena-run: could not load " << options.scenario.string() << '\n';
            return kLoadFailed;
        }
        scenario = std::move(document.scenario);
        road_network = document.road_network.logic_file;
    }

    // --replay declares that the host will drive an entity, which is the host's
    // call to make: control ownership belongs to the embedder, not to the
    // scenario file (ADR-0003, ADR-0017). OpenSCENARIO XML has no way to say
    // "the host drives this one", so scena-run says it here, before init, and
    // the engine's poll_state path does the rest.
    for (const auto& [entity, path] : options.replay) {
        (void)path;
        bool found = false;
        for (scena::ir::Entity& declared : scenario.entities) {
            if (declared.id == entity) {
                declared.control_mode = scena::ir::ControlMode::HostControlled;
                found = true;
            }
        }
        if (!found) {
            std::cerr << "scena-run: --replay names '" << entity
                      << "', which the scenario does not declare\n";
            return kReplayFailed;
        }
    }

    TraceGateway gateway;

    // The road network. A scenario names its own map in RoadNetwork/LogicFile,
    // relative to the scenario file; --map overrides it, which is how a host
    // points a scenario at a different network without editing it. Without
    // either, the engine runs road-free and the flat-world model applies.
    std::filesystem::path map = options.map;
    if (map.empty() && !road_network.empty()) {
        map = std::filesystem::path(road_network);
        if (map.is_relative()) {
            map = options.scenario.parent_path() / map;
        }
    }
    if (!map.empty()) {
        scena::DiagnosticSink map_sink;
        scena::opendrive::Map network;
        const scena::Status loaded_map = scena::opendrive::load_file(map, network, map_sink);
        print_diagnostics(map_sink.diagnostics(), options.quiet);
        if (loaded_map != scena::Status::Ok) {
            std::cerr << "scena-run: could not load the road network " << map.string() << '\n';
            return kMapFailed;
        }
        gateway.set_road_query(
            std::make_unique<scena::opendrive::OpenDriveRoadQuery>(std::move(network)));
    }

    for (const auto& [entity, path] : options.replay) {
        std::vector<Sample> rows;
        if (!read_trace_csv(path, rows, entity)) {
            std::cerr << "scena-run: could not read replay trace " << path.string() << " for '"
                      << entity << "'\n";
            return kReplayFailed;
        }
        gateway.set_replay(entity, std::move(rows));
    }

    scena::Engine engine(&gateway);
    const scena::Status initialized = engine.init(std::move(scenario));
    print_diagnostics(engine.diagnostics(), options.quiet);
    if (initialized != scena::Status::Ok) {
        std::cerr << "scena-run: could not initialize the scenario\n";
        return kLoadFailed;
    }

    // Whole steps only, so the trace has a fixed cadence and dt never varies
    // mid-run — a variable last step would make the tail of the trace depend on
    // how the duration divides.
    const long long steps = static_cast<long long>(options.duration / options.dt);
    for (long long i = 0; i < steps; ++i) {
        gateway.set_time(static_cast<double>(i + 1) * options.dt);
        const scena::Status stepped = engine.step(options.dt);
        if (stepped != scena::Status::Ok) {
            print_diagnostics(engine.diagnostics(), options.quiet);
            std::cerr << "scena-run: step " << (i + 1) << " failed\n";
            return kRunFailed;
        }
    }
    print_diagnostics(engine.diagnostics(), options.quiet);

    if (!options.trace.empty()) {
        std::string format = options.trace_format;
        if (format.empty()) {
            format = options.trace.extension() == ".json" ? "json" : "csv";
        }
        const bool written = format == "json" ? write_json(options.trace, gateway.samples)
                                              : write_csv(options.trace, gateway.samples);
        if (!written) {
            std::cerr << "scena-run: could not write " << options.trace.string() << '\n';
            return kOutputFailed;
        }
    }

    if (!options.quiet) {
        std::cerr << "scena-run: " << steps << " steps, " << gateway.samples.size()
                  << " samples, t=" << format_double(engine.time()) << " s\n";
    }
    return kOk;
}
