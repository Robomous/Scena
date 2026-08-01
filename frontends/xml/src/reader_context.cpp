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

#include "reader_context.h"

#include <utility>
#include <vector>

#include "parameters.h"

namespace scena::xml::detail {

namespace {

/// 1-based position of `node` among its same-named element siblings, or 0
/// when it is the only one — 0 means "no predicate needed".
int sibling_index(const pugi::xml_node& node) {
    const pugi::xml_node parent = node.parent();
    if (!parent) {
        return 0;
    }
    const std::string_view name = node.name();
    int index = 0;
    int seen = 0;
    for (pugi::xml_node sibling : parent.children()) {
        if (sibling.type() != pugi::node_element || sibling.name() != name) {
            continue;
        }
        ++seen;
        if (sibling == node) {
            index = seen;
        }
    }
    return seen > 1 ? index : 0;
}

} // namespace

std::string element_path(const pugi::xml_node& node) {
    std::vector<std::string> steps;
    for (pugi::xml_node current = node; current && current.type() == pugi::node_element;
         current = current.parent()) {
        std::string step = current.name();
        const int index = sibling_index(current);
        if (index > 0) {
            step += "[" + std::to_string(index) + "]";
        }
        steps.push_back(std::move(step));
    }
    std::string path;
    for (auto step = steps.rbegin(); step != steps.rend(); ++step) {
        path += "/";
        path += *step;
    }
    return path.empty() ? std::string("/") : path;
}

std::string attribute_path(const pugi::xml_node& node, std::string_view attribute) {
    std::string path = element_path(node);
    path += "/@";
    path.append(attribute);
    return path;
}

LineColumn line_column_at(std::string_view source, std::ptrdiff_t offset) {
    if (offset < 0 || source.empty()) {
        return {};
    }
    const std::size_t limit =
        std::min(static_cast<std::size_t>(offset), source.size() - 1); // clamp past-the-end
    LineColumn position{1, 1};
    for (std::size_t i = 0; i < limit; ++i) {
        // A lone '\r' also ends a line, and "\r\n" must count as one break:
        // documents saved on Windows are read in binary mode precisely so
        // their bytes are not rewritten, so both forms reach this loop.
        if (source[i] == '\n') {
            ++position.line;
            position.column = 1;
        } else if (source[i] == '\r') {
            if (i + 1 < source.size() && source[i + 1] == '\n') {
                continue; // counted at the '\n'
            }
            ++position.line;
            position.column = 1;
        } else {
            ++position.column;
        }
    }
    return position;
}

ReadContext::ReadContext(DiagnosticSink& sink, std::string_view source, std::string file)
    : sink_(sink), parameters_(std::make_unique<ParameterScope>()), source_(source),
      file_(std::move(file)) {}

// Out of line so the header can forward-declare ParameterScope and keep the
// expression machinery out of every reader's include graph.
ReadContext::~ReadContext() = default;

void ReadContext::report(Severity severity, Status code, std::string path, std::string message,
                         std::string rule_id) {
    emit(severity, code, std::move(path), std::move(message), std::move(rule_id), LineColumn{});
}

void ReadContext::report_at(const pugi::xml_node& node, Severity severity, Status code,
                            std::string path, std::string message, std::string rule_id) {
    emit(severity, code, std::move(path), std::move(message), std::move(rule_id),
         line_column_at(source_, node.offset_debug()));
}

void ReadContext::report_at_offset(std::ptrdiff_t offset, Severity severity, Status code,
                                   std::string path, std::string message, std::string rule_id) {
    emit(severity, code, std::move(path), std::move(message), std::move(rule_id),
         line_column_at(source_, offset));
}

void ReadContext::emit(Severity severity, Status code, std::string path, std::string message,
                       std::string rule_id, LineColumn position) {
    if (severity == Severity::Error && first_error_ == Status::Ok) {
        first_error_ = code;
    }
    Diagnostic diagnostic;
    diagnostic.severity = severity;
    diagnostic.code = code;
    diagnostic.message = std::move(message);
    diagnostic.path = std::move(path);
    diagnostic.location.file = file_;
    diagnostic.location.line = position.line;
    diagnostic.location.column = position.column;
    diagnostic.rule_id = std::move(rule_id);
    sink_.report(std::move(diagnostic));
}

} // namespace scena::xml::detail
