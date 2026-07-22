// SPDX-License-Identifier: MIT
#include "scena/runtime/element_ref.h"

#include <cstddef>

namespace scena::runtime {

std::vector<std::string> split_element_ref(std::string_view ref) {
    std::vector<std::string> segments;
    std::string_view::size_type begin = 0;
    while (true) {
        const auto pos = ref.find("::", begin);
        if (pos == std::string_view::npos) {
            segments.emplace_back(ref.substr(begin));
            break;
        }
        segments.emplace_back(ref.substr(begin, pos - begin));
        begin = pos + 2; // skip "::"
    }
    return segments;
}

bool element_ref_matches(const std::vector<std::string>& segments,
                         const std::vector<std::string>& chain) {
    if (segments.empty() || segments.size() > chain.size()) {
        return false;
    }
    // segments is [prefix_n, ..., prefix_1, name]; chain is [self, parent, ...].
    // The last segment is the name (chain[0]), the one before it the direct
    // parent (chain[1]), and so on outward.
    for (std::size_t i = 0; i < segments.size(); ++i) {
        if (segments[segments.size() - 1 - i] != chain[i]) {
            return false;
        }
    }
    return true;
}

} // namespace scena::runtime
