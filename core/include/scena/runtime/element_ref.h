// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace scena::runtime {

/// Splits a storyboard element reference into its name segments on the "::"
/// separator, per ASAM OpenSCENARIO XML 1.4.0 naming conventions (preface):
/// a reference is a name optionally prefixed by the names of directly
/// enclosing elements, "prefix_n::...::prefix_1::name", most-enclosing first.
/// The "::" never appears in a name itself, so the split is unambiguous.
[[nodiscard]] std::vector<std::string> split_element_ref(std::string_view ref);

/// True when the reference `segments` (as produced by split_element_ref)
/// names an element whose ancestor-name chain — the element's own name first,
/// then its parent, grandparent, up to the root — is `chain`.
///
/// The last segment must equal the element name, and each preceding segment
/// (walking outward) must equal the next enclosing ancestor. Fewer prefixes
/// than the full depth are allowed (a globally unique name may be used bare);
/// the resolution is unique only when exactly one element in the tree matches,
/// which the caller checks by counting matches.
[[nodiscard]] bool element_ref_matches(const std::vector<std::string>& segments,
                                       const std::vector<std::string>& chain);

} // namespace scena::runtime
