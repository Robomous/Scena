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

// p3-s3: the OpenDRIVE backend passes the same executable IRoadQuery
// contract the FlatWorldRoadQuery null object pins (p3-s1 exit criterion).

#include <memory>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "scena/opendrive/reader.h"
#include "scena/opendrive/road_query.h"
#include "support/road_query_contract.h"

namespace {

// A straight two-lane road covering the contract's probe grid: reference
// line from (-60, 0) to (60, 0), one 3.5 m driving lane per side.
constexpr const char* kContractMap =
    "<?xml version=\"1.0\"?><OpenDRIVE><header revMajor=\"1\" revMinor=\"8\"/>"
    "<road id=\"1\" length=\"120\">"
    "<planView><geometry s=\"0\" x=\"-60\" y=\"0\" hdg=\"0\" length=\"120\">"
    "<line/></geometry></planView>"
    "<lanes><laneSection s=\"0\">"
    "<left><lane id=\"1\" type=\"driving\">"
    "<width sOffset=\"0\" a=\"3.5\" b=\"0\" c=\"0\" d=\"0\"/></lane></left>"
    "<center><lane id=\"0\"/></center>"
    "<right><lane id=\"-1\" type=\"driving\">"
    "<width sOffset=\"0\" a=\"3.5\" b=\"0\" c=\"0\" d=\"0\"/></lane></right>"
    "</laneSection></lanes>"
    "</road></OpenDRIVE>";

struct OpenDriveTraits {
    static std::unique_ptr<scena::gateway::IRoadQuery> make() {
        scena::opendrive::Map map;
        scena::DiagnosticSink sink;
        const scena::Status status = scena::opendrive::load_string(kContractMap, map, sink);
        EXPECT_EQ(status, scena::Status::Ok);
        return std::make_unique<scena::opendrive::OpenDriveRoadQuery>(std::move(map));
    }
};

} // namespace

// The instantiation macro must expand inside the namespace that declared the
// typed suite (p3-s1 note in road_query_contract_test.cpp).
namespace scena::testsupport {
using OpenDriveBackend = ::testing::Types<OpenDriveTraits>;
INSTANTIATE_TYPED_TEST_SUITE_P(OpenDrive, RoadQueryContractTest, OpenDriveBackend);
} // namespace scena::testsupport
