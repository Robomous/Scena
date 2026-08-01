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

// Catalogs (§9.4-9.6): reference resolution against a committed fixture
// tree, parameter assignment into catalog entries, the isolation of a
// catalog's parameters, controller assignment, and the reproducibility the
// sprint's exit criterion asks for.

#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

#include "scena/diagnostic.h"
#include "scena/ir/action.h"
#include "scena/ir/entity.h"
#include "scena/xml/loader.h"

namespace {

using scena::Diagnostic;
using scena::DiagnosticSink;
using scena::Severity;
using scena::Status;
using scena::xml::Document;

const std::filesystem::path& catalog_root() {
    static const std::filesystem::path root(SCENA_TEST_CATALOG_DIR);
    return root;
}

/// Writes `text` as a scenario file inside the fixture tree's parent, so the
/// relative catalog directories in it resolve exactly as a real scenario's
/// would, and removes it afterwards.
class ScenarioFile {
public:
    ScenarioFile(std::string_view name, std::string_view text) : path_(catalog_root() / name) {
        std::ofstream stream(path_, std::ios::binary | std::ios::trunc);
        stream.write(text.data(), static_cast<std::streamsize>(text.size()));
    }
    ~ScenarioFile() {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }
    ScenarioFile(const ScenarioFile&) = delete;
    ScenarioFile& operator=(const ScenarioFile&) = delete;

    [[nodiscard]] const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};

constexpr std::string_view kLocations = R"(<CatalogLocations>
  <VehicleCatalog><Directory path="vehicles"/></VehicleCatalog>
  <ControllerCatalog><Directory path="controllers"/></ControllerCatalog>
  <PedestrianCatalog><Directory path="pedestrians"/></PedestrianCatalog>
  <ManeuverCatalog><Directory path="maneuvers"/></ManeuverCatalog>
</CatalogLocations>)";

std::string scenario_with(std::string_view body, std::string_view locations = kLocations) {
    return std::string(R"(<OpenSCENARIO><FileHeader revMajor="1" revMinor="2"
        date="2026-08-01T00:00:00" description="catalog fixture" author="Scena"/>)") +
           std::string(locations) + std::string(body) + "</OpenSCENARIO>";
}

bool has_message_containing(const DiagnosticSink& sink, std::string_view needle) {
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        if (diagnostic.message.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

bool has_rule_id(const DiagnosticSink& sink, std::string_view rule_id) {
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        if (diagnostic.rule_id == rule_id) {
            return true;
        }
    }
    return false;
}

// --- entity entries -------------------------------------------------------

constexpr std::string_view kVehicleReference = R"(<Entities>
  <ScenarioObject name="ego">
    <CatalogReference catalogName="ScenaVehicles" entryName="compact"/>
  </ScenarioObject>
</Entities><Storyboard/>)";

TEST(Catalogs, AVehicleEntryResolvesWithItsDefaults) {
    const ScenarioFile file("scena_catalog_vehicle.xosc", scenario_with(kVehicleReference));
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_file(file.path(), document, sink), Status::Ok);
    ASSERT_EQ(document.scenario.entities.size(), 1U);

    const auto* vehicle =
        std::get_if<scena::ir::Vehicle>(&*document.scenario.entities.front().object);
    ASSERT_NE(vehicle, nullptr);
    EXPECT_EQ(vehicle->category, scena::ir::VehicleCategory::Car);
    // The entry's own ParameterDeclarations supply the defaults (§9.5).
    EXPECT_DOUBLE_EQ(vehicle->performance.max_speed, 50.0);
    EXPECT_DOUBLE_EQ(vehicle->bounding_box.length, 4.0);
}

TEST(Catalogs, ParameterAssignmentsOverrideTheEntryDefaults) {
    constexpr std::string_view kBody = R"(<Entities>
      <ScenarioObject name="ego">
        <CatalogReference catalogName="ScenaVehicles" entryName="compact">
          <ParameterAssignments>
            <ParameterAssignment parameterRef="maxSpeed" value="70.0"/>
          </ParameterAssignments>
        </CatalogReference>
      </ScenarioObject>
    </Entities><Storyboard/>)";
    const ScenarioFile file("scena_catalog_assign.xosc", scenario_with(kBody));
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_file(file.path(), document, sink), Status::Ok);

    const auto* vehicle =
        std::get_if<scena::ir::Vehicle>(&*document.scenario.entities.front().object);
    ASSERT_NE(vehicle, nullptr);
    EXPECT_DOUBLE_EQ(vehicle->performance.max_speed, 70.0); // assigned
    EXPECT_DOUBLE_EQ(vehicle->bounding_box.length, 4.0);    // default kept
}

TEST(Catalogs, AnAssignmentValueMayItselfBeAParameter) {
    constexpr std::string_view kBody = R"(<ParameterDeclarations>
        <ParameterDeclaration name="fleet_speed" parameterType="double" value="33.0"/>
      </ParameterDeclarations>
      <Entities>
        <ScenarioObject name="ego">
          <CatalogReference catalogName="ScenaVehicles" entryName="compact">
            <ParameterAssignments>
              <ParameterAssignment parameterRef="maxSpeed" value="$fleet_speed"/>
            </ParameterAssignments>
          </CatalogReference>
        </ScenarioObject>
      </Entities><Storyboard/>)";
    const ScenarioFile file("scena_catalog_assign_param.xosc", scenario_with(kBody));
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_file(file.path(), document, sink), Status::Ok);
    const auto* vehicle =
        std::get_if<scena::ir::Vehicle>(&*document.scenario.entities.front().object);
    ASSERT_NE(vehicle, nullptr);
    // The assignment is evaluated in the *referencing* scope, before the
    // entry's isolation begins.
    EXPECT_DOUBLE_EQ(vehicle->performance.max_speed, 33.0);
}

TEST(Catalogs, AnEntryCannotSeeTheScenariosOwnParameters) {
    // §9.5: "No other parameters may be referenced from within the catalog."
    // The scenario declares `length`, but the entry's own declaration is the
    // one that applies inside it.
    constexpr std::string_view kBody = R"(<ParameterDeclarations>
        <ParameterDeclaration name="length" parameterType="double" value="99.0"/>
      </ParameterDeclarations>
      <Entities>
        <ScenarioObject name="ego">
          <CatalogReference catalogName="ScenaVehicles" entryName="compact"/>
        </ScenarioObject>
      </Entities><Storyboard/>)";
    const ScenarioFile file("scena_catalog_isolation.xosc", scenario_with(kBody));
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_file(file.path(), document, sink), Status::Ok);
    const auto* vehicle =
        std::get_if<scena::ir::Vehicle>(&*document.scenario.entities.front().object);
    ASSERT_NE(vehicle, nullptr);
    EXPECT_DOUBLE_EQ(vehicle->bounding_box.length, 4.0);
}

TEST(Catalogs, APedestrianEntryResolvesFromItsOwnCatalogDirectory) {
    // Each catalog kind has its own directory (§9.6), and a ScenarioObject's
    // reference does not say which kind it names — the loader tries them in
    // the order the ScenarioObject choice declares.
    constexpr std::string_view kBody = R"(<Entities>
      <ScenarioObject name="walker">
        <CatalogReference catalogName="ScenaPedestrians" entryName="walker"/>
      </ScenarioObject>
    </Entities><Storyboard/>)";
    const ScenarioFile file("scena_catalog_pedestrian.xosc", scenario_with(kBody));
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_file(file.path(), document, sink), Status::Ok);
    ASSERT_EQ(document.scenario.entities.size(), 1U);
    EXPECT_EQ(*scena::ir::object_type_of(document.scenario.entities.front()),
              scena::ir::ObjectType::Pedestrian);
}

// --- missing references ---------------------------------------------------

TEST(Catalogs, AMissingEntryCitesTheResolvabilityRule) {
    constexpr std::string_view kBody = R"(<Entities>
      <ScenarioObject name="ego">
        <CatalogReference catalogName="ScenaVehicles" entryName="spaceship"/>
      </ScenarioObject>
    </Entities><Storyboard/>)";
    const ScenarioFile file("scena_catalog_missing_entry.xosc", scenario_with(kBody));
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_file(file.path(), document, sink), Status::SemanticError);
    EXPECT_TRUE(
        has_rule_id(sink, "asam.net:xosc:1.0.0:reference_control.catalog_reference_resolvability"));
    // The entity survives, unclassified: one bad reference does not lose the
    // rest of the document.
    ASSERT_EQ(document.scenario.entities.size(), 1U);
    EXPECT_FALSE(document.scenario.entities.front().object.has_value());
}

TEST(Catalogs, AnUndeclaredDirectoryCitesTheDirectoryRule) {
    constexpr std::string_view kBody = R"(<Storyboard><Init><Actions>
      <GlobalAction><EnvironmentAction>
        <CatalogReference catalogName="ScenaEnvironments" entryName="dusk"/>
      </EnvironmentAction></GlobalAction>
    </Actions></Init></Storyboard>)";
    const ScenarioFile file("scena_catalog_no_dir.xosc", scenario_with(kBody));
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_file(file.path(), document, sink), Status::SemanticError);
    EXPECT_TRUE(has_rule_id(
        sink, "asam.net:xosc:1.0.0:reference_control.catalogs_referenced_by_directory"));
}

TEST(Catalogs, ARelativeDirectoryNeedsAScenarioFile) {
    // load_string has no location on disk, so a relative catalog directory
    // cannot be resolved — reported, never guessed.
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string(scenario_with(kVehicleReference), document, sink),
              Status::SemanticError);
    EXPECT_TRUE(has_message_containing(sink, "no file location to resolve it against"));
}

// --- other catalog kinds --------------------------------------------------

TEST(Catalogs, AControllerEntryResolvesThroughObjectController) {
    constexpr std::string_view kBody = R"(<Entities>
      <ScenarioObject name="ego">
        <Vehicle name="v" vehicleCategory="car">
          <BoundingBox><Center x="0" y="0" z="0"/><Dimensions width="2" length="4" height="1.5"/></BoundingBox>
          <Performance maxSpeed="60" maxAcceleration="5" maxDeceleration="9"/>
          <Axles><RearAxle maxSteering="0" wheelDiameter="0.6" trackWidth="1.7" positionX="0" positionZ="0.3"/></Axles>
        </Vehicle>
        <ObjectController>
          <CatalogReference catalogName="ScenaControllers" entryName="external_driver"/>
        </ObjectController>
      </ScenarioObject>
    </Entities><Storyboard/>)";
    const ScenarioFile file("scena_catalog_controller.xosc", scenario_with(kBody));
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_file(file.path(), document, sink), Status::Ok);

    // The controller reaches the engine as an init action; control ownership
    // stays with the engine until the host says otherwise (ADR-0023).
    ASSERT_EQ(document.scenario.init_actions.size(), 1U);
    const auto* assign = dynamic_cast<const scena::ir::AssignControllerAction*>(
        document.scenario.init_actions.front().get());
    ASSERT_NE(assign, nullptr);
    EXPECT_EQ(assign->entity_id(), "ego");
    EXPECT_EQ(assign->controller().name, "external_driver");
    EXPECT_EQ(assign->controller().type, scena::ir::ControllerType::Movement);
    ASSERT_EQ(assign->controller().properties.size(), 1U);
    EXPECT_EQ(assign->controller().properties.front().value, "host");
    EXPECT_EQ(document.scenario.entities.front().control_mode,
              scena::ir::ControlMode::EngineControlled);
}

TEST(Catalogs, AManeuverEntryResolvesIntoItsManeuverGroup) {
    constexpr std::string_view kBody = R"(<Entities>
        <ScenarioObject name="ego">
          <CatalogReference catalogName="ScenaVehicles" entryName="compact"/>
        </ScenarioObject>
      </Entities>
      <Storyboard><Story name="s"><Act name="a"><ManeuverGroup name="g">
        <Actors selectTriggeringEntities="false"><EntityRef entityRef="ego"/></Actors>
        <CatalogReference catalogName="ScenaManeuvers" entryName="brake">
          <ParameterAssignments>
            <ParameterAssignment parameterRef="target" value="5.0"/>
          </ParameterAssignments>
        </CatalogReference>
      </ManeuverGroup></Act></Story></Storyboard>)";
    const ScenarioFile file("scena_catalog_maneuver.xosc", scenario_with(kBody));
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_file(file.path(), document, sink), Status::Ok);

    const auto& group = document.scenario.storyboard.stories.at(0).acts.at(0).groups.at(0);
    ASSERT_EQ(group.maneuvers.size(), 1U);
    EXPECT_EQ(group.maneuvers.front().name, "brake");
    ASSERT_EQ(group.maneuvers.front().events.size(), 1U);
    const auto& actions = group.maneuvers.front().events.front().actions;
    ASSERT_EQ(actions.size(), 1U);
    EXPECT_EQ(actions.front()->kind(), "SpeedAction");
    EXPECT_EQ(actions.front()->entity_id(), "ego");
}

// --- reproducibility ------------------------------------------------------

TEST(Catalogs, TheFixtureTreeLoadsReproducibly) {
    // The sprint's exit criterion: filesystem enumeration order must not
    // reach the IR. Loading the same tree twice must produce the same
    // entities, the same values and the same diagnostics.
    const ScenarioFile file("scena_catalog_repeat.xosc", scenario_with(kVehicleReference));

    const auto load = [&file]() {
        Document document;
        DiagnosticSink sink;
        EXPECT_EQ(scena::xml::load_file(file.path(), document, sink), Status::Ok);
        return document;
    };
    const Document first = load();
    const Document second = load();

    ASSERT_EQ(first.scenario.entities.size(), second.scenario.entities.size());
    const auto* first_vehicle =
        std::get_if<scena::ir::Vehicle>(&*first.scenario.entities.front().object);
    const auto* second_vehicle =
        std::get_if<scena::ir::Vehicle>(&*second.scenario.entities.front().object);
    ASSERT_NE(first_vehicle, nullptr);
    ASSERT_NE(second_vehicle, nullptr);
    EXPECT_EQ(first_vehicle->performance.max_speed, second_vehicle->performance.max_speed);
    EXPECT_EQ(first_vehicle->bounding_box.length, second_vehicle->bounding_box.length);
}

} // namespace
