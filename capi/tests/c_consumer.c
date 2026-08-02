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

/*
 * A pure-C consumer of the Scena C ABI.
 *
 * Compiled as C11 with no C++ in sight, which is the point: capi.h has to stay
 * C-clean, and the only way to keep it that way is to have a C compiler read it
 * on every platform in CI. The program also runs, so the header being parseable
 * is not mistaken for the library being callable.
 *
 * Deliberately uses only the C standard library: an embedder integrating Scena
 * into a C codebase should need nothing else.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "scena/capi.h"

#define CHECK(condition)                                                                           \
    do {                                                                                           \
        if (!(condition)) {                                                                        \
            fprintf(stderr, "%s:%d: FAILED %s\n", __FILE__, __LINE__, #condition);                 \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)

static const char* const kScenario =
    "<?xml version=\"1.0\"?>"
    "<OpenSCENARIO>"
    "  <FileHeader revMajor=\"1\" revMinor=\"3\" date=\"2026-08-01T00:00:00\""
    "              description=\"c consumer\" author=\"scena\"/>"
    "  <Entities>"
    "    <ScenarioObject name=\"ego\"><Vehicle name=\"ego_v\" vehicleCategory=\"car\">"
    "      <BoundingBox><Center x=\"0\" y=\"0\" z=\"0\"/>"
    "        <Dimensions width=\"2\" length=\"4\" height=\"1.5\"/></BoundingBox>"
    "      <Performance maxSpeed=\"60\" maxAcceleration=\"5\" maxDeceleration=\"9\"/>"
    "      <Axles><RearAxle maxSteering=\"0\" wheelDiameter=\"0.6\" trackWidth=\"1.7\""
    "                       positionX=\"0\" positionZ=\"0.3\"/></Axles>"
    "    </Vehicle></ScenarioObject>"
    "  </Entities>"
    "  <Storyboard>"
    "    <Init><Actions><Private entityRef=\"ego\"><PrivateAction><TeleportAction>"
    "      <Position><WorldPosition x=\"0\" y=\"0\" z=\"0\" h=\"0\"/></Position>"
    "    </TeleportAction></PrivateAction></Private></Actions></Init>"
    "    <Story name=\"story\"><Act name=\"act\">"
    "      <ManeuverGroup name=\"group\" maximumExecutionCount=\"1\">"
    "        <Actors selectTriggeringEntities=\"false\"><EntityRef entityRef=\"ego\"/></Actors>"
    "        <Maneuver name=\"maneuver\"><Event name=\"event\" priority=\"parallel\">"
    "          <Action name=\"go\"><PrivateAction><LongitudinalAction><SpeedAction>"
    "            <SpeedActionDynamics dynamicsShape=\"step\" dynamicsDimension=\"time\""
    "                                 value=\"0\"/>"
    "            <SpeedActionTarget><AbsoluteTargetSpeed value=\"10\"/></SpeedActionTarget>"
    "          </SpeedAction></LongitudinalAction></PrivateAction></Action>"
    "          <StartTrigger><ConditionGroup><Condition name=\"c\" delay=\"0\""
    "              conditionEdge=\"none\"><ByValueCondition>"
    "              <SimulationTimeCondition value=\"0\" rule=\"greaterThan\"/>"
    "            </ByValueCondition></Condition></ConditionGroup></StartTrigger>"
    "        </Event></Maneuver>"
    "      </ManeuverGroup>"
    "    </Act></Story>"
    "  </Storyboard>"
    "</OpenSCENARIO>";

/* OpenSCENARIO DSL, checked through scn_check_dsl_string. `length` comes from
 * the bundled standard library, so this source also proves the implicit import
 * reached the check. */
static const char* const kDslSource = "struct marker:\n"
                                      "    x: length\n"
                                      "    y: length\n";

/* The same source with a type nothing declares — one error, one diagnostic. */
static const char* const kBadDslSource = "struct marker:\n"
                                         "    x: no_such_type\n";

static int check_dsl_surface(void) {
    scn_dsl_check_options options;
    scn_dsl_check* check = NULL;
    size_t diagnostic_count = 1;
    size_t type_count = 0;
    size_t file_count = 0;
    scn_diagnostic diagnostic;

    memset(&options, 0, sizeof(options));
    options.implicit_standard_library = 1;

    CHECK(scn_check_dsl_string(kDslSource, "marker.osc", &options, &check) == SCN_OK);
    CHECK(check != NULL);
    CHECK(scn_dsl_check_diagnostic_count(check, &diagnostic_count) == SCN_OK);
    CHECK(diagnostic_count == 0);
    CHECK(scn_dsl_check_type_count(check, &type_count) == SCN_OK);
    CHECK(type_count > 0);
    /* The source itself plus the standard library's two sub-modules. */
    CHECK(scn_dsl_check_file_count(check, &file_count) == SCN_OK);
    CHECK(file_count > 1);
    scn_dsl_check_destroy(check);

    /* A failing check still produces a handle: the diagnostics are the point. */
    check = NULL;
    CHECK(scn_check_dsl_string(kBadDslSource, "marker.osc", NULL, &check) != SCN_OK);
    CHECK(check != NULL);
    CHECK(scn_dsl_check_diagnostic_count(check, &diagnostic_count) == SCN_OK);
    CHECK(diagnostic_count > 0);
    memset(&diagnostic, 0, sizeof(diagnostic));
    CHECK(scn_dsl_check_diagnostic_at(check, 0, &diagnostic) == SCN_OK);
    CHECK(diagnostic.severity == SCN_SEVERITY_ERROR);
    CHECK(strcmp(diagnostic.file, "marker.osc") == 0);
    CHECK(diagnostic.line > 0);
    /* The DSL standard defines no rule ids; the citation is in the message. */
    CHECK(strcmp(diagnostic.rule_id, "") == 0);
    CHECK(scn_dsl_check_diagnostic_at(check, diagnostic_count, &diagnostic) ==
          SCN_ERROR_INVALID_ARGUMENT);
    scn_dsl_check_destroy(check);

    /* A path that cannot be read is host misuse, and still hands back the
     * handle carrying the diagnostic that says so. */
    check = NULL;
    CHECK(scn_check_dsl_file("no/such/file.osc", NULL, &check) == SCN_ERROR_INVALID_ARGUMENT);
    CHECK(check != NULL);
    CHECK(scn_dsl_check_diagnostic_count(check, &diagnostic_count) == SCN_OK);
    CHECK(diagnostic_count > 0);
    scn_dsl_check_destroy(check);

    /* Null arguments are rejected without producing a handle. */
    check = NULL;
    CHECK(scn_check_dsl_string(NULL, NULL, NULL, &check) == SCN_ERROR_INVALID_ARGUMENT);
    CHECK(check == NULL);
    CHECK(scn_check_dsl_file(NULL, NULL, &check) == SCN_ERROR_INVALID_ARGUMENT);
    CHECK(check == NULL);
    CHECK(scn_check_dsl_string(kDslSource, NULL, NULL, NULL) == SCN_ERROR_INVALID_ARGUMENT);
    CHECK(scn_dsl_check_diagnostic_count(NULL, &diagnostic_count) == SCN_ERROR_INVALID_ARGUMENT);
    /* Destroying NULL is a no-op, so a cleanup path needs no guard. */
    scn_dsl_check_destroy(NULL);

    return 0;
}

int main(void) {
    /* An embedder that dlopen()s the library checks the ABI major first. */
    CHECK(scn_abi_version() / 10000u == SCN_ABI_VERSION / 10000u);
    CHECK(scn_version() != NULL);
    CHECK(strlen(scn_version()) > 0);

    scn_engine* engine = scn_engine_create();
    CHECK(engine != NULL);

    int initialized = 1;
    CHECK(scn_engine_initialized(engine, &initialized) == SCN_OK);
    CHECK(initialized == 0);

    CHECK(scn_engine_load_xml_string(engine, kScenario) == SCN_OK);
    CHECK(scn_engine_initialized(engine, &initialized) == SCN_OK);
    CHECK(initialized == 1);

    size_t entity_count = 0;
    CHECK(scn_engine_entity_count(engine, &entity_count) == SCN_OK);
    CHECK(entity_count == 1);

    const char* entity_id = NULL;
    CHECK(scn_engine_entity_id_at(engine, 0, &entity_id) == SCN_OK);
    CHECK(strcmp(entity_id, "ego") == 0);
    CHECK(scn_engine_entity_id_at(engine, 1, &entity_id) == SCN_ERROR_INVALID_ARGUMENT);

    scn_object_type type = SCN_OBJECT_MISC;
    CHECK(scn_engine_entity_object_type(engine, "ego", &type) == SCN_OK);
    CHECK(type == SCN_OBJECT_VEHICLE);

    scn_element_state state = SCN_ELEMENT_COMPLETE;
    CHECK(scn_engine_element_state(engine, "story/act/group/maneuver/event", &state) == SCN_OK);
    CHECK(state == SCN_ELEMENT_STANDBY);
    CHECK(scn_engine_element_state(engine, "story/act/nope", &state) == SCN_ERROR_UNKNOWN_NAME);

    for (int i = 0; i < 10; ++i) {
        CHECK(scn_engine_step(engine, 0.1) == SCN_OK);
    }

    double time = -1.0;
    CHECK(scn_engine_get_time(engine, &time) == SCN_OK);
    CHECK(time > 0.9 && time < 1.1);

    scn_entity_state entity_state;
    memset(&entity_state, 0, sizeof(entity_state));
    CHECK(scn_engine_get_state(engine, "ego", &entity_state) == SCN_OK);
    CHECK(entity_state.speed == 10.0);
    CHECK(entity_state.x > 0.0);

    CHECK(scn_engine_element_state(engine, "story/act/group/maneuver/event", &state) == SCN_OK);
    CHECK(state == SCN_ELEMENT_COMPLETE);
    scn_element_transition transition = SCN_TRANSITION_NONE;
    CHECK(scn_engine_element_transition(engine, "story/act/group/maneuver/event", &transition) ==
          SCN_OK);

    /* Host-side signal publication, and reading it back. */
    const char* signal_state = NULL;
    CHECK(scn_engine_set_traffic_signal_state(engine, "s1", "green") == SCN_OK);
    CHECK(scn_engine_traffic_signal_state(engine, "s1", &signal_state) == SCN_OK);
    CHECK(strcmp(signal_state, "green") == 0);

    /* Variables round-trip through the same borrowed-string convention. */
    const char* value = NULL;
    CHECK(scn_engine_set_user_defined_value(engine, "k", "v") == SCN_OK);
    CHECK(scn_engine_get_user_defined_value(engine, "k", &value) == SCN_OK);
    CHECK(strcmp(value, "v") == 0);

    size_t diagnostic_count = 0;
    CHECK(scn_engine_diagnostic_count(engine, &diagnostic_count) == SCN_OK);

    CHECK(scn_engine_close(engine) == SCN_OK);
    scn_engine_destroy(engine);

    /* Checking DSL needs no engine at all — it is a frontend service. */
    CHECK(check_dsl_surface() == 0);

    printf("pure-C consumer: OK\n");
    return 0;
}
