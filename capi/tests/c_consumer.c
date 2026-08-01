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

    printf("pure-C consumer: OK\n");
    return 0;
}
