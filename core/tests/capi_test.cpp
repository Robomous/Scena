// SPDX-License-Identifier: MIT
#include "scena/capi.h"

#include <gtest/gtest.h>

TEST(CApiTest, VersionString) {
    const char* version = scn_version();
    ASSERT_NE(version, nullptr);
    EXPECT_STREQ(version, "0.1.0");
}

TEST(CApiTest, FullLifecycle) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);

    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego vehicle", SCN_CONTROL_ENGINE), SCN_OK);
    ASSERT_EQ(scn_engine_add_entity(engine, "npc", "host vehicle", SCN_CONTROL_HOST), SCN_OK);
    ASSERT_EQ(scn_engine_add_speed_action(engine, "ego", 10.0, 1.0), SCN_OK);

    ASSERT_EQ(scn_engine_init(engine), SCN_OK);

    // 2 simulated seconds at 100 Hz; the speed action triggers at t = 1.0 s.
    for (int i = 0; i < 200; ++i) {
        ASSERT_EQ(scn_engine_step(engine, 0.01), SCN_OK);
    }

    scn_entity_state ego_state{};
    ASSERT_EQ(scn_engine_get_state(engine, "ego", &ego_state), SCN_OK);
    EXPECT_EQ(ego_state.speed, 10.0);
    EXPECT_GT(ego_state.x, 0.0);

    const scn_entity_state reported{4.0, 5.0, 0.0, 1.5, 3.0};
    ASSERT_EQ(scn_engine_report_state(engine, "npc", &reported), SCN_OK);
    scn_entity_state npc_state{};
    ASSERT_EQ(scn_engine_get_state(engine, "npc", &npc_state), SCN_OK);
    EXPECT_EQ(npc_state.x, reported.x);
    EXPECT_EQ(npc_state.heading, reported.heading);
    EXPECT_EQ(npc_state.speed, reported.speed);

    ASSERT_EQ(scn_engine_close(engine), SCN_OK);
    scn_engine_destroy(engine);
}

TEST(CApiTest, ErrorPaths) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);

    EXPECT_EQ(scn_engine_step(engine, 0.01), SCN_ERROR_NOT_INITIALIZED);
    EXPECT_EQ(scn_engine_close(engine), SCN_ERROR_NOT_INITIALIZED);

    scn_entity_state state{};
    EXPECT_EQ(scn_engine_get_state(engine, "ego", &state), SCN_ERROR_NOT_INITIALIZED);

    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego vehicle", SCN_CONTROL_ENGINE), SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);
    EXPECT_EQ(scn_engine_init(engine), SCN_ERROR_ALREADY_INITIALIZED);

    EXPECT_EQ(scn_engine_get_state(engine, "missing", &state), SCN_ERROR_UNKNOWN_ENTITY);
    EXPECT_EQ(scn_engine_report_state(engine, "ego", &state), SCN_ERROR_INVALID_CONTROL_MODE);
    EXPECT_EQ(scn_engine_step(engine, -1.0), SCN_ERROR_INVALID_ARGUMENT);

    scn_engine_destroy(engine);
}

TEST(CApiTest, NullArgumentsAreRejected) {
    EXPECT_EQ(scn_engine_step(nullptr, 0.01), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_init(nullptr), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_close(nullptr), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_get_state(nullptr, "ego", nullptr), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_entity(nullptr, "ego", "ego", SCN_CONTROL_ENGINE),
              SCN_ERROR_INVALID_ARGUMENT);

    size_t count = 42;
    scn_diagnostic diagnostic{};
    EXPECT_EQ(scn_engine_diagnostic_count(nullptr, &count), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_diagnostic_at(nullptr, 0, &diagnostic), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_clear_diagnostics(nullptr), SCN_ERROR_INVALID_ARGUMENT);

    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    EXPECT_EQ(scn_engine_add_entity(engine, nullptr, "ego", SCN_CONTROL_ENGINE),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_get_state(engine, "ego", nullptr), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_diagnostic_count(engine, nullptr), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_diagnostic_at(engine, 0, nullptr), SCN_ERROR_INVALID_ARGUMENT);
    scn_engine_destroy(engine);

    scn_engine_destroy(nullptr); // must be a safe no-op
}

TEST(CApiTest, DiagnosticsRoundTrip) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    // A speed action targeting an entity the scenario never declares: init
    // fails with a semantic error and records a diagnostic.
    ASSERT_EQ(scn_engine_add_speed_action(engine, "missing", 10.0, 1.0), SCN_OK);
    EXPECT_EQ(scn_engine_init(engine), SCN_ERROR_SEMANTIC);

    size_t count = 0;
    ASSERT_EQ(scn_engine_diagnostic_count(engine, &count), SCN_OK);
    ASSERT_GE(count, 1U);

    scn_diagnostic diagnostic{};
    ASSERT_EQ(scn_engine_diagnostic_at(engine, 0, &diagnostic), SCN_OK);
    EXPECT_EQ(diagnostic.severity, SCN_SEVERITY_ERROR);
    EXPECT_EQ(diagnostic.code, SCN_ERROR_SEMANTIC);
    // Borrowed strings are never NULL, absent fields are "".
    ASSERT_NE(diagnostic.message, nullptr);
    ASSERT_NE(diagnostic.path, nullptr);
    ASSERT_NE(diagnostic.file, nullptr);
    ASSERT_NE(diagnostic.rule_id, nullptr);
    EXPECT_STRNE(diagnostic.message, "");
    EXPECT_STRNE(diagnostic.path, "");
    EXPECT_STREQ(diagnostic.file, ""); // no source location in a built scenario
    EXPECT_EQ(diagnostic.line, 0);

    scn_engine_destroy(engine);
}

TEST(CApiTest, DiagnosticAtRejectsOutOfRange) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego vehicle", SCN_CONTROL_ENGINE), SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);

    size_t count = 7;
    ASSERT_EQ(scn_engine_diagnostic_count(engine, &count), SCN_OK);
    EXPECT_EQ(count, 0U); // a valid scenario produces none

    scn_diagnostic diagnostic{};
    diagnostic.line = 99; // sentinel: must be left untouched on out-of-range
    EXPECT_EQ(scn_engine_diagnostic_at(engine, 0, &diagnostic), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(diagnostic.line, 99);

    scn_engine_destroy(engine);
}

TEST(CApiTest, ClearDiagnosticsEmptiesTheRecord) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_speed_action(engine, "missing", 10.0, 1.0), SCN_OK);
    EXPECT_EQ(scn_engine_init(engine), SCN_ERROR_SEMANTIC);

    size_t count = 0;
    ASSERT_EQ(scn_engine_diagnostic_count(engine, &count), SCN_OK);
    ASSERT_GE(count, 1U);

    ASSERT_EQ(scn_engine_clear_diagnostics(engine), SCN_OK);
    ASSERT_EQ(scn_engine_diagnostic_count(engine, &count), SCN_OK);
    EXPECT_EQ(count, 0U);

    scn_engine_destroy(engine);
}

TEST(CApiTest, AddSpeedActionExDefaultsMatchPlainVariant) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego vehicle", SCN_CONTROL_ENGINE), SCN_OK);
    EXPECT_EQ(scn_engine_add_speed_action_ex(engine, "ego", 12.0, 0.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);

    scn_entity_state state{};
    ASSERT_EQ(scn_engine_get_state(engine, "ego", &state), SCN_OK);
    EXPECT_EQ(state.speed, 12.0);
    scn_engine_destroy(engine);
}

TEST(CApiTest, AddSpeedActionExRepeatsUpToMaximumExecutionCount) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego vehicle", SCN_CONTROL_ENGINE), SCN_OK);
    // Trigger holds from t = 1 on; the event spends two executions on two
    // consecutive evaluations, then completes (§8.3.3.2).
    ASSERT_EQ(scn_engine_add_speed_action_ex(engine, "ego", 4.0, 1.0, SCN_PRIORITY_PARALLEL, 2),
              SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);

    ASSERT_EQ(scn_engine_step(engine, 1.0), SCN_OK);
    scn_entity_state state{};
    ASSERT_EQ(scn_engine_get_state(engine, "ego", &state), SCN_OK);
    EXPECT_EQ(state.speed, 4.0);

    // A second execution reapplies the same speed; a third never happens.
    ASSERT_EQ(scn_engine_report_state(engine, "ego", &state), SCN_ERROR_INVALID_CONTROL_MODE);
    ASSERT_EQ(scn_engine_step(engine, 1.0), SCN_OK);
    ASSERT_EQ(scn_engine_step(engine, 1.0), SCN_OK);
    scn_engine_destroy(engine);
}

TEST(CApiTest, AddSpeedActionExRejectsInvalidArguments) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    EXPECT_EQ(scn_engine_add_speed_action_ex(engine, "ego", 1.0, 0.0, SCN_PRIORITY_PARALLEL, -1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_speed_action_ex(engine, "ego", 1.0, 0.0,
                                             static_cast<scn_event_priority>(99), 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_speed_action_ex(nullptr, "ego", 1.0, 0.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_speed_action_ex(engine, nullptr, 1.0, 0.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    scn_engine_destroy(engine);
}
