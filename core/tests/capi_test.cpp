// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
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

    const scn_entity_state reported{4.0, 5.0, 0.0, 1.5, 3.0, 0.2, -0.1};
    ASSERT_EQ(scn_engine_report_state(engine, "npc", &reported), SCN_OK);
    scn_entity_state npc_state{};
    ASSERT_EQ(scn_engine_get_state(engine, "npc", &npc_state), SCN_OK);
    EXPECT_EQ(npc_state.x, reported.x);
    EXPECT_EQ(npc_state.heading, reported.heading);
    EXPECT_EQ(npc_state.speed, reported.speed);
    // The full pose (pitch/roll) round-trips through the C ABI.
    EXPECT_EQ(npc_state.pitch, reported.pitch);
    EXPECT_EQ(npc_state.roll, reported.roll);

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

    // Entity taxonomy builders and metadata queries reject nulls.
    const scn_bounding_box box{0.0, 0.0, 0.0, 4.0, 2.0, 1.5};
    const scn_performance perf{50.0, 3.0, 9.0, -1.0, -1.0};
    EXPECT_EQ(
        scn_engine_add_vehicle(nullptr, "v", "v", SCN_CONTROL_HOST, SCN_VEHICLE_CAR, &box, &perf),
        SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(
        scn_engine_add_vehicle(engine, "v", "v", SCN_CONTROL_HOST, SCN_VEHICLE_CAR, nullptr, &perf),
        SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(
        scn_engine_add_vehicle(engine, "v", "v", SCN_CONTROL_HOST, SCN_VEHICLE_CAR, &box, nullptr),
        SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_pedestrian(engine, "p", "p", SCN_CONTROL_HOST,
                                        SCN_PEDESTRIAN_PEDESTRIAN, nullptr),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(
        scn_engine_add_misc_object(engine, "m", "m", SCN_CONTROL_HOST, SCN_MISC_POLE, nullptr),
        SCN_ERROR_INVALID_ARGUMENT);
    scn_object_type type{};
    scn_bounding_box out_box{};
    scn_performance out_perf{};
    EXPECT_EQ(scn_engine_entity_object_type(engine, nullptr, &type), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_entity_object_type(engine, "v", nullptr), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_entity_bounding_box(engine, "v", nullptr), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_entity_performance(nullptr, "v", &out_perf), SCN_ERROR_INVALID_ARGUMENT);
    (void)out_box;

    // Longitudinal builders reject nulls.
    const scn_transition_dynamics dynamics{SCN_DYNAMICS_SHAPE_LINEAR, SCN_DYNAMICS_DIMENSION_TIME,
                                           1.0, SCN_FOLLOWING_MODE_POSITION};
    EXPECT_EQ(scn_engine_add_speed_action_dyn(engine, nullptr, 8.0, &dynamics, 0.0,
                                              SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(
        scn_engine_add_speed_action_dyn(engine, "ego", 8.0, nullptr, 0.0, SCN_PRIORITY_PARALLEL, 1),
        SCN_ERROR_INVALID_ARGUMENT);
    const scn_speed_profile_entry entries[] = {{5.0, 1.0}};
    EXPECT_EQ(scn_engine_add_speed_profile_action(engine, nullptr, entries, 1,
                                                  SCN_FOLLOWING_MODE_POSITION, 0.0,
                                                  SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_speed_profile_action(engine, "ego", nullptr, 1,
                                                  SCN_FOLLOWING_MODE_POSITION, 0.0,
                                                  SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);

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

TEST(CApiTest, AddSpeedActionDynRampsAcrossSteps) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego vehicle", SCN_CONTROL_ENGINE), SCN_OK);
    const scn_transition_dynamics dynamics{SCN_DYNAMICS_SHAPE_LINEAR, SCN_DYNAMICS_DIMENSION_TIME,
                                           4.0, SCN_FOLLOWING_MODE_POSITION};
    ASSERT_EQ(scn_engine_add_speed_action_dyn(engine, "ego", 8.0, &dynamics, 0.0,
                                              SCN_PRIORITY_PARALLEL, 1),
              SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);

    scn_entity_state state{};
    ASSERT_EQ(scn_engine_get_state(engine, "ego", &state), SCN_OK);
    EXPECT_EQ(state.speed, 0.0); // installed, not yet advanced

    ASSERT_EQ(scn_engine_step(engine, 2.0), SCN_OK);
    ASSERT_EQ(scn_engine_get_state(engine, "ego", &state), SCN_OK);
    EXPECT_EQ(state.speed, 4.0); // halfway up the 4 s ramp
    ASSERT_EQ(scn_engine_step(engine, 2.0), SCN_OK);
    ASSERT_EQ(scn_engine_get_state(engine, "ego", &state), SCN_OK);
    EXPECT_EQ(state.speed, 8.0); // target reached
    scn_engine_destroy(engine);
}

TEST(CApiTest, AddSpeedProfileActionFollowsTargets) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego vehicle", SCN_CONTROL_ENGINE), SCN_OK);
    const scn_speed_profile_entry entries[] = {{10.0, 2.0}, {4.0, 2.0}};
    ASSERT_EQ(scn_engine_add_speed_profile_action(engine, "ego", entries, 2,
                                                  SCN_FOLLOWING_MODE_POSITION, 0.0,
                                                  SCN_PRIORITY_PARALLEL, 1),
              SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);

    ASSERT_EQ(scn_engine_step(engine, 2.0), SCN_OK);
    scn_entity_state state{};
    ASSERT_EQ(scn_engine_get_state(engine, "ego", &state), SCN_OK);
    EXPECT_EQ(state.speed, 10.0); // first target
    ASSERT_EQ(scn_engine_step(engine, 2.0), SCN_OK);
    ASSERT_EQ(scn_engine_get_state(engine, "ego", &state), SCN_OK);
    EXPECT_EQ(state.speed, 4.0); // second target
    scn_engine_destroy(engine);
}

TEST(CApiTest, LongitudinalBuildersRejectInvalidArguments) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    const scn_transition_dynamics dynamics{SCN_DYNAMICS_SHAPE_LINEAR, SCN_DYNAMICS_DIMENSION_TIME,
                                           4.0, SCN_FOLLOWING_MODE_POSITION};
    // NULL dynamics, out-of-range shape, NULL engine/entity.
    EXPECT_EQ(
        scn_engine_add_speed_action_dyn(engine, "ego", 8.0, nullptr, 0.0, SCN_PRIORITY_PARALLEL, 1),
        SCN_ERROR_INVALID_ARGUMENT);
    const scn_transition_dynamics bad_shape{static_cast<scn_dynamics_shape>(99),
                                            SCN_DYNAMICS_DIMENSION_TIME, 4.0,
                                            SCN_FOLLOWING_MODE_POSITION};
    EXPECT_EQ(scn_engine_add_speed_action_dyn(engine, "ego", 8.0, &bad_shape, 0.0,
                                              SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_speed_action_dyn(nullptr, "ego", 8.0, &dynamics, 0.0,
                                              SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    // Profile: NULL entries, zero count.
    const scn_speed_profile_entry entries[] = {{5.0, 1.0}};
    EXPECT_EQ(scn_engine_add_speed_profile_action(engine, "ego", nullptr, 1,
                                                  SCN_FOLLOWING_MODE_POSITION, 0.0,
                                                  SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_speed_profile_action(engine, "ego", entries, 0,
                                                  SCN_FOLLOWING_MODE_POSITION, 0.0,
                                                  SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    scn_engine_destroy(engine);
}

TEST(CApiTest, AddRelativeSpeedActionResolvesAgainstReference) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_entity(engine, "lead", "lead", SCN_CONTROL_ENGINE), SCN_OK);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego", SCN_CONTROL_ENGINE), SCN_OK);
    // lead cruises at 8 m/s from t=0; ego jumps to lead + 4 = 12 m/s (Step).
    ASSERT_EQ(scn_engine_add_speed_action(engine, "lead", 8.0, 0.0), SCN_OK);
    const scn_transition_dynamics step{SCN_DYNAMICS_SHAPE_STEP, SCN_DYNAMICS_DIMENSION_TIME, 0.0,
                                       SCN_FOLLOWING_MODE_POSITION};
    ASSERT_EQ(scn_engine_add_relative_speed_action(engine, "ego", "lead", 4.0,
                                                   SCN_SPEED_TARGET_DELTA, 0, &step, 0.0,
                                                   SCN_PRIORITY_PARALLEL, 1),
              SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);
    scn_entity_state state{};
    ASSERT_EQ(scn_engine_get_state(engine, "ego", &state), SCN_OK);
    EXPECT_EQ(state.speed, 12.0);
    scn_engine_destroy(engine);
}

TEST(CApiTest, AddTeleportActionMovesEntity) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego", SCN_CONTROL_ENGINE), SCN_OK);
    ASSERT_EQ(scn_engine_add_teleport_action(engine, "ego", 12.0, -3.0, 0.5, 1.0,
                                             SCN_PRIORITY_PARALLEL, 1),
              SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);
    ASSERT_EQ(scn_engine_step(engine, 1.0), SCN_OK); // t=1: teleport fires
    scn_entity_state state{};
    ASSERT_EQ(scn_engine_get_state(engine, "ego", &state), SCN_OK);
    EXPECT_EQ(state.x, 12.0);
    EXPECT_EQ(state.y, -3.0);
    EXPECT_EQ(state.z, 0.5);
    scn_engine_destroy(engine);
}

TEST(CApiTest, PrivateActionBuildersRejectInvalidArguments) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    const scn_transition_dynamics step{SCN_DYNAMICS_SHAPE_STEP, SCN_DYNAMICS_DIMENSION_TIME, 0.0,
                                       SCN_FOLLOWING_MODE_POSITION};
    // Relative speed: NULL dynamics, NULL reference, out-of-range value_type, NULL engine.
    EXPECT_EQ(scn_engine_add_relative_speed_action(engine, "ego", "lead", 4.0,
                                                   SCN_SPEED_TARGET_DELTA, 0, nullptr, 0.0,
                                                   SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_relative_speed_action(engine, "ego", nullptr, 4.0,
                                                   SCN_SPEED_TARGET_DELTA, 0, &step, 0.0,
                                                   SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_relative_speed_action(engine, "ego", "lead", 4.0,
                                                   static_cast<scn_speed_target_value_type>(99), 0,
                                                   &step, 0.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_relative_speed_action(nullptr, "ego", "lead", 4.0,
                                                   SCN_SPEED_TARGET_DELTA, 0, &step, 0.0,
                                                   SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    // Teleport: NULL engine/entity, negative execution count.
    EXPECT_EQ(scn_engine_add_teleport_action(nullptr, "ego", 0.0, 0.0, 0.0, 0.0,
                                             SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_teleport_action(engine, nullptr, 0.0, 0.0, 0.0, 0.0,
                                             SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_teleport_action(engine, "ego", 0.0, 0.0, 0.0, 0.0,
                                             SCN_PRIORITY_PARALLEL, -1),
              SCN_ERROR_INVALID_ARGUMENT);
    scn_engine_destroy(engine);
}

TEST(CApiTest, NamedValueHostInterfaceRoundTrip) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    // Builders take effect at the next init.
    ASSERT_EQ(scn_engine_set_parameter(engine, "speedLimit", "30"), SCN_OK);
    ASSERT_EQ(scn_engine_declare_variable(engine, "v", "0"), SCN_OK);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego vehicle", SCN_CONTROL_ENGINE), SCN_OK);
    // A user-defined value staged before init survives it.
    ASSERT_EQ(scn_engine_set_user_defined_value(engine, "sig", "on"), SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);

    const char* value = nullptr;
    ASSERT_EQ(scn_engine_get_variable(engine, "v", &value), SCN_OK);
    EXPECT_STREQ(value, "0"); // seeded from the declaration
    ASSERT_EQ(scn_engine_set_variable(engine, "v", "42"), SCN_OK);
    ASSERT_EQ(scn_engine_get_variable(engine, "v", &value), SCN_OK);
    EXPECT_STREQ(value, "42");

    ASSERT_EQ(scn_engine_get_user_defined_value(engine, "sig", &value), SCN_OK);
    EXPECT_STREQ(value, "on");
    ASSERT_EQ(scn_engine_set_user_defined_value(engine, "sig", "off"), SCN_OK);
    ASSERT_EQ(scn_engine_get_user_defined_value(engine, "sig", &value), SCN_OK);
    EXPECT_STREQ(value, "off");

    // A valid date-time is accepted; an out-of-range one is rejected.
    EXPECT_EQ(scn_engine_set_date_time(engine, 2000, 1, 1, 12, 0, 0, 0, 0), SCN_OK);
    EXPECT_EQ(scn_engine_set_date_time(engine, 2001, 2, 29, 0, 0, 0, 0, 0),
              SCN_ERROR_INVALID_ARGUMENT);

    scn_engine_destroy(engine);
}

TEST(CApiTest, NamedValueErrorPaths) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);

    // Runtime setters need init.
    EXPECT_EQ(scn_engine_set_variable(engine, "v", "1"), SCN_ERROR_NOT_INITIALIZED);

    ASSERT_EQ(scn_engine_declare_variable(engine, "v", "0"), SCN_OK);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego vehicle", SCN_CONTROL_ENGINE), SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);

    EXPECT_EQ(scn_engine_set_variable(engine, "ghost", "1"), SCN_ERROR_UNKNOWN_NAME);

    // Unknown name leaves the out sentinel untouched.
    const char* out = "sentinel";
    EXPECT_EQ(scn_engine_get_variable(engine, "ghost", &out), SCN_ERROR_UNKNOWN_NAME);
    EXPECT_STREQ(out, "sentinel");
    EXPECT_EQ(scn_engine_get_user_defined_value(engine, "never-set", &out), SCN_ERROR_UNKNOWN_NAME);
    EXPECT_STREQ(out, "sentinel");

    scn_engine_destroy(engine);
}

TEST(CApiTest, NamedValueBorrowedStringStaysValidUntilNextAccess) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_declare_variable(engine, "v", "first"), SCN_OK);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego vehicle", SCN_CONTROL_ENGINE), SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);

    const char* value = nullptr;
    ASSERT_EQ(scn_engine_get_variable(engine, "v", &value), SCN_OK);
    // The borrowed pointer is readable before any further engine call.
    EXPECT_STREQ(value, "first");
    scn_engine_destroy(engine);
}

TEST(CApiTest, NullArgumentsAreRejectedForNamedValues) {
    EXPECT_EQ(scn_engine_set_parameter(nullptr, "p", "1"), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_declare_variable(nullptr, "v", "1"), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_set_variable(nullptr, "v", "1"), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_get_variable(nullptr, "v", nullptr), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_set_user_defined_value(nullptr, "u", "1"), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_get_user_defined_value(nullptr, "u", nullptr), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_set_date_time(nullptr, 2000, 1, 1, 0, 0, 0, 0, 0),
              SCN_ERROR_INVALID_ARGUMENT);

    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    const char* out = nullptr;
    EXPECT_EQ(scn_engine_set_parameter(engine, nullptr, "1"), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_declare_variable(engine, "v", nullptr), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_set_variable(engine, nullptr, "1"), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_get_variable(engine, nullptr, &out), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_get_variable(engine, "v", nullptr), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_get_user_defined_value(engine, "u", nullptr), SCN_ERROR_INVALID_ARGUMENT);
    scn_engine_destroy(engine);
}

TEST(CApiTest, DeprecatedFeatureStatusHasStableAbiValue) {
    // ABI: appended after SCN_ERROR_UNKNOWN_NAME (11); never renumbered.
    EXPECT_EQ(SCN_ERROR_UNKNOWN_NAME, 11);
    EXPECT_EQ(SCN_ERROR_DEPRECATED_FEATURE, 12);
}

TEST(CApiTest, EntityTaxonomyBuildersAndQueries) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);

    const scn_bounding_box car_box{1.4, 0.0, 0.8, 4.6, 2.0, 1.5};
    const scn_performance car_perf{60.0, 5.0, 9.0, 2.5, -1.0};
    ASSERT_EQ(scn_engine_add_vehicle(engine, "ego", "ego", SCN_CONTROL_HOST, SCN_VEHICLE_CAR,
                                     &car_box, &car_perf),
              SCN_OK);
    const scn_bounding_box ped_box{0.0, 0.0, 0.9, 0.5, 0.6, 1.8};
    ASSERT_EQ(scn_engine_add_pedestrian(engine, "ped", "ped", SCN_CONTROL_HOST,
                                        SCN_PEDESTRIAN_PEDESTRIAN, &ped_box),
              SCN_OK);
    const scn_bounding_box pole_box{0.0, 0.0, 1.5, 0.2, 0.2, 3.0};
    ASSERT_EQ(scn_engine_add_misc_object(engine, "pole", "pole", SCN_CONTROL_HOST, SCN_MISC_POLE,
                                         &pole_box),
              SCN_OK);

    // Object type reflects the alternative.
    scn_object_type type{};
    ASSERT_EQ(scn_engine_entity_object_type(engine, "ego", &type), SCN_OK);
    EXPECT_EQ(type, SCN_OBJECT_VEHICLE);
    ASSERT_EQ(scn_engine_entity_object_type(engine, "ped", &type), SCN_OK);
    EXPECT_EQ(type, SCN_OBJECT_PEDESTRIAN);
    ASSERT_EQ(scn_engine_entity_object_type(engine, "pole", &type), SCN_OK);
    EXPECT_EQ(type, SCN_OBJECT_MISC);

    // Bounding box round-trips for any object type.
    scn_bounding_box box{};
    ASSERT_EQ(scn_engine_entity_bounding_box(engine, "ego", &box), SCN_OK);
    EXPECT_EQ(box.length, 4.6);
    EXPECT_EQ(box.center_x, 1.4);
    ASSERT_EQ(scn_engine_entity_bounding_box(engine, "pole", &box), SCN_OK);
    EXPECT_EQ(box.height, 3.0);

    // Performance is vehicle-only; the absent rate reads back as a negative
    // sentinel while the supplied one survives.
    scn_performance perf{};
    ASSERT_EQ(scn_engine_entity_performance(engine, "ego", &perf), SCN_OK);
    EXPECT_EQ(perf.max_speed, 60.0);
    EXPECT_EQ(perf.max_acceleration_rate, 2.5);
    EXPECT_LT(perf.max_deceleration_rate, 0.0);
    // A pedestrian has no performance.
    EXPECT_EQ(scn_engine_entity_performance(engine, "ped", &perf), SCN_ERROR_INVALID_ARGUMENT);

    // The queries survive init (they read the authored scenario).
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);
    ASSERT_EQ(scn_engine_entity_object_type(engine, "ego", &type), SCN_OK);
    EXPECT_EQ(type, SCN_OBJECT_VEHICLE);

    scn_engine_destroy(engine);
}

TEST(CApiTest, EntityTaxonomyErrorPaths) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);

    const scn_bounding_box box{0.0, 0.0, 0.0, 4.0, 2.0, 1.5};
    const scn_performance perf{50.0, 3.0, 9.0, -1.0, -1.0};

    // A category outside its enumeration is rejected (as unsigned, so a
    // negative value wraps above the max).
    EXPECT_EQ(scn_engine_add_vehicle(engine, "v", "v", SCN_CONTROL_HOST,
                                     static_cast<scn_vehicle_category>(999), &box, &perf),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_misc_object(engine, "m", "m", SCN_CONTROL_HOST,
                                         static_cast<scn_misc_object_category>(-1), &box),
              SCN_ERROR_INVALID_ARGUMENT);

    // Unknown id vs. present-but-unclassified: a bare participant has no object.
    ASSERT_EQ(scn_engine_add_entity(engine, "bare", "bare", SCN_CONTROL_HOST), SCN_OK);
    scn_object_type type{};
    scn_bounding_box out_box{};
    EXPECT_EQ(scn_engine_entity_object_type(engine, "missing", &type), SCN_ERROR_UNKNOWN_ENTITY);
    EXPECT_EQ(scn_engine_entity_object_type(engine, "bare", &type), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_entity_bounding_box(engine, "bare", &out_box), SCN_ERROR_INVALID_ARGUMENT);

    scn_engine_destroy(engine);
}

// --- p5-s5: routing, distance keeping, controllers, visibility -------------

TEST(CApiTest, LongitudinalDistanceActionKeepsTheGap) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_entity(engine, "lead", "lead", SCN_CONTROL_ENGINE), SCN_OK);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego", SCN_CONTROL_ENGINE), SCN_OK);
    ASSERT_EQ(scn_engine_add_teleport_action(engine, "lead", 100.0, 0.0, 0.0, 0.0,
                                             SCN_PRIORITY_PARALLEL, 1),
              SCN_OK);
    ASSERT_EQ(scn_engine_add_speed_action(engine, "lead", 10.0, 0.0), SCN_OK);
    ASSERT_EQ(scn_engine_add_speed_action(engine, "ego", 10.0, 0.0), SCN_OK);

    const scn_dynamic_constraints constraints{2.0, -1.0, 3.0, -1.0, 40.0};
    ASSERT_EQ(scn_engine_add_longitudinal_distance_action(
                  engine, "ego", "lead", /*distance=*/25.0, /*time_gap=*/-1.0, /*freespace=*/0,
                  /*continuous=*/1, SCN_COORDINATE_SYSTEM_ENTITY,
                  SCN_LONGITUDINAL_DISPLACEMENT_TRAILING, &constraints, 1.0, SCN_PRIORITY_PARALLEL,
                  1),
              SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);
    for (int i = 0; i < 400; ++i) {
        ASSERT_EQ(scn_engine_step(engine, 0.1), SCN_OK);
    }
    scn_entity_state ego{};
    scn_entity_state lead{};
    ASSERT_EQ(scn_engine_get_state(engine, "ego", &ego), SCN_OK);
    ASSERT_EQ(scn_engine_get_state(engine, "lead", &lead), SCN_OK);
    EXPECT_NEAR(lead.x - ego.x, 25.0, 1e-6);
    scn_engine_destroy(engine);
}

TEST(CApiTest, LongitudinalDistanceActionRejectsBadTargets) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    // Neither target given.
    EXPECT_EQ(scn_engine_add_longitudinal_distance_action(
                  engine, "ego", "lead", -1.0, -1.0, 0, 0, SCN_COORDINATE_SYSTEM_ENTITY,
                  SCN_LONGITUDINAL_DISPLACEMENT_ANY, nullptr, 0.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    // Both targets given.
    EXPECT_EQ(scn_engine_add_longitudinal_distance_action(
                  engine, "ego", "lead", 10.0, 2.0, 0, 0, SCN_COORDINATE_SYSTEM_ENTITY,
                  SCN_LONGITUDINAL_DISPLACEMENT_ANY, nullptr, 0.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    // Out-of-range enums and a negative execution count.
    EXPECT_EQ(scn_engine_add_longitudinal_distance_action(
                  engine, "ego", "lead", 10.0, -1.0, 0, 0, static_cast<scn_coordinate_system>(99),
                  SCN_LONGITUDINAL_DISPLACEMENT_ANY, nullptr, 0.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_longitudinal_distance_action(
                  engine, "ego", "lead", 10.0, -1.0, 0, 0, SCN_COORDINATE_SYSTEM_ENTITY,
                  static_cast<scn_longitudinal_displacement>(99), nullptr, 0.0,
                  SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_longitudinal_distance_action(
                  engine, "ego", "lead", 10.0, -1.0, 0, 0, SCN_COORDINATE_SYSTEM_ENTITY,
                  SCN_LONGITUDINAL_DISPLACEMENT_ANY, nullptr, 0.0, SCN_PRIORITY_PARALLEL, -1),
              SCN_ERROR_INVALID_ARGUMENT);
    scn_engine_destroy(engine);
}

TEST(CApiTest, RoutingActionsBuildAndQueryRoutes) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego", SCN_CONTROL_ENGINE), SCN_OK);
    ASSERT_EQ(scn_engine_add_speed_action(engine, "ego", 10.0, 0.0), SCN_OK);

    const scn_waypoint waypoints[] = {
        {0.0, 0.0, 0.0, SCN_ROUTE_STRATEGY_SHORTEST},
        {120.0, 8.0, 0.0, SCN_ROUTE_STRATEGY_FASTEST},
    };
    ASSERT_EQ(scn_engine_add_assign_route_action(engine, "ego", "r1", waypoints, 2, /*closed=*/0,
                                                 1.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);

    // Before the action fires the entity has no route.
    size_t count = 42;
    EXPECT_EQ(scn_engine_entity_route_info(engine, "ego", &count, nullptr),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(count, 42U); // out params untouched on error
    EXPECT_EQ(scn_engine_entity_route_info(engine, "missing", &count, nullptr),
              SCN_ERROR_UNKNOWN_ENTITY);

    ASSERT_EQ(scn_engine_step(engine, 1.0), SCN_OK);
    int closed = 7;
    ASSERT_EQ(scn_engine_entity_route_info(engine, "ego", &count, &closed), SCN_OK);
    EXPECT_EQ(count, 2U);
    EXPECT_EQ(closed, 0);
    scn_waypoint out{};
    ASSERT_EQ(scn_engine_entity_route_waypoint_at(engine, "ego", 1, &out), SCN_OK);
    EXPECT_EQ(out.x, 120.0);
    EXPECT_EQ(out.y, 8.0);
    EXPECT_EQ(out.strategy, SCN_ROUTE_STRATEGY_FASTEST);
    EXPECT_EQ(scn_engine_entity_route_waypoint_at(engine, "ego", 2, &out),
              SCN_ERROR_INVALID_ARGUMENT);

    // AcquirePositionAction overwrites it with the implicit two-waypoint route.
    scn_engine_destroy(engine);
}

TEST(CApiTest, AcquirePositionActionInstallsTheImplicitRoute) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego", SCN_CONTROL_ENGINE), SCN_OK);
    ASSERT_EQ(scn_engine_add_speed_action(engine, "ego", 10.0, 0.0), SCN_OK);
    ASSERT_EQ(scn_engine_add_acquire_position_action(engine, "ego", 300.0, 0.0, 0.0, 2.0,
                                                     SCN_PRIORITY_PARALLEL, 1),
              SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);
    ASSERT_EQ(scn_engine_step(engine, 1.0), SCN_OK);
    ASSERT_EQ(scn_engine_step(engine, 1.0), SCN_OK);

    size_t count = 0;
    ASSERT_EQ(scn_engine_entity_route_info(engine, "ego", &count, nullptr), SCN_OK);
    EXPECT_EQ(count, 2U);
    scn_waypoint first{};
    scn_waypoint last{};
    ASSERT_EQ(scn_engine_entity_route_waypoint_at(engine, "ego", 0, &first), SCN_OK);
    ASSERT_EQ(scn_engine_entity_route_waypoint_at(engine, "ego", 1, &last), SCN_OK);
    EXPECT_NEAR(first.x, 10.0, 1e-9); // the position when the action fired
    EXPECT_EQ(last.x, 300.0);
    scn_engine_destroy(engine);
}

TEST(CApiTest, FollowTrajectoryActionMovesTheEntityAlongThePolyline) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego", SCN_CONTROL_ENGINE), SCN_OK);
    ASSERT_EQ(scn_engine_add_speed_action(engine, "ego", 10.0, 0.0), SCN_OK);
    const scn_trajectory_vertex vertices[] = {
        {0.0, 0.0, 0.0, 0.0, 0},
        {0.0, 100.0, 0.0, 0.0, 0},
    };
    ASSERT_EQ(scn_engine_add_follow_trajectory_action(
                  engine, "ego", "t1", vertices, 2, /*closed=*/0, SCN_FOLLOWING_MODE_POSITION,
                  /*timing=*/nullptr, /*initial_distance_offset=*/0.0, 1.0, SCN_PRIORITY_PARALLEL,
                  1),
              SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);
    for (int i = 0; i < 4; ++i) {
        ASSERT_EQ(scn_engine_step(engine, 1.0), SCN_OK);
    }
    scn_entity_state state{};
    ASSERT_EQ(scn_engine_get_state(engine, "ego", &state), SCN_OK);
    // Teleported to the trajectory start, then 30 m along it.
    EXPECT_NEAR(state.x, 0.0, 1e-9);
    EXPECT_NEAR(state.y, 30.0, 1e-9);
    scn_engine_destroy(engine);
}

TEST(CApiTest, FollowTrajectoryActionWithTimingDrivesTheSpeed) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego", SCN_CONTROL_ENGINE), SCN_OK);
    const scn_trajectory_vertex vertices[] = {
        {0.0, 0.0, 0.0, 0.0, 1},
        {80.0, 0.0, 0.0, 8.0, 1},
    };
    const scn_timing timing{SCN_REFERENCE_CONTEXT_ABSOLUTE, 1.0, 0.0};
    ASSERT_EQ(scn_engine_add_follow_trajectory_action(engine, "ego", "t1", vertices, 2, 0,
                                                      SCN_FOLLOWING_MODE_POSITION, &timing, 0.0,
                                                      0.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);
    for (int i = 0; i < 4; ++i) {
        ASSERT_EQ(scn_engine_step(engine, 1.0), SCN_OK);
    }
    scn_entity_state state{};
    ASSERT_EQ(scn_engine_get_state(engine, "ego", &state), SCN_OK);
    EXPECT_NEAR(state.x, 40.0, 1e-9);
    EXPECT_NEAR(state.speed, 10.0, 1e-9); // 80 m over 8 s
    scn_engine_destroy(engine);
}

TEST(CApiTest, ControllerAndVisibilityActionsRoundTrip) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego", SCN_CONTROL_ENGINE), SCN_OK);
    ASSERT_EQ(scn_engine_add_speed_action(engine, "ego", 10.0, 0.0), SCN_OK);

    const char* names[] = {"model", "aggressiveness"};
    const char* values[] = {"idm", "0.7"};
    ASSERT_EQ(scn_engine_add_assign_controller_action(
                  engine, "ego", "driver", SCN_CONTROLLER_TYPE_MOVEMENT, names, values, 2,
                  /*activate_lateral=*/-1, /*activate_longitudinal=*/1, 1.0, SCN_PRIORITY_PARALLEL,
                  1),
              SCN_OK);
    ASSERT_EQ(scn_engine_add_visibility_action(engine, "ego", /*graphics=*/0, /*sensors=*/1,
                                               /*traffic=*/0, 2.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_OK);
    ASSERT_EQ(scn_engine_add_activate_controller_action(engine, "ego", /*lateral=*/-1,
                                                        /*longitudinal=*/0, 3.0,
                                                        SCN_PRIORITY_PARALLEL, 1),
              SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);

    // Defaults before anything fires.
    scn_entity_visibility visibility{};
    ASSERT_EQ(scn_engine_entity_visibility(engine, "ego", &visibility), SCN_OK);
    EXPECT_EQ(visibility.graphics, 1);
    EXPECT_EQ(visibility.traffic, 1);
    scn_controller_activation activation{};
    ASSERT_EQ(scn_engine_entity_controller_activation(engine, "ego", &activation), SCN_OK);
    EXPECT_EQ(activation.lateral, 1);
    EXPECT_EQ(activation.longitudinal, 1);
    scn_controller_type type{};
    EXPECT_EQ(scn_engine_entity_controller_type(engine, "ego", &type), SCN_ERROR_INVALID_ARGUMENT);

    ASSERT_EQ(scn_engine_step(engine, 1.0), SCN_OK); // assign controller
    ASSERT_EQ(scn_engine_entity_controller_type(engine, "ego", &type), SCN_OK);
    EXPECT_EQ(type, SCN_CONTROLLER_TYPE_MOVEMENT);
    const char* name = nullptr;
    ASSERT_EQ(scn_engine_entity_controller_name(engine, "ego", &name), SCN_OK);
    ASSERT_NE(name, nullptr);
    EXPECT_STREQ(name, "driver");

    ASSERT_EQ(scn_engine_step(engine, 1.0), SCN_OK); // visibility
    ASSERT_EQ(scn_engine_entity_visibility(engine, "ego", &visibility), SCN_OK);
    EXPECT_EQ(visibility.graphics, 0);
    EXPECT_EQ(visibility.sensors, 1);
    EXPECT_EQ(visibility.traffic, 0);

    ASSERT_EQ(scn_engine_step(engine, 1.0), SCN_OK); // deactivate longitudinal
    ASSERT_EQ(scn_engine_entity_controller_activation(engine, "ego", &activation), SCN_OK);
    EXPECT_EQ(activation.lateral, 1); // tri-state -1 left it alone
    EXPECT_EQ(activation.longitudinal, 0);

    EXPECT_EQ(scn_engine_entity_visibility(engine, "missing", &visibility),
              SCN_ERROR_UNKNOWN_ENTITY);
    EXPECT_EQ(scn_engine_entity_controller_activation(engine, "missing", &activation),
              SCN_ERROR_UNKNOWN_ENTITY);
    scn_engine_destroy(engine);
}

TEST(CApiTest, ControllerActivationOutsideItsTypeFailsInit) {
    // per rule asam.net:xosc:1.2.0:scenario_logic.controller_activation
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego", SCN_CONTROL_ENGINE), SCN_OK);
    ASSERT_EQ(scn_engine_add_assign_controller_action(
                  engine, "ego", "lights", SCN_CONTROLLER_TYPE_LIGHTING, nullptr, nullptr, 0,
                  /*activate_lateral=*/1, /*activate_longitudinal=*/-1, 0.0, SCN_PRIORITY_PARALLEL,
                  1),
              SCN_OK);
    EXPECT_EQ(scn_engine_init(engine), SCN_ERROR_VALIDATION);
    scn_engine_destroy(engine);
}

TEST(CApiTest, NullArgumentsAreRejectedForPrivateActions) {
    const scn_waypoint waypoints[] = {
        {0.0, 0.0, 0.0, SCN_ROUTE_STRATEGY_SHORTEST},
        {1.0, 0.0, 0.0, SCN_ROUTE_STRATEGY_SHORTEST},
    };
    const scn_trajectory_vertex vertices[] = {
        {0.0, 0.0, 0.0, 0.0, 0},
        {1.0, 0.0, 0.0, 0.0, 0},
    };
    EXPECT_EQ(scn_engine_add_longitudinal_distance_action(
                  nullptr, "ego", "lead", 1.0, -1.0, 0, 0, SCN_COORDINATE_SYSTEM_ENTITY,
                  SCN_LONGITUDINAL_DISPLACEMENT_ANY, nullptr, 0.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_assign_route_action(nullptr, "ego", "r", waypoints, 2, 0, 0.0,
                                                 SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_acquire_position_action(nullptr, "ego", 0.0, 0.0, 0.0, 0.0,
                                                     SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_follow_trajectory_action(nullptr, "ego", "t", vertices, 2, 0,
                                                      SCN_FOLLOWING_MODE_POSITION, nullptr, 0.0,
                                                      0.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_assign_controller_action(
                  nullptr, "ego", "c", SCN_CONTROLLER_TYPE_MOVEMENT, nullptr, nullptr, 0, -1, -1,
                  0.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_activate_controller_action(nullptr, "ego", -1, -1, 0.0,
                                                        SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(
        scn_engine_add_visibility_action(nullptr, "ego", 1, 1, 1, 0.0, SCN_PRIORITY_PARALLEL, 1),
        SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_entity_visibility(nullptr, "ego", nullptr), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_entity_controller_activation(nullptr, "ego", nullptr),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_entity_controller_type(nullptr, "ego", nullptr),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_entity_controller_name(nullptr, "ego", nullptr),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_entity_route_info(nullptr, "ego", nullptr, nullptr),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_entity_route_waypoint_at(nullptr, "ego", 0, nullptr),
              SCN_ERROR_INVALID_ARGUMENT);

    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    // Null ids, null arrays, and too-short arrays.
    EXPECT_EQ(scn_engine_add_assign_route_action(engine, nullptr, "r", waypoints, 2, 0, 0.0,
                                                 SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_assign_route_action(engine, "ego", "r", nullptr, 2, 0, 0.0,
                                                 SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_assign_route_action(engine, "ego", "r", waypoints, 1, 0, 0.0,
                                                 SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_follow_trajectory_action(engine, "ego", "t", vertices, 1, 0,
                                                      SCN_FOLLOWING_MODE_POSITION, nullptr, 0.0,
                                                      0.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_assign_controller_action(
                  engine, "ego", nullptr, SCN_CONTROLLER_TYPE_MOVEMENT, nullptr, nullptr, 0, -1, -1,
                  0.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    // A non-zero property count with null arrays.
    EXPECT_EQ(scn_engine_add_assign_controller_action(
                  engine, "ego", "c", SCN_CONTROLLER_TYPE_MOVEMENT, nullptr, nullptr, 1, -1, -1,
                  0.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(
        scn_engine_add_visibility_action(engine, nullptr, 1, 1, 1, 0.0, SCN_PRIORITY_PARALLEL, 1),
        SCN_ERROR_INVALID_ARGUMENT);
    scn_engine_destroy(engine);
}

// --- Global and infrastructure actions (p5-s6) ----------------------------

TEST(CApiTest, VariableAndParameterActionsThroughTheAbi) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego", SCN_CONTROL_ENGINE), SCN_OK);
    ASSERT_EQ(scn_engine_declare_variable(engine, "counter", "1"), SCN_OK);
    ASSERT_EQ(scn_engine_set_parameter(engine, "gap", "8"), SCN_OK);

    ASSERT_EQ(
        scn_engine_add_variable_set_action(engine, "counter", "10", 1.0, SCN_PRIORITY_PARALLEL, 1),
        SCN_OK);
    ASSERT_EQ(scn_engine_add_variable_modify_action(engine, "counter", SCN_MODIFY_MULTIPLY, 2.5,
                                                    2.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_OK);
    ASSERT_EQ(
        scn_engine_add_parameter_set_action(engine, "gap", "12", 3.0, SCN_PRIORITY_PARALLEL, 1),
        SCN_OK);
    ASSERT_EQ(scn_engine_add_parameter_modify_action(engine, "gap", SCN_MODIFY_ADD, 3.0, 4.0,
                                                     SCN_PRIORITY_PARALLEL, 1),
              SCN_OK);

    ASSERT_EQ(scn_engine_init(engine), SCN_OK);
    for (int i = 0; i < 5; ++i) {
        ASSERT_EQ(scn_engine_step(engine, 1.0), SCN_OK);
    }
    const char* value = nullptr;
    ASSERT_EQ(scn_engine_get_variable(engine, "counter", &value), SCN_OK);
    EXPECT_STREQ(value, "25");

    // The deprecated parameter actions warn through the diagnostic stream.
    size_t count = 0;
    ASSERT_EQ(scn_engine_diagnostic_count(engine, &count), SCN_OK);
    bool deprecated = false;
    for (size_t i = 0; i < count; ++i) {
        scn_diagnostic diagnostic{};
        ASSERT_EQ(scn_engine_diagnostic_at(engine, i, &diagnostic), SCN_OK);
        if (diagnostic.code == SCN_ERROR_DEPRECATED_FEATURE) {
            deprecated = true;
        }
    }
    EXPECT_TRUE(deprecated);
    scn_engine_destroy(engine);
}

TEST(CApiTest, EntityLifecycleThroughTheAbi) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego", SCN_CONTROL_ENGINE), SCN_OK);
    ASSERT_EQ(scn_engine_add_delete_entity_action(engine, "ego", 1.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_OK);
    ASSERT_EQ(scn_engine_add_add_entity_action(engine, "ego", 5.0, 6.0, 0.0, 2.0,
                                               SCN_PRIORITY_PARALLEL, 1),
              SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);

    int active = -1;
    ASSERT_EQ(scn_engine_entity_active(engine, "ego", &active), SCN_OK);
    EXPECT_EQ(active, 1);
    EXPECT_EQ(scn_engine_entity_active(engine, "ghost", &active), SCN_ERROR_UNKNOWN_ENTITY);

    ASSERT_EQ(scn_engine_step(engine, 1.0), SCN_OK);
    ASSERT_EQ(scn_engine_entity_active(engine, "ego", &active), SCN_OK);
    EXPECT_EQ(active, 0);
    scn_entity_state state{};
    EXPECT_EQ(scn_engine_get_state(engine, "ego", &state), SCN_ERROR_UNKNOWN_ENTITY);

    ASSERT_EQ(scn_engine_step(engine, 1.0), SCN_OK);
    ASSERT_EQ(scn_engine_entity_active(engine, "ego", &active), SCN_OK);
    EXPECT_EQ(active, 1);
    ASSERT_EQ(scn_engine_get_state(engine, "ego", &state), SCN_OK);
    EXPECT_EQ(state.x, 5.0);
    EXPECT_EQ(state.y, 6.0);
    scn_engine_destroy(engine);
}

TEST(CApiTest, EnvironmentActionThroughTheAbi) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego", SCN_CONTROL_ENGINE), SCN_OK);

    // Zero-initialized: every has_* flag off, so only what is set is merged.
    scn_environment environment{};
    environment.name = "dusk";
    environment.has_time_of_day = 1;
    environment.time_of_day_animation = 1;
    environment.year = 2026;
    environment.month = 7;
    environment.day = 22;
    environment.hour = 18;
    environment.has_weather = 1;
    environment.has_fog = 1;
    environment.fog_visual_range = 250.0;
    environment.has_precipitation = 1;
    environment.precipitation_type = SCN_PRECIPITATION_RAIN;
    environment.precipitation_intensity = 3.0;
    environment.has_road_condition = 1;
    environment.friction_scale_factor = 0.7;
    ASSERT_EQ(
        scn_engine_add_environment_action(engine, &environment, 1.0, SCN_PRIORITY_PARALLEL, 1),
        SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);

    double instant = 0.0;
    EXPECT_EQ(scn_engine_get_date_time(engine, &instant), SCN_ERROR_INVALID_ARGUMENT);
    ASSERT_EQ(scn_engine_step(engine, 1.0), SCN_OK);
    ASSERT_EQ(scn_engine_get_date_time(engine, &instant), SCN_OK);
    // The anchor advances with simulation time (animation is on).
    ASSERT_EQ(scn_engine_step(engine, 2.0), SCN_OK);
    double later = 0.0;
    ASSERT_EQ(scn_engine_get_date_time(engine, &later), SCN_OK);
    EXPECT_DOUBLE_EQ(later - instant, 2.0);
    scn_engine_destroy(engine);
}

TEST(CApiTest, FrozenTimeOfDayHoldsTheAbiClock) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego", SCN_CONTROL_ENGINE), SCN_OK);
    scn_environment environment{};
    environment.has_time_of_day = 1;
    environment.time_of_day_animation = 0; // frozen
    environment.year = 2026;
    environment.month = 7;
    environment.day = 22;
    environment.hour = 9;
    ASSERT_EQ(
        scn_engine_add_environment_action(engine, &environment, 0.0, SCN_PRIORITY_PARALLEL, 1),
        SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);
    double anchor = 0.0;
    ASSERT_EQ(scn_engine_get_date_time(engine, &anchor), SCN_OK);
    for (int i = 0; i < 5; ++i) {
        ASSERT_EQ(scn_engine_step(engine, 1.0), SCN_OK);
        double now = 0.0;
        ASSERT_EQ(scn_engine_get_date_time(engine, &now), SCN_OK);
        EXPECT_EQ(now, anchor);
    }
    scn_engine_destroy(engine);
}

TEST(CApiTest, TrafficSignalsThroughTheAbi) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego", SCN_CONTROL_ENGINE), SCN_OK);

    const scn_traffic_signal_state stop_states[] = {{"s1", "red"}};
    const scn_traffic_signal_state go_states[] = {{"s1", "green"}};
    const scn_signal_phase phases[] = {{"stop", 10.0, stop_states, 1}, {"go", 10.0, go_states, 1}};
    ASSERT_EQ(
        scn_engine_declare_traffic_signal_controller(engine, "group1", -1.0, nullptr, phases, 2),
        SCN_OK);
    ASSERT_EQ(scn_engine_add_traffic_signal_controller_action(engine, "group1", "go", 3.0,
                                                              SCN_PRIORITY_PARALLEL, 1),
              SCN_OK);
    ASSERT_EQ(scn_engine_add_traffic_signal_state_action(engine, "s1", "red;green", 6.0,
                                                         SCN_PRIORITY_PARALLEL, 1),
              SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);

    const char* state = nullptr;
    ASSERT_EQ(scn_engine_traffic_signal_state(engine, "s1", &state), SCN_OK);
    EXPECT_STREQ(state, "red");
    const char* phase = nullptr;
    ASSERT_EQ(scn_engine_traffic_signal_controller_phase(engine, "group1", &phase), SCN_OK);
    EXPECT_STREQ(phase, "stop");
    EXPECT_EQ(scn_engine_traffic_signal_state(engine, "unwritten", &state), SCN_ERROR_UNKNOWN_NAME);
    EXPECT_EQ(scn_engine_traffic_signal_controller_phase(engine, "nope", &phase),
              SCN_ERROR_UNKNOWN_NAME);

    for (int i = 0; i < 6; ++i) { // t = 3, the controller action fires
        ASSERT_EQ(scn_engine_step(engine, 0.5), SCN_OK);
    }
    ASSERT_EQ(scn_engine_traffic_signal_controller_phase(engine, "group1", &phase), SCN_OK);
    EXPECT_STREQ(phase, "go");

    for (int i = 0; i < 6; ++i) { // t = 6, the forced state lands
        ASSERT_EQ(scn_engine_step(engine, 0.5), SCN_OK);
    }
    ASSERT_EQ(scn_engine_traffic_signal_state(engine, "s1", &state), SCN_OK);
    EXPECT_STREQ(state, "red;green");
    scn_engine_destroy(engine);
}

TEST(CApiTest, TrafficSignalControllerActionUnknownPhaseFailsInit) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego", SCN_CONTROL_ENGINE), SCN_OK);
    const scn_signal_phase phases[] = {{"stop", 10.0, nullptr, 0}};
    ASSERT_EQ(
        scn_engine_declare_traffic_signal_controller(engine, "group1", -1.0, nullptr, phases, 1),
        SCN_OK);
    ASSERT_EQ(scn_engine_add_traffic_signal_controller_action(engine, "group1", "go", 1.0,
                                                              SCN_PRIORITY_PARALLEL, 1),
              SCN_OK);
    EXPECT_EQ(scn_engine_init(engine), SCN_ERROR_SEMANTIC);
    scn_engine_destroy(engine);
}

TEST(CApiTest, CustomCommandActionIsAcceptedWithoutAGateway) {
    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(scn_engine_add_entity(engine, "ego", "ego", SCN_CONTROL_ENGINE), SCN_OK);
    ASSERT_EQ(scn_engine_add_custom_command_action(engine, "script", "run.py", 1.0,
                                                   SCN_PRIORITY_PARALLEL, 1),
              SCN_OK);
    ASSERT_EQ(scn_engine_init(engine), SCN_OK);
    ASSERT_EQ(scn_engine_step(engine, 1.0), SCN_OK);
    // The C surface exposes no gateway, so the action is a documented no-op —
    // and emits no diagnostic.
    size_t count = 0;
    ASSERT_EQ(scn_engine_diagnostic_count(engine, &count), SCN_OK);
    EXPECT_EQ(count, 0U);
    scn_engine_destroy(engine);
}

TEST(CApiTest, NullArgumentsAreRejectedForGlobalActions) {
    const scn_signal_phase phases[] = {{"stop", 10.0, nullptr, 0}};
    scn_environment environment{};

    // A null engine is rejected by every entry point.
    EXPECT_EQ(scn_engine_add_variable_set_action(nullptr, "v", "1", 0.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_variable_modify_action(nullptr, "v", SCN_MODIFY_ADD, 1.0, 0.0,
                                                    SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_parameter_set_action(nullptr, "p", "1", 0.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_parameter_modify_action(nullptr, "p", SCN_MODIFY_ADD, 1.0, 0.0,
                                                     SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_add_entity_action(nullptr, "e", 0.0, 0.0, 0.0, 0.0,
                                               SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_delete_entity_action(nullptr, "e", 0.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(
        scn_engine_add_environment_action(nullptr, &environment, 0.0, SCN_PRIORITY_PARALLEL, 1),
        SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_traffic_signal_state_action(nullptr, "s", "on", 0.0,
                                                         SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_traffic_signal_controller_action(nullptr, "c", "p", 0.0,
                                                              SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(
        scn_engine_add_custom_command_action(nullptr, "t", "c", 0.0, SCN_PRIORITY_PARALLEL, 1),
        SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_declare_traffic_signal_controller(nullptr, "c", -1.0, nullptr, phases, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_traffic_signal_state(nullptr, "s", nullptr), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_traffic_signal_controller_phase(nullptr, "c", nullptr),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_entity_active(nullptr, "e", nullptr), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_get_date_time(nullptr, nullptr), SCN_ERROR_INVALID_ARGUMENT);

    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    // Null required strings and out pointers.
    EXPECT_EQ(
        scn_engine_add_variable_set_action(engine, nullptr, "1", 0.0, SCN_PRIORITY_PARALLEL, 1),
        SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(
        scn_engine_add_variable_set_action(engine, "v", nullptr, 0.0, SCN_PRIORITY_PARALLEL, 1),
        SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_add_entity_action(engine, nullptr, 0.0, 0.0, 0.0, 0.0,
                                               SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_environment_action(engine, nullptr, 0.0, SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_traffic_signal_state_action(engine, "s", nullptr, 0.0,
                                                         SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(
        scn_engine_add_custom_command_action(engine, "t", nullptr, 0.0, SCN_PRIORITY_PARALLEL, 1),
        SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(
        scn_engine_declare_traffic_signal_controller(engine, nullptr, -1.0, nullptr, phases, 1),
        SCN_ERROR_INVALID_ARGUMENT);
    // A non-zero phase count with a null array.
    EXPECT_EQ(scn_engine_declare_traffic_signal_controller(engine, "c", -1.0, nullptr, nullptr, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_traffic_signal_state(engine, "s", nullptr), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_entity_active(engine, "e", nullptr), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_get_date_time(engine, nullptr), SCN_ERROR_INVALID_ARGUMENT);
    // Out-of-range enums.
    EXPECT_EQ(scn_engine_add_variable_modify_action(engine, "v",
                                                    static_cast<scn_modify_operator>(9), 1.0, 0.0,
                                                    SCN_PRIORITY_PARALLEL, 1),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_variable_set_action(engine, "v", "1", 0.0,
                                                 static_cast<scn_event_priority>(9), 1),
              SCN_ERROR_INVALID_ARGUMENT);
    // A negative maximumExecutionCount.
    EXPECT_EQ(scn_engine_add_variable_set_action(engine, "v", "1", 0.0, SCN_PRIORITY_PARALLEL, -1),
              SCN_ERROR_INVALID_ARGUMENT);
    scn_engine_destroy(engine);
}
