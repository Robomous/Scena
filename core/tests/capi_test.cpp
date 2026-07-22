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
    EXPECT_EQ(scn_engine_add_vehicle(nullptr, "v", "v", SCN_CONTROL_HOST, SCN_VEHICLE_CAR, &box,
                                     &perf),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_vehicle(engine, "v", "v", SCN_CONTROL_HOST, SCN_VEHICLE_CAR, nullptr,
                                     &perf),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_vehicle(engine, "v", "v", SCN_CONTROL_HOST, SCN_VEHICLE_CAR, &box,
                                     nullptr),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_pedestrian(engine, "p", "p", SCN_CONTROL_HOST,
                                        SCN_PEDESTRIAN_PEDESTRIAN, nullptr),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_add_misc_object(engine, "m", "m", SCN_CONTROL_HOST, SCN_MISC_POLE, nullptr),
              SCN_ERROR_INVALID_ARGUMENT);
    scn_object_type type{};
    scn_bounding_box out_box{};
    scn_performance out_perf{};
    EXPECT_EQ(scn_engine_entity_object_type(engine, nullptr, &type), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_entity_object_type(engine, "v", nullptr), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_entity_bounding_box(engine, "v", nullptr), SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_entity_performance(nullptr, "v", &out_perf), SCN_ERROR_INVALID_ARGUMENT);
    (void)out_box;

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
