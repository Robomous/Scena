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

    scn_engine* engine = scn_engine_create();
    ASSERT_NE(engine, nullptr);
    EXPECT_EQ(scn_engine_add_entity(engine, nullptr, "ego", SCN_CONTROL_ENGINE),
              SCN_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(scn_engine_get_state(engine, "ego", nullptr), SCN_ERROR_INVALID_ARGUMENT);
    scn_engine_destroy(engine);

    scn_engine_destroy(nullptr); // must be a safe no-op
}
