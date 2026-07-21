// SPDX-License-Identifier: MIT
#include "kinema/capi.h"

#include <gtest/gtest.h>

TEST(CApiTest, VersionString) {
    const char* version = knm_version();
    ASSERT_NE(version, nullptr);
    EXPECT_STREQ(version, "0.1.0");
}

TEST(CApiTest, FullLifecycle) {
    knm_engine* engine = knm_engine_create();
    ASSERT_NE(engine, nullptr);

    ASSERT_EQ(knm_engine_add_entity(engine, "ego", "ego vehicle", KNM_CONTROL_ENGINE), KNM_OK);
    ASSERT_EQ(knm_engine_add_entity(engine, "npc", "host vehicle", KNM_CONTROL_HOST), KNM_OK);
    ASSERT_EQ(knm_engine_add_speed_action(engine, "ego", 10.0, 1.0), KNM_OK);

    ASSERT_EQ(knm_engine_init(engine), KNM_OK);

    // 2 simulated seconds at 100 Hz; the speed action triggers at t = 1.0 s.
    for (int i = 0; i < 200; ++i) {
        ASSERT_EQ(knm_engine_step(engine, 0.01), KNM_OK);
    }

    knm_entity_state ego_state{};
    ASSERT_EQ(knm_engine_get_state(engine, "ego", &ego_state), KNM_OK);
    EXPECT_EQ(ego_state.speed, 10.0);
    EXPECT_GT(ego_state.x, 0.0);

    const knm_entity_state reported{4.0, 5.0, 0.0, 1.5, 3.0};
    ASSERT_EQ(knm_engine_report_state(engine, "npc", &reported), KNM_OK);
    knm_entity_state npc_state{};
    ASSERT_EQ(knm_engine_get_state(engine, "npc", &npc_state), KNM_OK);
    EXPECT_EQ(npc_state.x, reported.x);
    EXPECT_EQ(npc_state.heading, reported.heading);
    EXPECT_EQ(npc_state.speed, reported.speed);

    ASSERT_EQ(knm_engine_close(engine), KNM_OK);
    knm_engine_destroy(engine);
}

TEST(CApiTest, ErrorPaths) {
    knm_engine* engine = knm_engine_create();
    ASSERT_NE(engine, nullptr);

    EXPECT_EQ(knm_engine_step(engine, 0.01), KNM_ERROR_NOT_INITIALIZED);
    EXPECT_EQ(knm_engine_close(engine), KNM_ERROR_NOT_INITIALIZED);

    knm_entity_state state{};
    EXPECT_EQ(knm_engine_get_state(engine, "ego", &state), KNM_ERROR_NOT_INITIALIZED);

    ASSERT_EQ(knm_engine_add_entity(engine, "ego", "ego vehicle", KNM_CONTROL_ENGINE), KNM_OK);
    ASSERT_EQ(knm_engine_init(engine), KNM_OK);
    EXPECT_EQ(knm_engine_init(engine), KNM_ERROR_ALREADY_INITIALIZED);

    EXPECT_EQ(knm_engine_get_state(engine, "missing", &state), KNM_ERROR_UNKNOWN_ENTITY);
    EXPECT_EQ(knm_engine_report_state(engine, "ego", &state), KNM_ERROR_INVALID_CONTROL_MODE);
    EXPECT_EQ(knm_engine_step(engine, -1.0), KNM_ERROR_INVALID_ARGUMENT);

    knm_engine_destroy(engine);
}

TEST(CApiTest, NullArgumentsAreRejected) {
    EXPECT_EQ(knm_engine_step(nullptr, 0.01), KNM_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(knm_engine_init(nullptr), KNM_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(knm_engine_close(nullptr), KNM_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(knm_engine_get_state(nullptr, "ego", nullptr), KNM_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(knm_engine_add_entity(nullptr, "ego", "ego", KNM_CONTROL_ENGINE),
              KNM_ERROR_INVALID_ARGUMENT);

    knm_engine* engine = knm_engine_create();
    ASSERT_NE(engine, nullptr);
    EXPECT_EQ(knm_engine_add_entity(engine, nullptr, "ego", KNM_CONTROL_ENGINE),
              KNM_ERROR_INVALID_ARGUMENT);
    EXPECT_EQ(knm_engine_get_state(engine, "ego", nullptr), KNM_ERROR_INVALID_ARGUMENT);
    knm_engine_destroy(engine);

    knm_engine_destroy(nullptr); // must be a safe no-op
}
