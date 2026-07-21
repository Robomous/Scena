/* SPDX-License-Identifier: MIT */
#ifndef KINEMA_CAPI_H
#define KINEMA_CAPI_H

/*
 * Stable C ABI over the Kinema engine.
 *
 * Skeleton in this phase: enough surface to create an engine, describe a
 * minimal scenario, step it, and query entity states — proving the ABI shape
 * end to end. No exceptions cross this boundary; every fallible call returns
 * an knm_status.
 */

#if defined(_WIN32)
#if defined(KNM_CAPI_EXPORTS)
#define KNM_API __declspec(dllexport)
#else
#define KNM_API __declspec(dllimport)
#endif
#else
#define KNM_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum knm_status {
    KNM_OK = 0,
    KNM_ERROR_ALREADY_INITIALIZED = 1,
    KNM_ERROR_NOT_INITIALIZED = 2,
    KNM_ERROR_UNKNOWN_ENTITY = 3,
    KNM_ERROR_INVALID_CONTROL_MODE = 4,
    KNM_ERROR_INVALID_ARGUMENT = 5,
    KNM_ERROR_INTERNAL = 6
} knm_status;

typedef enum knm_control_mode {
    KNM_CONTROL_ENGINE = 0, /* the engine integrates the entity's motion */
    KNM_CONTROL_HOST = 1    /* the host simulator reports the entity's state */
} knm_control_mode;

typedef struct knm_entity_state {
    double x;       /* world position, meters */
    double y;       /* world position, meters */
    double z;       /* world position, meters */
    double heading; /* yaw around +Z, radians; 0 points along +X */
    double speed;   /* longitudinal speed along the heading, m/s */
} knm_entity_state;

/* Opaque engine handle. */
typedef struct knm_engine knm_engine;

/* Library version as "major.minor.patch". The string is owned by the library
 * and valid for the lifetime of the process. */
KNM_API const char* knm_version(void);

/* Creates an engine with an empty scenario. Returns NULL on allocation
 * failure. Destroy with knm_engine_destroy. */
KNM_API knm_engine* knm_engine_create(void);

/* Destroys an engine. NULL is a no-op. */
KNM_API void knm_engine_destroy(knm_engine* engine);

/* Scenario building (before knm_engine_init). */
KNM_API knm_status knm_engine_add_entity(knm_engine* engine, const char* id, const char* name,
                                         knm_control_mode control_mode);

/* Adds a storyboard entry: a SpeedAction on `entity_id` triggered by a
 * SimulationTimeCondition at `at_time` seconds. */
KNM_API knm_status knm_engine_add_speed_action(knm_engine* engine, const char* entity_id,
                                               double target_speed, double at_time);

/* Lifecycle. */
KNM_API knm_status knm_engine_init(knm_engine* engine);
KNM_API knm_status knm_engine_step(knm_engine* engine, double dt);
KNM_API knm_status knm_engine_close(knm_engine* engine);

/* Entity state exchange. */
KNM_API knm_status knm_engine_get_state(knm_engine* engine, const char* entity_id,
                                        knm_entity_state* out);
KNM_API knm_status knm_engine_report_state(knm_engine* engine, const char* entity_id,
                                           const knm_entity_state* state);

#ifdef __cplusplus
}
#endif

#endif /* KINEMA_CAPI_H */
