/* SPDX-License-Identifier: MIT */
#ifndef SCENA_CAPI_H
#define SCENA_CAPI_H

/*
 * Stable C ABI over the Scena engine.
 *
 * Skeleton in this phase: enough surface to create an engine, describe a
 * minimal scenario, step it, and query entity states — proving the ABI shape
 * end to end. No exceptions cross this boundary; every fallible call returns
 * an scn_status.
 */

#if defined(_WIN32)
#if defined(SCN_CAPI_EXPORTS)
#define SCN_API __declspec(dllexport)
#else
#define SCN_API __declspec(dllimport)
#endif
#else
#define SCN_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum scn_status {
    SCN_OK = 0,
    SCN_ERROR_ALREADY_INITIALIZED = 1,
    SCN_ERROR_NOT_INITIALIZED = 2,
    SCN_ERROR_UNKNOWN_ENTITY = 3,
    SCN_ERROR_INVALID_CONTROL_MODE = 4,
    SCN_ERROR_INVALID_ARGUMENT = 5,
    SCN_ERROR_INTERNAL = 6
} scn_status;

typedef enum scn_control_mode {
    SCN_CONTROL_ENGINE = 0, /* the engine integrates the entity's motion */
    SCN_CONTROL_HOST = 1    /* the host simulator reports the entity's state */
} scn_control_mode;

/* Event priority per ASAM OpenSCENARIO XML 1.3 §7.3.2 / §8.4.2.2, resolved
 * in the scope of the enclosing Maneuver. The deprecated pre-1.3 spelling
 * "overwrite" is a lexical synonym of override and has no separate value. */
typedef enum scn_event_priority {
    SCN_PRIORITY_OVERRIDE = 0, /* stops the running events of the Maneuver */
    SCN_PRIORITY_PARALLEL = 1, /* starts regardless of the other events */
    SCN_PRIORITY_SKIP = 2      /* does not start while another event runs */
} scn_event_priority;

typedef struct scn_entity_state {
    double x;       /* world position, meters */
    double y;       /* world position, meters */
    double z;       /* world position, meters */
    double heading; /* yaw around +Z, radians; 0 points along +X */
    double speed;   /* longitudinal speed along the heading, m/s */
} scn_entity_state;

/* Opaque engine handle. */
typedef struct scn_engine scn_engine;

/* Library version as "major.minor.patch". The string is owned by the library
 * and valid for the lifetime of the process. */
SCN_API const char* scn_version(void);

/* Creates an engine with an empty scenario. Returns NULL on allocation
 * failure. Destroy with scn_engine_destroy. */
SCN_API scn_engine* scn_engine_create(void);

/* Destroys an engine. NULL is a no-op. */
SCN_API void scn_engine_destroy(scn_engine* engine);

/* Scenario building (before scn_engine_init). */
SCN_API scn_status scn_engine_add_entity(scn_engine* engine, const char* id, const char* name,
                                         scn_control_mode control_mode);

/* Adds a storyboard entry: a SpeedAction on `entity_id` triggered by a
 * SimulationTimeCondition at `at_time` seconds. Equivalent to
 * scn_engine_add_speed_action_ex with SCN_PRIORITY_PARALLEL and a single
 * execution. */
SCN_API scn_status scn_engine_add_speed_action(scn_engine* engine, const char* entity_id,
                                               double target_speed, double at_time);

/* As scn_engine_add_speed_action, with the event's priority (§7.3.2) and its
 * maximumExecutionCount (§8.3.3.2) — the number of times the event may
 * execute, counted as its startTransitions plus its skipTransitions. Zero
 * means the event never executes; a negative count is rejected with
 * SCN_ERROR_INVALID_ARGUMENT, as is a priority outside the enumeration.
 *
 * Every event added through this surface lands in the same Maneuver, which
 * is the scope priority is resolved over, so events added by consecutive
 * calls do interact. */
SCN_API scn_status scn_engine_add_speed_action_ex(scn_engine* engine, const char* entity_id,
                                                  double target_speed, double at_time,
                                                  scn_event_priority priority,
                                                  int maximum_execution_count);

/* Lifecycle. */
SCN_API scn_status scn_engine_init(scn_engine* engine);
SCN_API scn_status scn_engine_step(scn_engine* engine, double dt);
SCN_API scn_status scn_engine_close(scn_engine* engine);

/* Entity state exchange. */
SCN_API scn_status scn_engine_get_state(scn_engine* engine, const char* entity_id,
                                        scn_entity_state* out);
SCN_API scn_status scn_engine_report_state(scn_engine* engine, const char* entity_id,
                                           const scn_entity_state* state);

#ifdef __cplusplus
}
#endif

#endif /* SCENA_CAPI_H */
