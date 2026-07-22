/* SPDX-FileCopyrightText: 2026 Robomous */
/* SPDX-License-Identifier: Apache-2.0 */
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

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Mirrors scena::Status. Values are ABI: append new enumerators at the end,
 * never renumber or remove. SCN_ERROR_INTERNAL (6) is the C-ABI-only code for
 * an exception caught at the boundary; it has no scena::Status counterpart. */
typedef enum scn_status {
    SCN_OK = 0,
    SCN_ERROR_ALREADY_INITIALIZED = 1,
    SCN_ERROR_NOT_INITIALIZED = 2,
    SCN_ERROR_UNKNOWN_ENTITY = 3,
    SCN_ERROR_INVALID_CONTROL_MODE = 4,
    SCN_ERROR_INVALID_ARGUMENT = 5,
    SCN_ERROR_INTERNAL = 6,
    SCN_ERROR_PARSE = 7,                /* a frontend could not parse the source */
    SCN_ERROR_VALIDATION = 8,           /* scenario content violates a structural rule */
    SCN_ERROR_SEMANTIC = 9,             /* scenario content references something missing */
    SCN_ERROR_UNSUPPORTED_FEATURE = 10, /* a construct the engine does not implement */
    SCN_ERROR_UNKNOWN_NAME = 11,        /* a host referenced an undeclared named value */
    SCN_ERROR_DEPRECATED_FEATURE = 12   /* content uses a construct the standard deprecated */
} scn_status;

/* Mirrors scena::Severity. */
typedef enum scn_severity {
    SCN_SEVERITY_INFO = 0,
    SCN_SEVERITY_WARNING = 1,
    SCN_SEVERITY_ERROR = 2
} scn_severity;

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

/* Transparent struct: the layout is frozen ABI. Append fields only; never
 * reorder or remove. pitch/roll were appended after speed — a caller compiled
 * against an older header (without them) must be recompiled against this one so
 * it allocates the full struct. */
typedef struct scn_entity_state {
    double x;       /* world position, meters */
    double y;       /* world position, meters */
    double z;       /* world position, meters */
    double heading; /* yaw around +Z, radians; 0 points along +X */
    double speed;   /* longitudinal speed along the heading, m/s */
    double pitch;   /* pitch around the entity's +Y, radians */
    double roll;    /* roll around the entity's +X, radians */
} scn_entity_state;

/* Entity taxonomy (§7.2.2). The category/type enums mirror the C++
 * scena::ir enumerations one-for-one and in the same order; values are ABI,
 * append new enumerators at the end. */

typedef enum scn_object_type {
    SCN_OBJECT_VEHICLE = 0,
    SCN_OBJECT_PEDESTRIAN = 1,
    SCN_OBJECT_MISC = 2
} scn_object_type;

/* Mirrors scena::ir::VehicleCategory (§VehicleCategory). */
typedef enum scn_vehicle_category {
    SCN_VEHICLE_AIRCRAFT = 0,
    SCN_VEHICLE_BICYCLE = 1,
    SCN_VEHICLE_BUS = 2,
    SCN_VEHICLE_CAR = 3,
    SCN_VEHICLE_HEAVY_TRUCK = 4,
    SCN_VEHICLE_LAND_VEHICLE = 5,
    SCN_VEHICLE_MICROMOBILITY_DEVICE = 6,
    SCN_VEHICLE_MOTORBIKE = 7, /* DEPRECATED in 1.4: use SCN_VEHICLE_MOTORCYCLE */
    SCN_VEHICLE_MOTORCYCLE = 8,
    SCN_VEHICLE_OTHER = 9,
    SCN_VEHICLE_SEMITRACTOR = 10,
    SCN_VEHICLE_SEMITRAILER = 11,
    SCN_VEHICLE_STANDUP_SCOOTER = 12,
    SCN_VEHICLE_TRAILER = 13,
    SCN_VEHICLE_TRAIN = 14,
    SCN_VEHICLE_TRAM = 15,
    SCN_VEHICLE_TRUCK = 16, /* DEPRECATED in 1.4: use HEAVY_TRUCK/SEMITRACTOR */
    SCN_VEHICLE_VAN = 17,
    SCN_VEHICLE_WATERCRAFT = 18,
    SCN_VEHICLE_WHEELCHAIR = 19,
    SCN_VEHICLE_WORK_MACHINE = 20
} scn_vehicle_category;

/* Mirrors scena::ir::PedestrianCategory (§PedestrianCategory). */
typedef enum scn_pedestrian_category {
    SCN_PEDESTRIAN_ANIMAL = 0,
    SCN_PEDESTRIAN_PEDESTRIAN = 1,
    SCN_PEDESTRIAN_WHEELCHAIR = 2 /* DEPRECATED in 1.4 */
} scn_pedestrian_category;

/* Mirrors scena::ir::MiscObjectCategory (§MiscObjectCategory). */
typedef enum scn_misc_object_category {
    SCN_MISC_BARRIER = 0,
    SCN_MISC_BUILDING = 1,
    SCN_MISC_CROSSWALK = 2,
    SCN_MISC_GANTRY = 3,
    SCN_MISC_NONE = 4,
    SCN_MISC_OBSTACLE = 5,
    SCN_MISC_PARKING_SPACE = 6,
    SCN_MISC_PATCH = 7,
    SCN_MISC_POLE = 8,
    SCN_MISC_RAILING = 9,
    SCN_MISC_ROAD_MARK = 10,
    SCN_MISC_SOUND_BARRIER = 11,
    SCN_MISC_STREET_LAMP = 12,
    SCN_MISC_TRAFFIC_ISLAND = 13,
    SCN_MISC_TREE = 14,
    SCN_MISC_VEGETATION = 15,
    SCN_MISC_WIND = 16 /* DEPRECATED in 1.4 */
} scn_misc_object_category;

/* Mirrors scena::ir::Role (§Role); the default when unspecified is SCN_ROLE_NONE. */
typedef enum scn_role {
    SCN_ROLE_NONE = 0,
    SCN_ROLE_AGRICULTURE = 1,
    SCN_ROLE_AMBULANCE = 2,
    SCN_ROLE_CIVIL = 3,
    SCN_ROLE_CONSTRUCTION = 4,
    SCN_ROLE_DANGEROUS_GOODS_TRANSPORT = 5,
    SCN_ROLE_FIRE = 6, /* DEPRECATED in 1.4: use SCN_ROLE_FIRE_BRIGADE */
    SCN_ROLE_FIRE_BRIGADE = 7,
    SCN_ROLE_FREIGHT_TRANSPORT = 8,
    SCN_ROLE_GARBAGE_COLLECTION = 9,
    SCN_ROLE_MILITARY = 10,
    SCN_ROLE_OTHER = 11,
    SCN_ROLE_POLICE = 12,
    SCN_ROLE_PUBLIC_TRANSPORT = 13,
    SCN_ROLE_ROAD_ASSISTANCE = 14, /* DEPRECATED in 1.4: use ROADSIDE_ASSISTANCE */
    SCN_ROLE_ROADSIDE_ASSISTANCE = 15,
    SCN_ROLE_SPECIAL_TRANSPORT = 16,
    SCN_ROLE_TRAFFIC_CONTROL = 17
} scn_role;

/* A three-dimensional bounding box (§BoundingBox): the geometric center in the
 * entity body frame plus the length/width/height dimensions. Transparent
 * struct; append fields only. */
typedef struct scn_bounding_box {
    double center_x; /* m, entity frame, +x forward */
    double center_y; /* m, entity frame, +y left */
    double center_z; /* m, entity frame, +z up */
    double length;   /* m, extent along the body x axis */
    double width;    /* m, extent along the body y axis */
    double height;   /* m, extent along the body z axis */
} scn_bounding_box;

/* Vehicle performance limits (§Performance). The rate limits are optional in
 * the standard (absent ⇒ infinite): pass a negative value to mean "unspecified"
 * on a builder, and read back a negative value when the limit is absent.
 * Transparent struct; append fields only. */
typedef struct scn_performance {
    double max_speed;             /* m/s, Range [0..inf[ */
    double max_acceleration;      /* m/s^2, Range [0..inf[ */
    double max_deceleration;      /* m/s^2, Range [0..inf[ */
    double max_acceleration_rate; /* m/s^3; negative ⇒ unspecified (infinite) */
    double max_deceleration_rate; /* m/s^3; negative ⇒ unspecified (infinite) */
} scn_performance;

/* Shape of a speed transition (§DynamicsShape). Values mirror the IR enum. */
typedef enum scn_dynamics_shape {
    SCN_DYNAMICS_SHAPE_LINEAR = 0,
    SCN_DYNAMICS_SHAPE_CUBIC = 1,
    SCN_DYNAMICS_SHAPE_SINUSOIDAL = 2,
    SCN_DYNAMICS_SHAPE_STEP = 3 /* instantaneous; value must be 0 */
} scn_dynamics_shape;

/* How the target is acquired (§DynamicsDimension). Values mirror the IR enum. */
typedef enum scn_dynamics_dimension {
    SCN_DYNAMICS_DIMENSION_TIME = 0,     /* value is a duration [s] */
    SCN_DYNAMICS_DIMENSION_DISTANCE = 1, /* value is a distance [m] */
    SCN_DYNAMICS_DIMENSION_RATE = 2      /* value is a rate [delta/s] */
} scn_dynamics_dimension;

/* Shape-following behavior (§FollowingMode). Values mirror the IR enum. Scena
 * models position; follow is accepted but treated as position (ADR-0011). */
typedef enum scn_following_mode {
    SCN_FOLLOWING_MODE_POSITION = 0,
    SCN_FOLLOWING_MODE_FOLLOW = 1
} scn_following_mode;

/* Dynamics of a speed transition (§TransitionDynamics). Transparent struct;
 * append fields only. */
typedef struct scn_transition_dynamics {
    scn_dynamics_shape shape;
    scn_dynamics_dimension dimension;
    double value; /* s | m | delta/s, Range [0..inf[ */
    scn_following_mode following_mode;
} scn_transition_dynamics;

/* One speed target of a SpeedProfileAction (§SpeedProfileEntry). Transparent
 * struct; append fields only. */
typedef struct scn_speed_profile_entry {
    double speed; /* m/s */
    double time;  /* s; negative ⇒ unspecified (reach as fast as performance allows) */
} scn_speed_profile_entry;

/* One structured diagnostic read back from the engine.
 *
 * The string members are borrowed from the engine and are never NULL — an
 * absent field is the empty string "". They stay valid until the next
 * scn_engine_init, scn_engine_step, scn_engine_clear_diagnostics, or
 * scn_engine_destroy on the same engine (the only calls that mutate the
 * backing store); copy them out before calling any of those.
 *
 * Transparent struct: the layout is frozen ABI. Append fields only; never
 * reorder or remove. */
typedef struct scn_diagnostic {
    scn_severity severity;
    scn_status code;     /* machine-readable category (mirrors the diagnostic's code) */
    const char* message; /* human-readable description */
    const char* path;    /* element path; "" addresses the whole scenario */
    const char* file;    /* source path; "" when unknown */
    int line;            /* 1-based line; 0 when unknown */
    int column;          /* 1-based column; 0 when unknown */
    const char* rule_id; /* ASAM checker rule UID; "" when the standard names none */
} scn_diagnostic;

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

/* Adds a classified entity — a ScenarioObject whose EntityObject is a Vehicle,
 * Pedestrian, or MiscObject (§7.2.2). `bounding_box` is required (non-NULL);
 * `performance` is required for a vehicle. A NULL required pointer or a
 * category outside its enumeration returns SCN_ERROR_INVALID_ARGUMENT and adds
 * nothing. Role defaults to None and mass is left unspecified; the richer
 * fields (role, mass, axles, properties) are reachable from C++ / Python. */
SCN_API scn_status scn_engine_add_vehicle(scn_engine* engine, const char* id, const char* name,
                                          scn_control_mode control_mode,
                                          scn_vehicle_category category,
                                          const scn_bounding_box* bounding_box,
                                          const scn_performance* performance);
SCN_API scn_status scn_engine_add_pedestrian(scn_engine* engine, const char* id, const char* name,
                                             scn_control_mode control_mode,
                                             scn_pedestrian_category category,
                                             const scn_bounding_box* bounding_box);
SCN_API scn_status scn_engine_add_misc_object(scn_engine* engine, const char* id, const char* name,
                                              scn_control_mode control_mode,
                                              scn_misc_object_category category,
                                              const scn_bounding_box* bounding_box);

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

/* As scn_engine_add_speed_action_ex, with transition dynamics governing how the
 * target speed is reached (§SpeedAction). A NULL `dynamics`, an out-of-range
 * shape/dimension/following_mode, or a negative maximum_execution_count is
 * rejected with SCN_ERROR_INVALID_ARGUMENT. */
SCN_API scn_status scn_engine_add_speed_action_dyn(scn_engine* engine, const char* entity_id,
                                                   double target_speed,
                                                   const scn_transition_dynamics* dynamics,
                                                   double at_time, scn_event_priority priority,
                                                   int maximum_execution_count);

/* Adds a SpeedProfileAction (§SpeedProfileAction) on `entity_id` triggered at
 * `at_time`. `entries` points to `entry_count` speed targets (>= 1); an entry
 * whose time is negative is reached as fast as the Performance envelope allows.
 * A NULL `entries`, a zero `entry_count`, an out-of-range following_mode, or a
 * negative maximum_execution_count is rejected with SCN_ERROR_INVALID_ARGUMENT. */
SCN_API scn_status scn_engine_add_speed_profile_action(scn_engine* engine, const char* entity_id,
                                                       const scn_speed_profile_entry* entries,
                                                       size_t entry_count,
                                                       scn_following_mode following_mode,
                                                       double at_time, scn_event_priority priority,
                                                       int maximum_execution_count);

/* Lifecycle. */
SCN_API scn_status scn_engine_init(scn_engine* engine);
SCN_API scn_status scn_engine_step(scn_engine* engine, double dt);
SCN_API scn_status scn_engine_close(scn_engine* engine);

/* Diagnostics: structured findings from validation (at init) and the runtime
 * (at step). Read by count then by index. */

/* Writes the number of diagnostics into *out_count. */
SCN_API scn_status scn_engine_diagnostic_count(scn_engine* engine, size_t* out_count);

/* Writes the diagnostic at `index` into *out. An index >= the count returns
 * SCN_ERROR_INVALID_ARGUMENT with *out left untouched. The borrowed strings in
 * *out follow the lifetime documented on scn_diagnostic. */
SCN_API scn_status scn_engine_diagnostic_at(scn_engine* engine, size_t index, scn_diagnostic* out);

/* Drops every collected diagnostic. */
SCN_API scn_status scn_engine_clear_diagnostics(scn_engine* engine);

/* Entity state exchange. */
SCN_API scn_status scn_engine_get_state(scn_engine* engine, const char* entity_id,
                                        scn_entity_state* out);
SCN_API scn_status scn_engine_report_state(scn_engine* engine, const char* entity_id,
                                           const scn_entity_state* state);

/* Entity metadata queries (§7.2.2). Each reads the authored scenario, so it is
 * valid before and after init. An unknown id returns SCN_ERROR_UNKNOWN_ENTITY;
 * a known entity that lacks the requested metadata (no classified object, or —
 * for performance — a non-vehicle) returns SCN_ERROR_INVALID_ARGUMENT. On any
 * error *out is left untouched. */
SCN_API scn_status scn_engine_entity_object_type(scn_engine* engine, const char* id,
                                                 scn_object_type* out);
SCN_API scn_status scn_engine_entity_bounding_box(scn_engine* engine, const char* id,
                                                  scn_bounding_box* out);
SCN_API scn_status scn_engine_entity_performance(scn_engine* engine, const char* id,
                                                 scn_performance* out);

/* Named values (by-value conditions).
 *
 * Deliberately no C builder for conditions themselves: the route from C to a
 * scenario's storyboard is the XML frontend (P4), as with the trigger model.
 * These functions cover only the host-value interface those conditions read. */

/* Declares a global parameter (§9.1) on the scenario under construction. Takes
 * effect at the next scn_engine_init; parameters are immutable at runtime. */
SCN_API scn_status scn_engine_set_parameter(scn_engine* engine, const char* name,
                                            const char* value);

/* Declares a global variable (§6.12) with its initialization value on the
 * scenario under construction. Takes effect at the next scn_engine_init, which
 * seeds it into the runtime store. */
SCN_API scn_status scn_engine_declare_variable(scn_engine* engine, const char* name,
                                               const char* value);

/* Sets a declared variable's current value at runtime. Requires init; a name
 * with no declaration returns SCN_ERROR_UNKNOWN_NAME and changes nothing. */
SCN_API scn_status scn_engine_set_variable(scn_engine* engine, const char* name, const char* value);

/* Writes the current value of a variable into *out as a borrowed string. An
 * undeclared name (or before init) returns SCN_ERROR_UNKNOWN_NAME with *out
 * left untouched. The borrowed string is owned by the engine and stays valid
 * until the next scn_engine_get_variable / scn_engine_get_user_defined_value
 * on this engine, or any mutating call (step/init/close/destroy); copy it out
 * before then. */
SCN_API scn_status scn_engine_get_variable(scn_engine* engine, const char* name, const char** out);

/* Creates or updates an external user-defined value. Any name is accepted;
 * values may be staged before init and persist across init until close. */
SCN_API scn_status scn_engine_set_user_defined_value(scn_engine* engine, const char* name,
                                                     const char* value);

/* Writes the current value of a user-defined value into *out as a borrowed
 * string (same lifetime as scn_engine_get_variable). An unset name returns
 * SCN_ERROR_UNKNOWN_NAME with *out left untouched. */
SCN_API scn_status scn_engine_get_user_defined_value(scn_engine* engine, const char* name,
                                                     const char** out);

/* Anchors the simulated time of day (TimeOfDayCondition): the given date-time
 * holds at the current simulation instant and advances with simulation time.
 * The fields are ISO-8601 components (year, 1-based month/day, hour, minute,
 * second, millisecond) plus the RFC-822 zone as minutes east of UTC. An
 * out-of-range date-time returns SCN_ERROR_INVALID_ARGUMENT. */
SCN_API scn_status scn_engine_set_date_time(scn_engine* engine, int year, int month, int day,
                                            int hour, int minute, int second, int millisecond,
                                            int utc_offset_minutes);

#ifdef __cplusplus
}
#endif

#endif /* SCENA_CAPI_H */
