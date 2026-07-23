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

/* How a relative speed target combines with the reference entity's speed
 * (§SpeedTargetValueType). Values mirror the IR enum. */
typedef enum scn_speed_target_value_type {
    SCN_SPEED_TARGET_DELTA = 0, /* target = reference + value [m/s] */
    SCN_SPEED_TARGET_FACTOR = 1 /* target = reference * value (unitless) */
} scn_speed_target_value_type;

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

/* The referential a distance is measured in (§CoordinateSystem, §6.4). Values
 * mirror the IR enum. The road-based systems need a road network and are not
 * implemented yet: an action using one reports UnsupportedFeature and ends. */
typedef enum scn_coordinate_system {
    SCN_COORDINATE_SYSTEM_ENTITY = 0,
    SCN_COORDINATE_SYSTEM_LANE = 1,
    SCN_COORDINATE_SYSTEM_ROAD = 2,
    SCN_COORDINATE_SYSTEM_TRAJECTORY = 3,
    SCN_COORDINATE_SYSTEM_WORLD = 4
} scn_coordinate_system;

/* Which side of the reference entity a longitudinal distance applies to
 * (§LongitudinalDisplacement). Values mirror the IR enum. */
typedef enum scn_longitudinal_displacement {
    SCN_LONGITUDINAL_DISPLACEMENT_ANY = 0,
    SCN_LONGITUDINAL_DISPLACEMENT_TRAILING = 1, /* the actor stays behind (default) */
    SCN_LONGITUDINAL_DISPLACEMENT_LEADING = 2   /* the actor stays ahead */
} scn_longitudinal_displacement;

/* Strategy for path selection between route waypoints (§RouteStrategy). Values
 * mirror the IR enum. Stored but not interpreted: choosing a path needs a road
 * network, and SCN_ROUTE_STRATEGY_RANDOM never reaches a random generator. */
typedef enum scn_route_strategy {
    SCN_ROUTE_STRATEGY_FASTEST = 0,
    SCN_ROUTE_STRATEGY_LEAST_INTERSECTIONS = 1,
    SCN_ROUTE_STRATEGY_RANDOM = 2,
    SCN_ROUTE_STRATEGY_SHORTEST = 3
} scn_route_strategy;

/* Whether trajectory vertex times are absolute or relative to the action's
 * start (§ReferenceContext). Values mirror the IR enum. */
typedef enum scn_reference_context {
    SCN_REFERENCE_CONTEXT_ABSOLUTE = 0,
    SCN_REFERENCE_CONTEXT_RELATIVE = 1
} scn_reference_context;

/* The operational domains a controller acts on (§ControllerType). Values
 * mirror the IR enum; the default when unspecified is
 * SCN_CONTROLLER_TYPE_MOVEMENT. */
typedef enum scn_controller_type {
    SCN_CONTROLLER_TYPE_LATERAL = 0,
    SCN_CONTROLLER_TYPE_LONGITUDINAL = 1,
    SCN_CONTROLLER_TYPE_LIGHTING = 2,
    SCN_CONTROLLER_TYPE_ANIMATION = 3,
    SCN_CONTROLLER_TYPE_MOVEMENT = 4,
    SCN_CONTROLLER_TYPE_APPEARANCE = 5,
    SCN_CONTROLLER_TYPE_ALL = 6
} scn_controller_type;

/* Limits a distance controller may use (§DynamicConstraints). Every field is
 * optional in the standard, where a missing value means infinite; on the ABI a
 * negative value means "unspecified", the same convention scn_performance uses
 * for its rate limits. Transparent struct; append fields only. */
typedef struct scn_dynamic_constraints {
    double max_acceleration;      /* m/s^2; negative ⇒ unspecified (infinite) */
    double max_acceleration_rate; /* m/s^3; negative ⇒ unspecified (infinite) */
    double max_deceleration;      /* m/s^2; negative ⇒ unspecified (infinite) */
    double max_deceleration_rate; /* m/s^3; negative ⇒ unspecified (infinite) */
    double max_speed;             /* m/s;   negative ⇒ unspecified (infinite) */
} scn_dynamic_constraints;

/* One waypoint of a route (§Waypoint): a world position plus the strategy for
 * reaching it. Transparent struct; append fields only. */
typedef struct scn_waypoint {
    double x; /* m */
    double y; /* m */
    double z; /* m */
    scn_route_strategy strategy;
} scn_waypoint;

/* One vertex of a polyline trajectory (§Vertex). `time` is meaningful only
 * when `has_time` is non-zero. Transparent struct; append fields only. */
typedef struct scn_trajectory_vertex {
    double x;     /* m */
    double y;     /* m */
    double z;     /* m */
    double time;  /* s; read only when has_time != 0 */
    int has_time; /* 0 ⇒ the vertex carries no time */
} scn_trajectory_vertex;

/* Timing adjustment applied to trajectory vertex times (§Timing): the
 * effective time of a vertex is time * scale + offset, read in `domain`.
 * Transparent struct; append fields only. */
typedef struct scn_timing {
    scn_reference_context domain;
    double scale;  /* Range ]0..inf[; 1.0 means no scaling */
    double offset; /* s */
} scn_timing;

/* Detectability of an entity (§VisibilityAction); non-zero means visible.
 * Transparent struct; append fields only. */
typedef struct scn_entity_visibility {
    int graphics; /* visible in the host's image generator(s) */
    int sensors;  /* visible to the host's sensor model(s) */
    int traffic;  /* visible to other traffic participants */
} scn_entity_visibility;

/* Which movement domains the engine currently controls for an entity
 * (§ActivateControllerAction); non-zero means the engine drives that domain.
 * Transparent struct; append fields only. */
typedef struct scn_controller_activation {
    int lateral;
    int longitudinal;
} scn_controller_activation;

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

/* Adds a SpeedAction whose target is relative to `reference_entity_id`
 * (§RelativeTargetSpeed): the target is that entity's speed combined with
 * `value` per `value_type` (delta ⇒ reference + value, factor ⇒ reference *
 * value). With `continuous` zero the target is resolved once at start and
 * reached through `dynamics`; with `continuous` non-zero a controller keeps
 * matching the reference and the action never ends by itself (§7.5.3) — which
 * must not be combined with a time- or distance-dimensioned transition. A NULL
 * `dynamics`, an out-of-range value_type/shape/dimension/following_mode/
 * priority, or a negative maximum_execution_count is rejected with
 * SCN_ERROR_INVALID_ARGUMENT. */
SCN_API scn_status scn_engine_add_relative_speed_action(
    scn_engine* engine, const char* entity_id, const char* reference_entity_id, double value,
    scn_speed_target_value_type value_type, int continuous, const scn_transition_dynamics* dynamics,
    double at_time, scn_event_priority priority, int maximum_execution_count);

/* Adds a TeleportAction (§TeleportAction) that moves `entity_id` to the world
 * position (`x`, `y`, `z`) [m] when triggered at `at_time`. A step
 * (instantaneous) action; Scena resolves the world-frame target only (the other
 * §6.3.8 position variants arrive with p2-s4/p3-s4). An out-of-range priority or
 * a negative maximum_execution_count is rejected with
 * SCN_ERROR_INVALID_ARGUMENT. */
SCN_API scn_status scn_engine_add_teleport_action(scn_engine* engine, const char* entity_id,
                                                  double x, double y, double z, double at_time,
                                                  scn_event_priority priority,
                                                  int maximum_execution_count);

/* Adds a LongitudinalDistanceAction (§LongitudinalDistanceAction): the actor
 * keeps a distance [m] or a headway time gap [s] to `reference_entity_id`.
 * Exactly one of `distance` and `time_gap` must be non-negative; the other must
 * be negative, meaning "not used" (the two attributes are mutually exclusive).
 * `freespace` non-zero measures between the closest bounding-box points, zero
 * between reference points. `continuous` non-zero makes the action never end by
 * itself (§7.5.3). `constraints` may be NULL for unlimited dynamics.
 *
 * A NULL engine or id, both or neither of distance/time_gap, an out-of-range
 * enum, or a negative maximum_execution_count is rejected with
 * SCN_ERROR_INVALID_ARGUMENT. */
SCN_API scn_status scn_engine_add_longitudinal_distance_action(
    scn_engine* engine, const char* entity_id, const char* reference_entity_id, double distance,
    double time_gap, int freespace, int continuous, scn_coordinate_system coordinate_system,
    scn_longitudinal_displacement displacement, const scn_dynamic_constraints* constraints,
    double at_time, scn_event_priority priority, int maximum_execution_count);

/* Adds an AssignRouteAction (§AssignRouteAction) assigning a route of
 * `waypoint_count` waypoints (at least 2) to `entity_id`. `closed` non-zero
 * closes the route (§Route). `name` may be NULL for an unnamed route. The
 * action completes immediately (Annex A Table 10). A NULL engine/id/waypoints,
 * fewer than two waypoints, an out-of-range strategy, or a negative
 * maximum_execution_count is rejected with SCN_ERROR_INVALID_ARGUMENT. */
SCN_API scn_status scn_engine_add_assign_route_action(scn_engine* engine, const char* entity_id,
                                                      const char* name,
                                                      const scn_waypoint* waypoints,
                                                      size_t waypoint_count, int closed,
                                                      double at_time, scn_event_priority priority,
                                                      int maximum_execution_count);

/* Adds an AcquirePositionAction (§AcquirePositionAction): a route from the
 * entity's position at apply time to (`x`, `y`, `z`) is created and the action
 * completes immediately (§7.4.1.4, Table 10). */
SCN_API scn_status scn_engine_add_acquire_position_action(scn_engine* engine, const char* entity_id,
                                                          double x, double y, double z,
                                                          double at_time,
                                                          scn_event_priority priority,
                                                          int maximum_execution_count);

/* Adds a FollowTrajectoryAction (§FollowTrajectoryAction) over a polyline of
 * `vertex_count` vertices (at least 2). `timing` may be NULL for
 * §TimeReference "None" — the vertex times are then ignored and the entity's
 * own longitudinal control sets the pace; with a timing, every vertex must
 * carry a time and the action drives the speed as well.
 * `initial_distance_offset` truncates the trajectory to start at that arc
 * length [m]. Only the Polyline shape is modeled; the clothoid and NURBS
 * shapes arrive with a later phase. A NULL engine/id/vertices, fewer than two
 * vertices, an out-of-range following_mode/domain, or a negative
 * maximum_execution_count is rejected with SCN_ERROR_INVALID_ARGUMENT. */
SCN_API scn_status scn_engine_add_follow_trajectory_action(
    scn_engine* engine, const char* entity_id, const char* name,
    const scn_trajectory_vertex* vertices, size_t vertex_count, int closed,
    scn_following_mode following_mode, const scn_timing* timing, double initial_distance_offset,
    double at_time, scn_event_priority priority, int maximum_execution_count);

/* Adds an AssignControllerAction (§AssignControllerAction) assigning a
 * controller named `name` of `controller_type` to `entity_id`, with
 * `property_count` properties given as parallel key/value arrays (either array
 * may be NULL when the count is zero). The activation flags are tri-state:
 * negative leaves the domain unchanged, zero deactivates it, positive
 * activates it. Activating a domain the controller type does not define is
 * rejected at scn_engine_init, not here. The action completes immediately
 * (Table 10). */
SCN_API scn_status scn_engine_add_assign_controller_action(
    scn_engine* engine, const char* entity_id, const char* name,
    scn_controller_type controller_type, const char* const* property_names,
    const char* const* property_values, size_t property_count, int activate_lateral,
    int activate_longitudinal, double at_time, scn_event_priority priority,
    int maximum_execution_count);

/* Adds an ActivateControllerAction (§ActivateControllerAction) toggling the
 * engine's control of a movement domain. Both flags are tri-state: negative
 * leaves the domain unchanged, zero deactivates it, positive activates it.
 * Deactivating a domain releases the engine's control of it and suppresses
 * actions targeting it until it is activated again. Completes immediately
 * (Table 10). */
SCN_API scn_status scn_engine_add_activate_controller_action(scn_engine* engine,
                                                             const char* entity_id, int lateral,
                                                             int longitudinal, double at_time,
                                                             scn_event_priority priority,
                                                             int maximum_execution_count);

/* Adds a VisibilityAction (§VisibilityAction) setting an entity's
 * detectability; each flag is non-zero for visible. Completes immediately
 * (Table 10). */
SCN_API scn_status scn_engine_add_visibility_action(scn_engine* engine, const char* entity_id,
                                                    int graphics, int sensors, int traffic,
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

/* Runtime entity state installed by the p5-s5 private actions. Unlike the
 * entity metadata queries above, these read the running engine, so they need a
 * successful scn_engine_init; an unknown id returns SCN_ERROR_UNKNOWN_ENTITY
 * and an entity that has nothing to report returns SCN_ERROR_INVALID_ARGUMENT.
 * On any error *out is left untouched. */

/* Current detectability of an entity (§VisibilityAction); every flag is 1
 * until a VisibilityAction changes it. */
SCN_API scn_status scn_engine_entity_visibility(scn_engine* engine, const char* id,
                                                scn_entity_visibility* out);

/* Which movement domains the engine currently controls for an entity
 * (§ActivateControllerAction); both are 1 until a controller action says
 * otherwise. */
SCN_API scn_status scn_engine_entity_controller_activation(scn_engine* engine, const char* id,
                                                           scn_controller_activation* out);

/* The type of the controller assigned to an entity, or
 * SCN_ERROR_INVALID_ARGUMENT when it has none. */
SCN_API scn_status scn_engine_entity_controller_type(scn_engine* engine, const char* id,
                                                     scn_controller_type* out);

/* The name of the controller assigned to an entity, as a borrowed string with
 * the same lifetime as scn_engine_get_variable's. The controller's properties
 * are reachable from C++ and Python. */
SCN_API scn_status scn_engine_entity_controller_name(scn_engine* engine, const char* id,
                                                     const char** out);

/* The assigned route's waypoint count and closed flag (§6.8.2). Either out
 * pointer may be NULL when that half is not wanted; an entity with no route
 * returns SCN_ERROR_INVALID_ARGUMENT. */
SCN_API scn_status scn_engine_entity_route_info(scn_engine* engine, const char* id,
                                                size_t* out_waypoint_count, int* out_closed);

/* One waypoint of the assigned route, by index. An index at or past the
 * waypoint count returns SCN_ERROR_INVALID_ARGUMENT with *out untouched. */
SCN_API scn_status scn_engine_entity_route_waypoint_at(scn_engine* engine, const char* id,
                                                       size_t index, scn_waypoint* out);

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

/* Writes the current simulated instant as seconds since the Unix epoch into
 * *out. Returns SCN_ERROR_INVALID_ARGUMENT with *out untouched when no
 * time-of-day anchor has been set. Frozen at the anchor while a non-animated
 * §TimeOfDay is in force. */
SCN_API scn_status scn_engine_get_date_time(scn_engine* engine, double* out);

/* Global and infrastructure actions (§7.4.2, §7.4.3).
 *
 * Each builder appends one event to the same flat storyboard the private-action
 * builders use, triggered by a SimulationTimeCondition at `at_time`, with the
 * event's priority (§7.3.2) and maximumExecutionCount (§8.3.3.2). A NULL engine
 * or required string, an out-of-range enum, or a negative
 * maximum_execution_count is rejected with SCN_ERROR_INVALID_ARGUMENT and adds
 * nothing. Every action here completes in the evaluation it fires (Annex A
 * Tables 11 and 12). References are validated at scn_engine_init, not here. */

/* The arithmetic a modify action applies (§VariableModifyRule). */
typedef enum scn_modify_operator {
    SCN_MODIFY_ADD = 0,     /* current + value */
    SCN_MODIFY_MULTIPLY = 1 /* current * value */
} scn_modify_operator;

/* Sets a declared variable to `value` (§VariableSetAction, >= 1.2). */
SCN_API scn_status scn_engine_add_variable_set_action(scn_engine* engine, const char* variable_ref,
                                                      const char* value, double at_time,
                                                      scn_event_priority priority,
                                                      int maximum_execution_count);

/* Modifies a declared variable arithmetically (§VariableModifyAction). A
 * non-numeric current value reports rule
 * data_type.variable_modification_or_comparison_possible at runtime and leaves
 * the variable alone; a non-finite `value` is rejected at scn_engine_init. */
SCN_API scn_status scn_engine_add_variable_modify_action(
    scn_engine* engine, const char* variable_ref, scn_modify_operator op, double value,
    double at_time, scn_event_priority priority, int maximum_execution_count);

/* Sets a declared parameter (§ParameterSetAction) — deprecated with 1.2 in
 * favour of the variable actions, and executed against a runtime overlay so a
 * 1.0/1.1 file's ParameterCondition observes it. Each name warns once with
 * SCN_ERROR_DEPRECATED_FEATURE. */
SCN_API scn_status scn_engine_add_parameter_set_action(scn_engine* engine,
                                                       const char* parameter_ref, const char* value,
                                                       double at_time, scn_event_priority priority,
                                                       int maximum_execution_count);

/* Modifies a declared parameter (§ParameterModifyAction) — deprecated with 1.2;
 * same overlay and deprecation warning. */
SCN_API scn_status scn_engine_add_parameter_modify_action(
    scn_engine* engine, const char* parameter_ref, scn_modify_operator op, double value,
    double at_time, scn_event_priority priority, int maximum_execution_count);

/* Adds a declared entity to the running scenario at the world position
 * (`x`, `y`, `z`) [m] (§AddEntityAction). Adding an already active entity has
 * no effect. */
SCN_API scn_status scn_engine_add_add_entity_action(scn_engine* engine, const char* entity_ref,
                                                    double x, double y, double z, double at_time,
                                                    scn_event_priority priority,
                                                    int maximum_execution_count);

/* Removes an entity from the running scenario (§DeleteEntityAction). Deleting
 * an already inactive entity has no effect. */
SCN_API scn_status scn_engine_add_delete_entity_action(scn_engine* engine, const char* entity_ref,
                                                       double at_time, scn_event_priority priority,
                                                       int maximum_execution_count);

/* Types of precipitation (§PrecipitationType). */
typedef enum scn_precipitation_type {
    SCN_PRECIPITATION_DRY = 0,
    SCN_PRECIPITATION_RAIN = 1,
    SCN_PRECIPITATION_SNOW = 2
} scn_precipitation_type;

/* An environment update (§Environment). Every group is optional and carries a
 * `has_*` flag: a zero flag means "this condition doesn't change", the
 * standard's own reading of an absent member, and the corresponding value
 * fields are then ignored. `name` may be NULL or "" to leave the name alone.
 *
 * Transparent struct: the layout is frozen ABI. Append fields only; never
 * reorder or remove. Zero-initialize it (`scn_environment env = {0};`) and set
 * only what the update carries. */
typedef struct scn_environment {
    const char* name; /* NULL or "" leaves the current name */

    /* §TimeOfDay — the one member with runtime meaning: it re-anchors the
     * simulated clock, and `time_of_day_animation` decides whether that clock
     * advances with simulation time or freezes at the anchor. */
    int has_time_of_day;
    int time_of_day_animation;
    int year;
    int month; /* 1..12 */
    int day;   /* 1..days-in-month */
    int hour;
    int minute;
    int second;
    int millisecond;
    int utc_offset_minutes;

    /* §Weather; each member merges individually. */
    int has_weather;
    int has_sun;
    double sun_azimuth;     /* [rad], 0..2*PI */
    double sun_elevation;   /* [rad], -PI..PI */
    double sun_illuminance; /* [lx], 0..inf */
    int has_fog;
    double fog_visual_range; /* [m], 0..inf */
    int has_precipitation;
    scn_precipitation_type precipitation_type;
    double precipitation_intensity; /* [mm/h], 0..inf */
    int has_wind;
    double wind_direction; /* [rad], 0..2*PI */
    double wind_speed;     /* [m/s], 0..inf */
    int has_temperature;
    double temperature; /* [K], 170..340 */
    int has_atmospheric_pressure;
    double atmospheric_pressure; /* [Pa], 80000..120000 */
    int has_fractional_cloud_cover;
    int fractional_cloud_cover_oktas; /* 0..9 (§FractionalCloudCover, 1.2+) */

    /* §RoadCondition. */
    int has_road_condition;
    double friction_scale_factor; /* 0..inf; 1.0 is nominal */
} scn_environment;

/* Merges an environment update into the engine's environment store
 * (§EnvironmentAction). A NULL `environment` is rejected. Out-of-range values
 * are reported at scn_engine_init. Read-back of the merged store is C++ / Python
 * only in this phase; the C surface ships the builders and the small getters
 * below (see the p6-s1 C-ABI expansion). */
SCN_API scn_status scn_engine_add_environment_action(scn_engine* engine,
                                                     const scn_environment* environment,
                                                     double at_time, scn_event_priority priority,
                                                     int maximum_execution_count);

/* Forces a named traffic signal into an observable state
 * (§TrafficSignalStateAction). The forced state stands until the controlling
 * cycle's next phase transition. Both strings are opaque to the engine. */
SCN_API scn_status scn_engine_add_traffic_signal_state_action(scn_engine* engine, const char* name,
                                                              const char* state, double at_time,
                                                              scn_event_priority priority,
                                                              int maximum_execution_count);

/* Restarts a traffic signal controller's cycle at a named phase
 * (§TrafficSignalControllerAction). Both references must exist in the scenario
 * (rule traffic_signal_controller_action_references), checked at
 * scn_engine_init. */
SCN_API scn_status scn_engine_add_traffic_signal_controller_action(
    scn_engine* engine, const char* traffic_signal_controller_ref, const char* phase,
    double at_time, scn_event_priority priority, int maximum_execution_count);

/* Issues a user-defined command to the host (§CustomCommandAction). `type` and
 * `content` are a host-author contract handed over verbatim through the
 * gateway; without a gateway the action is a silent no-op. */
SCN_API scn_status scn_engine_add_custom_command_action(scn_engine* engine, const char* type,
                                                        const char* content, double at_time,
                                                        scn_event_priority priority,
                                                        int maximum_execution_count);

/* One signal's observable state within a phase (§TrafficSignalState).
 * Transparent struct; append fields only. */
typedef struct scn_traffic_signal_state {
    const char* traffic_signal_id; /* road-network signal id */
    const char* state;             /* e.g. "off;off;on"; notation is host-specific */
} scn_traffic_signal_state;

/* One phase of a signal cycle (§Phase). `states` may be NULL when
 * `state_count` is zero. Transparent struct; append fields only. */
typedef struct scn_signal_phase {
    const char* name; /* unique within its controller */
    double duration;  /* [s], 0..inf */
    const scn_traffic_signal_state* states;
    size_t state_count;
} scn_signal_phase;

/* Declares a TrafficSignalController (§6.11.2) on the scenario under
 * construction; takes effect at the next scn_engine_init. `phases` are the
 * ordered cycle and may be NULL when `phase_count` is zero.
 *
 * `reference` may be NULL for an unchained controller; `delay` is negative to
 * mean "unspecified" (the scn_performance rate-limit convention). A delay
 * without a reference, an unresolvable or cyclic reference, a duplicate
 * controller or phase name, and a negative phase duration are all reported at
 * scn_engine_init. A NULL engine or name, or a NULL phase name inside a
 * non-empty phase, is rejected here with SCN_ERROR_INVALID_ARGUMENT. */
SCN_API scn_status scn_engine_declare_traffic_signal_controller(scn_engine* engine,
                                                                const char* name, double delay,
                                                                const char* reference,
                                                                const scn_signal_phase* phases,
                                                                size_t phase_count);

/* Writes the current observable state of a traffic signal into *out as a
 * borrowed string with the same lifetime as scn_engine_get_variable's. A signal
 * nothing has written yet returns SCN_ERROR_UNKNOWN_NAME with *out untouched. */
SCN_API scn_status scn_engine_traffic_signal_state(scn_engine* engine, const char* name,
                                                   const char** out);

/* Writes the name of the phase a traffic signal controller is currently in into
 * *out, borrowed with the same lifetime. A controller that is unknown, has no
 * phases, or has not started yet (a §6.11.3 delay still running) returns
 * SCN_ERROR_UNKNOWN_NAME with *out untouched. */
SCN_API scn_status scn_engine_traffic_signal_controller_phase(scn_engine* engine, const char* name,
                                                              const char** out);

/* Writes 1 into *out when a declared entity is currently in the scenario and 0
 * when a DeleteEntityAction has removed it (§EntityAction). An id the scenario
 * does not declare at all returns SCN_ERROR_UNKNOWN_ENTITY with *out
 * untouched. */
SCN_API scn_status scn_engine_entity_active(scn_engine* engine, const char* id, int* out);

#ifdef __cplusplus
}
#endif

#endif /* SCENA_CAPI_H */
