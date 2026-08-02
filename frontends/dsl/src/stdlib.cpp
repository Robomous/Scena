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

#include "scena/dsl/stdlib.h"

#include <string>

namespace scena::dsl {
namespace {

// The library is written as several raw literals and joined once, because MSVC
// caps a single string literal at 16380 bytes (C2026) and the domain
// sub-module is far past that. Joining into a function-local static keeps the
// returned view stable for the lifetime of the program.

// §8.14.1 — scalar physical types and their units. Transcribed from the
// normative text of ASAM OpenSCENARIO DSL 2.2.0 §8.14.1.1–§8.14.1.16; the
// conversion factors are the ones the standard prints, not rounder values.
constexpr std::string_view kTypesScalars = R"OSC(namespace stdtypes

# --- §8.14.1.1 length --------------------------------------------------------
type length is SI(m: 1)
unit nanometer  of length is SI(m: 1, factor: 0.000000001)
unit nm         of length is SI(m: 1, factor: 0.000000001)
unit micrometer of length is SI(m: 1, factor: 0.000001)
unit millimeter of length is SI(m: 1, factor: 0.001)
unit mm         of length is SI(m: 1, factor: 0.001)
unit centimeter of length is SI(m: 1, factor: 0.01)
unit cm         of length is SI(m: 1, factor: 0.01)
unit meter      of length is SI(m: 1, factor: 1)
unit m          of length is SI(m: 1, factor: 1)
unit kilometer  of length is SI(m: 1, factor: 1000)
unit km         of length is SI(m: 1, factor: 1000)
unit inch       of length is SI(m: 1, factor: 0.0254)
unit feet       of length is SI(m: 1, factor: 0.3048)
unit mile       of length is SI(m: 1, factor: 1609.344)
unit mi         of length is SI(m: 1, factor: 1609.344)

# --- §8.14.1.2 time ----------------------------------------------------------
type time is SI(s: 1)
unit millisecond of time is SI(s: 1, factor: 0.001)
unit ms          of time is SI(s: 1, factor: 0.001)
unit second      of time is SI(s: 1, factor: 1)
unit sec         of time is SI(s: 1, factor: 1)
unit s           of time is SI(s: 1, factor: 1)
unit minute      of time is SI(s: 1, factor: 60)
unit min         of time is SI(s: 1, factor: 60)
unit hour        of time is SI(s: 1, factor: 3600)
unit h           of time is SI(s: 1, factor: 3600)

# --- §8.14.1.3 speed ---------------------------------------------------------
type speed is SI(m: 1, s: -1)
unit meter_per_second    of speed is SI(m: 1, s: -1, factor: 1)
unit mps                 of speed is SI(m: 1, s: -1, factor: 1)
unit kilometer_per_hour  of speed is SI(m: 1, s: -1, factor: 0.277777778)
unit kmph                of speed is SI(m: 1, s: -1, factor: 0.277777778)
unit kph                 of speed is SI(m: 1, s: -1, factor: 0.277777778)
unit mile_per_hour       of speed is SI(m: 1, s: -1, factor: 0.447038889)
unit mph                 of speed is SI(m: 1, s: -1, factor: 0.447038889)
unit miph                of speed is SI(m: 1, s: -1, factor: 0.447038889)
unit mmph                of speed is SI(m: 1, s: -1, factor: 0.000000278)
unit millimeter_per_hour of speed is SI(m: 1, s: -1, factor: 0.000000278)

# --- §8.14.1.4 acceleration --------------------------------------------------
type acceleration is SI(m: 1, s: -2)
unit meter_per_sec_sqr          of acceleration is SI(m: 1, s: -2, factor: 1)
unit mpsps                      of acceleration is SI(m: 1, s: -2, factor: 1)
unit mpss                       of acceleration is SI(m: 1, s: -2, factor: 1)
unit kilometer_per_hour_per_sec of acceleration is SI(m: 1, s: -2, factor: 0.277777778)
unit kmphps                     of acceleration is SI(m: 1, s: -2, factor: 0.277777778)
unit mile_per_hour_per_sec      of acceleration is SI(m: 1, s: -2, factor: 0.447038889)
unit miphps                     of acceleration is SI(m: 1, s: -2, factor: 0.447038889)

# --- §8.14.1.5 jerk ----------------------------------------------------------
type jerk is SI(m: 1, s: -3)
unit meter_per_sec_cubed of jerk is SI(m: 1, s: -3, factor: 1)
unit mpspsps             of jerk is SI(m: 1, s: -3, factor: 1)
unit mile_per_sec_cubed  of jerk is SI(m: 1, s: -3, factor: 1609.344)
unit mipspsps            of jerk is SI(m: 1, s: -3, factor: 1609.344)

# --- §8.14.1.6 angle ---------------------------------------------------------
type angle is SI(rad: 1)
unit degree of angle is SI(rad: 1, factor: 0.01745329252)
unit deg    of angle is SI(rad: 1, factor: 0.01745329252)
unit radian of angle is SI(rad: 1, factor: 1)
unit rad    of angle is SI(rad: 1, factor: 1)

# --- §8.14.1.7 angular_rate --------------------------------------------------
type angular_rate is SI(rad: 1, s: -1)
unit degree_per_sec of angular_rate is SI(rad: 1, s: -1, factor: 0.01745329252)
unit degps          of angular_rate is SI(rad: 1, s: -1, factor: 0.01745329252)
unit radian_per_sec of angular_rate is SI(rad: 1, s: -1, factor: 1)
unit radps          of angular_rate is SI(rad: 1, s: -1, factor: 1)

# --- §8.14.1.8 angular_acceleration ------------------------------------------
type angular_acceleration is SI(rad: 1, s: -2)
unit degree_per_sec_sqr of angular_acceleration is SI(rad: 1, s: -2, factor: 0.01745329252)
unit degpsps            of angular_acceleration is SI(rad: 1, s: -2, factor: 0.01745329252)
unit radian_per_sec_sqr of angular_acceleration is SI(rad: 1, s: -2, factor: 1)
unit radpsps            of angular_acceleration is SI(rad: 1, s: -2, factor: 1)

# --- §8.14.1.9 mass ----------------------------------------------------------
type mass is SI(kg: 1)
unit gram     of mass is SI(kg: 1, factor: 0.001)
unit kilogram of mass is SI(kg: 1, factor: 1)
unit kg       of mass is SI(kg: 1, factor: 1)
unit ton      of mass is SI(kg: 1, factor: 1000)
unit pound    of mass is SI(kg: 1, factor: 0.45359237)
unit lb       of mass is SI(kg: 1, factor: 0.45359237)

# --- §8.14.1.10 temperature --------------------------------------------------
type temperature is SI(K: 1)
unit K          of temperature is SI(K: 1, factor: 1)
unit kelvin     of temperature is SI(K: 1, factor: 1)
unit celsius    of temperature is SI(K: 1, factor: 1, offset: 273.15)
unit C          of temperature is SI(K: 1, factor: 1, offset: 273.15)
unit fahrenheit of temperature is SI(K: 1, factor: 0.555555556, offset: 255.372222222)
unit F          of temperature is SI(K: 1, factor: 0.555555556, offset: 255.372222222)

# --- §8.14.1.11 pressure -----------------------------------------------------
type pressure is SI(kg: 1, m: -1, s: -2)
unit newton_per_meter_sqr of pressure is SI(kg: 1, m: -1, s: -2, factor: 1)
unit Pa                   of pressure is SI(kg: 1, m: -1, s: -2, factor: 1)
unit pascal               of pressure is SI(kg: 1, m: -1, s: -2, factor: 1)
unit hPa                  of pressure is SI(kg: 1, m: -1, s: -2, factor: 100)
unit atm                  of pressure is SI(kg: 1, m: -1, s: -2, factor: 101325)

# --- §8.14.1.12 luminous_intensity -------------------------------------------
type luminous_intensity is SI(cd: 1)
unit cd      of luminous_intensity is SI(cd: 1, factor: 1)
unit candela of luminous_intensity is SI(cd: 1, factor: 1)

# --- §8.14.1.13 luminous_flux ------------------------------------------------
type luminous_flux is SI(cd: 1, rad: 2)
unit lm    of luminous_flux is SI(cd: 1, rad: 2, factor: 1)
unit lumen of luminous_flux is SI(cd: 1, rad: 2, factor: 1)

# --- §8.14.1.14 illuminance --------------------------------------------------
type illuminance is SI(cd: 1, rad: 2, m: -2)
unit lx  of illuminance is SI(cd: 1, rad: 2, m: -2, factor: 1)
unit lux of illuminance is SI(cd: 1, rad: 2, m: -2, factor: 1)

# --- §8.14.1.15 electrical_current -------------------------------------------
type electrical_current is SI(A: 1)
unit ampere of electrical_current is SI(A: 1, factor: 1)
unit A      of electrical_current is SI(A: 1, factor: 1)

# --- §8.14.1.16 amount_of_substance ------------------------------------------
type amount_of_substance is SI(mol: 1)
unit mole of amount_of_substance is SI(mol: 1, factor: 1)
unit mol  of amount_of_substance is SI(mol: 1, factor: 1)
)OSC";

// §8.14.2 — compound structs, and the §8.13 string methods.
//
// §8.13 heads its definitions as part of the types sub-module and the
// `stdtypes` namespace, then says the string methods are "in the `std`
// namespace". The two statements disagree; they are declared here, in the
// sub-module whose section they appear under. Nothing observable turns on the
// choice, because a method on a primitive is reached through a value
// (`"abc".length()`) and never through a namespace-qualified name.
constexpr std::string_view kTypesCompound = R"OSC(
# --- §8.14.2 compound types --------------------------------------------------
struct position

struct position_3d inherits position:
    x: length
    y: length
    z: length
    def norm() -> length is undefined

struct geodetic_position_2d inherits position:
    latitude: angle
    longitude: angle

struct celestial_position_2d inherits position:
    azimuth: angle
    elevation: angle

struct orientation_3d:
    roll: angle
    pitch: angle
    yaw: angle

struct pose_3d:
    position: position_3d
    orientation: orientation_3d

struct translational_velocity_3d:
    x: speed
    y: speed
    z: speed
    def norm() -> speed is undefined

struct orientation_rate_3d:
    roll: angular_rate
    pitch: angular_rate
    yaw: angular_rate

struct velocity_6d:
    translational: translational_velocity_3d
    angular: orientation_rate_3d

struct translational_acceleration_3d:
    x: acceleration
    y: acceleration
    z: acceleration
    def norm() -> acceleration is undefined

struct orientation_acceleration_3d:
    roll: angular_acceleration
    pitch: angular_acceleration
    yaw: angular_acceleration

struct acceleration_6d:
    translational: translational_acceleration_3d
    angular: orientation_acceleration_3d

# --- §8.13 string methods ----------------------------------------------------
extend string:
    def length() -> int is undefined
    def contains(substring: string) -> bool is undefined
    def substring(start: int, end: int) -> string is undefined
    def split(separator: string) -> list of string is undefined
    def join(parts: list of string) -> string is undefined
    def replace(old: string, new: string) -> string is undefined
    def trim() -> string is undefined

export *
)OSC";

// §8.7 — physical object actors, their structs and their enumerations.
// Transcribed from the normative text of ASAM OpenSCENARIO DSL 2.2.0
// §8.7.2–§8.7.25, which prints them as parameter tables rather than as code.
//
// A parameter marked "Mandatory: no" in a table is still a declared field: the
// DSL has no optional-field marker, and §7.3.11 lets a scenario leave a field
// unconstrained. State variables are declared as fields too, for the same
// reason — §8.4.4 describes them as readable members, and the language draws no
// declaration-level distinction.
//
// §8.7.26 (groups of traffic participants) is explicitly non-normative and is
// not part of the library.
constexpr std::string_view kDomainEntities = R"OSC(namespace std use stdtypes

# --- §8.7.15 enum color ------------------------------------------------------
enum color: [
    white, silver, gray, black, red, maroon, yellow, olive, lime, green, aqua,
    teal, blue, navy, fuchsia, purple, violet, orange, brown, other
]

# --- §8.7.16 enum vehicle_category -------------------------------------------
# `truck` and `vru_vehicle` are the spellings earlier releases used; the
# standard keeps them for backward compatibility, equal to their replacements.
enum vehicle_category: [
    car, bus, heavy_truck, truck = heavy_truck, trailer, micro_mobility_device,
    vru_vehicle = micro_mobility_device, other, van, semi_tractor, semi_trailer,
    motorcycle, bicycle, stand_up_scooter, wheelchair, work_machine, train,
    tram, watercraft, aircraft, land_vehicle
]

# --- §8.7.17 enum trailer_category -------------------------------------------
enum trailer_category: [semi_trailer, full_trailer, central_axle_trailer]

# --- §8.7.18 enum hitch_type -------------------------------------------------
enum hitch_type: [ball, pintle, fifth_wheel, other, none]

# --- §8.7.19 enum intended_infrastructure ------------------------------------
enum intended_infrastructure: [
    driving, sidewalk, biking, rail, tram, bus, taxi, hov
]

# --- §8.7.20 enum traffic_participant_role -----------------------------------
enum traffic_participant_role: [
    civil, ambulance, fire_brigade, fire = fire_brigade, military, police,
    public_transport, roadside_assistance,
    road_assistance = roadside_assistance, garbage_collection, construction,
    road_construction = construction, other, freight_transport,
    special_transport, dangerous_goods_transport, agriculture, traffic_control
]

# --- §8.7.21 enum distance_direction -----------------------------------------
enum distance_direction: [longitudinal, lateral, vertical, euclidean]

# --- §8.7.22 enum road_distance_direction ------------------------------------
enum road_distance_direction: [longitudinal, lateral]

# --- §8.7.23 enum distance_mode ----------------------------------------------
enum distance_mode: [reference_points, bounding_boxes]

# --- §8.7.24 enum on_route_type ----------------------------------------------
enum on_route_type: [on_road, on_lane_section, on_lane, on_crossing]

# --- §8.7.25 enum route_distance_enum ----------------------------------------
enum route_distance_enum: [from_start, from_end]

# --- §8.7.11 struct bounding_box ---------------------------------------------
struct bounding_box:
    center: position_3d
    length: length
    width: length
    height: length

# --- §8.7.12 struct axle -----------------------------------------------------
struct axle:
    max_steering: angle
    wheel_diameter: length
    track_width: length
    position_x: length
    position_z: length
    number_of_wheels: uint

# --- §8.7.13 struct hitch_receiver -------------------------------------------
struct hitch_receiver:
    hitch_type: hitch_type
    position_x: length
    position_z: length
    max_rotation: angle
    max_tilt: angle
    is_towing: bool

# --- §8.7.14 struct hitch_coupler --------------------------------------------
struct hitch_coupler:
    hitch_type: hitch_type
    position_x: length
    position_z: length
    is_towed: bool

# --- §8.7.2 actor osc_actor --------------------------------------------------
actor osc_actor

# --- §8.7.3 actor physical_object --------------------------------------------
actor physical_object inherits osc_actor:
    bounding_box: bounding_box
    color: color
    geometry_reference: string
    center_of_gravity: position_3d
    pose: pose_3d

# --- §8.7.4 actor stationary_object ------------------------------------------
actor stationary_object inherits physical_object

modifier stationary_object.location:
    pose: pose_3d

# --- §8.7.5 actor movable_object ---------------------------------------------
actor movable_object inherits physical_object:
    velocity: velocity_6d
    acceleration: acceleration_6d
    speed: speed

# --- §8.7.6 actor traffic_participant ----------------------------------------
actor traffic_participant inherits movable_object:
    intended_infrastructure: list of intended_infrastructure
    role: traffic_participant_role
    def time_to_collision(reference: physical_object) -> time is undefined
    def time_gap(reference: physical_object, direction: road_distance_direction) -> time is undefined
    def space_gap(reference: physical_object, direction: road_distance_direction) -> length is undefined

# --- §8.7.7 actor vehicle ----------------------------------------------------
actor vehicle inherits traffic_participant:
    vehicle_category: vehicle_category
    axles: list of axle
    rear_overhang: length
    trailer_receiver: hitch_receiver
    def time_headway(reference: physical_object) -> time is undefined
    def space_headway(reference: physical_object) -> length is undefined

modifier vehicle.tow_trailer:
    trailer: trailer

# --- §8.7.8 actor trailer ----------------------------------------------------
actor trailer inherits vehicle:
    trailer_category: trailer_category
    coupler: hitch_coupler
    tow_vehicle: vehicle

# --- §8.7.9 actor person -----------------------------------------------------
actor person inherits traffic_participant

# --- §8.7.10 actor animal ----------------------------------------------------
actor animal inherits traffic_participant

export *
)OSC";

// §8.12 — the road abstraction classes, which §8.12.1 calls the classes that
// "describe the road network in an abstract way". Transcribed from the
// parameter tables of §8.12.3–§8.12.41.
//
// The §8.12.2 `map` actor is not here yet; it lands with the next slice of #43.
//
// This chunk also carries the §8.7 methods that were deferred until their
// argument types existed: the standard prints every one of them as an `extend`
// block, so they arrive exactly as written.
constexpr std::string_view kDomainRoads = R"OSC(
# --- §8.12.3 enum driving_rule -----------------------------------------------
enum driving_rule: [left_hand_traffic, right_hand_traffic]

# --- §8.12.6 enum directionality ---------------------------------------------
enum directionality: [uni_direction, bi_direction, split, free, none, other]

# --- §8.12.12 enum lane_type -------------------------------------------------
enum lane_type: [driving, non_driving, vru_vehicles, pedestrian, other]

# --- §8.12.13 enum lane_use --------------------------------------------------
enum lane_use: [
    normal, exit, entry, on_ramp, off_ramp, connecting_ramp, hov, bus,
    mixed_traffic_vru, parking, stop, restricted, border, shoulder, curb,
    median, bicycle, motorcycle, sidewalk, protected_sidewalk, none, other
]

# --- §8.12.14 enum side_left_right -------------------------------------------
enum side_left_right: [left, right]

# --- §8.12.15 enum lon_lat ---------------------------------------------------
enum lon_lat: [longitudinal, lateral]

# --- §8.12.17 enum crossing_marking ------------------------------------------
enum crossing_marking: [unmarked, marked, zebra, other]

# --- §8.12.18 enum crossing_use ----------------------------------------------
enum crossing_use: [pedestrian, animal, bicycle, rail_road, other]

# --- §8.12.19 enum crossing_elevation ----------------------------------------
enum crossing_elevation: [road_level, curb_level, refuge_island, other]

# --- §8.12.22 enum junction_direction ----------------------------------------
enum junction_direction: [straight, right, u_turn, left, other]

# --- §8.12.23 enum route_overlap_kind ----------------------------------------
enum route_overlap_kind: [equal, start, end, inside, any, other]

# --- §8.12.24 enum lateral_overlap_kind --------------------------------------
enum lateral_overlap_kind: [never, sometimes, always]

# --- §8.12.29 enum connect_route_points --------------------------------------
enum connect_route_points: [road, lane_section, lane, crossing, waypoint]

# --- §8.12.35 enum path_interpolation ----------------------------------------
enum path_interpolation: [straight_line, smooth]

# --- §8.12.36 enum relative_transform ----------------------------------------
enum relative_transform: [
    world_relative, object_relative, road_relative, lane_relative
]

# --- §8.12.5 struct route ----------------------------------------------------
struct route:
    length: length
    directionality: directionality
    min_lanes: uint
    max_lanes: uint
    anchors: list of string
    def start_point() -> route_point is undefined
    def end_point() -> route_point is undefined

# --- §8.12.7 struct route_element --------------------------------------------
struct route_element inherits route

# --- §8.12.4 struct junction -------------------------------------------------
struct junction:
    roads: list of road

# --- §8.12.8 struct road -----------------------------------------------------
struct road inherits route_element:
    s_positive: list of lane_section
    s_negative: list of lane_section

# --- §8.12.9 struct lane_section ---------------------------------------------
struct lane_section inherits route_element:
    road: road
    lanes: list of lane
    s_axis: lane

# --- §8.12.10 struct lane ----------------------------------------------------
struct lane inherits route_element:
    lane_section: lane_section
    lane_type: lane_type
    lane_use: lane_use
    width: length

# --- §8.12.16 struct crossing_type -------------------------------------------
struct crossing_type:
    marking: crossing_marking
    use: crossing_use
    elevation: crossing_elevation

# --- §8.12.11 struct crossing ------------------------------------------------
struct crossing inherits route_element:
    start_lane: lane
    end_lane: lane
    start_s_coord: length
    end_s_coord: length
    width: length
    crossing_type: crossing_type

# --- §8.12.20 struct compound_route ------------------------------------------
struct compound_route inherits route:
    route_elements: list of route_element

# --- §8.12.21 struct compound_lane -------------------------------------------
struct compound_lane inherits route:
    lanes: list of lane

# --- §8.12.25 struct route_point ---------------------------------------------
struct route_point inherits route_element:
    route: route
    s: length
    t: length

# --- §8.12.26 struct xyz_point -----------------------------------------------
struct xyz_point inherits route_element:
    position: position_3d

# --- §8.12.27 struct odr_point -----------------------------------------------
struct odr_point inherits route_element:
    road_id: string
    lane_id: string
    s: length
    t: length

# --- §8.12.28 struct geodetic_point ------------------------------------------
struct geodetic_point inherits route_element:
    latitude: angle
    longitude: angle
    altitude: length

# --- §8.12.30 struct path ----------------------------------------------------
struct path inherits route_element:
    points: list of pose_3d
    interpolation: path_interpolation

# --- §8.12.31 struct relative_path -------------------------------------------
struct relative_path:
    interpolation: path_interpolation

# --- §8.12.32 struct relative_path_pose_3d -----------------------------------
struct relative_path_pose_3d inherits relative_path:
    points: list of pose_3d

# --- §8.12.33 struct relative_path_st ----------------------------------------
struct relative_path_st inherits relative_path:
    points: list of route_point

# --- §8.12.34 struct relative_path_odr ---------------------------------------
struct relative_path_odr inherits relative_path:
    points: list of odr_point

# --- §8.12.37 struct trajectory ----------------------------------------------
struct trajectory:
    points: list of pose_3d
    time_stamps: list of time
    interpolation: path_interpolation

# --- §8.12.38 struct relative_trajectory -------------------------------------
struct relative_trajectory:
    time_stamps: list of time
    interpolation: path_interpolation

# --- §8.12.39 struct relative_trajectory_pose_3d -----------------------------
struct relative_trajectory_pose_3d inherits relative_trajectory:
    points: list of pose_3d

# --- §8.12.40 struct relative_trajectory_st ----------------------------------
struct relative_trajectory_st inherits relative_trajectory:
    points: list of route_point

# --- §8.12.41 struct relative_trajectory_odr ---------------------------------
struct relative_trajectory_odr inherits relative_trajectory:
    points: list of odr_point

# --- §8.7.3.1 physical_object methods over the road network ------------------
# Held back from the §8.7 slice until their argument types existed. The
# standard prints each of these as an `extend` block, which is how they arrive.
extend physical_object:
    def object_distance(reference: physical_object, direction: distance_direction, mode: distance_mode = reference_points) -> length is undefined
    def road_distance(reference: physical_object, direction: road_distance_direction, mode: distance_mode = reference_points, route_type: on_route_type = on_road) -> length is undefined
    def distance_to_xyz_point(point: xyz_point, direction: distance_direction, mode: distance_mode = reference_points) -> length is undefined
    def distance_to_route_point(point: route_point, direction: road_distance_direction, mode: distance_mode = reference_points, route_type: on_route_type = on_road) -> length is undefined
    def distance_to_odr_point(point: odr_point, direction: road_distance_direction, mode: distance_mode = reference_points, route_type: on_route_type = on_road) -> length is undefined
    def get_s_coord(route_type: on_route_type = on_road) -> length is undefined
    def get_t_coord(route_type: on_route_type = on_road) -> length is undefined
    def get_route_point(route_type: on_route_type = on_road) -> route_point is undefined

# --- §8.7.5.1.1 distance_along_route -----------------------------------------
extend traffic_participant:
    def distance_along_route(route: route, from: route_distance_enum = from_start) -> length is undefined
)OSC";

// §8.8.1 the action hierarchy, and §8.10/§8.11 the environment actor with its
// structs and its actions. Transcribed from the parameter tables of
// §8.8.1.1, §8.10.2–§8.10.9 and §8.11.2–§8.11.8.
constexpr std::string_view kDomainEnvironment = R"OSC(
# --- §8.8.1.1 action osc_action ----------------------------------------------
# "The parent action `osc_action` is the base class for all actions in the ASAM
# OpenSCENARIO domain model and it is associated with the parent actor
# `osc_actor`", with `action_for_environment` and `action_for_movable_object`
# as its two children.
action osc_actor.osc_action

action environment.action_for_environment inherits osc_actor.osc_action

action movable_object.action_for_movable_object inherits osc_actor.osc_action

# --- §8.10.4 struct air ------------------------------------------------------
struct air:
    temperature: temperature
    pressure: pressure
    relative_humidity: float

# --- §8.10.5 struct precipitation --------------------------------------------
# §8.10.5: volumetric flux reduces to the same dimension as speed, and the
# language cannot declare two physical types over one unit, so the standard
# uses `speed` here too.
struct precipitation:
    intensity: speed

# --- §8.10.6 struct wind -----------------------------------------------------
struct wind:
    speed: speed
    direction: angle

# --- §8.10.7 struct fog ------------------------------------------------------
struct fog:
    visual_range: length

# --- §8.10.8 struct clouds ---------------------------------------------------
struct clouds:
    cloudiness: uint

# --- §8.10.9 struct celestial_light_source -----------------------------------
struct celestial_light_source:
    position: celestial_position_2d

# --- §8.10.3 struct weather --------------------------------------------------
struct weather:
    air: air
    rain: precipitation
    snow: precipitation
    wind: wind
    fog: fog
    clouds: clouds

# --- §8.10.2 actor environment -----------------------------------------------
actor environment inherits osc_actor:
    geodetic_position: geodetic_position_2d
    datetime: time
    sun: celestial_light_source
    moon: celestial_light_source
    weather: weather
    def local_to_unix_time(year: uint, month: uint, day: uint, hour: uint, minute: uint, second: uint, time_zone: float) -> time is undefined

# --- §8.11.2 action air ------------------------------------------------------
action environment.air inherits environment.action_for_environment:
    temperature: temperature
    pressure: pressure
    relative_humidity: float

# --- §8.11.3 action rain -----------------------------------------------------
action environment.rain inherits environment.action_for_environment:
    intensity: speed

# --- §8.11.4 action snow -----------------------------------------------------
action environment.snow inherits environment.action_for_environment:
    intensity: speed

# --- §8.11.5 action wind -----------------------------------------------------
action environment.wind inherits environment.action_for_environment:
    speed: speed
    direction: angle

# --- §8.11.6 action fog ------------------------------------------------------
action environment.fog inherits environment.action_for_environment:
    visual_range: length

# --- §8.11.7 action clouds ---------------------------------------------------
action environment.clouds inherits environment.action_for_environment:
    cloudiness: uint

# --- §8.11.8 action assign_celestial_position --------------------------------
action environment.assign_celestial_position inherits environment.action_for_environment:
    light_source: celestial_light_source
    azimuth: angle
    elevation: angle
)OSC";

// §8.15 the traffic lights: the bulb enums and struct, the semantic-state enum,
// the traffic-light and group structs with their methods, the stop line, the
// phase and cycle structs, the controller actor and its seven actions.
// Transcribed from the parameter tables of §8.15.2–§8.15.9; the four method
// prototypes are the chapter's only printed code.
// Translation worksheet: docs/dev/stdlib-worksheets/08-15-traffic-lights.md
constexpr std::string_view kDomainTrafficLights = R"OSC(
# --- §8.15.2.1 enum bulb_icon ------------------------------------------------
# Table 302 pairs each icon with the OpenDRIVE signal type that draws it; that
# mapping is documentation and is not part of the declaration.
enum bulb_icon: [
    unknown, circle, pedestrian_walking, pedestrian_standing, tram, bus,
    bicycle, horse_rider, person_bicycle, bicycle_left, bicycle_right,
    arrow_left, arrow_right, arrow_straight, arrow_left_straight,
    arrow_right_straight, arrow_diagonal_left, arrow_diagonal_right,
    arrow_u_turn_left, arrow_u_turn_right, arrow_left_right, lane_arrow_down,
    lane_arrow_down_right, lane_arrow_down_left, lane_cross, txt_walk,
    txt_dont_walk, countdown, pt_horizontal_bar, pt_vertical_bar, pt_slash_bar,
    pt_backslash_bar, pt_small_circle, pt_triangle, switch_x, switch_v_flipped,
    switch_v_left, switch_v_right, switch_t, switch_a, switch_bar_v,
    switch_bar_v_flipped, switch_bar_v_right, switch_bar_v_left,
    switch_dotted_circle
]

# --- §8.15.2.2 enum bulb_color -----------------------------------------------
# Every member also names a member of `color` (§8.7.15), so §7.3.3 makes a
# scenario write `bulb_color!red` wherever both are in scope.
enum bulb_color: [unknown, red, yellow, green, blue, white]

# --- §8.15.2.3 enum bulb_state -----------------------------------------------
enum bulb_state: [unknown, is_off, is_on, is_flashing]

# --- §8.15.2.4 struct traffic_light_bulb -------------------------------------
# Table 308 calls the first four parameters and Table 309 calls `state` a state
# variable; the language has one kind of member, so both become fields.
struct traffic_light_bulb:
    map_id: string
    icon: bulb_icon
    color: bulb_color
    icon_positive: bool
    state: bulb_state

# --- §8.15.3.1 enum semantic_traffic_light_state -----------------------------
enum semantic_traffic_light_state: [
    off, stop, attention, caution, stop_attention, go, go_exclusive,
    non_functional
]

# --- §8.15.4.1 struct traffic_light ------------------------------------------
# A traffic light has no state member of its own: its state IS the state of its
# bulbs, which is what the three methods below read and convert.
struct traffic_light:
    map_id: string
    bulbs: list of traffic_light_bulb
    pose: pose_3d
    height: length
    width: length
    group: traffic_light_group
    def state_equal(bulbs: list of traffic_light_bulb) -> bool is undefined
    def semantic_state_to_state(state: semantic_traffic_light_state) -> list of traffic_light_bulb is undefined
    def state_to_semantic_state(bulbs: list of traffic_light_bulb) -> semantic_traffic_light_state is undefined

# --- §8.15.4.2 struct traffic_light_group ------------------------------------
# §8.15.4.2.1's prototype prints `extend traffic_light:`, but its heading, its
# prose and Table 319 all place `state_equal` on the GROUP; the lone printed
# receiver is a slip and the method is declared here.
struct traffic_light_group:
    map_id: string
    bulbs: list of traffic_light_bulb
    traffic_lights: list of traffic_light
    cycle: traffic_light_cycle
    def state_equal(bulbs: list of traffic_light_bulb) -> bool is undefined

# --- §8.15.5.1 enum stop_line_marking ----------------------------------------
enum stop_line_marking: [none, solid, broken]

# --- §8.15.5.2 struct traffic_light_stop_line --------------------------------
struct traffic_light_stop_line inherits route_element:
    traffic_light_group: traffic_light_group
    route: route
    rightmost_lane: uint
    leftmost_lane: uint
    offset: length
    secondary_stop_offset: length
    primary_stop_line_marking: stop_line_marking
    secondary_stop_line_marking: stop_line_marking

# --- §8.15.6.1 struct traffic_light_phase ------------------------------------
struct traffic_light_phase:
    group: traffic_light_group
    bulbs_state: list of traffic_light_bulb
    duration: time

# --- §8.15.6.2 struct traffic_light_cycle ------------------------------------
struct traffic_light_cycle:
    phases: list of traffic_light_phase
    synchronization_group_id: uint
    start_offset: time

# --- §8.15.8.1 actor traffic_light_controller --------------------------------
actor traffic_light_controller inherits osc_actor

# --- §8.15.9 actions for controlling traffic lights --------------------------
# §8.8.1 declares only `action_for_environment` and `action_for_movable_object`
# as intermediate bases, so these inherit `osc_action` directly rather than
# invent a third one. Every action but `play_cycles` is instantaneous; action
# ending is runtime semantics and has no place in a declaration.
action traffic_light_controller.set_bulb_state inherits osc_actor.osc_action:
    traffic_light: traffic_light
    bulb_color: bulb_color
    bulb_kind: bulb_icon
    bulb_state: bulb_state
    sync: bool

action traffic_light_controller.set_state inherits osc_actor.osc_action:
    traffic_light: traffic_light
    state: list of bulb_state
    sync: bool

action traffic_light_controller.set_semantic_state inherits osc_actor.osc_action:
    traffic_light: traffic_light
    state: semantic_traffic_light_state
    sync: bool

# Table 337 names this action's first parameter `traffic_light` of type
# `traffic_light` while its description says "the traffic light GROUP affected".
# The parameter table is the surface a conforming scenario is written against,
# so the printed name and type are carried verbatim — the same rule that keeps
# §8.14.1.3's rounded conversion factors (ADR-0029).
action traffic_light_controller.set_group_bulb_state inherits osc_actor.osc_action:
    traffic_light: traffic_light
    bulb_color: bulb_color
    bulb_kind: bulb_icon
    bulb_state: bulb_state
    sync: bool

action traffic_light_controller.set_group_state inherits osc_actor.osc_action:
    group: traffic_light_group
    state: list of bulb_state
    sync: bool

action traffic_light_controller.set_group_semantic_state inherits osc_actor.osc_action:
    traffic_light_group: traffic_light_group
    state: semantic_traffic_light_state
    sync: bool

action traffic_light_controller.play_cycles inherits osc_actor.osc_action:
    cycles: list of traffic_light_cycle
)OSC";

// §8.12.2 the `map` actor: the top-level holder of the abstract road network,
// its 18 query methods and its 12 search-space modifiers. The methods are the
// chapter's printed `extend map:` prototypes; the fields and the modifiers come
// from the parameter tables of §8.12.2 and §8.12.2.2.
// Translation worksheet: docs/dev/stdlib-worksheets/08-12-02-map.md
constexpr std::string_view kDomainMap = R"OSC(
# --- §8.12.2 actor map -------------------------------------------------------
# §8.15.7 prints `traffic_light_groups` and `traffic_light_control` as a
# separate `extend map:` block. §7.3.15 makes a type the union of its
# declarations, so declaring them inline here is the same program.
actor map inherits osc_actor:
    map_file: string
    routes: list of route
    junctions: list of junction
    driving_rule: driving_rule
    traffic_light_groups: list of traffic_light_group
    traffic_light_control: list of traffic_light_cycle

    # §8.12.2.1 methods. Each is a query a runtime answers; the library only
    # has to give it a signature to check calls against.
    def odr_to_route_point(road_id: string, lane_id: string, s: length, t: length) -> route_point is undefined
    def xyz_to_route_point(x: length, y: length, z: length) -> route_point is undefined
    def route_point_to_xyz(route_point: route_point) -> xyz_point is undefined
    def outer_side() -> side_left_right is undefined
    def inner_side() -> side_left_right is undefined
    def create_route(routes: list of route, connect_points_by: connect_route_points, legal_route: bool) -> compound_route is undefined
    def create_route_point(route: route, s: length, t: length) -> route_point is undefined
    def create_xyz_point(x: length, y: length, z: length) -> xyz_point is undefined
    def create_odr_point(road_id: string, lane_id: string, s: length, t: length) -> odr_point is undefined
    def create_path(points: list of pose_3d, interpolation: path_interpolation) -> path is undefined
    def create_path_odr_points(points: list of odr_point, interpolation: path_interpolation, on_road_network: bool) -> path is undefined
    def create_path_route_points(points: list of route_point, interpolation: path_interpolation, on_road_network: bool) -> path is undefined
    def create_trajectory(points: list of pose_3d, time_stamps: list of time, interpolation: path_interpolation) -> trajectory is undefined
    def create_trajectory_odr_points(points: list of odr_point, time_stamps: list of time, interpolation: path_interpolation, on_road_network: bool) -> trajectory is undefined
    def create_trajectory_route_points(points: list of route_point, time_stamps: list of time, interpolation: path_interpolation, on_road_network: bool) -> trajectory is undefined
    def resolve_relative_path(relative_path: relative_path, reference: physical_object, transform: relative_transform) -> path is undefined
    def resolve_relative_trajectory(relative_trajectory: relative_trajectory, reference: physical_object, transform: relative_transform) -> trajectory is undefined
    def get_map_file() -> string is undefined

# --- §8.12.2.2 modifiers -----------------------------------------------------
# An actor's modifier takes the §7.2.2.2.9 PREFIXED form: §7.3.12.2's `of`
# names a scenario or an action, never an actor.
#
# Many of the field names below are also type names in this namespace
# (`route`, `road`, `lane`, `lane_section`, `crossing`, `junction`,
# `lane_type`, `lane_use`, `directionality`). Members and types are separate
# lookup spaces, so this checks cleanly; it is pinned by test because it reads
# like a collision.
modifier map.number_of_lanes:
    route: route
    num_of_lanes: uint
    lane_type: lane_type
    lane_use: lane_use
    directionality: directionality

modifier map.routes_are_in_sequence:
    preceding: route
    succeeding: route
    road: road

modifier map.roads_follow_in_junction:
    junction: junction
    in_road: road
    out_road: road
    direction: junction_direction
    clockwise_count: uint
    number_of_roads: uint
    in_lane: lane
    out_lane: lane
    junction_route: route
    resulting_route: route

modifier map.routes_overlap:
    route1: route
    route2: route
    overlap_kind: route_overlap_kind

modifier map.lane_side:
    lane1: lane
    side: side_left_right
    lane2: lane
    count: uint
    lane_section: lane_section

modifier map.compound_lane_side:
    lane1: compound_lane
    side: side_left_right
    lane2: compound_lane
    count: uint
    route: route

modifier map.end_lane:
    lane: lane

modifier map.start_lane:
    lane: lane

modifier map.crossing_connects:
    crossing: crossing
    start_lane: lane
    end_lane: lane
    start_s_coord: length
    start_angle: angle

modifier map.routes_are_opposite:
    route1: route
    route2: route
    containing_road: road
    lateral_overlap: lateral_overlap_kind

modifier map.set_map_file:
    file: string

modifier map.set_traffic_lights_control_file:
    file: string
)OSC";

// §8.8.2–§8.8.4 the movement actions: fifteen for `movable_object`, thirteen
// for `vehicle` and one for `person`, plus the four enums they use and the two
// intermediate bases §8.8.1 names only in prose. Each action's table gives it a
// parent, its controlled states and how it ends; only the parent and the
// parameters translate, the other two being runtime semantics.
// Translation worksheet: docs/dev/stdlib-worksheets/08-08-movement-actions.md
constexpr std::string_view kDomainMovementActions = R"OSC(
# --- §8.8.2/§8.8.3/§8.8.4 the intermediate action bases ----------------------
# §8.8.1 declares `osc_action` with exactly two children. §8.8.3's and §8.8.4's
# "Parents" rows then name `action_for_vehicle` and `action_for_person`, which
# no table declares; §8.8.1's prose supplies them — "Actions for actors that
# are children of `movable_object`, like `vehicle` or `person`, inherit from
# `action_for_movable_object`".
action vehicle.action_for_vehicle inherits movable_object.action_for_movable_object

action person.action_for_person inherits movable_object.action_for_movable_object

# --- §8.8.2.18 enum dynamic_profile ------------------------------------------
enum dynamic_profile: [none, constant, smooth, asap]

# --- §8.8.2 actions for movable object ---------------------------------------
# The tables give each action a parent, its controlled states and how it ends.
# Only the parent and the parameters translate: the other two are runtime
# semantics with no place in a declaration.
action movable_object.move inherits movable_object.action_for_movable_object

# §8.8.2.4: "Use only one of the three possible arguments" — the language has no
# choice group, and §7.3.11 already lets a scenario leave a field unconstrained.
action movable_object.assign_position inherits movable_object.action_for_movable_object:
    position: position_3d
    route_point: route_point
    odr_point: odr_point

action movable_object.assign_orientation inherits movable_object.action_for_movable_object:
    orientation: orientation_3d

action movable_object.assign_speed inherits movable_object.action_for_movable_object:
    speed: speed

action movable_object.assign_acceleration inherits movable_object.action_for_movable_object:
    acceleration: acceleration

action movable_object.replay_path inherits movable_object.action_for_movable_object:
    absolute: path
    relative: relative_path
    reference: physical_object
    transform: relative_transform
    start_offset: length
    end_offset: length

action movable_object.replay_trajectory inherits movable_object.action_for_movable_object:
    absolute: trajectory
    relative: relative_trajectory
    reference: physical_object
    transform: relative_transform
    start_offset: length
    end_offset: length

action movable_object.remain_stationary inherits movable_object.action_for_movable_object

# `target_xyz` is marked deprecated in the same table that declares it, in
# favour of `target_position`. It is declared anyway: the language has no
# deprecation marker, and dropping a field the standard prints would reject a
# conforming scenario.
action movable_object.change_position inherits movable_object.action_for_movable_object:
    target_position: position
    target_st: route_point
    target_odr: odr_point
    target_xyz: position_3d
    interpolation: path_interpolation
    on_road_network: bool

action movable_object.change_speed inherits movable_object.action_for_movable_object:
    target: speed
    rate_profile: dynamic_profile
    rate_peak: acceleration

action movable_object.keep_speed inherits movable_object.action_for_movable_object

action movable_object.change_acceleration inherits movable_object.action_for_movable_object:
    target: acceleration
    rate_profile: dynamic_profile
    rate_peak: jerk

action movable_object.keep_acceleration inherits movable_object.action_for_movable_object

# §8.8.2.16/§8.8.2.17 are the target-behaviour counterparts of replay_path and
# replay_trajectory (§8.8.2.1 vs §8.8.2.2): same parameters, different promise
# about the actor's dynamic limits.
action movable_object.follow_path inherits movable_object.action_for_movable_object:
    absolute: path
    relative: relative_path
    reference: physical_object
    transform: relative_transform
    start_offset: length
    end_offset: length

action movable_object.follow_trajectory inherits movable_object.action_for_movable_object:
    absolute: trajectory
    relative: relative_trajectory
    reference: physical_object
    transform: relative_transform
    start_offset: length
    end_offset: length

# --- §8.8.3.14 enum lane_change_side -----------------------------------------
enum lane_change_side: [left, right, inside, outside, same]

# --- §8.8.3.15 enum gap_direction --------------------------------------------
enum gap_direction: [ahead, behind, left, right, inside, outside]

# --- §8.8.3.16 enum headway_direction ----------------------------------------
enum headway_direction: [ahead, behind]

# --- §8.8.3 actions for vehicle ----------------------------------------------
action vehicle.drive inherits vehicle.action_for_vehicle

action vehicle.follow_lane inherits vehicle.action_for_vehicle:
    offset: length
    rate_profile: dynamic_profile
    rate_peak: speed
    target: lane

# `reference` documents `Default=it.actor`; a default naming the invoking actor
# is not a constant expression, so the field is simply left unconstrained.
action vehicle.change_lane inherits vehicle.action_for_vehicle:
    num_of_lanes: uint
    side: lane_change_side
    reference: physical_object
    offset: length
    rate_profile: dynamic_profile
    rate_peak: speed
    target: lane

# The change_/keep_ pairs below are deliberately asymmetric: a `change_*` takes
# a `gap_direction` (six values, longitudinal and lateral), a `keep_*` takes a
# `road_distance_direction` (§8.7.22, just longitudinal and lateral).
action vehicle.change_time_gap inherits vehicle.action_for_vehicle:
    target: time
    direction: gap_direction
    reference: physical_object

action vehicle.keep_time_gap inherits vehicle.action_for_vehicle:
    reference: physical_object
    direction: road_distance_direction

action vehicle.change_space_gap inherits vehicle.action_for_vehicle:
    target: length
    direction: gap_direction
    reference: physical_object

action vehicle.keep_space_gap inherits vehicle.action_for_vehicle:
    reference: physical_object
    direction: road_distance_direction

action vehicle.change_time_headway inherits vehicle.action_for_vehicle:
    target: time
    direction: headway_direction
    reference: physical_object

action vehicle.keep_time_headway inherits vehicle.action_for_vehicle:
    reference: physical_object

action vehicle.change_space_headway inherits vehicle.action_for_vehicle:
    target: length
    direction: headway_direction
    reference: physical_object

action vehicle.keep_space_headway inherits vehicle.action_for_vehicle:
    reference: physical_object

action vehicle.connect_trailer inherits vehicle.action_for_vehicle:
    trailer: trailer

action vehicle.disconnect_trailer inherits vehicle.action_for_vehicle

# --- §8.8.4 actions for person -----------------------------------------------
# §8.8.4's prose says a person OR an animal walks, but the parent it names is
# `action_for_person` and no `action_for_animal` exists. Declared on `person` as
# printed; an animal reaches `move` through `action_for_movable_object`.
action person.walk inherits person.action_for_person
)OSC";

const std::string& types_module() {
    static const std::string source = std::string(kTypesScalars) + std::string(kTypesCompound);
    return source;
}

const std::string& domain_module() {
    static const std::string source = std::string(kDomainEntities) + std::string(kDomainRoads) +
                                      std::string(kDomainEnvironment) +
                                      std::string(kDomainTrafficLights) + std::string(kDomainMap) +
                                      std::string(kDomainMovementActions);
    return source;
}

} // namespace

bool is_reserved_module(std::string_view module_reference) {
    // §7.7.5.1.2: "Any structured identifier that starts with the `osc`
    // identifier is reserved". `oscar.foo` does not start with the *identifier*
    // `osc`, so the check is on the first component, not on a prefix.
    return module_reference == "osc" || module_reference.rfind("osc.", 0) == 0;
}

bool is_standard_module(std::string_view module_reference) {
    return module_reference == kStandardTypesModule || module_reference == kStandardDomainModule ||
           module_reference == kStandardAllModule || module_reference == kStandardLegacyModule;
}

std::vector<std::string_view> standard_submodules(std::string_view module_reference) {
    if (module_reference == kStandardTypesModule) {
        return {kStandardTypesModule};
    }
    if (module_reference == kStandardDomainModule || module_reference == kStandardAllModule ||
        module_reference == kStandardLegacyModule) {
        // The domain sub-module is written in the types sub-module's physical
        // types, so it always arrives with it — §7.7.5.1's import-once rule
        // makes the duplicate reference free.
        return {kStandardTypesModule, kStandardDomainModule};
    }
    return {};
}

bool standard_module_auto_uses(std::string_view module_reference) {
    // §7.7.5.2.3: only the legacy `import osc.standard` adds `std` and
    // `stdtypes` to the use list of the null namespace. `osc.standard.all`
    // makes the definitions reachable but leaves the use list alone.
    return module_reference == kStandardLegacyModule;
}

std::string_view standard_module_source(std::string_view module_reference) {
    if (module_reference == kStandardTypesModule) {
        return types_module();
    }
    if (module_reference == kStandardDomainModule) {
        return domain_module();
    }
    return {};
}

std::string_view standard_module_namespace(std::string_view module_reference) {
    if (module_reference == kStandardTypesModule) {
        return "stdtypes";
    }
    if (module_reference == kStandardDomainModule) {
        return "std";
    }
    return {};
}

} // namespace scena::dsl
