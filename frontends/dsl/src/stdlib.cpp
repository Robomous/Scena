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

const std::string& types_module() {
    static const std::string source = std::string(kTypesScalars) + std::string(kTypesCompound);
    return source;
}

const std::string& domain_module() {
    static const std::string source = std::string(kDomainEntities);
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
