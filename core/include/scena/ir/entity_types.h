// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <optional>
#include <string>
#include <vector>

/// Value types for the OpenSCENARIO entity taxonomy: the object/category
/// enumerations and the plain-data performance, axle, and property records
/// carried by the concrete entity objects (Vehicle/Pedestrian/MiscObject,
/// see entity.h). Pure data — no runtime behavior lives here.
///
/// Version note: Scena targets ASAM OpenSCENARIO XML 1.0–1.3; the local spec
/// copy is 1.4.0. The category enumerations below are the full 1.4.0 sets so a
/// 1.0–1.3 document (a subset) always maps, with the post-1.3 additions and the
/// 1.3→1.4 deprecations called out per enumerator (targeted version wins per
/// the standards rule, handled explicitly).
namespace scena::ir {

/// The kind of entity object, per ASAM OpenSCENARIO XML 1.4.0 §ObjectType.
/// The standard also defines `external` (an ExternalObjectReference); Scena
/// executes only the three concrete inline object types, so an external
/// reference is out of scope for v0.0.1 and has no enumerator here.
enum class ObjectType {
    Vehicle,
    Pedestrian,
    MiscObject,
};

/// Category of a Vehicle, per ASAM OpenSCENARIO XML 1.4.0 §VehicleCategory.
/// The 1.3 baseline is {Bicycle, Bus, Car, Motorbike, Semitrailer, Trailer,
/// Train, Tram, Truck, Van}; 1.4 added the remaining literals (harmonized with
/// the ASAM TrafficParticipants standard) and deprecated two — see per-value
/// notes. Enumerator order is the spec document order.
enum class VehicleCategory {
    Aircraft,            ///< 1.4 addition.
    Bicycle,             ///< 1.3 baseline.
    Bus,                 ///< 1.3 baseline.
    Car,                 ///< 1.3 baseline.
    HeavyTruck,          ///< 1.4 addition.
    LandVehicle,         ///< 1.4 addition.
    MicromobilityDevice, ///< 1.4 addition.
    Motorbike,           ///< 1.3 baseline. DEPRECATED in 1.4: use Motorcycle.
    Motorcycle,          ///< 1.4 addition.
    Other,               ///< 1.4 addition.
    Semitractor,         ///< 1.4 addition.
    Semitrailer,         ///< 1.3 baseline.
    StandupScooter,      ///< 1.4 addition.
    Trailer,             ///< 1.3 baseline.
    Train,               ///< 1.3 baseline.
    Tram,                ///< 1.3 baseline.
    Truck,               ///< 1.3 baseline. DEPRECATED in 1.4: use HeavyTruck/Semitractor.
    Van,                 ///< 1.3 baseline.
    Watercraft,          ///< 1.4 addition.
    Wheelchair,          ///< 1.4 addition.
    WorkMachine,         ///< 1.4 addition.
};

/// Category of a Pedestrian, per ASAM OpenSCENARIO XML 1.4.0
/// §PedestrianCategory.
enum class PedestrianCategory {
    Animal,
    Pedestrian,
    Wheelchair, ///< DEPRECATED in 1.4: use a Vehicle of category Wheelchair.
};

/// Category of a MiscObject, per ASAM OpenSCENARIO XML 1.4.0
/// §MiscObjectCategory. Enumerator order is the spec document order.
enum class MiscObjectCategory {
    Barrier,
    Building,
    Crosswalk,
    Gantry,
    None,
    Obstacle,
    ParkingSpace,
    Patch,
    Pole,
    Railing,
    RoadMark,
    SoundBarrier,
    StreetLamp,
    TrafficIsland,
    Tree,
    Vegetation,
    Wind, ///< DEPRECATED in 1.4.
};

/// Role of a Vehicle or Pedestrian, per ASAM OpenSCENARIO XML 1.4.0 §Role.
/// The default role when the attribute is omitted is `None` (§Vehicle,
/// §Pedestrian). Enumerator order is the spec document order.
enum class Role {
    None,
    Agriculture,
    Ambulance,
    Civil,
    Construction,
    DangerousGoodsTransport,
    Fire, ///< DEPRECATED in 1.4: use FireBrigade.
    FireBrigade,
    FreightTransport,
    GarbageCollection,
    Military,
    Other,
    Police,
    PublicTransport,
    RoadAssistance, ///< DEPRECATED in 1.4: use RoadsideAssistance.
    RoadsideAssistance,
    SpecialTransport,
    TrafficControl,
};

/// Performance limits of a vehicle, per ASAM OpenSCENARIO XML 1.4.0
/// §Performance. maxSpeed/maxAcceleration/maxDeceleration are required
/// (1..1); the rate limits are optional (0..1) and mean "unbounded" (i.e. an
/// infinite rate) when absent. All non-negative — ranges [0..inf[ — validated
/// at Engine::init. The default controller (p2-s2) clamps target-speed
/// setpoints against these.
struct Performance {
    double max_speed = 0.0;                      ///< m/s. Range [0..inf[.
    double max_acceleration = 0.0;               ///< m/s^2. Range [0..inf[.
    double max_deceleration = 0.0;               ///< m/s^2 (magnitude). Range [0..inf[.
    std::optional<double> max_acceleration_rate; ///< m/s^3; absent ⇒ infinite.
    std::optional<double> max_deceleration_rate; ///< m/s^3; absent ⇒ infinite.
};

/// One axle of a vehicle, per ASAM OpenSCENARIO XML 1.4.0 §Axle. Carried as
/// data for downstream consumers (e.g. steering models); the straight-line
/// runtime does not read it in this phase.
struct Axle {
    double max_steering = 0.0;   ///< rad, symmetrical. Range [0..PI].
    double position_x = 0.0;     ///< m from the reference point. Range [0..inf[.
    double position_z = 0.0;     ///< m from the reference point. Range [0..inf[.
    double track_width = 0.0;    ///< m between wheel centre lines. Range [0..inf[.
    double wheel_diameter = 0.0; ///< m. Range ]0..inf[.
};

/// The axle set of a vehicle, per ASAM OpenSCENARIO XML 1.4.0 §Axles: a
/// required rear axle, an optional front axle, and zero or more additional
/// axles (order preserved as authored).
struct Axles {
    Axle rear;                    ///< Required (1..1).
    std::optional<Axle> front;    ///< Optional (0..1).
    std::vector<Axle> additional; ///< Zero or more, in document order.
};

/// A user-defined name/value property, per ASAM OpenSCENARIO XML 1.4.0
/// §Property. The semantics are a contract between scenario author and host;
/// Scena preserves them verbatim and in document order (a vector, not a map,
/// so authored order and duplicate names survive round-trip).
struct Property {
    std::string name;
    std::string value;
};

} // namespace scena::ir
