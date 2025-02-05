//
// This is an AUTO-GENERATED Cyphal DSDL data type implementation. Curious? See https://opencyphal.org.
// You shouldn't attempt to edit this file.
//
// Checking this file under version control is not recommended since metadata in this header will change for each
// build invocation. TODO: add --reproducible option to prevent any volatile metadata from being generated.
//
// Generator:     nunavut-2.3.1 (serialization was enabled)
// Source file:   /tmp/public_regulated_data_types/reg/udral/service/actuator/esc/_.0.1.dsdl
// Generated at:  2024-01-12 03:09:35.826615 UTC
// Is deprecated: no
// Fixed port-ID: None
// Full name:     reg.udral.service.actuator.esc._
// Type Version:  0.1
// Support
//    Support Namespace: nunavut.lang.cpp.support
//    Support Version:   (1, 0, 0)
// Template Set (package)
//    priority: 0
//    package:  nunavut.lang.cpp.templates
//    version:  (1, 0, 0)
// Platform
//     python_implementation:  CPython
//     python_version:  3.10.12
//     python_release_level:  final
//     python_build:  ('main', 'Nov 20 2023 15:14:05')
//     python_compiler:  GCC 11.4.0
//     python_revision:
//     python_xoptions:  {}
//     runtime_platform:  Linux-5.15.0-91-generic-aarch64-with-glibc2.35
// Language Options
//     target_endianness:  any
//     omit_float_serialization_support:  False
//     enable_serialization_asserts:  False
//     enable_override_variable_array_capacity:  False
//     std:  c++17
//     cast_format:  static_cast<{type}>({value})
//     variable_array_type_include:  <vector>
//     variable_array_type_template:  std::vector<{TYPE}>
//     variable_array_type_constructor_args:
//     allocator_include:
//     allocator_type:
//     allocator_is_default_constructible:  True
//     ctor_convention:  default
// Uses Language Features
//     Uses std_variant:yes
#ifndef REG_UDRAL_SERVICE_ACTUATOR_ESC___0_1_HPP_INCLUDED
#define REG_UDRAL_SERVICE_ACTUATOR_ESC___0_1_HPP_INCLUDED

#include <nunavut/support/serialization.hpp>
#include <limits>

namespace reg
{
namespace udral
{
namespace service
{
namespace actuator
{
namespace esc
{
// +-------------------------------------------------------------------------------------------------------------------+
// | LANGUAGE OPTION ASSERTIONS
// |    These static assertions ensure that the header is being used with
// | Nunavut C++ serialization support that is compatible with the language
// | options in effect when that support code was generated.
// +-------------------------------------------------------------------------------------------------------------------+
static_assert( nunavut::support::options::target_endianness == 1693710260,
              "/tmp/public_regulated_data_types/reg/udral/service/actuator/esc/_.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::omit_float_serialization_support == 0,
              "/tmp/public_regulated_data_types/reg/udral/service/actuator/esc/_.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::enable_serialization_asserts == 0,
              "/tmp/public_regulated_data_types/reg/udral/service/actuator/esc/_.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::enable_override_variable_array_capacity == 0,
              "/tmp/public_regulated_data_types/reg/udral/service/actuator/esc/_.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::std == 628873475,
              "/tmp/public_regulated_data_types/reg/udral/service/actuator/esc/_.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::cast_format == 1407868567,
              "/tmp/public_regulated_data_types/reg/udral/service/actuator/esc/_.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::variable_array_type_include == 3320664631,
              "/tmp/public_regulated_data_types/reg/udral/service/actuator/esc/_.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::variable_array_type_template == 4227611599,
              "/tmp/public_regulated_data_types/reg/udral/service/actuator/esc/_.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::variable_array_type_constructor_args == 0,
              "/tmp/public_regulated_data_types/reg/udral/service/actuator/esc/_.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::allocator_include == 0,
              "/tmp/public_regulated_data_types/reg/udral/service/actuator/esc/_.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::allocator_type == 0,
              "/tmp/public_regulated_data_types/reg/udral/service/actuator/esc/_.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::allocator_is_default_constructible == 1,
              "/tmp/public_regulated_data_types/reg/udral/service/actuator/esc/_.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::ctor_convention == 3814588639,
              "/tmp/public_regulated_data_types/reg/udral/service/actuator/esc/_.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );

// +-------------------------------------------------------------------------------------------------------------------+
// | This implementation uses the C++17 standard library variant type with wrappers for the emplace and
// | get_if methods to support forward-compatibility with the C++14 version of this object. The union_value type
// | extends std::variant and can be used with the entire set of variant methods. Using std::variant directly does mean
// | your code will not be backwards compatible with the C++14 version of this object.
// +-------------------------------------------------------------------------------------------------------------------+
///
/// The electronic speed controller (ESC) service is designed for controlling and monitoring electric drives.
/// From the standpoint of this standard, an electric drive is just a special case of a servo. For generality,
/// COTS electric drives are recommended to also support the servo interface defined in the adjacent namespace.
///
/// ESCs (drives) are segregated into groups. Each ESC in a group has an index that is unique within the group.
/// Drives in a group are commanded synchronously by publishing a message containing an array of setpoints.
/// There are several subjects defined:
///
///   - Setpoint array subject. Every participant subscribes to the same setpoint subject.
///     Every message is consumed by all participants according to their index in the group.
///     The setpoint subject defines the group. There may be an arbitrary number of such groups in the network.
///
///   - Readiness subject. Every participant subscribes to the same readiness control subject which is used to command
///     the state of the group: sleep, standby, or engaged. In many cases there will be one global subject controlling
///     the state of the entire system; in other cases there will be dedicated controls on a per-subsystem basis.
///
///   - Feedback subjects published by each ESC separately, as shown on the diagram below.
///
///                                                   SUBJECT NAME        SUBJECT TYPE
///  +----------------+
///  |   Controller   |---------+------------+----... setpoint            reg.udral.service.actuator.common.sp.*
///  |                |-------+-)----------+-)----... readiness           reg.udral.service.common.Readiness
///  +----------------+       | |          | |
///   ^ ^ ^ ^  ^ ^ ^ ^        v v          v v
///   | | | |  | | | |   +---------+  +---------+
///   | | | |  | | | |   |Drive i=0|  |Drive i=1| ...
///   | | | |  | | | |   +---------+  +---------+
///   | | | |  | | | |     | | | |      | | | |
///   | | | |  | | | +-----+ | | |      | | | |       feedback            reg.udral.service.actuator.common.Feedback
///   | | | |  | | +---------+ | |      | | | |       status              reg.udral.service.actuator.common.Status
///   | | | |  | +-------------+ |      | | | |       power               reg.udral.physics.electricity.PowerTs
///   | | | |  +-----------------+      | | | |       dynamics            reg.udral.physics.dynamics.rotation.PlanarTs
///   | | | |                           | | | |
///   | | | +---------------------------+ | | |
///   | | +-------------------------------+ | |
///   | +-----------------------------------+ |
///   +---------------------------------------+
///
/// Notice that the physics subjects are timestamped.
///
/// Vendor/application-specific subjects are not shown here.
/// Vendors are encouraged to publish additional data (e.g., temperatures) on separate subjects.
///
///
///   SETPOINT SUBJECT
///
/// The setpoint subject is ignored unless the drive is ENGAGED. As long as the drive is not ENGAGED, it shall not apply
/// any power to the load excepting non-operational scenarios such as maintenance and diagnostics, which are
/// outside of the scope of this service definition. More on readiness and safety in the next section.
///
/// Upon reception of a setpoint message, a group participant fetches its setpoint from the array using the array
/// element whose index equals the index of the group participant. By virtue of the Implicit Zero Extension Rule,
/// if the message is too short, the setpoint will be interpreted as zero.
///
/// If no valid setpoint was received in CONTROL_TIMEOUT or a lower implementation-specific value,
/// the drive should assume a zero setpoint for safety reasons.
/// The minimum setpoint publication period should be at least twice lower than its timeout.
///
/// While stopped, the drive may either allow the load to freewheel or it may force it to a particular parking position,
/// depending on the implementation requirements. The actual state of the load may be continuously reported using the
/// dynamics subject. Notice that per the safety rule introduced earlier, the parking position may be impossile
/// to enforce unless the drive is ENGAGED because it may require delivering power to the load.
///
/// The setpoint message types that can be used to command a group of drives are defined in
/// reg.udral.service.actuator.common.sp; please read the documentation related to that namespace for further
/// information.
/// Servo setpoint message types may also be supported on an implementation-specific basis for enhanced
/// interoperability.
/// If the group is controlled using different setpoint subjects concurrently, the behavior is implementation-defined.
///
/// The following control modes are defined, none of which are mandatory to support.
/// The control mode in use is to be specified using the register API.
/// This service does not support switching the control mode or setting the motion profile at runtime;
/// for that, please refer to the servo service.
///
///   0. Ratiometric voltage control. Each setpoint scalar is a value normalized/saturated in [-1, +1] representing
///      the Q-axis/phase/armature (depending on the type of the drive) voltage as a fraction of the maximum.
///      This control mode emulates the behavior of a typical RCPWM-controlled BLDC drive.
///
///   1. Ratiometric current/torque control. Each setpoint scalar is a value normalized/saturated in [-1, +1]
/// representing
///      the Q-axis/phase/armature (depending on the type of the drive) current as a fraction of the maximum.
///      A negative setpoint during forward rotation (positive during reverse rotation) commands braking.
///
///   2. Speed control. Each setpoint scalar contains the target angular velocity of the load in radian/second.
///
///   -. More control modes may be added later. Which control modes are supported is implementation-defined.
///
/// Considerations that apply to all control modes:
///  -  Negative setpoint values represent reversal; a positive setpoint is co-directed with positive rotation/torque.
///  -  If reverse operation is not supported, negative values should be clamped to zero.
///  -  A non-finite setpoint is to be treated as zero.
///
///
///   READINESS SUBJECT
///
/// The default state is STANDBY. While in this state, the drive is not allowed to deliver power to the load,
/// and the setpoint subject is ignored. The drive shall enter this state automatically if the readiness subject
/// is not updated for CONTROL_TIMEOUT.
///
/// While the drive is ENGAGED, the setpoint commands are processed normally as described in the adjacent section.
/// If the drive does not support bidirectional operation, implementations are recommended to ensure that the load
/// is driven at some minimum power level (idling) while the drive is ENGAGED regardless of the commanded setpoint,
/// unless such behavior is deemed incompatible with the functional requirements of the controlled drive.
///
/// If the selected readiness state is SLEEP, the behavior is implementation-defined. Implementations are recommended to
/// power off the high-voltage circuitry and all non-essential components (e.g., LED indication, sensors, etc.)
/// to minimize the power consumption.
///
/// Implementations are recommended to announce transitions between the readiness states using audiovisual feedback.
///
/// The worst-case state transition latency is not defined. The controlling element (that is, the unit that publishes
/// to the setpoint and readiness subjects) is expected to monitor the actual readiness status of each component using
/// the feedback subject. For example, a sensorless electric motor drive may choose to spool-up before entering the
/// ENGAGED state, which would obviously take time; as soon as the spool-up is finished, the drive would switch its
/// reported status from STANDBY to ENGAGED, thereby indicating that it is ready for normal operation.
///
///
///   PUBLISHED SUBJECTS
///
/// The following subjects shall be published immediately after a new setpoint is applied even if the drive is STANDBY:
///
///   SUBJECT             RECOMMENDED PRIORITY
///   ---------------------------------------------
///   feedback            same as the setpoint
///   power               second to the setpoint
///   dynamics            second to the setpoint
///
/// If no setpoint is being published, these subjects should continue being updated at least at
/// 1/MAX_PUBLICATION_PERIOD.
/// The publication rate requirements do not apply if the readiness state is SLEEP.
///
/// If the setpoint publication rate exceeds 50 Hz, implementations are allowed (but not required) to throttle these
/// subjects by dropping some of the messages such that the publication rate of each subject does not exceed 50 Hz.
/// Implementations operating over Classic CAN are recommended to do this.
///
/// The other subjects may be published at an implementation-defined rate and priority,
/// which should be consistent across the group.
///
/// Implementations are encouraged to provide additional subjects for enhanced feedback and monitoring.
///
/// The measurements carried by the published messages should be low-pass filtered with an adequate cutoff frequency to
/// avoid aliasing effects. Implementations should strive to sample all parameters simultaneously.
///
/// If a float-typed reported quantity is unknown, the corresponding value should be NaN.
///
///
///   CONVENTIONS AND ASSUMPTIONS
///
/// A drive powers a rotary mechanical load that may be connected via a gearbox. It is the responsibility of
/// the drive to account for the gear ratio of the gearbox when calculating related parameters such as angular
/// velocity or torque.
///
/// It is assumed that there is a well-defined direction of rotation that is referred to as forward rotation.
/// A positive angular velocity represents forward rotation. Likewise, forward torque is positive.
///
/// It is assumed that the drive is powered from a DC electric power supply network. A positive electric current
/// represents current flowing from the network into the drive, also referred to as the state of driving/motoring.
/// The opposite -- braking/regeneration -- is represented by negative current.
///
/// Excepting edge cases and transients, torque and current are generally of the same sign.
/// The above is summarized on the following four-quadrant diagram:
///
///                                   +velocity
///                                       ^
///                               braking,| forward,
///                               negative| positive
///                               power   | power
///                            -----------+---------->  +torque/current
///                               reverse,| braking,
///                               positive| negative
///                               power   | power
///
struct zX005FzX005F0_1 final
{
    struct _traits_  // The name is surrounded with underscores to avoid collisions with DSDL attributes.
    {
        _traits_() = delete;
        /// This type does not have a fixed port-ID. See https://forum.opencyphal.org/t/choosing-message-and-service-ids/889
        static constexpr bool HasFixedPortID = false;

        static constexpr bool IsServiceType = false;

        /// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
        /// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
        /// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
        /// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
        /// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
        /// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
        /// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
        static constexpr std::size_t ExtentBytes                  = 0UL;
        static constexpr std::size_t SerializationBufferSizeBytes = 0UL;
        static_assert(ExtentBytes >= SerializationBufferSizeBytes, "Internal constraint violation");
        static_assert(ExtentBytes < (std::numeric_limits<std::size_t>::max() / 8U), "This message is too large to be handled by the selected types");

    };

};

inline nunavut::support::SerializeResult serialize(const zX005FzX005F0_1& obj,
                                                   nunavut::support::bitspan out_buffer)
{
    (void)(out_buffer);
    (void)(obj);
    return 0U;
}

inline nunavut::support::SerializeResult deserialize(zX005FzX005F0_1& obj,
                                                     nunavut::support::const_bitspan in_buffer)
{
    (void)(in_buffer);
    (void)(obj);
    return 0;
}

} // namespace esc
} // namespace actuator
} // namespace service
} // namespace udral
} // namespace reg

#endif // REG_UDRAL_SERVICE_ACTUATOR_ESC___0_1_HPP_INCLUDED
