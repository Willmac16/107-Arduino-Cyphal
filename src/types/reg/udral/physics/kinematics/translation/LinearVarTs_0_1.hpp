//
// This is an AUTO-GENERATED Cyphal DSDL data type implementation. Curious? See https://opencyphal.org.
// You shouldn't attempt to edit this file.
//
// Checking this file under version control is not recommended since metadata in this header will change for each
// build invocation. TODO: add --reproducible option to prevent any volatile metadata from being generated.
//
// Generator:     nunavut-2.3.1 (serialization was enabled)
// Source file:   /tmp/public_regulated_data_types/reg/udral/physics/kinematics/translation/LinearVarTs.0.1.dsdl
// Generated at:  2024-01-12 03:09:36.102154 UTC
// Is deprecated: no
// Fixed port-ID: None
// Full name:     reg.udral.physics.kinematics.translation.LinearVarTs
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
#ifndef REG_UDRAL_PHYSICS_KINEMATICS_TRANSLATION_LINEAR_VAR_TS_0_1_HPP_INCLUDED
#define REG_UDRAL_PHYSICS_KINEMATICS_TRANSLATION_LINEAR_VAR_TS_0_1_HPP_INCLUDED

#include <nunavut/support/serialization.hpp>
#include <types/reg/udral/physics/kinematics/translation/LinearTs_0_1.hpp>
#include <limits>

namespace reg
{
namespace udral
{
namespace physics
{
namespace kinematics
{
namespace translation
{
// +-------------------------------------------------------------------------------------------------------------------+
// | LANGUAGE OPTION ASSERTIONS
// |    These static assertions ensure that the header is being used with
// | Nunavut C++ serialization support that is compatible with the language
// | options in effect when that support code was generated.
// +-------------------------------------------------------------------------------------------------------------------+
static_assert( nunavut::support::options::target_endianness == 1693710260,
              "/tmp/public_regulated_data_types/reg/udral/physics/kinematics/translation/LinearVarTs.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::omit_float_serialization_support == 0,
              "/tmp/public_regulated_data_types/reg/udral/physics/kinematics/translation/LinearVarTs.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::enable_serialization_asserts == 0,
              "/tmp/public_regulated_data_types/reg/udral/physics/kinematics/translation/LinearVarTs.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::enable_override_variable_array_capacity == 0,
              "/tmp/public_regulated_data_types/reg/udral/physics/kinematics/translation/LinearVarTs.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::std == 628873475,
              "/tmp/public_regulated_data_types/reg/udral/physics/kinematics/translation/LinearVarTs.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::cast_format == 1407868567,
              "/tmp/public_regulated_data_types/reg/udral/physics/kinematics/translation/LinearVarTs.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::variable_array_type_include == 3320664631,
              "/tmp/public_regulated_data_types/reg/udral/physics/kinematics/translation/LinearVarTs.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::variable_array_type_template == 4227611599,
              "/tmp/public_regulated_data_types/reg/udral/physics/kinematics/translation/LinearVarTs.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::variable_array_type_constructor_args == 0,
              "/tmp/public_regulated_data_types/reg/udral/physics/kinematics/translation/LinearVarTs.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::allocator_include == 0,
              "/tmp/public_regulated_data_types/reg/udral/physics/kinematics/translation/LinearVarTs.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::allocator_type == 0,
              "/tmp/public_regulated_data_types/reg/udral/physics/kinematics/translation/LinearVarTs.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::allocator_is_default_constructible == 1,
              "/tmp/public_regulated_data_types/reg/udral/physics/kinematics/translation/LinearVarTs.0.1.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::ctor_convention == 3814588639,
              "/tmp/public_regulated_data_types/reg/udral/physics/kinematics/translation/LinearVarTs.0.1.dsdl "
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
/// This is a structural subtype of LinearTs.
/// Use best guess if the error variance is unknown.
///
struct LinearVarTs_0_1 final
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
        static constexpr std::size_t ExtentBytes                  = 25UL;
        static constexpr std::size_t SerializationBufferSizeBytes = 25UL;
        static_assert(ExtentBytes >= SerializationBufferSizeBytes, "Internal constraint violation");
        static_assert(ExtentBytes < (std::numeric_limits<std::size_t>::max() / 8U), "This message is too large to be handled by the selected types");

        struct TypeOf
        {
            TypeOf() = delete;
            using value = reg::udral::physics::kinematics::translation::LinearTs_0_1;
            using position_error_variance = float;
            using velocity_error_variance = float;
            using acceleration_error_variance = float;
        };
    };

    // +----------------------------------------------------------------------+
    // | FIELDS
    // +----------------------------------------------------------------------+

    _traits_::TypeOf::value value{};
    ///
    /// [meter^2]
    ///
    _traits_::TypeOf::position_error_variance position_error_variance{};
    ///
    /// [(meter/second)^2]
    ///
    _traits_::TypeOf::velocity_error_variance velocity_error_variance{};
    ///
    /// [(meter/second^2)^2]
    ///
    _traits_::TypeOf::acceleration_error_variance acceleration_error_variance{};
};

inline nunavut::support::SerializeResult serialize(const LinearVarTs_0_1& obj,
                                                   nunavut::support::bitspan out_buffer)
{
    const std::size_t capacity_bits = out_buffer.size();
    if ((static_cast<std::size_t>(capacity_bits)) < 200UL)
    {
        return -nunavut::support::Error::SerializationBufferTooSmall;
    }
    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    {   // reg.udral.physics.kinematics.translation.LinearTs.0.1 value
        std::size_t _size_bytes0_ = 19UL;  // Nested object (max) size, in bytes.
        auto _subspan0_ = out_buffer.subspan(0U, _size_bytes0_ * 8U);
        if(not _subspan0_){
            return -_subspan0_.error();
        }
        auto _err0_ = serialize(obj.value, _subspan0_.value());
        if (not _err0_)
        {
            return _err0_;
        }
        _size_bytes0_ = _err0_.value();
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        out_buffer.add_offset(_size_bytes0_ * 8U);
        //
    }
    {   // saturated float16 position_error_variance
        float _sat1_ = obj.position_error_variance;
        if (std::isfinite(_sat1_))
        {
            if (_sat1_ < static_cast<float>(-65504.0))
            {
                _sat1_ = static_cast<float>(-65504.0);
            }
            if (_sat1_ > static_cast<float>(65504.0))
            {
                _sat1_ = static_cast<float>(65504.0);
            }
        }
        auto _result4_ = out_buffer.setF16(_sat1_);
        if(not _result4_){
            return -_result4_.error();
        }
        out_buffer.add_offset(16U);
    }
    {   // saturated float16 velocity_error_variance
        float _sat1_ = obj.velocity_error_variance;
        if (std::isfinite(_sat1_))
        {
            if (_sat1_ < static_cast<float>(-65504.0))
            {
                _sat1_ = static_cast<float>(-65504.0);
            }
            if (_sat1_ > static_cast<float>(65504.0))
            {
                _sat1_ = static_cast<float>(65504.0);
            }
        }
        auto _result4_ = out_buffer.setF16(_sat1_);
        if(not _result4_){
            return -_result4_.error();
        }
        out_buffer.add_offset(16U);
    }
    {   // saturated float16 acceleration_error_variance
        float _sat1_ = obj.acceleration_error_variance;
        if (std::isfinite(_sat1_))
        {
            if (_sat1_ < static_cast<float>(-65504.0))
            {
                _sat1_ = static_cast<float>(-65504.0);
            }
            if (_sat1_ > static_cast<float>(65504.0))
            {
                _sat1_ = static_cast<float>(65504.0);
            }
        }
        auto _result4_ = out_buffer.setF16(_sat1_);
        if(not _result4_){
            return -_result4_.error();
        }
        out_buffer.add_offset(16U);
    }
    {
        const auto _result0_ = out_buffer.padAndMoveToAlignment(8U);
        if(not _result0_){
            return -_result0_.error();
        }
    }
    // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
    return out_buffer.offset_bytes_ceil();
}

inline nunavut::support::SerializeResult deserialize(LinearVarTs_0_1& obj,
                                                     nunavut::support::const_bitspan in_buffer)
{
    const auto capacity_bits = in_buffer.size();
    // reg.udral.physics.kinematics.translation.LinearTs.0.1 value
    {
        std::size_t _size_bytes1_ = in_buffer.size() / 8U;
        {
            const auto _err1_ = deserialize(obj.value, in_buffer.subspan());
            if(_err1_){
                _size_bytes1_ = _err1_.value();
            }else{
                return -_err1_.error();
            }
        }
        in_buffer.add_offset(_size_bytes1_ * 8U);  // Advance by the size of the nested serialized representation.
    }
    // saturated float16 position_error_variance
    obj.position_error_variance = in_buffer.getF16();
    in_buffer.add_offset(16U);
    // saturated float16 velocity_error_variance
    obj.velocity_error_variance = in_buffer.getF16();
    in_buffer.add_offset(16U);
    // saturated float16 acceleration_error_variance
    obj.acceleration_error_variance = in_buffer.getF16();
    in_buffer.add_offset(16U);
    in_buffer.align_offset_to<8U>();
    auto _bits_got_ = std::min<std::size_t>(in_buffer.offset(), capacity_bits);
    return { static_cast<std::size_t>(_bits_got_ / 8U) };
}

} // namespace translation
} // namespace kinematics
} // namespace physics
} // namespace udral
} // namespace reg

#endif // REG_UDRAL_PHYSICS_KINEMATICS_TRANSLATION_LINEAR_VAR_TS_0_1_HPP_INCLUDED
