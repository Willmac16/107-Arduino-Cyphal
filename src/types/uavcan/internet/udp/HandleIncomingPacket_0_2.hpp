//
// This is an AUTO-GENERATED Cyphal DSDL data type implementation. Curious? See https://opencyphal.org.
// You shouldn't attempt to edit this file.
//
// Checking this file under version control is not recommended since metadata in this header will change for each
// build invocation. TODO: add --reproducible option to prevent any volatile metadata from being generated.
//
// Generator:     nunavut-2.3.1 (serialization was enabled)
// Source file:   /tmp/public_regulated_data_types/uavcan/internet/udp/500.HandleIncomingPacket.0.2.dsdl
// Generated at:  2024-01-12 03:09:34.544920 UTC
// Is deprecated: no
// Fixed port-ID: 500
// Full name:     uavcan.internet.udp.HandleIncomingPacket
// Type Version:  0.2
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
#ifndef UAVCAN_INTERNET_UDP_HANDLE_INCOMING_PACKET_0_2_HPP_INCLUDED
#define UAVCAN_INTERNET_UDP_HANDLE_INCOMING_PACKET_0_2_HPP_INCLUDED

#include <nunavut/support/serialization.hpp>
#include <cstdint>
#include <limits>
#include <vector>

namespace uavcan
{
namespace internet
{
namespace udp
{
// +-------------------------------------------------------------------------------------------------------------------+
// | LANGUAGE OPTION ASSERTIONS
// |    These static assertions ensure that the header is being used with
// | Nunavut C++ serialization support that is compatible with the language
// | options in effect when that support code was generated.
// +-------------------------------------------------------------------------------------------------------------------+
static_assert( nunavut::support::options::target_endianness == 1693710260,
              "/tmp/public_regulated_data_types/uavcan/internet/udp/500.HandleIncomingPacket.0.2.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::omit_float_serialization_support == 0,
              "/tmp/public_regulated_data_types/uavcan/internet/udp/500.HandleIncomingPacket.0.2.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::enable_serialization_asserts == 0,
              "/tmp/public_regulated_data_types/uavcan/internet/udp/500.HandleIncomingPacket.0.2.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::enable_override_variable_array_capacity == 0,
              "/tmp/public_regulated_data_types/uavcan/internet/udp/500.HandleIncomingPacket.0.2.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::std == 628873475,
              "/tmp/public_regulated_data_types/uavcan/internet/udp/500.HandleIncomingPacket.0.2.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::cast_format == 1407868567,
              "/tmp/public_regulated_data_types/uavcan/internet/udp/500.HandleIncomingPacket.0.2.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::variable_array_type_include == 3320664631,
              "/tmp/public_regulated_data_types/uavcan/internet/udp/500.HandleIncomingPacket.0.2.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::variable_array_type_template == 4227611599,
              "/tmp/public_regulated_data_types/uavcan/internet/udp/500.HandleIncomingPacket.0.2.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::variable_array_type_constructor_args == 0,
              "/tmp/public_regulated_data_types/uavcan/internet/udp/500.HandleIncomingPacket.0.2.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::allocator_include == 0,
              "/tmp/public_regulated_data_types/uavcan/internet/udp/500.HandleIncomingPacket.0.2.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::allocator_type == 0,
              "/tmp/public_regulated_data_types/uavcan/internet/udp/500.HandleIncomingPacket.0.2.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::allocator_is_default_constructible == 1,
              "/tmp/public_regulated_data_types/uavcan/internet/udp/500.HandleIncomingPacket.0.2.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );
static_assert( nunavut::support::options::ctor_convention == 3814588639,
              "/tmp/public_regulated_data_types/uavcan/internet/udp/500.HandleIncomingPacket.0.2.dsdl "
              "is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not "
              "allowed." );

namespace HandleIncomingPacket
{

// +-------------------------------------------------------------------------------------------------------------------+
// | This implementation uses the C++17 standard library variant type with wrappers for the emplace and
// | get_if methods to support forward-compatibility with the C++14 version of this object. The union_value type
// | extends std::variant and can be used with the entire set of variant methods. Using std::variant directly does mean
// | your code will not be backwards compatible with the C++14 version of this object.
// +-------------------------------------------------------------------------------------------------------------------+
///
/// This message carries UDP packets sent from a remote host on the Internet or a LAN to a node on the local Cyphal bus.
/// Please refer to the definition of the message type OutgoingPacket for a general overview of the packet forwarding
/// logic.
///
/// This data type has been made a service type rather than a message type in order to make its transfers addressable,
/// allowing nodes to employ hardware acceptance filters for filtering out forwarded datagrams that are not addressed
/// to them. Additionally, requiring the destination nodes to always respond upon reception of the forwarded datagram
/// opens interesting opportunities for future extensions of the forwarding protocol. If the service invocation times
/// out, the modem node is permitted to remove the corresponding entry from the NAT table immediately, not waiting
/// for its TTL to expire.
///
/// It should be noted that this data type definition intentionally leaves out the source address. This is done in
/// order to simplify the implementation, reduce the bus traffic overhead, and because the nature of the
/// communication patterns proposed by this set of messages does not provide a valid way to implement server hosts
/// on the local Cyphal bus. It is assumed that local nodes can be only clients, and therefore, they will be able to
/// determine the address of the sender simply by mapping the field session_id to their internally maintained states.
/// Furthermore, it is uncertain what is the optimal way of representing the source address for
/// client nodes: it is assumed that the local nodes will mostly use DNS names rather than IP addresses, so if there
/// was a source address field, modem nodes would have to perform reverse mapping from the IP address they received
/// the datagram from to the corresponding DNS name that was used by the local node with the outgoing message. This
/// approach creates a number of troubling corner cases and adds a fair amount of hidden complexities to the
/// implementation of modem nodes.
///
/// It is recommended to perform service invocations at the same transfer priority level as was used for broadcasting
/// the latest matching message of type OutgoingPacket. However, meeting this recommendation would require the modem
/// node to implement additional logic, which may be undesirable. Therefore, implementers are free to deviate from
/// this recommendation and resort to a fixed priority level instead. In the case of a fixed priority level, it is
/// advised to use the lowest transfer priority level.
///
struct Request_0_2 final
{
    struct _traits_  // The name is surrounded with underscores to avoid collisions with DSDL attributes.
    {
        _traits_() = delete;
        static constexpr bool HasFixedPortID = true;
        static constexpr std::uint16_t FixedPortId = 500U;
        static constexpr bool IsServiceType  = true;
        static constexpr bool IsService      = false;
        static constexpr bool IsRequest      = true;
        static constexpr bool IsResponse     = false;
        /// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
        /// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
        /// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
        /// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
        /// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
        /// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
        /// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
        static constexpr std::size_t ExtentBytes                  = 600UL;
        static constexpr std::size_t SerializationBufferSizeBytes = 512UL;
        static_assert(ExtentBytes >= SerializationBufferSizeBytes, "Internal constraint violation");
        static_assert(ExtentBytes < (std::numeric_limits<std::size_t>::max() / 8U), "This message is too large to be handled by the selected types");

        struct TypeOf
        {
            TypeOf() = delete;
            using session_id = std::uint16_t;
            using payload = std::vector<std::uint8_t>;
        };
    };

    // +----------------------------------------------------------------------+
    // | FIELDS
    // +----------------------------------------------------------------------+
    ///
    /// This field shall contain the same value that was used by the local node when sending the corresponding outgoing
    /// packet using the message type OutgoingPacket. This value will be used by the local node to match the response
    /// with its local context.
    ///
    _traits_::TypeOf::session_id session_id{};
    ///
    /// Effective payload. This data will be forwarded from the remote host verbatim.
    /// UDP packets that contain more than 508 bytes of payload may be dropped by some types of
    /// communication equipment. Refer to RFC 791 and 2460 for an in-depth review.
    /// Datagrams that exceed the capacity of this field should be discarded by the modem node.
    ///
    _traits_::TypeOf::payload payload{};
};

inline nunavut::support::SerializeResult serialize(const Request_0_2& obj,
                                                   nunavut::support::bitspan out_buffer)
{
    const std::size_t capacity_bits = out_buffer.size();
    if ((static_cast<std::size_t>(capacity_bits)) < 4096UL)
    {
        return -nunavut::support::Error::SerializationBufferTooSmall;
    }
    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    {   // saturated uint16 session_id
        // Saturation code not emitted -- native representation matches the serialized representation.
        const auto _result3_ = out_buffer.setUxx(obj.session_id, 16U);
        if(not _result3_){
            return -_result3_.error();
        }
        out_buffer.add_offset(16U);
    }
    {   // saturated uint8[<=508] payload
        if (obj.payload.size() > 508)
        {
            return -nunavut::support::Error::SerializationBadArrayLength;
        }
        // Array length prefix: truncated uint16
        const auto _result3_ = out_buffer.setUxx(obj.payload.size(), 16U);
        if(not _result3_){
            return -_result3_.error();
        }
        out_buffer.add_offset(16U);
        for (std::size_t _index2_ = 0U; _index2_ < obj.payload.size(); ++_index2_)
        {
            // Saturation code not emitted -- native representation matches the serialized representation.
            const auto _result3_ = out_buffer.setUxx(obj.payload[_index2_], 8U);
            if(not _result3_){
                return -_result3_.error();
            }
            out_buffer.add_offset(8U);
        }
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

inline nunavut::support::SerializeResult deserialize(Request_0_2& obj,
                                                     nunavut::support::const_bitspan in_buffer)
{
    const auto capacity_bits = in_buffer.size();
    // saturated uint16 session_id
    obj.session_id = in_buffer.getU16(16U);
    in_buffer.add_offset(16U);
    // saturated uint8[<=508] payload
    {
        // Array length prefix: truncated uint16
    const std::size_t _size0_ = in_buffer.getU16(16U);
    in_buffer.add_offset(16U);
        if ( _size0_ > 508U)
        {
            return -nunavut::support::Error::SerializationBadArrayLength;
        }
        obj.payload.reserve(_size0_);
        for (std::size_t _index5_ = 0U; _index5_ < _size0_; ++_index5_)
        {
            std::uint8_t _tmp0_ = std::uint8_t();
            _tmp0_ = in_buffer.getU8(8U);
        in_buffer.add_offset(8U);
            obj.payload.push_back(std::move(_tmp0_));
        }
    }
    in_buffer.align_offset_to<8U>();
    auto _bits_got_ = std::min<std::size_t>(in_buffer.offset(), capacity_bits);
    return { static_cast<std::size_t>(_bits_got_ / 8U) };
}

// +-------------------------------------------------------------------------------------------------------------------+
// | This implementation uses the C++17 standard library variant type with wrappers for the emplace and
// | get_if methods to support forward-compatibility with the C++14 version of this object. The union_value type
// | extends std::variant and can be used with the entire set of variant methods. Using std::variant directly does mean
// | your code will not be backwards compatible with the C++14 version of this object.
// +-------------------------------------------------------------------------------------------------------------------+

struct Response_0_2 final
{
    struct _traits_  // The name is surrounded with underscores to avoid collisions with DSDL attributes.
    {
        _traits_() = delete;
        static constexpr bool HasFixedPortID = true;
        static constexpr std::uint16_t FixedPortId = 500U;
        static constexpr bool IsServiceType  = true;
        static constexpr bool IsService      = false;
        static constexpr bool IsRequest      = false;
        static constexpr bool IsResponse     = true;
        /// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
        /// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
        /// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
        /// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
        /// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
        /// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
        /// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
        static constexpr std::size_t ExtentBytes                  = 63UL;
        static constexpr std::size_t SerializationBufferSizeBytes = 0UL;
        static_assert(ExtentBytes >= SerializationBufferSizeBytes, "Internal constraint violation");
        static_assert(ExtentBytes < (std::numeric_limits<std::size_t>::max() / 8U), "This message is too large to be handled by the selected types");

    };

};

inline nunavut::support::SerializeResult serialize(const Response_0_2& obj,
                                                   nunavut::support::bitspan out_buffer)
{
    (void)(out_buffer);
    (void)(obj);
    return 0U;
}

inline nunavut::support::SerializeResult deserialize(Response_0_2& obj,
                                                     nunavut::support::const_bitspan in_buffer)
{
    (void)(in_buffer);
    (void)(obj);
    return 0;
}

}  // namespace HandleIncomingPacket

struct HandleIncomingPacket_0_2
{
    HandleIncomingPacket_0_2() = delete;

    using Request  = HandleIncomingPacket::Request_0_2;
    using Response = HandleIncomingPacket::Response_0_2;

    struct _traits_
    {
        _traits_() = delete;

        static constexpr bool IsServiceType = true;
        static constexpr bool IsService = true;
        static constexpr bool IsRequest = false;
        static constexpr bool IsResponse = false;
    };
};

} // namespace udp
} // namespace internet
} // namespace uavcan

#endif // UAVCAN_INTERNET_UDP_HANDLE_INCOMING_PACKET_0_2_HPP_INCLUDED
