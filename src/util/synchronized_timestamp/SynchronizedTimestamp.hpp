/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020-2023 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-Cyphal/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "../../Node.hpp"
#include "../../DSDL_Types.h"

#include "registry_impl.hpp"

// #if !defined(__GNUC__) || (__GNUC__ >= 11)

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace cyphal
{

  namespace impl
  {

    enum SyncTimestampState
    {
      STATE_UPDATE,
      STATE_ADJUST
    }

    /**************************************************************************************
     * CLASS DECLARATION
     **************************************************************************************/

    class SynchronizedTimestamp final
    {
      static constexpr std::uint64_t ONE_HERTZ_MICROS = 1'000'000U;

      static constexpr std::uint64_t PUBLISHER_TIMEOUT = uavcan::time::Synchronization_1_0::PUBLISHER_TIMEOUT_PERIOD_MULTIPLIER * uavcan::time::Synchronization_1_0::MAX_PUBLICATION_PERIOD * ONE_HERTZ_MICROS;

    public:
      typedef std::function<uint64_t(void)>
          MicrosFunc;

      SynchronizedTimestamp(Node &node_hdl, MicrosFunc const micros)
          : _micros{micros}
      {
        _sync_timestamp_sub = node_hdl.create_subscription<uavcan::time::Synchronization_1_0>(onSynchronization_1_0_Received);
      }

      int64_t clock_offset = 0;

    private:
      MicrosFunc const _micros;

      cyphal::Subscription _sync_timestamp_sub;

      CanardMicrosecond _previous_rx_real_timestamp = 0;      // This clock is being synchronized
      CanardMicrosecond _previous_rx_monotonic_timestamp = 0; // Monotonic time -- doesn't leap or change rate
      CanardTransferID _previous_transfer_id = 0;
      SyncTimestampState state = STATE_UPDATE; // Variants: STATE_UPDATE, STATE_ADJUST
      CanardNodeID _master_node_id = -1;

      SyncTimestampState _state;

      void adjust(uavcan::time::Synchronization_1_0 msg)
      {
        // Clock adjustment will be performed every second message
        const int64_t local_time_phase_error = _previous_rx_real_timestamp - msg.previous_transmission_timestamp_microsecond;
        clock_offset -= local_time_phase_error;
        _state = STATE_UPDATE;
      }

      void update(uavcan::time::Synchronization_1_0 msg, cyphal::TransferMetadata const &metadata)
      {
        // A message is assumed to have two timestamps:
        //   Real      - sampled from the clock that is being synchronized
        //   Monotonic - clock that never leaps and never changes rate
        _previous_rx_monotonic_timestamp = metadata.timestamp_usec;
        _previous_rx_real_timestamp = _previous_rx_monotonic_timestamp + clock_offset;
        _master_node_id = metadata.remote_node_id;
        _previous_transfer_id = metadata.transfer_id;
        _state = STATE_ADJUST;
      }

      // Accepts the message of type uavcan.time.Synchronization
      void onSynchronization_1_0_Received(uavcan::time::Synchronization_1_0 const &msg, cyphal::TransferMetadata const &metadata)
      {
        const uint64_t time_since_previous_msg = metadata.timestamp_usec - _previous_rx_monotonic_timestamp;
        const bool needs_init = (_master_node_id < 0);
        const bool switch_master = metadata.remote_node_id < _master_node_id;
        // The value publisher_timeout is computed as described in the specification (3x interval)
        const bool publisher_timed_out = time_since_previous_msg > PUBLISHER_TIMEOUT;
        if (needs_init or switch_master or publisher_timed_out)
        {
          update(msg, metadata);
        }
        else if ((metadata.remote_node_id == _master_node_id))
        {
          // Revert the state to STATE_UPDATE if needed
          if (_state == STATE_ADJUST)
          {
            const bool msg_invalid = msg.previous_transmission_timestamp_microsecond == 0;
            // Overflow shall be handled correctly
            const bool wrong_tid = metadata.transfer_id != (_previous_transfer_id + 1);
            const bool wrong_timing = time_since_previous_msg > uavcan::time::Synchronization_1_0::MAX_PUBLICATION_PERIOD;
            if (msg_invalid || wrong_tid || wrong_timing)
            {
              _state = STATE_UPDATE;
            }
          }
          // Handle the current state
          if (_state == STATE_ADJUST)
          {
            adjust(msg, metadata);
          }
          else
          {
            update(msg, metadata);
          }
        } // else ignore
      }
    };

    /**************************************************************************************
     * NAMESPACE
     **************************************************************************************/

  } /* impl */
} /* cyphal */

// #endif /* !defined(__GNUC__) || (__GNUC__ >= 11) */
