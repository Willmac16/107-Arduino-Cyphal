/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-Cyphal/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <array>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace impl
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

template<typename T>
bool Publisher::publish(T const & msg)
{
  CanardTransferMetadata const transfer_metadata =
  {
    .priority       = CanardPriorityNominal,
    .transfer_kind  = CanardTransferKindMessage,
    .port_id        = _port_id,
    .remote_node_id = CANARD_NODE_ID_UNSET,
    .transfer_id    = _transfer_id++,
  };

  /* Serialize message into payload buffer. */
  std::array<uint8_t, T::MAX_PAYLOAD_SIZE> payload_buf{};
  size_t const payload_buf_size = msg.serialize(payload_buf.data());

  /* Serialize transfer into a series of CAN frames */
  int32_t result = canardTxPush(&_canard_tx_queue,
                                &_canard_hdl,
                                CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                                &transfer_metadata,
                                payload_buf_size,
                                payload_buf.data());
  bool const success = (result >= 0);
  return success;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* impl */
