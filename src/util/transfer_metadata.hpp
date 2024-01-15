/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020-2023 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-Cyphal/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace cyphal
{

  /**************************************************************************************
   * TYPEDEF
   **************************************************************************************/

  struct TransferMetadata final
  {
    uint8_t priority;
    uint8_t transfer_kind;
    uint16_t port_id;
    uint8_t remote_node_id;
    uint8_t transfer_id;

    uint64_t timestamp_usec;
  };

  /**************************************************************************************
   * NAMESPACE
   **************************************************************************************/

} /* cyphal */
