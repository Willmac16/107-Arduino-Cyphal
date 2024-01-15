/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020-2023 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-Cyphal/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <memory>

#include "libcanard/canard.h"
#include "util/transfer_metadata.hpp"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace cyphal
{

namespace impl
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class SubscriptionBase
{
public:
  SubscriptionBase(CanardTransferKind const transfer_kind)
  : _transfer_kind{transfer_kind}
  {
    _canard_rx_sub.user_reference = static_cast<void *>(this);
  }
  virtual ~SubscriptionBase() { }
  SubscriptionBase(SubscriptionBase const &) = delete;
  SubscriptionBase(SubscriptionBase &&) = delete;
  SubscriptionBase &operator=(SubscriptionBase const &) = delete;
  SubscriptionBase &operator=(SubscriptionBase &&) = delete;


  virtual bool onTransferReceived(CanardRxTransfer const & transfer) = 0;


  [[nodiscard]] CanardRxSubscription &canard_rx_subscription() { return _canard_rx_sub; }



protected:
  [[nodiscard]] CanardTransferKind canard_transfer_kind() const { return _transfer_kind; }

  [[nodiscard]] TransferMetadata fillMetadata(CanardRxTransfer const & transfer)
  {
    TransferMetadata transfer_metadata;
    transfer_metadata.priority = static_cast<uint8_t>(transfer.metadata.priority);
    transfer_metadata.transfer_kind = static_cast<uint8_t>(transfer.metadata.transfer_kind);
    transfer_metadata.port_id = static_cast<uint16_t>(transfer.metadata.port_id);
    transfer_metadata.remote_node_id = static_cast<uint8_t>(transfer.metadata.remote_node_id);
    transfer_metadata.transfer_id = static_cast<uint8_t>(transfer.metadata.transfer_id);

    transfer_metadata.timestamp_usec = static_cast<uint64_t>(transfer.timestamp_usec);

    return transfer_metadata;
  }

private:
  CanardTransferKind const _transfer_kind;
  CanardRxSubscription _canard_rx_sub;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* impl */

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

using Subscription = std::shared_ptr<impl::SubscriptionBase>;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* cyphal */
