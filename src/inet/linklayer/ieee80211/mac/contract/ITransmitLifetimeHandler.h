//
// Copyright (C) 2016 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//

#ifndef __INET_ITRANSMITLIFETIMEHANDLER_H
#define __INET_ITRANSMITLIFETIMEHANDLER_H

#include "inet/linklayer/ieee80211/mac/Ieee80211Frame_m.h"

namespace inet {
namespace ieee80211 {

class INET_API ITransmitLifetimeHandler
{
  public:
    virtual ~ITransmitLifetimeHandler() {}

    virtual void frameGotInProgess(const Ptr<const Ieee80211DataHeader>& header) = 0;
    virtual void frameTransmitted(const Ptr<const Ieee80211DataHeader>& header) = 0;

    virtual bool isLifetimeExpired(const Ptr<const Ieee80211DataHeader>& header) = 0;
};

} // namespace ieee80211
} // namespace inet

#endif

