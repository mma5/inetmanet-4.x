//
// Copyright (C) 2018 OpenSim Ltd.
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

#include "inet/linklayer/ieee8021d/stp/StpProtocolDissector.h"

#include "inet/common/packet/dissector/ProtocolDissectorRegistry.h"
#include "inet/linklayer/ieee8021d/common/Ieee8021dBpdu_m.h"

namespace inet {

Register_Protocol_Dissector(&Protocol::stp, StpProtocolDissector);

void StpProtocolDissector::dissect(Packet *packet, const Protocol *protocol, ICallback& callback) const
{
    auto stpPacket = packet->popAtFront<BpduBase>();
    callback.startProtocolDataUnit(&Protocol::stp);
    callback.visitChunk(stpPacket, &Protocol::stp);
    callback.endProtocolDataUnit(&Protocol::stp);
}

} // namespace inet

