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

#include "inet/linklayer/ieee80211/mac/duplicateremoval/LegacyDuplicateRemoval.h"

#include "inet/common/stlutils.h"

namespace inet {
namespace ieee80211 {

bool LegacyDuplicateRemoval::isDuplicate(const Ptr<const Ieee80211DataOrMgmtHeader>& header)
{
    ASSERT(header->getType() != ST_DATA_WITH_QOS);
    const MacAddress& address = header->getTransmitterAddress();
    SequenceControlField seqVal(header->getSequenceNumber().get(), header->getFragmentNumber());
    auto it = lastSeenSeqNumCache.find(address);
    if (it == lastSeenSeqNumCache.end())
        lastSeenSeqNumCache.insert(std::pair<MacAddress, SequenceControlField>(address, seqVal));
    else if (it->second.getSequenceNumber() == seqVal.getSequenceNumber() && it->second.getFragmentNumber() == seqVal.getFragmentNumber() && header->getRetry())
        return true;
    else
        it->second = seqVal;
    return false;
}

} // namespace ieee80211
} // namespace inet

