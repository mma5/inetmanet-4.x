//
// Copyright (C) 2014 OpenSim Ltd.
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

#include "inet/physicallayer/wireless/ieee80211/mode/Ieee80211DsssOfdmMode.h"

namespace inet {

namespace physicallayer {

Ieee80211DsssOfdmMode::Ieee80211DsssOfdmMode(const char *name, const Ieee80211DsssPreambleMode *dsssPreambleMode, const Ieee80211DsssHeaderMode *dsssHeaderMode, const Ieee80211OfdmPreambleMode *ofdmPreambleMode, const Ieee80211OfdmSignalMode *ofdmSignalMode, const Ieee80211OfdmDataMode *ofdmDataMode) :
    Ieee80211ModeBase(name),
    dsssPreambleMode(dsssPreambleMode),
    dsssHeaderMode(dsssHeaderMode),
    ofdmPreambleMode(ofdmPreambleMode),
    ofdmSignalMode(ofdmSignalMode),
    ofdmDataMode(ofdmDataMode)
{
}

const simtime_t Ieee80211DsssOfdmMode::getRifsTime() const
{
    return -1;
}

} // namespace physicallayer

} // namespace inet

