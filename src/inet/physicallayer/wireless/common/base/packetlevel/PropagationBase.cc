//
// Copyright (C) 2013 OpenSim Ltd.
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

#include "inet/physicallayer/wireless/common/base/packetlevel/PropagationBase.h"

namespace inet {

namespace physicallayer {

PropagationBase::PropagationBase() :
    propagationSpeed(mps(NaN)),
    arrivalComputationCount(0)
{
}

void PropagationBase::initialize(int stage)
{
    if (stage == INITSTAGE_LOCAL)
        propagationSpeed = mps(par("propagationSpeed"));
}

std::ostream& PropagationBase::printToStream(std::ostream& stream, int level, int evFlags) const
{
    if (level <= PRINT_LEVEL_TRACE)
        stream << EV_FIELD(propagationSpeed);
    return stream;
}

void PropagationBase::finish()
{
    EV_INFO << "Radio signal arrival computation count = " << arrivalComputationCount << endl;
    recordScalar("Arrival computation count", arrivalComputationCount);
}

} // namespace physicallayer

} // namespace inet

