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

#include "inet/common/geometry/object/Box.h"

namespace inet {

const Box Box::NIL = Box(Coord::NIL, Coord::NIL);

Box::Box(const Coord& min, const Coord& max) :
    min(min),
    max(max)
{
}

Box inet::Box::computeBoundingBox(const std::vector<Coord>& points)
{
    Coord min = Coord::NIL;
    Coord max = Coord::NIL;
    if (points.begin() != points.end()) {
        min = *points.begin();
        max = min;
    }
    for (const auto& point : points) {
        min = min.min(point);
        max = max.max(point);
    }
    return Box(min, max);
}

} /* namespace inet */

