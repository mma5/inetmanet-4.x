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

#ifndef __INET_IINTERLEAVER_H
#define __INET_IINTERLEAVER_H

#include "inet/common/BitVector.h"
#include "inet/common/IPrintableObject.h"

namespace inet {
namespace physicallayer {

class INET_API IInterleaving : public IPrintableObject
{
};

class INET_API IInterleaver : public IPrintableObject
{
  public:
    virtual BitVector interleave(const BitVector& bits) const = 0;
    virtual BitVector deinterleave(const BitVector& bits) const = 0;
    virtual const IInterleaving *getInterleaving() const = 0;
};

} // namespace physicallayer
} // namespace inet

#endif

