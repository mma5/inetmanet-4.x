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

#ifndef __INET_APSKDECODER_H
#define __INET_APSKDECODER_H

#include "inet/physicallayer/wireless/apsk/bitlevel/ApskCode.h"
#include "inet/physicallayer/wireless/common/contract/bitlevel/IDecoder.h"

namespace inet {

namespace physicallayer {

class INET_API ApskDecoder : public cSimpleModule, public IDecoder
{
  protected:
    const ApskCode *code;
    const IScrambler *descrambler;
    const IFecCoder *fecDecoder;
    const IInterleaver *deinterleaver;

  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;

  public:
    ApskDecoder();
    virtual ~ApskDecoder();

    virtual std::ostream& printToStream(std::ostream& stream, int level, int evFlags = 0) const override;
    virtual const ApskCode *getCode() const { return code; }
    virtual const IReceptionPacketModel *decode(const IReceptionBitModel *bitModel) const override;
};

} // namespace physicallayer

} // namespace inet

#endif

