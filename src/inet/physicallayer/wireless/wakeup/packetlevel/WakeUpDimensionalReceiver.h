//
// Copyright (C) 2014 Florian Meier
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

#ifndef __INET_WAKEUPDIMENSIONALRECEIVER_H
#define __INET_WAKEUPDIMENSIONALRECEIVER_H

#include "inet/physicallayer/wireless/wakeup/packetlevel/WakeUpReceiverBase.h"

namespace inet {

namespace physicallayer {

class INET_API WakeUpDimensionalReceiver : public WakeUpReceiverBase
{
  protected:
    W minInterferencePower;

  public:
    WakeUpDimensionalReceiver();

    void initialize(int stage) override;

    bool computeIsReceptionPossible(const IListening *listening, const IReception *reception, IRadioSignal::SignalPart part) const override;

    virtual std::ostream& printToStream(std::ostream& stream, int level, int evFlags = 0) const override;

    virtual W getMinInterferencePower() const override { return minInterferencePower; }
};

} // namespace physicallayer

} // namespace inet

#endif

