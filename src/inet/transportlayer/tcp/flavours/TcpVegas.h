//
// Copyright (C) 2013 Maria Fernandez, Carlos Calafate, Juan-Carlos Cano and Pietro Manzoni
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

#ifndef __INET_TCPVEGAS_H
#define __INET_TCPVEGAS_H

#include "inet/transportlayer/tcp/flavours/TcpBaseAlg.h"
#include "inet/transportlayer/tcp/flavours/TcpSegmentTransmitInfoList.h"

namespace inet {
namespace tcp {

/**
 * State variables for TcpVegas.
 */
class INET_API TcpVegasStateVariables : public TcpBaseAlgStateVariables
{
  public:
    TcpVegasStateVariables();
    ~TcpVegasStateVariables();
    virtual std::string str() const override;
    virtual std::string detailedInfo() const override;

    uint32_t v_recoverypoint;
    simtime_t v_cwnd_changed; // last time cwnd changes because of a rtx.

    simtime_t v_baseRTT;
    simtime_t v_sumRTT; // sum of rtt's measured within one RTT
    int v_cntRTT; // # of rtt's measured within one RTT
    uint32_t v_begseq; // register next pkt to be sent,for rtt calculation in receivedDataAck
    simtime_t v_begtime; // register time for rtt calculation

    simtime_t v_rtt_timeout; // vegas fine-grained timeout
    simtime_t v_sa; // average for vegas fine-grained timeout
    simtime_t v_sd; // deviation for vegas fine-grained timeout

    TcpSegmentTransmitInfoList regions;

    uint32_t ssthresh; ///< slow start threshold

    bool v_inc_flag; // for slow start: "exponential growth only every other RTT"
    bool v_incr_ss; // to control no incr. cwnd if during slowstart ssthresh has been exceeded before the rtt is over
    int32_t v_incr; // incr/decr
    uint32_t v_worried; // pkts a to retransmit due to vegas fine-grained timeout
};

class INET_API TcpVegas : public TcpBaseAlg
{
  protected:
    TcpVegasStateVariables *& state; // alias to TcpAlgorithm's 'state'

    /** Create and return a TCPvegasStateVariables object. */
    virtual TcpStateVariables *createStateVariables() override
    {
        return new TcpVegasStateVariables();
    }

    /** Utility function to recalculate ssthresh */
    virtual void recalculateSlowStartThreshold();

    /** Redefine what should happen on retransmission */
    virtual void processRexmitTimer(TcpEventCode& event) override;

  public:
    /** Ctor */
    TcpVegas();

    /** Redefine what should happen when data got acked, to add congestion window management */
    virtual void receivedDataAck(uint32_t firstSeqAcked) override;

    /** Redefine what should happen when dupAck was received, to add congestion window management */
    virtual void receivedDuplicateAck() override;

    /** Called after we send data */
    virtual void dataSent(uint32_t fromseq) override;

    virtual void segmentRetransmitted(uint32_t fromseq, uint32_t toseq) override;
};

} // namespace tcp
} // namespace inet

#endif

