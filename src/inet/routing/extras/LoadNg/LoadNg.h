//
// Copyright (C) 2014 OpenSim Ltd.
// Author: Benjamin Seregi
// Copyright (C) 2019 Universidad de Malaga
// Author: Alfonso Ariza
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
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#ifndef __INET_AODV_H
#define __INET_AODV_H

#include <map>

#include "inet/common/INETDefs.h"
#include "inet/networklayer/contract/IInterfaceTable.h"
#include "inet/networklayer/contract/IL3AddressType.h"
#include "inet/networklayer/contract/INetfilter.h"
#include "inet/networklayer/contract/IRoutingTable.h"
#include "inet/routing/extras/LoadNg/LoadNgControlPackets_m.h"
#include "inet/routing/extras/LoadNg/LoadNgRouteData.h"
#include "inet/routing/base/RoutingProtocolBase.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"
#include "inet/transportlayer/udp/UdpHeader_m.h"
#include "inet/common/DijktraKShortest.h"
#include "inet/physicallayer/common/packetlevel/SignalTag_m.h"

namespace inet {
namespace inetmanet {

/*
 * This class implements AODV routing protocol and Netfilter hooks
 * in the IP-layer required by this protocol.
 */

class INET_API LoadNg : public RoutingProtocolBase, public NetfilterBase::HookBase, public cListener
{
  protected:
    /*
     * It implements a unique identifier for an arbitrary RREQ message
     * in the network. See: rreqsArrivalTime.
     */
    class RreqIdentifier
    {
      public:
        L3Address originatorAddr;
        unsigned int rreqID;
        RreqIdentifier(const L3Address& originatorAddr, unsigned int rreqID) : originatorAddr(originatorAddr), rreqID(rreqID) {};
        bool operator==(const RreqIdentifier& other) const
        {
            return this->originatorAddr == other.originatorAddr && this->rreqID == other.rreqID;
        }
    };

    class RreqIdentifierCompare
    {
    public:
        bool operator()(const RreqIdentifier& lhs, const RreqIdentifier& rhs) const
        {
            if (lhs.originatorAddr < rhs.originatorAddr)
                return true;
            else if (lhs.originatorAddr > rhs.originatorAddr)
                return false;
            else
                return lhs.rreqID < rhs.rreqID;
        }
    };

    class NodeStatus {
    public:
        bool isBidirectional = false;
        bool pendingConfirmation = false; // the latest notification has failed.
        int64_t seqNum = -1;
        uint8_t metric;
        uint8_t numHelloRec;
    };

    class NeigborElement {
    public:
        simtime_t lastNotification;
        simtime_t lifeTime;
        int64_t seqNumber = -1;
        uint64_t helloIdentify;
        bool isBidirectional = false;
        bool pendingConfirmation = false; // the latest notification has failed.
        std::map<L3Address, NodeStatus> listNeigbours;
        std::deque<simtime_t> helloTime;
        uint8_t metric = 255;
        int32_t distRoot = -1;
        int32_t metricToRoot;
    };


    std::map<L3Address, NeigborElement> neighbors;

    // DFF information sets
    bool activeFFD = false;
    uint64_t ffdForwardSeqNum = 0;


    class DffTupleId {
        L3Address origin;
        uint64_t seqNumber = 0;
        inline bool operator<(const DffTupleId& b) const
        {
            if (origin != b.origin)
                return origin < b.origin;
            else
                return seqNumber < b.seqNumber;
        };
        inline bool operator >(const DffTupleId& b) const
        {
            if (origin != b.origin)
                return origin > b.origin;
            else
                return seqNumber > b.seqNumber;
        };
        inline bool operator == (const DffTupleId& b) const
        {
            if (origin == b.origin && seqNumber == b.seqNumber)
                return true;
            else
                return false;
        };
        DffTupleId& operator = (const DffTupleId& b)
        {
            if (this == &b) return *this;
            origin = b.origin;
            seqNumber = b.seqNumber;
            return *this;
        };
    };

    class DffTuple {
    public:
        L3Address destination;
        L3Address prevHop;
        simtime_t time;
        std::deque<L3Address> nextHopNeigList;
    };
    std::map<DffTupleId, DffTuple> dffSet;

    std::map<L3Address, std::vector<std::vector<L3Address>>> alternativePaths;


    // End DFF information sets

    Dijkstra *dijkstra = nullptr;
    DijkstraKshortest *dijkstraKs = nullptr;

    // bool checkNeigborList();

    // context
    IL3AddressType *addressType = nullptr;    // to support both Ipv4 and v6 addresses.

    // environment
    cModule *host = nullptr;
    IRoutingTable *routingTable = nullptr;
    IInterfaceTable *interfaceTable = nullptr;
    INetfilter *networkProtocol = nullptr;

    // AODV parameters: the following parameters are configurable, see the NED file for more info.
    unsigned int rerrRatelimit = 0;
    unsigned int aodvUDPPort = 0;
    bool askGratuitousRREP = false;
    bool useHelloMessages = false;
    bool destinationOnlyFlag = false;

    //
    bool isRoot = false; // this node is a root;
    bool measureEtx = false;
    int numHellosEtx = 10;


    simtime_t maxJitter;
    simtime_t activeRouteTimeout;
    simtime_t helloInterval;
    simtime_t minHelloInterval;
    simtime_t maxHelloInterval;

    unsigned int netDiameter = 0;
    unsigned int rreqRetries = 0;
    unsigned int rreqRatelimit = 0;
    unsigned int timeoutBuffer = 0;
    unsigned int ttlStart = 0;
    unsigned int ttlIncrement = 0;
    unsigned int ttlThreshold = 0;
    unsigned int localAddTTL = 0;
    unsigned int allowedHelloLoss = 0;
    simtime_t nodeTraversalTime;
    cPar *jitterPar = nullptr;
    cPar *periodicJitter = nullptr;

    // the following parameters are calculated from the parameters defined above
    // see the NED file for more info
    simtime_t deletePeriod;
    simtime_t myRouteTimeout;
    simtime_t blacklistTimeout;
    simtime_t netTraversalTime;
    simtime_t nextHopWait;
    simtime_t pathDiscoveryTime;

    // state
    unsigned int rreqId = 0;    // Not used, to remove
    unsigned int sequenceNum = 0;    // it helps to prevent loops in the routes (RFC 3561 6.1 p11.)
    uint64_t helloIdentifier = 0; // used to identify the hello and no increase the seq number

    std::map<L3Address, WaitForRrep *> waitForRREPTimers;    // timeout for Route Replies
    std::map<RreqIdentifier, simtime_t, RreqIdentifierCompare> rreqsArrivalTime;    // maps RREQ id to its arriving time
    L3Address failedNextHop;    // next hop to the destination who failed to send us RREP-ACK
    std::map<L3Address, simtime_t> blacklist;    // we don't accept RREQs from blacklisted nodes
    unsigned int rerrCount = 0;    // num of originated RERR in the last second
    unsigned int rreqCount = 0;    // num of originated RREQ in the last second
    simtime_t lastBroadcastTime;    // the last time when any control packet was broadcasted
    std::map<L3Address, unsigned int> addressToRreqRetries;    // number of re-discovery attempts per address

    // self messages
    cMessage *helloMsgTimer = nullptr;    // timer to send hello messages (only if the feature is enabled)
    cMessage *expungeTimer = nullptr;    // timer to clean the routing table out
    cMessage *counterTimer = nullptr;    // timer to set rrerCount = rreqCount = 0 in each second
    cMessage *rrepAckTimer = nullptr;    // timer to wait for RREP-ACKs (RREP-ACK timeout)
    cMessage *blacklistTimer = nullptr;    // timer to clean the blacklist out

    // lifecycle
    simtime_t rebootTime;    // the last time when the node rebooted

    // internal
    std::multimap<L3Address, Packet *> targetAddressToDelayedPackets;    // queue for the datagrams we have no route for

  protected:
    void handleMessageWhenUp(cMessage *msg) override;
    void initialize(int stage) override;
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }

    /* Route Discovery */
    void startRouteDiscovery(const L3Address& target, unsigned int timeToLive = 0);
    void completeRouteDiscovery(const L3Address& target);
    bool hasOngoingRouteDiscovery(const L3Address& destAddr);
    void cancelRouteDiscovery(const L3Address& destAddr);

    /* Routing Table management */
    void updateRoutingTable(IRoute *route, const L3Address& nextHop, unsigned int hopCount, int64_t destSeqNum, bool isActive, simtime_t lifeTime, int metricType, unsigned int metric);
    IRoute *createRoute(const L3Address& destAddr, const L3Address& nextHop, unsigned int hopCount, int64_t destSeqNum, bool isActive, simtime_t lifeTime, int metricType, unsigned int metric);
    bool updateValidRouteLifeTime(const L3Address& destAddr, simtime_t lifetime);
    void scheduleExpungeRoutes();
    void expungeRoutes();

    /* Control packet creators */
    const Ptr<RrepAck> createRREPACK();
    const Ptr<Hello> createHelloMessage();
    const Ptr<Rreq> createRREQ(const L3Address& destAddr);
    const Ptr<Rrep> createRREP(const Ptr<Rreq>& rreq, IRoute *destRoute, IRoute *originatorRoute, const L3Address& sourceAddr);
    const Ptr<Rrep> createGratuitousRREP(const Ptr<Rreq>& rreq, IRoute *originatorRoute);
    const Ptr<Rerr> createRERR(const std::vector<UnreachableNode>& unreachableNodes);

    /* Control Packet handlers */
    void handleRREP(const Ptr<Rrep>& rrep, const L3Address& sourceAddr);
    void handleRREQ(const Ptr<Rreq>& rreq, const L3Address& sourceAddr, unsigned int timeToLive);
    void handleRERR(const Ptr<const Rerr>& rerr, const L3Address& sourceAddr);
    void handleHelloMessage(const Ptr<const Hello>& helloMessage, SignalPowerInd *, SnirInd *);
    void handleRREPACK(const Ptr<const RrepAck>& rrepACK, const L3Address& neighborAddr);

    /* Control Packet sender methods */
    void sendRREQ(const Ptr<Rreq>& rreq, const L3Address& destAddr, unsigned int timeToLive);
    void sendRREPACK(const Ptr<RrepAck>& rrepACK, const L3Address& destAddr);
    void sendRREP(const Ptr<Rrep>& rrep, const L3Address& destAddr, unsigned int timeToLive);
    void sendGRREP(const Ptr<Rrep>& grrep, const L3Address& destAddr, unsigned int timeToLive);

    /* Control Packet forwarders */
    void forwardRREP(const Ptr<Rrep>& rrep, const L3Address& destAddr, unsigned int timeToLive);
    void forwardRREQ(const Ptr<Rreq>& rreq, unsigned int timeToLive);

    /* Self message handlers */
    void handleRREPACKTimer();
    void handleBlackListTimer();
    void sendHelloMessagesIfNeeded();
    void handleWaitForRREP(WaitForRrep *rrepTimer);

    /* General functions to handle route errors */
    void sendRERRWhenNoRouteToForward(const L3Address& unreachableAddr);
    void handleLinkBreakSendRERR(const L3Address& unreachableAddr);
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details) override;

    /* Netfilter hooks */
    Result ensureRouteForDatagram(Packet *datagram);
    virtual Result datagramPreRoutingHook(Packet *datagram) override { Enter_Method("datagramPreRoutingHook"); return ensureRouteForDatagram(datagram); }
    virtual Result datagramForwardHook(Packet *datagram) override;
    virtual Result datagramPostRoutingHook(Packet *datagram) override { return ACCEPT; }
    virtual Result datagramLocalInHook(Packet *datagram) override { return ACCEPT; }
    virtual Result datagramLocalOutHook(Packet *datagram) override { Enter_Method("datagramLocalOutHook"); return ensureRouteForDatagram(datagram); }
    void delayDatagram(Packet *datagram);

    /* Helper functions */
    virtual void checkNeigList();
    virtual void runDijkstra();
    virtual void runDijkstraKs();
    L3Address getSelfIPAddress() const;
    void sendLoadNgPacket(const Ptr<LoadNgControlPacket>& packet, const L3Address& destAddr, unsigned int timeToLive, double delay);
    void clearState();

    /* Lifecycle */
    virtual void handleStartOperation(LifecycleOperation *operation) override;
    virtual void handleStopOperation(LifecycleOperation *operation) override;
    virtual void handleCrashOperation(LifecycleOperation *operation) override;

  public:
    LoadNg();
    virtual ~LoadNg();
};

} // namespace aodv
} // namespace inet

#endif // ifndef __INET_AODV_H

