//
// Copyright (C) 2016 OpenSim Ltd.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

#include <openssl/evp.h>
#include <openssl/conf.h>

#include "inet/flora/lorabase/LoRaMac.h"

#include "inet/flora/lorabase/LoRaTagInfo_m.h"
#include "inet/common/ModuleAccess.h"
#include "inet/linklayer/common/Ieee802Ctrl.h"
#include "inet/linklayer/common/UserPriority.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "inet/linklayer/csmaca/CsmaCaMac.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/linklayer/common/InterfaceTag_m.h"

namespace inet {
namespace flora {

Define_Module(LoRaMac);
simsignal_t LoRaMac::signalSFchanged = cComponent::registerSignal("signalSFchanged");

LoRaMac::~LoRaMac()
{
    cancelAndDelete(endTransmission);
    cancelAndDelete(endReception);
    cancelAndDelete(droppedPacket);
    cancelAndDelete(endDelay_1);
    cancelAndDelete(endListening_1);
    cancelAndDelete(endDelay_2);
    cancelAndDelete(endListening_2);
    cancelAndDelete(mediumStateChange);
}

/****************************************************************
 * Initialization functions.
 */
void LoRaMac::initialize(int stage)
{
    MacProtocolBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        EV << "Initializing stage 0\n";

        //maxQueueSize = par("maxQueueSize");
        headerLength = par("headerLength");
        ackLength = par("ackLength");
        ackTimeout = par("ackTimeout");
        retryLimit = par("retryLimit");
        bitrate = bps(par("bitrate"));

        waitDelay1Time = 1;
        listening1Time = 1;
        waitDelay2Time = 1;
        listening2Time = 1;

        // beacon parameters
                beaconStart = par("beaconStart");
                beaconPeriodTime = par("beaconPeriodTime");
                beaconReservedTime = par("beaconReservedTime");
                beaconGuardTime = par("beaconGuardTime");

                // class B parameters
                 classBslotTime = par("classBslotTime");
                 timeToNextSlot = par("timeToNextSlot");
                 pingOffset = par("pingOffset");


        const char *addressString = par("address");
        if (!strcmp(addressString, "auto")) {
            // assign automatic address
            address = MacAddress::generateAutoAddress();
            // change module parameter from "auto" to concrete address
            par("address").setStringValue(address.str().c_str());
        }
        else
            address.setAddress(addressString);

        // subscribe for the information of the carrier sense
        cModule *radioModule = getModuleFromPar<cModule>(par("radioModule"), this);
        radioModule->subscribe(IRadio::receptionStateChangedSignal, this);
        radioModule->subscribe(IRadio::transmissionStateChangedSignal, this);
        radioModule->subscribe(LoRaRadio::droppedPacket, this);
        radio = check_and_cast<IRadio *>(radioModule);

        // initialize self messages
        endTransmission = new cMessage("Transmission");
        endReception = new cMessage("Reception");
        droppedPacket = new cMessage("Dropped Packet");
        endDelay_1 = new cMessage("Delay_1");
        endListening_1 = new cMessage("Listening_1");
        endDelay_2 = new cMessage("Delay_2");
        endListening_2 = new cMessage("Listening_2");
        mediumStateChange = new cMessage("MediumStateChange");

        beaconGuardStart = new cMessage("Beacon_Guard_Start");
        beaconGuardEnd = new cMessage("Beacon_Guard_End");
        beaconPeriod = new cMessage("Beacon_Period");
        beaconReservedEnd = new cMessage("Beacon_Close");

        pingPeriod = new cMessage("Ping_Period");
        endPingSlot = new cMessage("Ping_Slot_Close");

        beginTXslot = new cMessage("UplinkSlot_Start");

        // set up internal queue
        txQueue = getQueue(gate(upperLayerInGateId));//check_and_cast<queueing::IPacketQueue *>(getSubmodule("queue"));

        // schedule beacon when using class B
                const char *usedClass = par("classUsed");
                  if (strcmp(usedClass,"A"))
                       {
                        scheduleAt(simTime() + beaconStart, beaconPeriod);
                        isClassA = false;
                        isClassB = true;
                       }


        // state variables
        fsm.setName("LoRaMac State Machine");
        backoffPeriod = -1;
        retryCounter = 0;

        // sequence number for messages
        sequenceNumber = 0;
        lastSentSequenceNumber=-1;
        // statistics
        numRetry = 0;
        numSentWithoutRetry = 0;
        numGivenUp = 0;
        numCollision = 0;
        numSent = 0;
        numReceived = 0;
        numSentBroadcast = 0;
        numReceivedBroadcast = 0;
        numReceivedBeacons = 0;

        // initialize watches
        WATCH(fsm);
        WATCH(backoffPeriod);
        WATCH(retryCounter);
        WATCH(numRetry);
        WATCH(numSentWithoutRetry);
        WATCH(numGivenUp);
        WATCH(numCollision);
        WATCH(numSent);
        WATCH(numReceived);
        WATCH(numSentBroadcast);
        WATCH(numReceivedBroadcast);
        WATCH(numReceivedBeacons);
    }
    else if (stage == INITSTAGE_LINK_LAYER)
        radio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
}

void LoRaMac::finish()
{
    recordScalar("numRetry", numRetry);
    recordScalar("numSentWithoutRetry", numSentWithoutRetry);
    recordScalar("numGivenUp", numGivenUp);
    //recordScalar("numCollision", numCollision);
    recordScalar("numSent", numSent);
    recordScalar("numReceived", numReceived);
    recordScalar("numSentBroadcast", numSentBroadcast);
    recordScalar("numReceivedBroadcast", numReceivedBroadcast);
    recordScalar("numReceivedBeacons", numReceivedBeacons);
}

void LoRaMac::configureNetworkInterface()
{
    //NetworkInterface *e = new NetworkInterface(this);

    // data rate
    networkInterface->setDatarate(bitrate.get());
    networkInterface->setMacAddress(address);

    // capabilities
    //interfaceEntry->setMtu(par("mtu"));
    networkInterface->setMtu(std::numeric_limits<int>::quiet_NaN());
    networkInterface->setMulticast(true);
    networkInterface->setBroadcast(true);
    networkInterface->setPointToPoint(false);
}

/****************************************************************
 * Message handling functions.
 */
void LoRaMac::handleSelfMessage(cMessage *msg)
{
    if (msg == beaconPeriod)
        {
            beaconGuard = false;
            beaconScheduling();
        }
    if (msg == beaconReservedEnd)
        {
            EV<<"msg is beaconReservedEnd "<<msg->getName()<<endl;
            EV<<"the value of iGotBeacon is "<<iGotBeacon<<" and the value of isClassB is "<<isClassB<< endl;
            if(iGotBeacon)
            {
                numReceivedBeacons++;
                iGotBeacon = false;
                if (isClassB)
                    schedulePingPeriod();

                if (hasGUI())
                    getParentModule()->getParentModule()->bubble(beaconReceivedText);
            }
        }
    if (msg == beaconGuardStart)
               beaconGuard = true;
    EV << "received self message: " << msg << endl;
    handleWithFsm(msg);
}
//#if 0
void LoRaMac::handleUpperPacket(Packet *packet)//cMessage *msg)
{//needs track
   // if(fsm.getState() != IDLE) {
        //    error("Wrong, it should not happen erroneous state: %s", fsm.getStateName());
   // }
    //auto pkt = check_and_cast<Packet *>(msg);

    packet->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&Protocol::lora);
//    LoRaMacControlInfo *cInfo = check_and_cast<LoRaMacControlInfo *>(msg->getControlInfo());
    auto pktEncap = encapsulate(packet);

    const auto &frame = pktEncap->peekAtFront<LoRaMacFrame>();

    EV << "frame " << pktEncap << " received from higher layer, receiver = " << frame->getReceiverAddress() << endl;

  //  txQueue->enqueuePacket(pktEncap);
   // if (fsm.getState() != IDLE)
       // EV << "deferring upper message transmission in " << fsm.getStateName() << " state\n";
   // else {
       // EV<<"befor ecalling popTxQueue() 246 "<<endl;
      //  popTxQueue();
      //  handleWithFsm(currentTxFrame);
   // }
    if (frame == nullptr)
            throw cRuntimeError("Header LoRaMacFrame not found");

        if (currentTxFrame != nullptr)
            throw cRuntimeError("Model error: incomplete transmission exists");
        currentTxFrame = pktEncap;
        handleWithFsm(currentTxFrame);
}
//#endif
/*void LoRaMac::handleUpperPacket(Packet *packet)
{
    //if(fsm.getState() != IDLE) {
       //  error("Wrong, it should not happen erroneous state: %s", fsm.getStateName());
   // }
    packet->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&Protocol::lora);

    EV << "frame " << packet << " received from higher layer " << endl;
    auto pktEncap = encapsulate(packet);
    const auto &frame = pktEncap->peekAtFront<LoRaMacFrame>();
    if (frame == nullptr)
        throw cRuntimeError("Header LoRaMacFrame not found");

    if (currentTxFrame != nullptr)
        throw cRuntimeError("Model error: incomplete transmission exists");
    currentTxFrame = pktEncap;
    handleWithFsm(currentTxFrame);
}*/


void LoRaMac::handleLowerPacket(Packet *msg)
{

    if( (fsm.getState() == RECEIVING_1) || (fsm.getState() == RECEIVING_2) ||
            (fsm.getState()== RECEIVING) || (fsm.getState()==RECEIVING_BEACON) )
    {
        const auto &frame = msg->peekAtFront<LoRaMacFrame>();
        auto tag = msg->getTag<LoRaTag>();
        EV<<"Tag:: "<<tag->getSignalRSSI_dBm()<<endl;
        int sfToPassUp=sfBasedOnRSSI(tag->getSignalRSSI_dBm());
        EV<<"value of sfToPassUp is :"<<sfToPassUp<<endl;
        emit(LoRaMac::signalSFchanged, sfToPassUp);
        if (isBeacon(frame))
                      {
                          int ping = pow(2,12)/frame->getPingNb();
                          timeToNextSlot = ping*classBslotTime;
                          EV << "time to next slot: " << timeToNextSlot<< endl;
                      }
        handleWithFsm(msg);
    }
    else
        {
        EV << "Received lower message but MAC FSM is not on a valid state for reception" << endl;
        EV << "Deleting message " << msg << endl;
        delete msg;
        }
}

void LoRaMac::processUpperPacket()
{
    EV<<"processUpperPacket is called 300"<<endl;
    Packet *packet = dequeuePacket();
    handleUpperMessage(packet);
}

queueing::IPassivePacketSource *LoRaMac::getProvider(const cGate *gate)
{
    return (gate->getId() == upperLayerInGateId) ? txQueue.get() : nullptr;
}

void LoRaMac::handleCanPullPacketChanged(const cGate *gate)
{
    Enter_Method("handleCanPullPacketChanged");
    if (fsm.getState() == IDLE && !txQueue->isEmpty()) {
        processUpperPacket();
    }
}

void LoRaMac::handlePullPacketProcessed(Packet *packet, const cGate *gate, bool successful)
{
    Enter_Method("handlePullPacketProcessed");
    throw cRuntimeError("Not supported callback");
}

void LoRaMac::handleWithFsm(cMessage *msg)
{
    Ptr<LoRaMacFrame>frame = nullptr;

    auto pkt = dynamic_cast<Packet *>(msg);
    if (pkt) {
        const auto &chunk = pkt->peekAtFront<Chunk>();
        frame = dynamicPtrCast<LoRaMacFrame>(constPtrCast<Chunk>(chunk));
    }
    EV<<"handleWithFsm is called and time is "<<simTime()<<endl;
    if (isClassA)
       {
    FSMA_Switch(fsm)
    {
        FSMA_State(IDLE)
        {
            FSMA_Enter(turnOffReceiver());
            FSMA_Event_Transition(Idle-Transmit,
                                  isUpperMessage(msg),
                                  TRANSMIT,
                                  EV << "CLASS A: starting transmission" << endl;
                                    );
        }
        FSMA_State(TRANSMIT)
        {
            FSMA_Enter(sendDataFrame(getCurrentTransmission()));
            FSMA_Event_Transition(Transmit-Wait_Delay_1,
                                  msg == endTransmission,
                                  WAIT_DELAY_1,
                                  EV << "CLASS A: transmission concluded" << endl;
                                  finishCurrentTransmission();
                                  numSent++;
                                   );
        }
        FSMA_State(WAIT_DELAY_1)
        {
            FSMA_Enter(turnOffReceiver());
            FSMA_Event_Transition(Wait_Delay_1-Listening_1,
                                  msg == endDelay_1 || endDelay_1->isScheduled() == false,
                                  LISTENING_1,
                                  EV << "CLASS A: opening receive window 1" << endl;
                                   );
        }
        FSMA_State(LISTENING_1)
        {
            FSMA_Enter(turnOnReceiver());
            FSMA_Event_Transition(Listening_1-Wait_Delay_2,
                                  msg == endListening_1 || endListening_1->isScheduled() == false,
                                  WAIT_DELAY_2,
                                  EV << "CLASS A: didn't receive downlink on receive window 1" << endl;
                                   );
            FSMA_Event_Transition(Listening_1-Receiving1,
                                  msg == mediumStateChange && isReceiving(),
                                  RECEIVING_1,
                                  EV << "CLASS A: receiving a message on receive window 1, analyzing packet..." << endl;
                                   );
        }
        FSMA_State(RECEIVING_1)
        {
            FSMA_Event_Transition(Receive-Unicast-Not-For,
                                  isLowerMessage(msg) && !isForUs(frame),
                                  LISTENING_1,
                                  EV << "CLASS A: wrong address downlink received, back to listening on window 1" << endl;
                                   );
            FSMA_Event_Transition(Receive-Unicast,
                                  isLowerMessage(msg) && isForUs(frame),
                                  IDLE,
                                  EV << "CLASS A: received downlink successfully on window 1, back to IDLE" << endl;
                                  sendUp(decapsulate(pkt));
                                  numReceived++;
                                  cancelEvent(endListening_1);
                                  cancelEvent(endDelay_2);
                                  cancelEvent(endListening_2);
                                    );
            FSMA_Event_Transition(Receive-BelowSensitivity,
                                  msg == droppedPacket,
                                  LISTENING_1,
                                  EV << "CLASS A: low power downlink, back to listening on window 1" << endl;
                                    );

        }
        FSMA_State(WAIT_DELAY_2)
        {
            FSMA_Enter(turnOffReceiver());
            FSMA_Event_Transition(Wait_Delay_2-Listening_2,
                                  msg == endDelay_2 || endDelay_2->isScheduled() == false,
                                  LISTENING_2,
                                  EV << "CLASS A: opening receive window 2" << endl;
                                    );
        }
        FSMA_State(LISTENING_2)
        {
            FSMA_Enter(turnOnReceiver());
            FSMA_Event_Transition(Listening_2-idle,
                                  msg == endListening_2 || endListening_2->isScheduled() == false,
                                  IDLE,
                                  EV << "CLASS A: didn't receive downlink on receive window 2" << endl;
                                    );
            FSMA_Event_Transition(Listening_2-Receiving2,
                                  msg == mediumStateChange && isReceiving(),
                                  RECEIVING_2,
                                  EV << "CLASS A: receiving a message on receive window 2, analyzing packet..." << endl;
                                    );
        }
        FSMA_State(RECEIVING_2)
        {
            FSMA_Event_Transition(Receive2-Unicast-Not-For,
                                  isLowerMessage(msg) && !isForUs(frame),
                                  LISTENING_2,
                                  EV << "CLASS A: wrong address downlink received, back to listening on window 2" << endl;
                                    );
            FSMA_Event_Transition(Receive2-Unicast,
                                  isLowerMessage(msg) && isForUs(frame),
                                  IDLE,
                                  EV << "CLASS A: received downlink successfully on window 2, back to IDLE" << endl;
                                  sendUp(pkt);
                                  numReceived++;
                                  cancelEvent(endListening_2);
                                    );
            FSMA_Event_Transition(Receive2-BelowSensitivity,
                                  msg == droppedPacket,
                                  LISTENING_2,
                                  EV << "CLASS A: low power downlink, back to listening on window 2" << endl;
                                    );
        }
        }
    }


    // THE FSM FOR CLASS B
        if (isClassB)
        {
            EV<<" class is B and state is: "<<fsm.getState()<<endl;
            FSMA_Switch(fsm)
            {
                FSMA_State(IDLE)
                {
                    FSMA_Enter(turnOffReceiver());
                    FSMA_Event_Transition(Idle-BeaconReception,
                                          msg == beaconPeriod,
                                          BEACON_RECEPTION,
                                          EV << "CLASS B: Going to Beacon Reception" << endl;
                                          );
                    FSMA_Event_Transition(Idle-Transmit,
                                          isUpperMessage(msg),
                                          TRANSMIT,
                                          EV << "CLASS B: starting transmission" << endl;
                                          );
                    FSMA_Event_Transition(Idle-ListeningOnPingSlot,
                                          msg == pingPeriod && !beaconGuard,
                                          PING_SLOT,
                                          EV << "CLASS B: starting Ping Slot" << endl;
                                          );
                }

                FSMA_State(BEACON_RECEPTION)
                {
                    FSMA_Enter(turnOnReceiver());
                    FSMA_Event_Transition(BeaconReception-Idle,
                                          msg == beaconReservedEnd,
                                          IDLE,
                                          EV << "CLASS B: no beacon detected, increasing beacon time" << endl;
                                          increaseBeaconTime();
                                          );
                    FSMA_Event_Transition(BeaconReception-ReceivingBeacon,
                                          msg == mediumStateChange && isReceiving(),
                                          RECEIVING_BEACON,
                                          EV << "CLASS B: Going to Receiving Beacon" << endl;
                                          );
                }

                FSMA_State(RECEIVING_BEACON)
                {
                    EV<<"Entring RECEIVING_BEACON state "<<endl;
                    FSMA_Event_Transition(ReceivingBeacon-Unicast-Not-For,
                                          isLowerMessage(msg) && isBeacon(frame),  //  && !isForUs(frame)
                                          IDLE,
                                          EV << "CLASS B: beacon received" << endl;
                                          calculatePingPeriod(frame);
                                          );
                    FSMA_Event_Transition(ReceivingBeacon-BelowSensitivity,
                                          msg == droppedPacket,
                                          IDLE,
                                          EV << "CLASS B: beacon below sensitivity" << endl;
                                          increaseBeaconTime();
                                          );

                   FSMA_Event_Transition(ReceivingBeacon-NotBeacon,
                                         isLowerMessage(msg) && !isBeacon(frame),
                                         IDLE,
                                         EV << "CLASS B: the recieved msg is not a beacon" << endl;
                                          //increaseBeaconTime();
                                         );
                }

                FSMA_State(PING_SLOT)
                {
                    FSMA_Enter(turnOnReceiver());
                    FSMA_Event_Transition(ListeningOnPingSlot-Idle,
                                          msg == endPingSlot && !isReceiving(),
                                          IDLE,
                                          EV << "CLASS B: no downlink detected, back to IDLE" << endl;
                                          );
                    FSMA_Event_Transition(ListeningOnPingSlot-ReceivingOnPingSlot,
                                          msg == mediumStateChange && isReceiving(),
                                          RECEIVING,
                                          EV << "CLASS B: going to receive downlink on ping slot" << endl;
                                          );
                }

                FSMA_State(RECEIVING)
                {
                    FSMA_Event_Transition(ReceivingOnPingSlot-Unicast-Not-For,
                                          isLowerMessage(msg) && !isForUs(frame),
                                          IDLE,
                                          EV << "CLASS B: wrong address downlink on ping slot, back to IDLE" << endl;
                                          );
                    FSMA_Event_Transition(ReceivingOnPingSlot-Unicast,
                                          isLowerMessage(msg) && isForUs(frame),
                                          IDLE,
                                          EV << "CLASS B: received downlink on ping slot, back to IDLE" << endl;
                                          sendUp(decapsulate(pkt));
                                          numReceived++;
                                          );
                    FSMA_Event_Transition(ReceivingOnPingSlot-BelowSensitivity,
                                          msg == droppedPacket,
                                          IDLE,
                                          EV << "CLASS B: downlink below sensitivity, back to IDLE" << endl;
                                          );
                }

                FSMA_State(TRANSMIT)
                {
                    EV<<"Entring TRANSMIT_Sate"<<endl;
                    const auto & myframe = getCurrentTransmission()->peekAtFront<LoRaMacFrame>();
                    EV<<"value of getSequenceNumber is "<<myframe->getSequenceNumber()<<endl;

                    while (myframe->getSequenceNumber()!= lastSentSequenceNumber)
                    {
                        FSMA_Enter(sendDataFrame(getCurrentTransmission()));

                        EV<<"value of getSequenceNumber in the frame is "<<myframe->getSequenceNumber()<<endl;
                        lastSentSequenceNumber=myframe->getSequenceNumber();
                        EV<<"value of lastSentSequenceNumber in my counter is "<<lastSentSequenceNumber<<endl;
                    }
                    FSMA_Event_Transition(Transmit-Wait_Delay_1,
                                          msg == endTransmission,
                                          WAIT_DELAY_1,
                                          EV << "CLASS B: transmission concluded" << endl;
                                          finishCurrentTransmission();
                                          numSent++;
                                          );
                   /* FSMA_Enter(sendDataFrame(getCurrentTransmission()));
                    FSMA_Event_Transition(Transmit-Wait_Delay_1,
                                           msg == endTransmission,
                                           WAIT_DELAY_1,
                                           EV << "CLASS B: transmission concluded" << endl;
                                           finishCurrentTransmission();
                                           numSent++;
                                                          );*/
                }

                FSMA_State(WAIT_DELAY_1)
                {
                    FSMA_Enter(turnOffReceiver());
                    FSMA_Event_Transition(Wait_Delay_1-Listening_1,
                                          msg == endDelay_1 || endDelay_1->isScheduled() == false,
                                          LISTENING_1,
                                          EV << "CLASS B: opening receive window 1" << endl;
                                          );
                }

                FSMA_State(LISTENING_1)
                {
                    FSMA_Enter(turnOnReceiver());
                    FSMA_Event_Transition(Listening_1-Wait_Delay_2,
                                          msg == endListening_1 || endListening_1->isScheduled() == false,
                                          WAIT_DELAY_2,
                                          EV << "CLASS B: didn't receive downlink on receive window 1" << endl;
                                          );
                    FSMA_Event_Transition(Listening_1-Receiving1,
                                          msg == mediumStateChange && isReceiving(),
                                          RECEIVING_1,
                                          EV << "CLASS B: receiving a message on receive window 1, analyzing packet..." << endl;
                                          );
                }

                FSMA_State(RECEIVING_1)
                {
                    FSMA_Event_Transition(Receive-Unicast-Not-For,
                                          isLowerMessage(msg) && !isForUs(frame),
                                          LISTENING_1,
                                          EV << "CLASS B: wrong address downlink received, back to listening on window 1" << endl;
                                          );
                    FSMA_Event_Transition(Receive-Unicast,
                                          isLowerMessage(msg) && isForUs(frame),
                                          IDLE,
                                          EV << "CLASS B: received downlink successfully on window 1, back to IDLE" << endl;
                                          sendUp(decapsulate(pkt));
                                          numReceived++;
                                          cancelEvent(endListening_1);
                                          cancelEvent(endDelay_2);
                                          cancelEvent(endListening_2);
                                          );
                    FSMA_Event_Transition(Receive-BelowSensitivity,
                                          msg == droppedPacket,
                                          LISTENING_1,
                                          EV << "CLASS B: low power downlink, back to listening on window 2" << endl;
                                          );
                }

                FSMA_State(WAIT_DELAY_2)
                {
                    FSMA_Enter(turnOffReceiver());
                    FSMA_Event_Transition(Wait_Delay_2-Listening_2,
                                          msg == endDelay_2 || endDelay_2->isScheduled() == false,
                                          LISTENING_2,
                                          EV << "CLASS B: opening receive window 2" << endl;
                                          );
                }

                FSMA_State(LISTENING_2)
                {
                    FSMA_Enter(turnOnReceiver());
                    FSMA_Event_Transition(Listening_2-idle,
                                          msg == endListening_2 || endListening_2->isScheduled() == false,
                                          IDLE,
                                          EV << "CLASS B: didn't receive downlink on receive window 2" << endl;
                                          );
                    FSMA_Event_Transition(Listening_2-Receiving2,
                                          msg == mediumStateChange && isReceiving(),
                                          RECEIVING_2,
                                          EV << "CLASS B: receiving a message on receive window 2, analyzing packet..." << endl;
                                          );
                }

                FSMA_State(RECEIVING_2)
                {
                    FSMA_Event_Transition(Receive2-Unicast-Not-For,
                                          isLowerMessage(msg) && !isForUs(frame),
                                          LISTENING_2,
                                          EV << "CLASS B: wrong address downlink received, back to listening on window 2" << endl;
                    );
                    FSMA_Event_Transition(Receive2-Unicast,
                                          isLowerMessage(msg) && isForUs(frame),
                                          IDLE,
                                          EV << "CLASS B: received downlink successfully on window 2, back to IDLE" << endl;
                                          sendUp(pkt);
                                          numReceived++;
                                          cancelEvent(endListening_2);
                                          );
                    FSMA_Event_Transition(Receive2-BelowSensitivity,
                                          msg == droppedPacket,
                                          LISTENING_2,
                                          EV << "CLASS B: low power downlink, back to listening on window 2" << endl;
                                          );
                }
            }
        }


//    if (fsm.getState() == IDLE) {
//        if (isReceiving())
//            handleWithFsm(mediumStateChange);
//        else if (currentTxFrame != nullptr)
//            handleWithFsm(currentTxFrame);
//        else if (!txQueue->isEmpty()) {
//            popTxQueue();
//            handleWithFsm(currentTxFrame);
//        }
//    }



    if (fsm.getState() == IDLE) {
        EV<<" 695 state is idle "<<endl;
        EV<<" Is the queue empty? "<<txQueue->isEmpty()<<endl;
        if (isReceiving())
            handleWithFsm(mediumStateChange);
        else if (currentTxFrame != nullptr){
            EV<<" 699 insdie  "<<endl;
            handleWithFsm(currentTxFrame);}
        else if (!txQueue->isEmpty()) {
            EV<<" 701 not empty "<<endl;
            processUpperPacket();
          //  popTxQueue();
            //handleWithFsm(currentTxFrame);
        }
    }

    if (endSifs) {
        if (isLowerMessage(msg) && pkt->getOwner() == this && (endSifs->getContextPointer() != pkt))
            delete pkt;
    }
    else {
        if (isLowerMessage(msg) && pkt->getOwner() == this)
            delete pkt;
    }
    getDisplayString().setTagArg("t", 0, fsm.getStateName());
}

void LoRaMac::receiveSignal(cComponent *source, simsignal_t signalID, intval_t value, cObject *details)
{
    Enter_Method_Silent();
    if (signalID == IRadio::receptionStateChangedSignal) {
        IRadio::ReceptionState newRadioReceptionState = (IRadio::ReceptionState)value;
        if (receptionState == IRadio::RECEPTION_STATE_RECEIVING) {
            radio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
        }
        receptionState = newRadioReceptionState;
        handleWithFsm(mediumStateChange);
    }
    else if (signalID == LoRaRadio::droppedPacket) {
        radio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
        handleWithFsm(droppedPacket);
    }
    else if (signalID == IRadio::transmissionStateChangedSignal) {
        IRadio::TransmissionState newRadioTransmissionState = (IRadio::TransmissionState)value;
        if (transmissionState == IRadio::TRANSMISSION_STATE_TRANSMITTING && newRadioTransmissionState == IRadio::TRANSMISSION_STATE_IDLE) {
            handleWithFsm(endTransmission);
            radio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
        }
        transmissionState = newRadioTransmissionState;
    }
}

Packet *LoRaMac::encapsulate(Packet *msg)
{
    auto frame = makeShared<LoRaMacFrame>();
    frame->setChunkLength(B(headerLength));
    msg->setArrival(msg->getArrivalModuleId(), msg->getArrivalGateId());
    auto tag = msg->getTag<LoRaTag>();

    frame->setTransmitterAddress(address);
    frame->setLoRaTP(tag->getPower().get());
    frame->setLoRaCF(tag->getCenterFrequency());
    frame->setLoRaSF(tag->getSpreadFactor());
    frame->setLoRaBW(tag->getBandwidth());
    frame->setLoRaCR(tag->getCodeRendundance());
    frame->setSequenceNumber(sequenceNumber);
    frame->setReceiverAddress(MacAddress::BROADCAST_ADDRESS);

    ++sequenceNumber;
    //frame->setLoRaUseHeader(cInfo->getLoRaUseHeader());
    frame->setLoRaUseHeader(tag->getUseHeader());

    msg->insertAtFront(frame);

    return msg;
}

Packet *LoRaMac::decapsulate(Packet *frame)
{

    auto loraHeader = frame->popAtFront<LoRaMacFrame>();
    frame->addTagIfAbsent<MacAddressInd>()->setSrcAddress(loraHeader->getTransmitterAddress());
    frame->addTagIfAbsent<MacAddressInd>()->setDestAddress(loraHeader->getReceiverAddress());
    frame->addTagIfAbsent<InterfaceInd>()->setInterfaceId(networkInterface->getInterfaceId());
//    auto payloadProtocol = ProtocolGroup::ethertype.getProtocol(loraHeader->getNetworkProtocol());
//    frame->addTagIfAbsent<DispatchProtocolReq>()->setProtocol(payloadProtocol);
//    frame->addTagIfAbsent<PacketProtocolTag>()->setProtocol(payloadProtocol);

    return frame;
}

/****************************************************************
 * Frame sender functions.
 */
void LoRaMac::sendDataFrame(Packet *frameToSend)
{
    EV << "sending Data frame\n";
    radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);

    auto frameCopy = frameToSend->dup();

    //LoRaMacControlInfo *ctrl = new LoRaMacControlInfo();
    //ctrl->setSrc(frameCopy->getTransmitterAddress());
    //ctrl->setDest(frameCopy->getReceiverAddress());
//    frameCopy->setControlInfo(ctrl);
    auto macHeader = frameCopy->peekAtFront<LoRaMacFrame>();

    auto macAddressInd = frameCopy->addTagIfAbsent<MacAddressInd>();
    macAddressInd->setSrcAddress(macHeader->getTransmitterAddress());
    macAddressInd->setDestAddress(macHeader->getReceiverAddress());

    //frameCopy->addTag<PacketProtocolTag>()->setProtocol(&Protocol::lora);

    sendDown(frameCopy);
}

void LoRaMac::sendAckFrame()
{
    auto frameToAck = static_cast<Packet *>(endSifs->getContextPointer());
    endSifs->setContextPointer(nullptr);
    auto macHeader = makeShared<CsmaCaMacAckHeader>();
    macHeader->setReceiverAddress(MacAddress(frameToAck->peekAtFront<LoRaMacFrame>()->getTransmitterAddress().getInt()));

    EV << "sending Ack frame\n";
    //auto macHeader = makeShared<CsmaCaMacAckHeader>();
    macHeader->setChunkLength(B(ackLength));
    auto frame = new Packet("CsmaAck");
    frame->insertAtFront(macHeader);
//    frame->addTag<PacketProtocolTag>()->setProtocol(&Protocol::lora);
    radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);

    auto macAddressInd = frame->addTagIfAbsent<MacAddressInd>();
    macAddressInd->setSrcAddress(macHeader->getTransmitterAddress());
    macAddressInd->setDestAddress(macHeader->getReceiverAddress());

    sendDown(frame);
}

/****************************************************************
 * Helper functions.
 */
// schedule beacon signals
void LoRaMac::beaconScheduling()
{
    EV<<" beaconScheduling 951 "<<endl;
    cancelEvent(beaconPeriod);
    cancelEvent(beaconReservedEnd);
    cancelEvent(beaconGuardStart);
    scheduleAt(simTime() + beaconPeriodTime, beaconPeriod);
    scheduleAt(simTime() + beaconReservedTime, beaconReservedEnd);
    scheduleAt(simTime() + beaconPeriodTime - beaconGuardTime, beaconGuardStart);
}
void LoRaMac::increaseBeaconTime()
{
    beaconReservedTime = beaconReservedTime + 1;
}
void LoRaMac::schedulePingPeriod()
{
    cancelEvent(pingPeriod);
    cancelEvent(endPingSlot);
    scheduleAt(simTime() + (pingOffset*classBslotTime), pingPeriod);
    scheduleAt(simTime() + (pingOffset*classBslotTime) + classBslotTime, endPingSlot);
}
//calculate the pingSlotPeriod using Aes128 encryption for randomization
void LoRaMac::calculatePingPeriod(const Ptr<const LoRaMacFrame> &frame)
{
    iGotBeacon = true;
    beaconReservedTime = 2.120;
    unsigned char cipher[7];

    cipher[0]=(unsigned char)(frame->getBeaconTimer()
            + getAddress().getAddressByte(0)
            + getAddress().getAddressByte(1)
            + getAddress().getAddressByte(2)
            + getAddress().getAddressByte(3)
            + getAddress().getAddressByte(4)
            + getAddress().getAddressByte(5)
            );
    cipher[1]=(unsigned char)(getAddress().getAddressByte(0));
    cipher[2]=(unsigned char)(getAddress().getAddressByte(1));
    cipher[3]=(unsigned char)(getAddress().getAddressByte(2));
    cipher[4]=(unsigned char)(getAddress().getAddressByte(3));
    cipher[5]=(unsigned char)(getAddress().getAddressByte(4));
    cipher[6]=(unsigned char)(getAddress().getAddressByte(5));

    int message_len = strlen((const char*)cipher);
    unsigned char cipher2[64];
    unsigned char* key = (unsigned char*)"00000000000000000000000000000000";

    int cipher_len = aesEncrypt(cipher,message_len,key,cipher2);
    int period = pow(2,12)/(frame->getPingNb());

    pingOffset = (cipher2[0]+(cipher2[1]*256))% period;
}
int LoRaMac::aesEncrypt(unsigned char *message, int message_len, unsigned char *key, unsigned char *cipher)
{
    int cipher_len = 0;
    int len = 0;

    EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();

    if(!ctx){
        perror("EVP_SIPHER_CTX_new() failed");
        exit(-1);
    }
    if (!EVP_EncryptInit_ex(ctx, EVP_aes_128_ecb(), NULL, key, NULL)){
        perror("EVP_EncryptInit_ex() failed");
        exit(-1);
    }
    if (!EVP_EncryptUpdate(ctx, cipher, &len, message, message_len)){
        perror("EVP_EncryptUpdate() failed");
        exit(-1);
    }
    cipher_len += len;
    if (!EVP_EncryptFinal_ex(ctx, cipher + len, &len)){
        perror("EVP_EnryptFinal_ex() failed");
        exit(-1);
    }
    cipher_len += len;
    EVP_CIPHER_CTX_free(ctx);
    return cipher_len;
}

void LoRaMac::finishCurrentTransmission()
{
    scheduleAt(simTime() + waitDelay1Time, endDelay_1);
    scheduleAt(simTime() + waitDelay1Time + listening1Time, endListening_1);
    scheduleAt(simTime() + waitDelay1Time + listening1Time + waitDelay2Time, endDelay_2);
    scheduleAt(simTime() + waitDelay1Time + listening1Time + waitDelay2Time + listening2Time, endListening_2);
    deleteCurrentTxFrame();
    //popTxQueue();
}

Packet *LoRaMac::getCurrentTransmission()
{
    ASSERT(currentTxFrame != nullptr);
    return currentTxFrame;
}

bool LoRaMac::isReceiving()
{
    return radio->getReceptionState() == IRadio::RECEPTION_STATE_RECEIVING;
}

bool LoRaMac::isAck(const Ptr<const LoRaMacFrame> &frame)
{
    return false;//dynamic_cast<LoRaMacFrame *>(frame);
}
bool LoRaMac::isBeacon(const Ptr<const LoRaMacFrame> &frame)
{
    return frame->getPktType() == BEACON;
}
int LoRaMac::sfBasedOnRSSI(double x)
{
    int calculatedSF=0;
    if ((x >= -137 && x < -134.5) || x <-137 ) calculatedSF=12;//if ((MaxRSSIinGW >= -137 && MaxRSSIinGW < -135) || MaxRSSIinGW <-137 ) calculatedSF=12;
              if (x >= -134.5 && x < -132) calculatedSF=11; // if (MaxRSSIinGW >= -135 && MaxRSSIinGW < -133) calculatedSF=11;//
              if (x >= -132 && x < -129) calculatedSF=10;//if (MaxRSSIinGW >= -133 && MaxRSSIinGW < -130) calculatedSF=10; //
              if (x >= -129 && x < -126) calculatedSF=9;//if (MaxRSSIinGW >= -130 && MaxRSSIinGW < -127) calculatedSF=9;//
              if (x >= -126 && x < -123) calculatedSF=8;//if (MaxRSSIinGW >= -127 && MaxRSSIinGW < -124) calculatedSF=8;//
              if (x > -123) calculatedSF=7; //if (MaxRSSIinGW > -124) calculatedSF=7;//
         EV<<"After: calculatedSF is "<<calculatedSF<<endl;
         return calculatedSF;
}
bool LoRaMac::isBroadcast(const Ptr<const LoRaMacFrame> &frame)
{
    return frame->getReceiverAddress().isBroadcast();
}

bool LoRaMac::isForUs(const Ptr<const LoRaMacFrame> &frame)
{
    return frame->getReceiverAddress() == address;
}

void LoRaMac::turnOnReceiver()
{
    LoRaRadio *loraRadio;
    loraRadio = check_and_cast<LoRaRadio *>(radio);
    loraRadio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);
}

void LoRaMac::turnOffReceiver()
{
    LoRaRadio *loraRadio;
    loraRadio = check_and_cast<LoRaRadio *>(radio);
    loraRadio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
}

MacAddress LoRaMac::getAddress()
{
    return address;
}

}
} // namespace inet
