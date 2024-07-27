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

#include "inet/flora/lorabase/LoRaGWMac.h"

#include "inet/flora/loraphy/LoRaPhyPreamble_m.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/ProtocolTag_m.h"


#include "inet/physicallayer/wireless/common/contract/packetlevel/IRadio.h"


namespace inet {
namespace flora {

Define_Module(LoRaGWMac);

void LoRaGWMac::initialize(int stage)
{
    MacProtocolBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        // subscribe for the information of the carrier sense
        cModule *radioModule = getModuleFromPar<cModule>(par("radioModule"), this);
        //radioModule->subscribe(IRadio::radioModeChangedSignal, this);
        radioModule->subscribe(IRadio::transmissionStateChangedSignal, this);
        radio = check_and_cast<IRadio *>(radioModule);
        waitingForDC = false;
        dutyCycleTimer = new cMessage("Duty Cycle Timer");
        beaconPeriod = new cMessage("Beacon Timer");
                endTXslot = new cMessage("UplinkSlot_End");
                beaconGuardStart = new cMessage("Beacon_Guard_Start");
                beaconReservedEnd = new cMessage("Beacon_Reserved_End");

                // beacon parameters
                beaconStart = par("beaconStart");
                beaconPeriodTime = par("beaconPeriodTime");
                beaconReservedTime = par("beaconReservedTime");
                beaconGuardTime = par("beaconGuardTime");

                pingNumber = par("pingNumber");

                // lora parameters for beacon
                        beaconSF = par("beaconSF");
                        beaconTP = par("beaconTP");
                        beaconCF = par("beaconCF");
                        beaconBW = par("beaconBW");
                        beaconCR = par("beaconCR");

                        // schedule beacon when using class B or S
                                const char *usedClass = par("classUsed");
                                if (strcmp(usedClass,"A"))
                                {
                                    EV<<"LoRaGWMac:: line 105 "<< usedClass<<endl;
                                    scheduleAt(simTime() + beaconStart, beaconPeriod);
                                    isClassA = false;

                                    if (!strcmp(usedClass,"B"))
                                        isClassB = true;
                                }


        const char *addressString = par("address");
        GW_forwardedDown = 0;
        GW_droppedDC = 0;
        if (!strcmp(addressString, "auto")) {
            // assign automatic address
            address = MacAddress::generateAutoAddress();
            // change module parameter from "auto" to concrete address
            par("address").setStringValue(address.str().c_str());
        }
        else
            address.setAddress(addressString);
    }
    else if (stage == INITSTAGE_LINK_LAYER) {
        radio->setRadioMode(IRadio::RADIO_MODE_TRANSCEIVER);
    }
}

void LoRaGWMac::finish()
{
    recordScalar("GW_forwardedDown", GW_forwardedDown);
    recordScalar("GW_droppedDC", GW_droppedDC);
    cancelAndDelete(dutyCycleTimer);
    dutyCycleTimer = nullptr;
    cancelAndDelete(beaconPeriod);
        cancelAndDelete(endTXslot);
        cancelAndDelete(beaconGuardStart);
        cancelAndDelete(beaconReservedEnd);
}

LoRaGWMac::~LoRaGWMac() {
    if (dutyCycleTimer != nullptr)
        cancelAndDelete(dutyCycleTimer);
}

void LoRaGWMac::configureNetworkInterface()
{
    //NetworkInterface *e = new NetworkInterface(this);

    // data rate
    //e->setDatarate(bitrate);

    // generate a link-layer address to be used as interface token for IPv6
    //e->setMACAddress(address);
    //e->setInterfaceToken(address.formInterfaceIdentifier());

    // capabilities
    //e->setMtu(par("mtu"));
    //e->setMulticast(true);
    //e->setBroadcast(true);
    //e->setPointToPoint(false);
    MacAddress address = parseMacAddressParameter(par("address"));

    // generate a link-layer address to be used as interface token for IPv6
    networkInterface->setMacAddress(address);
    // data rate
    //interfaceEntry->setDatarate(bitrate);

    // capabilities
    networkInterface->setMtu(par("mtu"));
    networkInterface->setMulticast(true);
    networkInterface->setBroadcast(true);
    networkInterface->setPointToPoint(false);
}

void LoRaGWMac::handleSelfMessage(cMessage *msg)
{
    if (msg == beaconPeriod)
        {
        EV << "beacon msg is released" << endl;

            beaconNumber++;
            beaconGuard = false;
            beaconScheduling();
            sendBeacon();
        }
    if (msg == beaconGuardStart)
        {
        EV << "beaconGuardStart msg is released" << endl;
            beaconGuard = true;
          //  if (isClassS)
               // cancelEvent(endTXslot);
        }
    if(msg == dutyCycleTimer) waitingForDC = false;
}

void LoRaGWMac::handleUpperMessage(cMessage *msg)
{
    if(waitingForDC == false)
    {
//        LoRaMacFrame *frame = check_and_cast<LoRaMacFrame *>(msg);
//        frame->removeControlInfo();
        auto pkt = check_and_cast<Packet *>(msg);
        const auto &frame = pkt->peekAtFront<LoRaMacFrame>();
        if (pkt->getControlInfo())
            delete pkt->removeControlInfo();

        auto tag = pkt->addTagIfAbsent<MacAddressReq>();
        tag->setDestAddress(frame->getReceiverAddress());
//        LoRaMacControlInfo *ctrl = new LoRaMacControlInfo();
//        ctrl->setSrc(address);
//        ctrl->setDest(frame->getReceiverAddress());
//        frame->setControlInfo(ctrl);
//        sendDown(frame);


        waitingForDC = true;
        double delta;
        if(frame->getLoRaSF() == 7) delta = 0.61696;
        if(frame->getLoRaSF() == 8) delta = 1.23392;
        if(frame->getLoRaSF() == 9) delta = 2.14016;
        if(frame->getLoRaSF() == 10) delta = 4.28032;
        if(frame->getLoRaSF() == 11) delta = 7.24992;
        if(frame->getLoRaSF() == 12) delta = 14.49984;
        scheduleAt(simTime() + delta, dutyCycleTimer);
        GW_forwardedDown++;
        pkt->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&Protocol::apskPhy);
        sendDown(pkt);
    }
    else
    {
        GW_droppedDC++;
        delete msg;
    }
}

void LoRaGWMac::handleLowerMessage(cMessage *msg)
{
    auto pkt = check_and_cast<Packet *>(msg);
    auto header = pkt->popAtFront<flora::LoRaPhyPreamble>();
    const auto &frame = pkt->peekAtFront<LoRaMacFrame>();
    if(frame->getReceiverAddress() == MacAddress::BROADCAST_ADDRESS)
        sendUp(pkt);
    else
        delete pkt;
}

void LoRaGWMac::sendPacketBack(Packet *receivedFrame)
{
    const auto &frame = receivedFrame->peekAtFront<LoRaMacFrame>();
    EV << "sending Data frame back" << endl;
    auto pktBack = new Packet("LoraPacket");
    auto frameToSend = makeShared<LoRaMacFrame>();
    frameToSend->setChunkLength(B(par("headerLength").intValue()));

    frameToSend->setReceiverAddress(frame->getTransmitterAddress());
    pktBack->insertAtFront(frameToSend);
    sendDown(pktBack);
}

void LoRaGWMac::createFakeLoRaMacFrame()
{

}
// schedule beacon signals
void LoRaGWMac::beaconScheduling()
{
    scheduleAt(simTime() + beaconPeriodTime, beaconPeriod);
    scheduleAt(simTime() + beaconReservedTime, beaconReservedEnd);
    scheduleAt(simTime() + beaconPeriodTime - beaconGuardTime, beaconGuardStart);
}
//this function send beacon message when the class is set to B or S
void LoRaGWMac::sendBeacon()
{
    EV << "sending Beacon" << endl;

    auto beacon = new Packet("Beacon");
    auto frame = makeShared<LoRaMacFrame>();
    frame->setPktType(BEACON);
    frame->setChunkLength(B(par("headerLength").intValue()));
    auto tag = beacon->addTagIfAbsent<MacAddressReq>();
    tag->setDestAddress(MacAddress::BROADCAST_ADDRESS);
    beacon->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&Protocol::apskPhy);

    units::values::Hz loRaBW = inet::units::values::Hz(beaconBW);
    units::values::Hz loRaCF = inet::units::values::Hz(beaconCF);

    frame->setLoRaSF(beaconSF);
    frame->setLoRaTP(beaconTP);
    frame->setLoRaBW(loRaBW);
    frame->setLoRaCF(loRaCF);

    frame->setBeaconTimer(beaconPeriodTime);
    frame->setPingNb(pingNumber);

    beacon->insertAtFront(frame);
    sendDown(beacon);
    if (hasGUI())
        getParentModule()->getParentModule()->bubble(beaconSentText);
}
void LoRaGWMac::receiveSignal(cComponent *source, simsignal_t signalID, intval_t value, cObject *details)
{
    Enter_Method_Silent();
    if (signalID == IRadio::transmissionStateChangedSignal) {
        IRadio::TransmissionState newRadioTransmissionState = (IRadio::TransmissionState)value;
        if (transmissionState == IRadio::TRANSMISSION_STATE_TRANSMITTING && newRadioTransmissionState == IRadio::TRANSMISSION_STATE_IDLE) {
            //transmissin is finished
            radio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);
        }
        transmissionState = newRadioTransmissionState;
    }
}

MacAddress LoRaGWMac::getAddress()
{
    return address;
}

}
}
