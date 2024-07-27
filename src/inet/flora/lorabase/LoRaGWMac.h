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

#ifndef LORA_LORAGWMAC_H_
#define LORA_LORAGWMAC_H_

#include "inet/flora/lorabase/LoRaMacFrame_m.h"
#include "inet/common/INETDefs.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/IRadio.h"
#include "inet/linklayer/contract/IMacProtocol.h"
#include "inet/linklayer/base/MacProtocolBase.h"
#include "inet/linklayer/common/InterfaceTag_m.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "inet/common/ModuleAccess.h"

namespace inet {
namespace flora {

using namespace physicallayer;

class LoRaGWMac: public MacProtocolBase {
public:
    bool waitingForDC;
    cMessage *dutyCycleTimer = nullptr;
    virtual void initialize(int stage) override;
    virtual void finish() override;
    virtual void configureNetworkInterface() override;
    long GW_forwardedDown;
    long GW_droppedDC;

    virtual ~LoRaGWMac();
    virtual void handleUpperMessage(cMessage *msg) override;
    virtual void handleLowerMessage(cMessage *msg) override;
    virtual void handleSelfMessage(cMessage *message) override;

    void sendPacketBack(Packet *receivedFrame);
    void beaconScheduling();
    void sendBeacon();
    void createFakeLoRaMacFrame();
    virtual MacAddress getAddress();

protected:

    const char beaconSentText[13] = "Beacon sent!";

    bool isClassA = true;
    bool isClassB = false;
    bool beaconGuard = false;
    int beaconNumber = -1;
        int pingNumber;

        int beaconSF;
        int beaconCR = -1;
        double beaconTP;
        double beaconCF;
        double beaconBW;

        simtime_t beaconStart = -1;
        simtime_t beaconGuardTime = -1;
        simtime_t beaconReservedTime = -1;
        double beaconPeriodTime = -1;

    /** End of the beacon period */
        cMessage *beaconPeriod = nullptr;

        /** End of the beacon reserved period */
        cMessage *beaconReservedEnd = nullptr;

        /** Start of the beacon guard period */
        cMessage *beaconGuardStart = nullptr;

        /** End of uplink transmission slot */
        cMessage *endTXslot = nullptr;

    MacAddress address;

    IRadio *radio = nullptr;
    IRadio::TransmissionState transmissionState = IRadio::TRANSMISSION_STATE_UNDEFINED;

    virtual void receiveSignal(cComponent *source, simsignal_t signalID, intval_t value, cObject *details) override;
};

}
}

#endif /* LORA_LORAGWMAC_H_ */
