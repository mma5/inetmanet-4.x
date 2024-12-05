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

#ifndef __LORA_OMNET_SIMPLELORAAPP_H_
#define __LORA_OMNET_SIMPLELORAAPP_H_

#include <omnetpp.h>

#include "inet/flora/loraapp/LoRaAppPacket_m.h"
#include "inet/common/lifecycle/ILifecycle.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/LifecycleOperation.h"
#include "inet/flora/lorabase/LoRaMac.h"


using namespace omnetpp;

namespace inet {
namespace flora{

/**
 * TODO - Generated class
 */
class INET_API SimpleLoRaApp : public cSimpleModule, public ILifecycle,public cListener
{
    protected:
        virtual void initialize(int stage) override;
        virtual void finish() override;
        virtual int numInitStages() const override { return NUM_INIT_STAGES; }
        virtual void handleMessage(cMessage *msg) override;
        virtual bool handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback) override;

        virtual void handleMessageFromLowerLayer(cMessage *msg);
        std::pair<double,double> generateUniformCircleCoordinates(double radius, double gatewayX, double gatewayY);
        virtual void sendJoinRequest();
        //virtual void sendDownMgmtPacket();
        virtual void receiveSignal(cComponent *source, simsignal_t signalID, intval_t value, cObject *details) override;


        int numberOfPacketsToSend = 0;
        int sentPackets = 0;
        int receivedADRCommands = 0;
        int lastSentMeasurement = 0;
        simtime_t timeToFirstPacket;
        simtime_t timeToNextPacket;

        cMessage *configureLoRaParameters = nullptr;
        cMessage *sendMeasurements = nullptr;

        //history of sent packets;
        cOutVector sfVector;
        cOutVector tpVector;
// to be used for subscribing to LoRaMac::signalSFchanged
        cModule *LoRaMacModule=nullptr;

        //variables to control ADR
        bool evaluateADRinNode;
        int ADR_ACK_CNT = 0;
        int ADR_ACK_LIMIT = 64; //64;
        int ADR_ACK_DELAY = 32; //32;
        bool sendNextPacketWithADRACKReq = false;
        void increaseSFIfPossible();

    public:
        SimpleLoRaApp() {}
        simsignal_t LoRa_AppPacketSent;
        //LoRa physical layer parameters
        double loRaTP = -1;
        units::values::Hz loRaCF = units::values::Hz(NaN);
        int loRaSF;
        units::values::Hz loRaBW = units::values::Hz(NaN);
        int loRaCR = -1;
        bool loRaUseHeader = false;
};

}
}
#endif
