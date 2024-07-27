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

//based on Ieee802154NarrowbandMac

package inet.flora.lorabase;


import inet.linklayer.csmaca.CsmaCaMac; 


module LoRaMac extends CsmaCaMac
{
    parameters:
        string classUsed  = default("A");
        
           // RX1 RX2 receive windows parameters
        int waitDelay1Time = default(1);
        int listening1Time = default(1);
        int waitDelay2Time = default(1);
        int listening2Time = default(1);
        
        // beacon parameters
        double beaconStart = default(1);
        double beaconPeriodTime = default(128);
        double beaconReservedTime = default(2.120);
        double beaconGuardTime = default(3);
        
        // class B parameters
        double classBslotTime = default(0.030);
        int timeToNextSlot = default(5);
        int pingOffset = default(5);
        
        bitrate = default(250kbps);
        @class(LoRaMac);
        gates:
        	input upperMgmtIn;
        	output upperMgmtOut;
}
