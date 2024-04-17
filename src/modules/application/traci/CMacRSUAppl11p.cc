//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "application/traci/CMacRSUAppl11p.h"

using Veins::AnnotationManagerAccess;

Define_Module(CMacRSUAppl11p);

void CMacRSUAppl11p::initialize(int stage) {
	CMacBaseWaveApplLayer::initialize(stage);
	if (stage == 0) {
	    mobi = dynamic_cast<BaseMobility*> (getParentModule()->getSubmodule("mobility"));
//	    mobi = dynamic_cast<TraCIMobility*> (getParentModule()->getSubmodule("mobility"));

	    ASSERT(mobi);
	    annotations = AnnotationManagerAccess().getIfExists();
	    ASSERT(annotations);

	    // Chris
		sendDataEvt = new cMessage("data evt", SEND_DATA_EVT);

		double offSet = dblrand() * (par("beaconInterval").doubleValue()/2);
		offSet = offSet + floor(offSet/0.050)*0.050;

//		findHost()->subscribe(mobilityStateChangedSignal, this);

//		std::cout << " SENDDATA RSU: " <<sendData << std::endl;
//		if (sendData){
//			scheduleAt(simTime() + offSet, sendDataEvt);
//		}

		// Chris end
	}
}

void CMacRSUAppl11p::onBeacon(CMACWSM* wsm) {

}

void CMacRSUAppl11p::onData(CMACWSM* wsm) {
	findHost()->getDisplayString().updateWith("r=16,green");

	annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), mobi->getCurrentPosition(), "blue"));

	if (!sentMessage) sendMessage(wsm->getWsmData());
}

void CMacRSUAppl11p::sendMessage(std::string blockedRoadId) {
	sentMessage = true;
	t_channel channel = dataOnSch ? type_SCH : type_CCH;
	CMACWSM* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
	wsm->setWsmData(blockedRoadId.c_str());
	sendWSM(wsm);
}
void CMacRSUAppl11p::sendWSM(CMACWSM* wsm) {
	sendDelayedDown(wsm,individualOffset);

}

void CMacRSUAppl11p::handleSelfMsg(cMessage* msg) {
	switch (msg->getKind()) {
//		case SEND_BEACON_EVT: {
//			sendWSM(prepareWSM("beacon", beaconLengthBits, type_CCH, beaconPriority, 0, -1));
//			scheduleAt(simTime() + par("beaconInterval").doubleValue(), sendBeaconEvt);
//			break;
//		}
		case SEND_DATA_EVT:{
			sendWSM(prepareWSM("data", dataLengthBits, type_CCH, dataPriority, 0, -1));
			scheduleAt(simTime() + par("beaconInterval").doubleValue(), sendDataEvt);
			break;
		}
		default: {
			if (msg)
				DBG << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
			break;
		}
	}
}
