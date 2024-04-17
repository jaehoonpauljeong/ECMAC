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

#ifndef CMACApp11p_H
#define CMACApp11p_H

#include "CMacBaseWaveApplLayer.h"
#include "modules/mobility/traci/TraCIMobility.h"
#include <algorithm>

using Veins::TraCIMobility;
using Veins::AnnotationManager;


#include "modules/mobility/traci/TraCIScenarioManager.h"
#include "modules/mobility/traci/TraCIScenarioManagerLaunchd.h"
#include "base/connectionManager/ConnectionManager.h"
using Veins::TraCIScenarioManager;
using Veins::TraCIScenarioManagerAccess;
using Veins::TraCIScenarioManagerLaunchd;

/**
 * Small IVC Demo using 11p
 */
class CMACApp11p : public CMacBaseWaveApplLayer {
	public:
		virtual void initialize(int stage);
		virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj);
	protected:
		TraCIMobility* traci;
		AnnotationManager* annotations;
		simtime_t lastDroveAt;
		bool sentMessage;
		bool isParking;
		bool sendWhileParking;
		static const simsignalwrap_t parkingStateChangedSignal;

		// variable Chris
		void handleSelfMsg(cMessage*);
		void handleLowerMsg(cMessage*);
		CMACWSM* prepareWSM(std::string name,
							int lengthBits,
							t_channel channel,
							int priority,
							int rcvId,
							int serial);
		double getOffsetRatio(Veins::TraCIMobility* );

		bool debugApp;

		bool appSaveToFile;

		bool ecmacApp;
		bool dmmacApp;
		bool waveApp;

        int seed;
        int density;
        std::string schemeName;

		cMessage* sendDataEvt;
		bool sendPkt;
		bool sendRandPkt;
		int dataLengthBits;
		double sendInterval;

		// emergency situation, flood
		bool sendEmgPkt;
		int emgPktCount;
		simtime_t sendEmgInterval;

		cMessage* sendFloodEvt;
		cMessage* decideVehEvt;
		cMessage* transitEvt;

		int pktSerial;
		std::map<std::string,int> rxSerMap;
		virtual void finish();

		std::string destIDdebug;

		double pktSendEndTime;

		// performance
		long peTxPackets;
		long peRxPackets;
		long peRxDropPkts;
		long peEmgTXPackets;
		long peEmgRxPackets;

		double peEmgPktRxTime;

		bool b_backgroundTraffic;
		double d_bgTrafficInterval;
		cMessage* eventSendWSM;
		// end Chris
	protected:
		virtual void onBeacon(CMACWSM* wsm);
		virtual void onData(CMACWSM* wsm);
		virtual void onStatus(CMACWSM* wsm);
		void sendMessage(std::string blockedRoadId);
		virtual void handlePositionUpdate(cObject* obj);
		virtual void handleParkingUpdate(cObject* obj);
		virtual void sendWSM(CMACWSM* wsm);
};

#endif
