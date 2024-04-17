//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
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

#ifndef CMACBASEWAVEAPPLLAYER_H_
#define CMACBASEWAVEAPPLLAYER_H_

#include <map>
#include <BaseApplLayer.h>
#include <Consts80211p.h>
#include <CMACWSM_m.h>
#include "base/connectionManager/ChannelAccess.h"
#include <WaveAppToMac1609_4Interface.h>

// SCMAC
//#include "modules/mobility/traci/TraCIMobility.h"

//using Veins::TraCIMobility;


#ifndef DBG
#define DBG EV
#endif
//#define DBG std::cerr << "[" << simTime().raw() << "] " << getParentModule()->getFullPath() << " "

/**
 * @brief
 * WAVE application layer base class.
 *
 * @author David Eckhoff
 * @reviser Chris Shen
 *
 * @ingroup applLayer
 *
 * @see CMacBaseWaveApplLayer
 * @see Mac1609_4
 * @see PhyLayer80211p
 * @see Decider80211p
 */
class CMacBaseWaveApplLayer : public BaseApplLayer {

	public:
//		CMacBaseWaveApplLayer();
		~CMacBaseWaveApplLayer();
		virtual void initialize(int stage);
		virtual void finish();

		virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj);

		enum WaveApplMessageKinds {
			SERVICE_PROVIDER = LAST_BASE_APPL_MESSAGE_KIND,
			SEND_BEACON_EVT,
			SEND_DATA_EVT,
			SEND_FLOOD_EVT
		};

	protected:

		static const simsignalwrap_t mobilityStateChangedSignal;

		/** @brief handle messages from below */
		virtual void handleLowerMsg(cMessage* msg);
		/** @brief handle self messages */
		virtual void handleSelfMsg(cMessage* msg);

		virtual CMACWSM* prepareWSM(	std::string name,
										int dataLengthBits,
										t_channel channel,
										int priority,
										int rcvId,
										int serial=0);

		virtual void sendWSM(CMACWSM* wsm);
		virtual void onBeacon(CMACWSM* wsm) = 0;
		virtual void onData(CMACWSM* wsm) = 0;

		virtual void handlePositionUpdate(cObject* obj);

	protected:
		int beaconLengthBits;
		int beaconPriority;
		bool sendData;
		bool sendBeacons;
		simtime_t individualOffset;
		int dataLengthBits;
		bool dataOnSch;
		int dataPriority;
		Coord curPosition;
		int mySCH;
		int myId;

		cMessage* sendBeaconEvt;

		WaveAppToMac1609_4Interface* myMac;
};

#endif /* CMACBASEWAVEAPPLLAYER_H_ */
