//
// Copyright (C) 2012 David Eckhoff <eckhoff@cs.fau.de>
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

#include "DMMAC.h"
#include <iterator>
#define FSM_DEBUG

#define DBG_MAC EV
//#define DBG_MAC std::cerr << "[" << simTime().raw() << "] " << myId << " "

const simsignalwrap_t DMMAC::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);

// for debug
static bool chris_dmmac = true;
static bool chris_dmmac_2 = false;
static bool chris_dmmac_3 = false;
static bool chris_dmmac_4 = false;
static bool chris_dmmac_5 = false;

Define_Module(DMMAC);
using namespace std;
#define ROADSPEEDLIMIT 30.556 // 30.556m/s = 110 km/h = 68.35 mph
#define TS 2
#define STATUSMSGINTERVAL 0.2
#define TF 10
#define CHANNELSWITCHGUARD 0.002
const SimTime CCI = SimTime().setRaw(100000000000UL);
const SimTime TA = 6 * SLOTLENGTH_11P;

static bool gFlagClustering = false;
static bool g_bNewVeh = false;

void DMMAC::initialize(int stage) {
	BaseMacLayer::initialize(stage);
	if (stage == 0) {

		phy11p = FindModule<Mac80211pToPhy11pInterface*>::findSubModule(
		             getParentModule());
		assert(phy11p);

		//this is required to circumvent double precision issues with constants from CONST80211p.h
		assert(simTime().getScaleExp() == -12);

		txPower = par("txPower").doubleValue();
		bitrate = par("bitrate");
		n_dbps = 0;
		setParametersForBitrate(bitrate);

		//mac-adresses
		myMacAddress = intuniform(0,0xFFFFFFFE);
		myId = getParentModule()->getParentModule()->getFullPath();
		//create frequency mappings
		frequency.insert(std::pair<int, double>(Channels::CRIT_SOL, 5.86e9));
		frequency.insert(std::pair<int, double>(Channels::SCH1, 5.87e9));
		frequency.insert(std::pair<int, double>(Channels::SCH2, 5.88e9));
		frequency.insert(std::pair<int, double>(Channels::CCH, 5.89e9));
		frequency.insert(std::pair<int, double>(Channels::SCH3, 5.90e9));
		frequency.insert(std::pair<int, double>(Channels::SCH4, 5.91e9));
		frequency.insert(std::pair<int, double>(Channels::HPPS, 5.92e9));

		//create two edca systems

		myEDCA[type_CCH] = new EDCA(type_CCH,par("queueSize").longValue());
		myEDCA[type_CCH]->myId = myId;
		myEDCA[type_CCH]->myId.append(" CCH");

		myEDCA[type_CCH]->createQueue(2,(((CWMIN_11P+1)/4)-1),(((CWMIN_11P +1)/2)-1),AC_VO);
		myEDCA[type_CCH]->createQueue(3,(((CWMIN_11P+1)/2)-1),CWMIN_11P,AC_VI);
		myEDCA[type_CCH]->createQueue(6,CWMIN_11P,CWMAX_11P,AC_BE);
		myEDCA[type_CCH]->createQueue(9,CWMIN_11P,CWMAX_11P,AC_BK);

		myEDCA[type_CCH]->maxQueueSize = 1024;

		myEDCA[type_SCH] = new EDCA(type_SCH,par("queueSize").longValue());
		myEDCA[type_SCH]->myId = myId;
		myEDCA[type_SCH]->myId.append(" SCH");
		myEDCA[type_SCH]->createQueue(2,(((CWMIN_11P+1)/4)-1),(((CWMIN_11P +1)/2)-1),AC_VO);
		myEDCA[type_SCH]->createQueue(3,(((CWMIN_11P+1)/2)-1),CWMIN_11P,AC_VI);
		myEDCA[type_SCH]->createQueue(6,CWMIN_11P,CWMAX_11P,AC_BE);
		myEDCA[type_SCH]->createQueue(9,CWMIN_11P,CWMAX_11P,AC_BK);

		myEDCA[type_SCH]->maxQueueSize = 1024;

		useSCH = par("useServiceChannel").boolValue();
		if (useSCH) {
			//set the initial service channel
			switch (par("serviceChannel").longValue()) {
				case 1: mySCH = Channels::SCH1; break;
				case 2: mySCH = Channels::SCH2; break;
				case 3: mySCH = Channels::SCH3; break;
				case 4: mySCH = Channels::SCH4; break;
				default: opp_error("Service Channel must be between 1 and 4"); break;
			}
		}

		headerLength = par("headerLength");

		nextMacEvent = new cMessage("next Mac Event");

		if (useSCH) {
			// introduce a little asynchronization between radios, but no more than .3 milliseconds
			uint64_t currenTime = simTime().raw();
			uint64_t switchingTime = SWITCHING_INTERVAL_11P.raw();
			// CCI is 100ms
			switchingTime = CCI.raw();

			double timeToNextSwitch = (double)(switchingTime
							   - (currenTime % switchingTime)) / simTime().getScale();
			if ((currenTime / switchingTime) % 2 == 0) {
				setActiveChannel(type_CCH);
			}
			else {
//				setActiveChannel(type_SCH);
				setActiveChannel(type_CCH);
			}

			countCCI = currenTime / switchingTime;
//			cout << "NEW! " << countCCI << endl;
			// channel switching active
			nextChannelSwitch = new cMessage("CCI Interval Start Event");
			simtime_t offset = dblrand() * 0.0005;

			scheduleAt(simTime() + offset + timeToNextSwitch, nextChannelSwitch);
		}
		else {
			// no channel switching
			nextChannelSwitch = 0;
			setActiveChannel(type_CCH);
		}


        findHost()->subscribe(mobilityStateChangedSignal, this);

        /*set channel to CCH*/
        phy11p->changeListeningFrequency(frequency[Channels::CCH]);

        nextStatusMsgEvent = new cMessage("next Status Message Event");
//        simtime_t offset = dblrand() * par("syncOffset").doubleValue(); // syncOffset = 3ms
//        scheduleAt( simTime() + offset, nextStatusMsgEvent );



        mNodeID = findHost()->getFullName();

        mVehMob = (Veins::TraCIMobility*)getOwner()->getOwner()->findObject(
                					"veinsmobility", true);
        cm = (ConnectionManager*)getOwner()->getOwner()->getOwner()->findObject(
        					"connectionManager",true);
//        cout << cm->calcInterfDist() << endl; // 10mw:360.99m

        phyLayerPtr = FindModule<PhyLayer80211p*>::findSubModule(
                        getParentModule());

        emgPktSenderID = "";
        emgReceived = false;
        emgReceivedReply = false;

        mCHID = "";
        mCHBK = "";
        mClusterRole = lone;
        commRange = 300;

        mWSF = 0.0;
        mPreWSF = 0.0;
        mPreSpeed = 0.0;
        mActAcc = 0.0;
        // initial value is -2 because the predicted acc can be -1, 0, 1
        mPredictAcc = -2.0;
        mAlpha = 1.0;
        mGamma = 1.0;
        mLeaveCHTime = simTime();
        mCHChanID = 0;
        mCHWaitTime = 0.0;

        distriCount = 0;

        mbCCIDuration = false;

        mUpperLayerPktQueue.clear();

        nextTestEvent = new cMessage("next Test Event");
        nextTfEvent = new cMessage("next Tf Event");
        nextClusterFormEvent = new cMessage("next Clustering Form Event");
        nextCMUploadEvent = new cMessage("next CM Upload Event");
        nextChanChangeEvent = new cMessage("next Channel Switch Event");
//        nextPreCH1stMsgEvent = new cMessage("next Pre CH 1st Message Event");
        nextCH1stMsgEvent = new cMessage("next CH 1st Message Event");
        nextCH2ndMsgEvent = new cMessage("next CH 2nd Message Event");
        nextCH3rdMsgEvent = new cMessage("next CH 3rd Message Event");
        nextEmgMsgEvent = new cMessage("next Emg Message Event");
        nextPreCH2ndMsgEvent = new cMessage("next Prepare CH2nd Msg Event");
        nextForwardEmgMsgEvent = new cMessage("next Forward Emg Message Event");
        nextRepeatEmgMsgEvent = new cMessage("next Repeat Emg Message Event");
        nextDistriEmgMsgEvent = new cMessage("next Distribute Emg Message Event");

        if ( mNodeID.compare("node[107]") == 0 ){
//        	scheduleAt(simTime() + 1, nextTestEvent);
        }


        if ( simTime() > 0.2 )
        	g_bNewVeh = true;

        m_pReEmgPkt = nullptr;
        m_pForEmgPkt = nullptr;
        m_sLastEmgSender.clear();

		//stats
		statsReceivedPackets = 0;
		statsReceivedBroadcasts = 0;
		statsSentPackets = 0;
		statsTXRXLostPackets = 0;
		statsSNIRLostPackets = 0;
		statsDroppedPackets = 0;
		statsNumTooLittleTime = 0;
		statsNumInternalContention = 0;
		statsNumBackoff = 0;
		statsSlotsBackoff = 0;
		statsTotalBusyTime = 0;

		idleChannel = true;
		lastBusy = simTime();
		channelIdle(true);
	}
}

void DMMAC::handleSelfMsg(cMessage* msg) {
	if (msg == nextChannelSwitch) {
		ASSERT(useSCH);

//		if ( chris_dmmac ){
//			cout << "mPos: " << mVehMob->getSumoPosition() << ", omnet Pos: " << mVehMob->getCurrentPosition() << endl;
//		}

		if ( chris_dmmac_4){
			std::list<std::string> rIdL = mVehMob->getPlannedEdgeIdList();

			for (auto tt: rIdL){
				cout << tt << " ";
			}
			cout << endl;
		}

		if ( nextCH1stMsgEvent->isScheduled() )
			cancelEvent(nextCH1stMsgEvent);

		if ( nextCH2ndMsgEvent->isScheduled() )
			cancelEvent(nextCH2ndMsgEvent);

		if ( nextPreCH2ndMsgEvent->isScheduled())
			cancelEvent(nextPreCH2ndMsgEvent);

		if ( nextCH3rdMsgEvent->isScheduled() )
			cancelEvent(nextCH3rdMsgEvent);

		if ( nextEmgMsgEvent->isScheduled() )
			cancelEvent(nextEmgMsgEvent);

		if ( nextForwardEmgMsgEvent->isScheduled())
			cancelEvent(nextForwardEmgMsgEvent);

		if ( nextRepeatEmgMsgEvent->isScheduled())
			cancelEvent(nextRepeatEmgMsgEvent);

		if ( nextDistriEmgMsgEvent->isScheduled())
			cancelEvent(nextDistriEmgMsgEvent);

		if ( getOffsetRatio(mVehMob) < 0.98 ){
			if ( mVehMob->getPlannedEdgeIdList().front().compare(mVehMob->getCurrentEdgeId()) == 0 ){
				/* channel switching modified for DMMAC */
		//		scheduleAt(simTime() + SWITCHING_INTERVAL_11P, nextChannelSwitch);
				scheduleAt(simTime() + CCI, nextChannelSwitch);
				countCCI++;
				DBG_MAC << " CCI Count: " << countCCI << endl;

				if ( chris_dmmac )
					cout << "Time[" << simTime() << "] CCI Count: " << countCCI << endl;
				// centralized

				// initial sampling speed and time
				if (countCCI == 2){
					mPreSpeed = mVehMob->getSpeed();
					mPreSpeedSampleTime = simTime();
				}

				if ( countCCI >= 10 ){

					distriCount = 0;
					m_sLastEmgSender.clear();

					std::map<std::string, cModule*> allVehMap = mVehMob->getManager()->getManagedHosts();

					// reset transmission power
					par("txPower").setDoubleValue(10);
					txPower = par("txPower");
					// connection manager
					cModule* cmModule = check_and_cast<cModule*>(cm);
					cmModule->par("pMax").setDoubleValue(10);

					phyLayerPtr->par("maxTXPower").setDoubleValue(5);
					phyLayerPtr->setMaxTXPower(5);

					if ( mobilityUpdateTime != simTime() ){
						mobilityUpdateTime = simTime();
						mVehMob->changePosition();
					}

					DBG_MAC << "CH: " << mNodeID << " alters TX power: "<< txPower << ", Inter: "
							<< cm->calcInterfDist() << ", Connect Size: "
							<< cm->getGateList(getParentModule()->getId() ).size() << endl;
					DBG_MAC << "Phy: " << mNodeID << " alters Phy TX power: "
							<< phyLayerPtr->getMaxTXPower() << endl;

					// every 1s, do Tf and cluster maintenance
					if ( countCCI % 10 == 0 ){
						vehStatusMap.clear();

		//				if ( mNodeID.compare(vehMinOffset) == 0 ){
						if ( gFlagClustering == false ){

		//					startTf();
							startTf(allVehMap);
							gFlagClustering = true;
						}
					}
					else if ( g_bNewVeh == true ){
		//				startTf();
						startTf(allVehMap);
						g_bNewVeh = false;
					}

		//			cout << simTime() << " countCCI: " << countCCI << endl;
					mbCCIDuration = true;

					if ( countCCI % 9 == 0 ){
						gFlagClustering = false;
					}

					// cancel status message event;
					if ( nextStatusMsgEvent->isScheduled())
						cancelEvent(nextStatusMsgEvent);

		//			cout << "SSS activeChannel " << activeChannel << ", " << type_SCH
		//					<< ", "<<  mNodeID << endl;

					if ( activeChannel == type_SCH ){
						scheduleAt( simTime(), nextChanChangeEvent );
						if ( chris_dmmac )
							cout << mNodeID << ", activeChannel: " << activeChannel
								<< ": ChanChange: " << simTime() << ", msg Name: START" << endl;
					}

		//			mCHMap.clear();
		//			cout << mNodeID << ": " << mClusterRole << endl;

					if ( mClusterRole == CH ){
						if ( chris_dmmac )
							cout << mNodeID << " : call 1st MsgEvent " << endl;

						if ( chris_dmmac_3 ){
						}

						scheduleAt( simTime() + CHANNELSWITCHGUARD, nextCH1stMsgEvent );
					}

		//			scheduleAt( simTime(), nextPreCH1stMsgEvent);

				}
			}
		}
	}
	else if (msg ==  nextMacEvent) {

		//we actually came to the point where we can send a packet
		channelBusySelf(true);
		CMACWSM* pktToSend = myEDCA[activeChannel]->initiateTransmit(lastIdle);

		lastAC = mapPriority(pktToSend->getPriority());

		DBG_MAC << "MacEvent received. Trying to send packet with priority " << lastAC << std::endl;

		//send the packet
		Mac80211Pkt* mac = new Mac80211Pkt(pktToSend->getName(), pktToSend->getKind());
		mac->setDestAddr(LAddress::L2BROADCAST);
		mac->setSrcAddr(myMacAddress);
		mac->encapsulate(pktToSend->dup());

		simtime_t sendingDuration = RADIODELAY_11P + getFrameDuration(mac->getBitLength());
		DBG_MAC << "Sending duration will be" << sendingDuration << std::endl;
		if ((!useSCH) || (timeLeftInSlot() > sendingDuration)) {
			if (useSCH) DBG_MAC << " Time in this slot left: " << timeLeftInSlot() << std::endl;
			// give time for the radio to be in Tx state before transmitting
			phy->setRadioState(Radio::TX);

			double freq = (activeChannel == type_CCH) ? frequency[Channels::CCH] : frequency[mySCH];

			attachSignal(mac, simTime()+RADIODELAY_11P, freq);
			MacToPhyControlInfo* phyInfo = dynamic_cast<MacToPhyControlInfo*>(mac->getControlInfo());
			assert(phyInfo);
			DBG_MAC << "Sending a Packet. Frequency " << freq << " Priority " << lastAC << std::endl;
			sendDelayed(mac, RADIODELAY_11P, lowerLayerOut);
			statsSentPackets++;
		}
		else {   //not enough time left now
			DBG_MAC << "Too little Time left. This packet cannot be send in this slot." << std::endl;
			statsNumTooLittleTime++;
			//revoke TXOP
			myEDCA[activeChannel]->revokeTxOPs();
			delete mac;
			channelIdle();
			//do nothing. contention will automatically start after channel switch
		}
	}
	else if ( msg == nextStatusMsgEvent ){

		int pktSizeinBit = 144;
		int prio = 1;
		int type = 0;
		int chaID = 0;
//		sendBroadcastPacket(prepareStatusMsg());
		handleMACLayerMsg(dynamic_cast<cMessage*>(prepareStatusMsg(	"status",
																	pktSizeinBit,
																	type_CCH,
																	prio,
																	type,
																	chaID)) );

        simtime_t offset = dblrand() * par("syncOffset").doubleValue(); // syncOffset = 3ms
        scheduleAt( simTime() + STATUSMSGINTERVAL + offset, nextStatusMsgEvent );

        // update my preSpeed
        if ( mPreSpeed == 0.0 )
        	mActAcc = 0.0;
        else
        	mActAcc = (mVehMob->getSpeed()-mPreSpeed)/(double)(simTime().dbl()-mPreSpeedSampleTime.dbl());

        mPreSpeed = mVehMob->getSpeed();
        mPreSpeedSampleTime = simTime();

	}
	else if ( msg == nextTestEvent ){
		cout << simTime() <<": Test ID: " << mNodeID << ", " << mVehMob->getSpeed() << ", "
				<< mVehMob->getCurrentEdgeId() << endl;

		scheduleAt(simTime()+1, nextTestEvent);
	}
	else if ( msg == nextTfEvent ){

	}
	else if ( msg == nextClusterFormEvent ){

	}
	else if ( msg == nextCMUploadEvent ){
		// AC1
		if ( mCHChanID == 0 )
			opp_error("nextCMUploadEvent: mCHChanID is 0, not SCH!");

		int pktSizeinBit = 144;
		int acPriority = 1;
		int pktType = 0;

//		CMACWSM* pktToSend = prepareStatusMsg(	"status",
//												pktSizeinBit,
//												type_SCH,
//												acPriority,
//												pktType,
//												mCHChanID);

		CMACWSM* pktToSend = prepareStatusMsg(	"status",
												pktSizeinBit,
												type_CCH,
												acPriority,
												pktType,
												mCHChanID);

//		handleMACLayerMsg( dynamic_cast<cMessage*>(pktToSend) );

		sendBroadcastPacket(pktToSend);
		// CMs leave CCI duration
		mbCCIDuration = false;

	}
	else if ( msg == nextChanChangeEvent ){

//		if (idleChannel){
			switch (activeChannel) {
				case type_CCH:
					DBG_MAC << "CCH --> SCH" << mySCH << ", "<< mNodeID << std::endl;
					if ( chris_dmmac )
						cout << "CCH --> SCH" << mySCH << ", "<< mNodeID << std::endl;
					channelBusySelf(false);
					setActiveChannel(type_SCH);
					channelIdle(true);
					phy11p->changeListeningFrequency(frequency[mySCH]);
					break;
				case type_SCH:
					DBG_MAC << "SCH --> CCH" << ", "<< mNodeID << std::endl;
					if ( chris_dmmac )
						cout << "SCH --> CCH" << ", "<< mNodeID << std::endl;
					channelBusySelf(false);
					setActiveChannel(type_CCH);
					channelIdle(true);
					phy11p->changeListeningFrequency(frequency[Channels::CCH]);
					break;
			}
//		}
//		else {
//			scheduleAt(simTime()+0.0001, nextChanChangeEvent);
//			DBG_MAC << " Change Channel is delayed 100us due to current channel is not idle!! " << endl;
//			cout << " Change Channel is delayed 100us due to current channel is not idle!! " << endl;
//		}

	}
	else if ( msg == nextPreCH1stMsgEvent ){


	}
	else if ( msg == nextCH1stMsgEvent ){
		int chanID;

		// decide channel
		if ( mCHMap.size() > 0 ){
			std::map<string, CMInfo> frontCHMap;
			std::map<string, CMInfo> rearCHMap;
			// find all front and rear CHs, chose a channel different from them
			for ( auto i: mCHMap){
				// x axis moving, rad is between  (-45, 45),
				// or (135, -135)
				double angle = mVehMob->getAngleRad();
				if ( angle < 0.785 && angle > -0.785 ) {
					if (i.second.pos.x > mVehMob->getSumoPosition().x){
						frontCHMap.insert(i);
					}
					else {
						rearCHMap.insert(i);
					}
				}
				else if ( angle > 2.356 && angle < -2.356 ){
					if (i.second.pos.x < mVehMob->getSumoPosition().x){
						frontCHMap.insert(i);
					}
					else {
						rearCHMap.insert(i);
					}
				}
				// y axis moving, rad is between (45, 135) or (-45, -135)
				else if ( angle > -2.356 && angle < -0.785 ) {
					if (i.second.pos.y < mVehMob->getSumoPosition().y){
						frontCHMap.insert(i);
					}
					else {
						rearCHMap.insert(i);
					}
				}

				else if ( angle > 0.785 && angle < 2.356){
					if (i.second.pos.y > mVehMob->getSumoPosition().y){
						frontCHMap.insert(i);
					}
					else {
						rearCHMap.insert(i);
					}
				}
			}

			/*find the nearest front and rear CH*/
			double xMin = 0.0;
			double xMax = 0.0;
			string xMinID;
			string xMaxID;

			for ( auto j: frontCHMap ){
				if(xMin == 0.0){
					xMin = j.second.pos.x;
					xMinID = j.first;
				}
				else if (xMin > j.second.pos.x){
					xMin = j.second.pos.x;
					xMinID = j.first;
				}
			}

			for ( auto j: rearCHMap ){
				if(xMax == 0.0){
					xMax = j.second.pos.x;
					xMaxID = j.first;
				}
				else if (xMin < j.second.pos.x){
					xMax = j.second.pos.x;
					xMaxID = j.first;
				}
			}
//			cout << "AT4: " << xMinID << ", " << xMaxID <<  endl;

			if ( xMinID.empty() && xMaxID .empty() ){
				chanID = intuniform(1, 3);
			}
			else if ( xMinID.empty() ){
				do{
					chanID = intuniform(1, 3);
				}while ( chanID == mCHMap.at(xMaxID).chanID );
			}
			else if ( xMaxID.empty() ){
				do{
					chanID = intuniform(1, 3);
				}while (chanID == mCHMap.at(xMinID).chanID );
			}
			else {
				do{
					chanID = intuniform(1, 3);
				}while (chanID == mCHMap.at(xMinID).chanID
							|| chanID == mCHMap.at(xMaxID).chanID );
			}
		}
		else {
			chanID = intuniform(1, 3);
		}



		int pktSizeinBit = 144;
		int acPriority = 2;
		int pktType = 1;

		chanID = mCHChanID;

		CMACWSM* pktTosend = prepareCHConsoMsg(	"CH1stPkt",
												pktSizeinBit,
												type_CCH,
												acPriority,
												pktType,
												chanID);
		// using AC2, 1st msg;
//		handleMACLayerMsg(dynamic_cast<cMessage*>(pktTosend) );
//		if (idleChannel){
			sendBroadcastPacket(pktTosend);
//		}
//		else{
//			sendBroadcastPacket(pktTosend, RADIODELAY_11P + 0.0001);
//		}

		DBG << "CH " << mNodeID << " sends 1st CH message in Priority " << acPriority
				<< ", Type " << pktType  << ", Cluster Channel: " << chanID << endl;

		// synchronize with time slot position
//		t_access_category ac = mapPriority(pktTosend->getPriority());
//		simtime_t DIFS = myEDCA[activeChannel]->myQueues[ac].aifsn * SLOTLENGTH_11P + SIFS_11P;
//		simtime_t base = DIFS + lastIdle;
		// macEventTime includes current time
//		simtime_t macEventTime = simTime() - simtime_t().setRaw((simTime() - base).raw() % SLOTLENGTH_11P.raw()) + SLOTLENGTH_11P;

		// switch channel to SCH
//		mCHChanID = chanID;
		if (mCHChanID == 0)
			opp_error("nextCH1stMsgEvent : mCHChanID == 0");
		selectSCH(mCHChanID);

		// * 1.001 is to ensure channel switch is after the transmission
		simtime_t afterMacEventTime;

//		if ( idleChannel )
			afterMacEventTime = simTime() + getFrameDuration(pktTosend->getBitLength()) + RADIODELAY_11P;
//		else
//			afterMacEventTime = simTime() + getFrameDuration(pktTosend->getBitLength()) + RADIODELAY_11P + 0.0001;

		scheduleAt( afterMacEventTime, nextChanChangeEvent );
		if ( chris_dmmac )
			cout << mNodeID << " CH: ChanChange: " << simTime() << ", msg Name: CH1rdPkt" << endl;

//		cout << "nextCH2ndMsgEvent: " << afterMacEventTime + CHANNELSWITCHGUARD + 2 * TA << endl;
//		cout << "nextCH2ndMsgEvent: " << afterMacEventTime + CHANNELSWITCHGUARD + 0.02 << endl;

//		simtime_t afterMacEventTime = macEventTime + getFrameDuration(pktTosend->getBitLength()) + RADIODELAY_11P * 1.001;
//		scheduleAt( afterMacEventTime + CHANNELSWITCHGUARD + 0.02, nextChanChangeEvent );
		simtime_t cmSendingTime;
		if ( emgPktSenderID.empty() ){
			cmSendingTime = mCMMap.size() * ( getFrameDuration(pktTosend->getBitLength()) + RADIODELAY_11P );
		}
		else{
			cmSendingTime= 0.0;
		}
		if ( chris_dmmac )
			cout << " --------> cmSendingTime: " << cmSendingTime << endl;
		// 0.02s is CMs uploading duration.
		scheduleAt( afterMacEventTime + CHANNELSWITCHGUARD + cmSendingTime, nextPreCH2ndMsgEvent);

	}
	else if ( msg == nextPreCH2ndMsgEvent){

		// SCH -> CCH
//		cout << "activeChannel :" << activeChannel << endl;
//		if ( idleChannel == true ){
			// check the channel idle
		scheduleAt( simTime(), nextChanChangeEvent );
		if ( chris_dmmac )
			cout << mNodeID << " CH: ChanChange: " << simTime() << ", msg Name: CH2rdPkt" << endl;
//		}
//		else{
			// give 500us to deal with current receiving
//			scheduleAt( simTime() + 0.0005, nextChanChangeEvent);
//		}

		scheduleAt( simTime() + CHANNELSWITCHGUARD, nextCH2ndMsgEvent);
	}
	else if ( msg == nextCH2ndMsgEvent ){

		int pktSizeinBit = 144;
		int acPriority = 2;
		int pktType = 2;

		if ( mCHChanID == 0 ){
			opp_error("nextCH2ndMsgEvent: mCHChanID shall not be 0!");
		}
		// this pkt is sent in CCH
		CMACWSM* pktTosend = prepareCHConsoMsg(	"CH2ndPkt",
												pktSizeinBit,
												type_CCH,
												acPriority,
												pktType,
												mCHChanID);
		// using AC2, 2nd msg;
//		handleMACLayerMsg(dynamic_cast<cMessage*>(pktTosend) );

		sendBroadcastPacket(pktTosend);
		DBG << "CH " << mNodeID << " sends 2nd CH message in Priority " << acPriority
				<< ", Type " << pktType  << ", Cluster Channel: " << mCHChanID << endl;

		// synchronize with time slot position
//		t_access_category ac = mapPriority(pktTosend->getPriority());
//		simtime_t DIFS = myEDCA[activeChannel]->myQueues[ac].aifsn * SLOTLENGTH_11P + SIFS_11P;
//		simtime_t base = DIFS + lastIdle;
		// macEventTime includes current time
//		simtime_t macEventTime = simTime() - simtime_t().setRaw((simTime() - base).raw() % SLOTLENGTH_11P.raw()) + SLOTLENGTH_11P;

		simtime_t sendingDura = getFrameDuration(pktTosend->getBitLength());

		scheduleAt( simTime() + sendingDura + RADIODELAY_11P * 1.001, nextCH3rdMsgEvent );

	}
	else if ( msg == nextCH3rdMsgEvent ){
		// increase transmission power to 2.5R
		par("txPower").setDoubleValue(35);
		txPower = par("txPower");
		// connection manager
		cModule* cmModule = check_and_cast<cModule*>(cm);
		cmModule->par("pMax").setDoubleValue(35);

		phyLayerPtr->par("maxTXPower").setDoubleValue(17.5);
		phyLayerPtr->setMaxTXPower(17.5);

        if ( mobilityUpdateTime != simTime() ){
	        mobilityUpdateTime = simTime();
			mVehMob->changePosition();
        }

		DBG_MAC << "CH: " << mNodeID << " alters TX power: "<< txPower << ", Inter: "
				<< cm->calcInterfDist() << ", Connect Size: "
				<< cm->getGateList(getParentModule()->getId() ).size() << endl;
		DBG_MAC << "Phy: " << mNodeID << " alters Phy TX power: "
				<< phyLayerPtr->getMaxTXPower() << endl;

		// send CH 3rd packet
		int pktSizeinBit = 144;
		int acPriority = 2;
		int pktType = 3;

		if ( mCHChanID == 0 ){
			opp_error("nextCH3rdMsgEvent: mCHChanID shall not be 0!");
		}
		// this pkt is sent in CCH
		CMACWSM* pktTosend = prepareCH2CHMsg(	"CH3rdPkt",
												pktSizeinBit,
												type_CCH,
												acPriority,
												pktType,
												mCHChanID);

		if ( strcmp(pktTosend->getDestID(), "emergency") == 0 ){
			pktTosend->setNodeID(mNodeID.c_str());
			sendBroadcastPacket(pktTosend);
			if ( chris_dmmac )
				cout << mNodeID << " : sends an emg pkt." << endl;

			// repeat sending
//			nextRepeatEmgMsgEvent->setContextPointer(pktTosend);
			m_pReEmgPkt = pktTosend->dup();
			scheduleAt( simTime() + 0.001, nextRepeatEmgMsgEvent);

			DBG << "CH " << mNodeID << " sends 3rd CH EMG message in Priority " << acPriority
					<< ", Type " << pktType  << ", Cluster Channel: " << mCHChanID << endl;

			simtime_t sendingDura = getFrameDuration(m_pReEmgPkt->getBitLength());
			// switch channel to SCH

			if ( mCHChanID == 0 )
				opp_error("nextCH3rdMsgEvent: mCHChanID is 0");
			selectSCH(mCHChanID);

			// * 1.001 is to ensure channel switch is after the transmission
			simtime_t afterMacEventTime = simTime() + sendingDura + RADIODELAY_11P * 1.001 + CHANNELSWITCHGUARD;

			// add 0.01s waiting time for CH2CH communication in CCH
			if ( !nextChanChangeEvent->isScheduled())
				scheduleAt( afterMacEventTime + 0.01, nextChanChangeEvent );

			if ( chris_dmmac )
				cout << mNodeID << " CH: ChanChange: " << afterMacEventTime + 0.01 << ", msg Name: CH3rdPkt (emg Pkt)" << endl;
		}
		else {
			sendBroadcastPacket(pktTosend);

			DBG << "CH " << mNodeID << " sends 3rd CH message in Priority " << acPriority
					<< ", Type " << pktType  << ", Cluster Channel: " << mCHChanID << endl;

			// after transmission, switch channel from CCH to SCH
			// synchronize with time slot position
	//		t_access_category ac = mapPriority(pktTosend->getPriority());
	//		simtime_t DIFS = myEDCA[activeChannel]->myQueues[ac].aifsn * SLOTLENGTH_11P + SIFS_11P;
	//		simtime_t base = DIFS + lastIdle;
			// macEventTime includes current time
	//		simtime_t macEventTime = simTime() - simtime_t().setRaw((simTime() - base).raw() % SLOTLENGTH_11P.raw()) + SLOTLENGTH_11P;

			simtime_t sendingDura = getFrameDuration(pktTosend->getBitLength());
			// switch channel to SCH

			if ( mCHChanID == 0 )
				opp_error("nextCH3rdMsgEvent: mCHChanID is 0");
			selectSCH(mCHChanID);

			// * 1.001 is to ensure channel switch is after the transmission
			simtime_t afterMacEventTime = simTime() + sendingDura + RADIODELAY_11P * 1.001 + CHANNELSWITCHGUARD;

			// add 0.01s waiting time for CH2CH communication in CCH
			scheduleAt( afterMacEventTime + 0.01, nextChanChangeEvent );
			if ( chris_dmmac )
				cout << mNodeID << " CH: ChanChange: " << afterMacEventTime + 0.01 << ", msg Name: CH3rdPkt (not Emg)" << endl;

	//		cout << " nextChanChangeEvent : " << afterMacEventTime << ", curr: " << simTime() << endl;
			// CH leaves CCI duration
			mbCCIDuration = false;
		}
	}
	else if ( msg == nextEmgMsgEvent ){
		// increase transmission power to 2.5R
		par("txPower").setDoubleValue(10);
		txPower = par("txPower");
		// connection manager
		cModule* cmModule = check_and_cast<cModule*>(cm);
		cmModule->par("pMax").setDoubleValue(10);

		phyLayerPtr->par("maxTXPower").setDoubleValue(5);
		phyLayerPtr->setMaxTXPower(5);

		if ( mobilityUpdateTime != simTime() ){
			mobilityUpdateTime = simTime();
			mVehMob->changePosition();
		}

		DBG_MAC << "CH: " << mNodeID << " alters TX power: "<< txPower << ", Inter: "
				<< cm->calcInterfDist() << ", Connect Size: "
				<< cm->getGateList(getParentModule()->getId() ).size() << endl;
		DBG_MAC << "Phy: " << mNodeID << " alters Phy TX power: "
				<< phyLayerPtr->getMaxTXPower() << endl;

		// send CH 3rd packet
		int pktSizeinBit = 144;
		int acPriority = 3;
		int pktType = 3;

		if ( mCHChanID == 0 ){
			opp_error("nextEmgMsgEvent: mCHChanID shall not be 0!");
		}
		// this pkt is sent in CCH
//		if ( emgPktSenderID.empty() )
//			opp_error("emgPktSender is empty!");

		CMACWSM* pktTosend = prepareEmgMsg(	"emergency",
											pktSizeinBit,
											type_CCH,
											acPriority,
											pktType,
											mCHChanID,
											mNodeID);
		// using AC2, 3rd msg;
//		handleMACLayerMsg(dynamic_cast<cMessage*>(pktTosend) );

		sendBroadcastPacket(pktTosend);

		DBG << mNodeID << " sends EMG message in Priority " << acPriority
				<< ", Type " << pktType  << ", Cluster Channel: " << mCHChanID << endl;

		// after transmission, switch channel from CCH to SCH
		// synchronize with time slot position
//		t_access_category ac = mapPriority(pktTosend->getPriority());
//		simtime_t DIFS = myEDCA[activeChannel]->myQueues[ac].aifsn * SLOTLENGTH_11P + SIFS_11P;
//		simtime_t base = DIFS + lastIdle;
		// macEventTime includes current time
//		simtime_t macEventTime = simTime() - simtime_t().setRaw((simTime() - base).raw() % SLOTLENGTH_11P.raw()) + SLOTLENGTH_11P;

//		simtime_t sendingDura = getFrameDuration(pktTosend->getBitLength());
		// switch channel to SCH

//		if ( mCHChanID == 0 )
//			opp_error("nextEmgMsgEvent: mCHChanID is 0");
//		selectSCH(mCHChanID);
//
//		// * 1.001 is to ensure channel switch is after the transmission
//		simtime_t afterMacEventTime = simTime() + sendingDura + RADIODELAY_11P * 1.001;
//		scheduleAt( afterMacEventTime, nextChanChangeEvent );

	}
	else if ( msg == nextRepeatEmgMsgEvent ){

		if ( m_pReEmgPkt != nullptr ){
			if ( activeChannel == type_CCH ){
				if ( chris_dmmac_5 ){
					cout << mNodeID << ", idleChannel: " << idleChannel << ", nextRepeatEmgMsgEvent." << endl;
				}

				if ( idleChannel )
					sendBroadcastPacket(m_pReEmgPkt);

				if ( chris_dmmac ){
					cout << mNodeID << "--> CH repeat emg Pkt" << endl;
					cout << "-------> emg pkt sender: "<< m_pReEmgPkt->getEmgPktSender() << endl;
				}

				DBG << mNodeID << "--> CH repeat emg Pkt" << endl;
				DBG << "-------> emg pkt sender: "<< m_pReEmgPkt->getEmgPktSender() << endl;

				scheduleAt( simTime() + 0.001, nextRepeatEmgMsgEvent);
			}
			else if ( activeChannel == type_SCH ){
				sendBroadcastPacket(m_pReEmgPkt);
				if ( chris_dmmac ){
					cout << mNodeID << "--> CH repeat emg Pkt to its CMs in SCH " << mySCH <<  endl;
					cout << "-------> emg pkt sender: "<< m_pReEmgPkt->getEmgPktSender() << endl;
				}

				DBG << mNodeID << "--> CH repeat emg Pkt to its CMs in SCH " << mySCH <<  endl;
				DBG << "-------> emg pkt sender: "<< m_pReEmgPkt->getEmgPktSender() << endl;
			}
		}
		else{
			opp_error("m_pReEmgPkt is nullptr in nextRepeatEmgMsgEvent!");
		}
	}
	else if ( msg == nextForwardEmgMsgEvent ){

		if ( m_pForEmgPkt != nullptr ){
			if ( activeChannel == type_CCH ){

				if ( m_sLastEmgSender.empty() )
					m_sLastEmgSender = m_pForEmgPkt->getNodeID();

				if ( chris_dmmac ){
					cout << "########### " << m_pForEmgPkt->getNodeID() << " : " << m_sLastEmgSender << endl;
				}

				m_pForEmgPkt->setEmgPktLastSender(m_sLastEmgSender.c_str());
				m_pForEmgPkt->setNodeID(mNodeID.c_str());

				if ( chris_dmmac ){
					cout << mNodeID << "--> CH forwards emg Pkt" << endl;
					cout << "-------> emg pkt sender: "<< m_pForEmgPkt->getEmgPktSender() << endl;
				}

				if ( chris_dmmac_5 ){
					cout << mNodeID << ", idleChannel: " << idleChannel << ", nextForwardEmgMsgEvent." << endl;
				}

				if ( idleChannel )
					sendBroadcastPacket(m_pForEmgPkt);

				DBG << mNodeID << "--> CH forwards emg Pkt" << endl;
				DBG << "-------> emg pkt sender: "<< m_pForEmgPkt->getEmgPktSender() << endl;

				if ( nextCH2ndMsgEvent->isScheduled())
					cancelEvent(nextCH2ndMsgEvent);

				scheduleAt( simTime() + 0.001, nextForwardEmgMsgEvent );
			}
			else if ( activeChannel == type_SCH ) {
//				opp_error("nextForwardEmgMsgEvent is not in CCH channel");


			}

		}
		else {
			opp_error("m_pForEmgPkt is nullptr!");
		}
	}
	else if ( msg == nextDistriEmgMsgEvent ){

		if ( m_pReEmgPkt != nullptr ){
			if ( activeChannel == type_CCH ){
//				opp_error("CH distributing EMG pkt should done in SCH!");
				if ( nextDistriEmgMsgEvent->isScheduled())
					cancelEvent(nextDistriEmgMsgEvent);
			}
			else if ( activeChannel == type_SCH ){

				// increase transmission power to 2.5R
				par("txPower").setDoubleValue(10);
				txPower = par("txPower");
				// connection manager
				cModule* cmModule = check_and_cast<cModule*>(cm);
				cmModule->par("pMax").setDoubleValue(10);

				phyLayerPtr->par("maxTXPower").setDoubleValue(5);
				phyLayerPtr->setMaxTXPower(5);

		        if ( mobilityUpdateTime != simTime() ){
			        mobilityUpdateTime = simTime();
					mVehMob->changePosition();
		        }


				m_pReEmgPkt->setName("emergency");

				sendBroadcastPacket(m_pReEmgPkt);
				distriCount++;

				if ( distriCount <= 5 )
					scheduleAt( simTime() + 0.001, nextDistriEmgMsgEvent);
				else {
					if (chris_dmmac){
						cout << mNodeID<< ", distriCount reaches 5!" << endl;
					}
				}

				if ( chris_dmmac ){
					cout << mNodeID << "--> CH distributes EMG Pkt to CMs in SCH " << mySCH <<  endl;
					cout << "-------> emg pkt sender: "<< m_pReEmgPkt->getEmgPktSender() << endl;
				}

				DBG << mNodeID << "--> CH distributes EMG Pkt to CMs in SCH " << mySCH <<  endl;
				DBG << "-------> emg pkt sender: "<< m_pReEmgPkt->getEmgPktSender() << endl;
			}
		}
		else {
			opp_error("m_pForEmgPkt is nullptr in nextDistriEmgMsgEvent!");
		}
	}
}

void DMMAC::handleUpperControl(cMessage* msg) {
	assert(false);
}

void DMMAC::handleUpperMsg(cMessage* msg) {

	CMACWSM* thisMsg;
//	DMMACstatus* thisMsg_2;

	if ((thisMsg = dynamic_cast<CMACWSM*>(msg)) == NULL) {
		error("WaveMac only accepts WaveShortMessages");
	}

	thisMsg->setName("status");

//	if ( mbCCIDuration == false){

//		sendBroadcastPacket(thisMsg);
//		t_access_category ac;
//		t_channel chan;
//		int num;
//
//		ac = mapPriority(thisMsg->getPriority());
//
//		DBG_MAC << "Received a message from upper layer for channel "
//				<< thisMsg->getChannelNumber() << " Access Category (Priority):  "
//				<< ac << std::endl;
//
//		//rewrite SCH channel to actual SCH the Mac1609_4 is set to
//		if (thisMsg->getChannelNumber() == Channels::SCH1) {
//			ASSERT(useSCH);
//			thisMsg->setChannelNumber(mySCH);
//			chan = type_SCH;
//		}
//
//		//put this packet in its queue
//		if (thisMsg->getChannelNumber() == Channels::CCH) {
//			chan = type_CCH;
//		}
//
//		num = myEDCA[chan]->queuePacket(ac,thisMsg);
//
//		//packet was dropped in Mac
//		if (num == -1) {
//			statsDroppedPackets++;
//			return;
//		}
//
//		//if this packet is not at the front of a new queue we dont have to reevaluate times
//		DBG_MAC << "sorted packet into queue of EDCA " << chan << " this packet is now at position: " << num << std::endl;
//
//		if (chan == activeChannel) {
//			DBG_MAC << "this packet is for the currently active channel" << std::endl;
//		}
//		else {
//			DBG_MAC << "this packet is NOT for the currently active channel" << std::endl;
//		}
//
//		if (num == 1 && idleChannel == true && chan == activeChannel) {
//
//			simtime_t nextEvent = myEDCA[chan]->startContent(lastIdle,guardActive());
//
//			if (nextEvent != -1) {
//				if ((!useSCH) || (nextEvent <= nextChannelSwitch->getArrivalTime())) {
//					if (nextMacEvent->isScheduled()) {
//						cancelEvent(nextMacEvent);
//					}
//					scheduleAt(nextEvent,nextMacEvent);
//					DBG_MAC << "Updated nextMacEvent:" << nextMacEvent->getArrivalTime().raw() << std::endl;
//				}
//				else {
//					DBG_MAC << "Too little time in this interval. Will not schedule nextMacEvent" << std::endl;
//					//it is possible that this queue has an txop. we have to revoke it
//					myEDCA[activeChannel]->revokeTxOPs();
//					statsNumTooLittleTime++;
//				}
//			}
//			else {
//				cancelEvent(nextMacEvent);
//			}
//		}
//		if (num == 1 && idleChannel == false && myEDCA[chan]->myQueues[ac].currentBackoff == 0 && chan == activeChannel) {
//			myEDCA[chan]->backoff(ac);
//		}
//	}
//	else {

	    /*limit the size of pkt queue*/
		if ( strcmp(thisMsg->getDestID(), "emergency") != 0 ){
			if ( mUpperLayerPktQueue.length() < 1024){

				mUpperLayerPktQueue.insert(thisMsg);
				DBG_MAC << "Received a data message from upper layer, Q Length: "
						<< mUpperLayerPktQueue.length() << endl;
			}
			else {
	//	    	gPeQueueDelPkts++;
				delete(thisMsg);
				if ( chris_dmmac ){
					cout << mNodeID
							<< ": A data pkt rvced from upper is deleted!" << endl;
				}
				DBG_MAC << "Queue is full, delete the pkt, Q Length: "
						<< mUpperLayerPktQueue.length() << endl;
				/*
				 * TODO: remove expired pkts
				 * */
			}
		}
		else {
			if ( mUpperLayerEmgPktQueue.length() < 1024 ){
				mUpperLayerEmgPktQueue.insert(thisMsg);
				scheduleAt(simTime(), nextEmgMsgEvent);
				DBG_MAC << "Received a EMG message from upper layer, Q Length: "
						<< mUpperLayerEmgPktQueue.length() << endl;
			}
			else {
				delete(thisMsg);

				if ( chris_dmmac ){
					cout << mNodeID
							<< ": A EMG pkt rvced from upper is deleted!" << endl;
				}
				DBG_MAC << "EMG Queue is full, delete the pkt, Q Length: "
						<< mUpperLayerEmgPktQueue.length() << endl;
			}
		}
//	}
}

void DMMAC::handleLowerControl(cMessage* msg) {
	if (msg->getKind() == MacToPhyInterface::TX_OVER) {

		DBG_MAC << "Successfully transmitted a packet on " << lastAC << std::endl;
//		cout << "lowerControl: " << mNodeID << ", activeChannel: "<< activeChannel << endl;
		phy->setRadioState(Radio::RX);

		//message was sent
		//update EDCA queue. go into post-transmit backoff and set cwCur to cwMin
//		myEDCA[activeChannel]->postTransmit(lastAC);
		//channel just turned idle.
		//don't set the chan to idle. the PHY layer decides, not us.

		if (guardActive()) {
			opp_error("We shouldnt have sent a packet in guard!");
		}

		// DMMAC
//		cout << "TX_OVER: " << msg->getName() << ", "<< endl;
		// DMMAC end
	}
	else if (msg->getKind() == Mac80211pToPhy11pInterface::CHANNEL_BUSY) {
		channelBusy();
		DBG_MAC << "ChannelBusy from control msg of Phy Layer." << std::endl;
	}
	else if (msg->getKind() == Mac80211pToPhy11pInterface::CHANNEL_IDLE) {
		channelIdle();
	}
	else if (msg->getKind() == Decider80211p::BITERROR) {
		statsSNIRLostPackets++;
		DBG_MAC << "A packet was not received due to biterrors" << std::endl;
	}
	else if (msg->getKind() == Decider80211p::RECWHILESEND) {
		statsTXRXLostPackets++;
		DBG_MAC << "A packet was not received because we were sending while receiving" << std::endl;
	}
	else if (msg->getKind() == MacToPhyInterface::RADIO_SWITCHING_OVER) {
		DBG_MAC << "Phylayer said radio switching is done" << std::endl;
	}
	else if (msg->getKind() == BaseDecider::PACKET_DROPPED) {
		phy->setRadioState(Radio::RX);
		DBG_MAC << "Phylayer said packet was dropped" << std::endl;
	}
	else {
		DBG_MAC << "Invalid control message type (type=NOTHING) : name=" << msg->getName() << " modulesrc=" << msg->getSenderModule()->getFullPath() << "." << std::endl;
		assert(false);
	}
	delete msg;
}

void DMMAC::handleLowerMsg(cMessage* msg) {
	Mac80211Pkt* macPkt = static_cast<Mac80211Pkt*>(msg);
	ASSERT(macPkt);

//	CMACWSM*  wsm =  dynamic_cast<CMACWSM*>(macPkt->decapsulate());
	CMACWSM* statusMsg = dynamic_cast<CMACWSM*>(macPkt->decapsulate());

	long dest = macPkt->getDestAddr();

	DBG_MAC << "Received frame name= " << macPkt->getName()
	        << ", myState=" << " src=" << macPkt->getSrcAddr()
	        << " dst=" << macPkt->getDestAddr() << " myAddr="
	        << myMacAddress << std::endl;

	if (macPkt->getDestAddr() == myMacAddress) {
		DBG_MAC << "Received a data packet addressed to me." << std::endl;
		statsReceivedPackets++;
//		sendUp(wsm);
	}
	else if (dest == LAddress::L2BROADCAST) {
		statsReceivedBroadcasts++;

//		cout << mNodeID << " ---> " << statusMsg->getDestID() << endl;
		if ( getOffsetRatio(mVehMob) < 0.98 ){
			if ( mVehMob->getPlannedEdgeIdList().front().compare(mVehMob->getCurrentEdgeId()) == 0 ){
				if ( strcmp(statusMsg->getName(), "status") == 0 ){
					if ( statusMsg->getType() == 0 ){
						CMInfo entry;
						entry.beta = statusMsg->getBeta();
						entry.speed = statusMsg->getSpeed();
						entry.pos = statusMsg->getPosition();
						entry.acc = statusMsg->getAcc();
						entry.range = statusMsg->getRange();
						entry.CHID = statusMsg->getCHID();
						entry.CHBK = statusMsg->getCHBK();
						entry.timeStamp = statusMsg->getTimestamp();
						entry.ttl = statusMsg->getTTL(); // 10s
						entry.chanID = statusMsg->getChannelID();
						entry.destID = statusMsg->getDestID();
						entry.senderID = statusMsg->getNodeID();
						entry.CMNodeID = statusMsg->getNodeID();
						entry.serialNo = statusMsg->getSerial();

						if ( vehStatusMap.find(statusMsg->getNodeID()) == vehStatusMap.end() ){
							vehStatusMap.insert(std::pair<std::string, CMInfo>(statusMsg->getNodeID(), entry));
						}
						else{
							vehStatusMap.at(statusMsg->getNodeID()) = entry;
						}

						DBG_MAC << mNodeID << " vehStatusMap: " << endl;
						DBG_MAC << setw(10) << "ID" << " | "
								<< setw(9) << "betaWSF" << " | "
								<< setw(8) << "speed" << " | "
								<< setw(19) << "pos" << " | "
								<< setw(8) << "acc" << " | "
								<< setw(4) << "R"<< " | "
								<< setw(10) << "CHID" << " | "
								<< setw(10) << "CHBK" << " | "
								<< setw(14) << "timeStamp" << " | "
								<< setw(1) << "cID" << " | "
								<< setw(10) << "destID"<< " | "
								<< setw(10) << "senderID"<< " | "<< endl;
						for ( auto i: vehStatusMap){
							DBG_MAC << setw(10) << i.first << " | "
									<< setw(9) << i.second.beta << " | "
									<< setw(8) << i.second.speed << " | "
									<< setw(2) << i.second.pos << " | "
									<< setw(8) << i.second.acc << " | "
									<< setw(4) << i.second.range << " | "
									<< setw(10) << i.second.CHID << " | "
									<< setw(10) << i.second.CHBK << " | "
									<< setw(14) << i.second.timeStamp << " | "
									<< setw(3) << i.second.chanID << " | "
									<< setw(10) << i.second.destID << " | "
									<< setw(10) << i.second.senderID << " | "<< endl;
						}

						if ( mClusterRole == CH ){
							// in CCI start duration, receiving CM uploading packets
							if ( mbCCIDuration == true ){
								// the sender's CHID is my ID
								if ( strcmp(statusMsg->getCHID(), mNodeID.c_str()) == 0 ){
									// my vehStatusMap has the sender's ID
		//							if ( vehStatusMap.find(statusMsg->getNodeID()) != vehStatusMap.end() ){
									bool bMyCM = false;
									for ( auto i: mCMMap){
										if ( strcmp(i.first.c_str(),statusMsg->getNodeID()) == 0 ){
											bMyCM = true;
											if ( chris_dmmac ){
												cout << mNodeID << " CH, find my CM from CMUpload pkt " << statusMsg->getNodeID() << endl;
											}
											break;
										}
									}

									if ( bMyCM == true ){
										// 2*TA to schedule channel switch
										// update mCHWaitTime, 2 TA or 1+.5 TA
										mCHWaitTime = (1 + intuniform(0,1))*TA;

		//								if ( nextChanChangeEvent->isScheduled() ){
		//									cancelEvent(nextChanChangeEvent);
		//								}
		//								// Switch to CCH
		//								scheduleAt( simTime() + mCHWaitTime, nextChanChangeEvent );

		//								if ( nextCH2ndMsgEvent->isScheduled() )
		//									cancelEvent( nextCH2ndMsgEvent );

		//								scheduleAt( simTime() + mCHWaitTime + CHANNELSWITCHGUARD, nextCH2ndMsgEvent);

										// save CM information
										CMInfo CMEntry;
										CMEntry.beta = statusMsg->getBeta();
										CMEntry.speed = statusMsg->getSpeed();
										CMEntry.pos = statusMsg->getPosition();
										CMEntry.acc = statusMsg->getAcc();
										CMEntry.range = statusMsg->getRange();
										CMEntry.CHID = statusMsg->getCHID();
										CMEntry.CHBK = statusMsg->getCHBK();
										CMEntry.timeStamp = statusMsg->getTimestamp();
										CMEntry.ttl = statusMsg->getTTL(); // 10s
										CMEntry.chanID = statusMsg->getChannelID();
										CMEntry.destID = statusMsg->getDestID();
										CMEntry.CMNodeID = statusMsg->getNodeID();
										CMEntry.serialNo = statusMsg->getSerial();

										if ( mCMMap.find(statusMsg->getNodeID()) == mCMMap.end() ){
											mCMMap.insert(std::pair<std::string, CMInfo>(statusMsg->getNodeID(), CMEntry));
										}
										else{
											mCMMap.at(statusMsg->getNodeID()) = CMEntry;
										}

		//								cout << mNodeID << " f " << statusMsg->getNodeID() << endl;

										if ( strcmp(mNodeID.c_str(), statusMsg->getDestID()) == 0 ){
											sendUp(statusMsg);
										}
									}

								}
								else {
									// not my CM
								}
							}
							else {

								if ( strcmp(statusMsg->getNodeID(), mNodeID.c_str()) == 0 )
									sendUp(statusMsg);

								CMInfo CMEntry;
								CMEntry.beta = statusMsg->getBeta();
								CMEntry.speed = statusMsg->getSpeed();
								CMEntry.pos = statusMsg->getPosition();
								CMEntry.acc = statusMsg->getAcc();
								CMEntry.range = statusMsg->getRange();
								CMEntry.CHID = statusMsg->getCHID();
								CMEntry.CHBK = statusMsg->getCHBK();
								CMEntry.timeStamp = statusMsg->getTimestamp();
								CMEntry.ttl = statusMsg->getTTL(); // 10s
								CMEntry.chanID = statusMsg->getChannelID();
								CMEntry.destID = statusMsg->getDestID();
								CMEntry.senderID = statusMsg->getNodeID();
								CMEntry.CMNodeID = statusMsg->getNodeID();
								CMEntry.serialNo = statusMsg->getSerial();

								mRcvdRandomCMVec.push_back(CMEntry);
							}
						}
						else if ( mClusterRole == CM ){
							// in CCI start duration
							if ( mbCCIDuration == true ){
		//						if ( nextCMUploadEvent->isScheduled() ) {
		//							cancelEvent(nextCMUploadEvent);
		//
		//							double distCM2CM = mVehMob->getSumoPosition().distance(statusMsg->getPosition());
		//							double rangeCM = 75;
		//							simtime_t sendTime =  TA + TA/2.0*(1+distCM2CM/rangeCM);
		//
		//							scheduleAt( simTime() + sendTime, nextCMUploadEvent);
		//
		//						}
								if ( strcmp(mNodeID.c_str(), statusMsg->getDestID()) == 0 )
									sendUp(statusMsg);
							}
							else {
								/*
								// sender is my CH
								if ( strcmp(statusMsg->getNodeID(), mCHID.c_str()) == 0 ){
									// but sender itself is not a CH
									if ( strcmp(statusMsg->getCHID(), statusMsg->getNodeID()) != 0 ){
										mClusterRole = lone;
										mCHID.clear();
										mCHBK.clear();

										findHost()->getDisplayString().updateWith("r=0");
									}

									// tell backCH
									if ( strcmp(statusMsg->getCHBK(), "") != 0 ){
										if ( strcmp(statusMsg->getCHBK(), mNodeID.c_str()) == 0 ){
											mClusterRole = backCH;
											mCHBK = statusMsg->getCHBK();
										}
										else {
											mCHBK = statusMsg->getCHBK();
										}
									}
								}
								// sender is not my CH, but sender is a CH
								else if ( strcmp(statusMsg->getNodeID(), statusMsg->getCHID()) == 0 ){
										mCHID = statusMsg->getNodeID();
										mCHChanID = statusMsg->getChannelID();
								}
								*/
								if ( strcmp(mNodeID.c_str(), statusMsg->getDestID()) == 0 )
									sendUp(statusMsg);
							}
						}
					}
				}
				else if ( strcmp(statusMsg->getName(), "CH1stPkt") == 0 ){
					if ( statusMsg->getType() == 1){
						if ( mClusterRole == CM ){
							if ( strcmp(statusMsg->getNodeID(), mCHID.c_str()) == 0 ){
								DBG << mNodeID << " receives CH1st Msg from " << mCHID << endl;
		//						cout << mNodeID << " receives CH1st Msg from " << mCHID << endl;

		//						findHost()->getDisplayString().updateWith("r=6,white");
								// cancel current scheduled status message
								if ( nextStatusMsgEvent->isScheduled() )
									cancelEvent(nextStatusMsgEvent);

								simtime_t sendTime;
								if ( mVehMob->getSumoPosition().x > statusMsg->getPosition().x ){
									double dist = mVehMob->getSumoPosition().distance(statusMsg->getPosition());
									double range = statusMsg->getRange();
									range = 300;
									// equation (2), +d
									sendTime = TA + TA/2.0*(1+dist/range);
								}
								else {
									double dist = mVehMob->getSumoPosition().distance(statusMsg->getPosition());
									double range = statusMsg->getRange();
									range = 300;
									// equation (2), -d
									sendTime = TA + TA/2.0*(1-dist/range);
								}

		//						mCHChanID = statusMsg->getChannelID();

								if ( mCHChanID > 0 )
									selectSCH(mCHChanID);
								else
									opp_error("statusMsg type 1: mCHChanID is 0, should be 1-3!");

								// CCH --> SCH
								scheduleAt( simTime(), nextChanChangeEvent );
								if ( chris_dmmac ){
									cout << mNodeID << " CM: ChanChange: " << simTime() << " revd 1stCH pkt"<< endl;
								}
		//						scheduleAt( simTime() + CHANNELSWITCHGUARD + sendTime, nextCMUploadEvent);
		//						if ( statusMsg->getGatherCMArraySize() == 0 )
		//							opp_error("statusMsg->getGatherCMArraySize() is 0!");
								if ( chris_dmmac ){
									cout << "  -----> CMArraySize: "<< statusMsg->getGatherCMArraySize() << endl;
								}
		//						cout << " Sender: " << statusMsg->getNodeID() << endl;
		//						for (unsigned i =0; i < statusMsg->getGatherCMArraySize(); ++i){
		//							cout << statusMsg->getGatherCM(i).CMNodeID << endl;
		//						}

								for (unsigned i =0; i < statusMsg->getGatherCMArraySize(); ++i){
									if ( strcmp(mNodeID.c_str(), statusMsg->getGatherCM(i).CMNodeID.c_str()) == 0 ){
										if ( chris_dmmac ){
											cout << mNodeID << " receive CH1stMsg, my sending position " << i << endl;
										}
										sendTime = SIFS_11P + i * ( SIFS_11P + getFrameDuration(500) + RADIODELAY_11P * 2 );
										if ( chris_dmmac ){
											cout << " sendTime: " << sendTime << endl;
										}
										scheduleAt( simTime() + CHANNELSWITCHGUARD + sendTime , nextCMUploadEvent);
										DBG_MAC << mNodeID << " scheduled CM upload at "
												<< simTime() + CHANNELSWITCHGUARD + sendTime
												<< ", my CHID: " << mCHID << endl;
										break;
									}
								}

								if ( strcmp(mNodeID.c_str(), statusMsg->getDestID()) == 0 ){
									CMACWSM* pkt;
									pkt = statusMsg->dup();
									pkt->setName("status");
									sendUp(pkt);
								}
							}
						}
					}
					delete statusMsg;
				}
				else if ( strcmp(statusMsg->getName(), "CH2ndPkt") == 0 ){
					if ( statusMsg->getType() == 2 ){
						// invitation message, if I am lone or CM, I shall join
						if ( mClusterRole != CH && mClusterRole != backCH ){
							mCHID = statusMsg->getNodeID();
							mCHBK = statusMsg->getCHBK();
							mCHChanID = statusMsg->getChannelID();

							DBG << mNodeID << " receives CH2nd Msg from " << mCHID << endl;
		//					cout << mNodeID << " receives CH2nd Msg from " << mCHID << endl;

							// switch channel
							if ( mCHChanID > 0 )
								selectSCH(mCHChanID);
							else
								opp_error("statusMsg type 1: mCHChanID is 0, should be 1-3!");

							// CCH --> SCH
							scheduleAt( simTime(), nextChanChangeEvent );
							if ( chris_dmmac ){
								cout << mNodeID << " CM: ChanChange: " << simTime() << ", msg Name: "<< statusMsg->getName() << endl;
							}
							if ( strcmp(mNodeID.c_str(), statusMsg->getDestID()) == 0 ){
								CMACWSM* pkt;
								pkt = statusMsg->dup();
								pkt->setName("status");
								sendUp(pkt);
							}
						}
					}
					delete statusMsg;
				}
				else if ( strcmp(statusMsg->getName(), "CH3rdPkt") == 0){
					if ( statusMsg->getType() == 3 ){
						// CH2CH message
						if ( mClusterRole == CH ){
							if ( strcmp(statusMsg->getDestID(), "emergency") == 0 ){
								if ( chris_dmmac ){
									cout << " Emg Pkt last sender: " << statusMsg->getEmgPktLastSender() << endl;
								}
								// deal with the ack from other CHs
								if ( strcmp(statusMsg->getEmgPktLastSender(), mNodeID.c_str() ) == 0 ){
									// check if the distance is larger than 2km
									if ( mVehMob->getSumoPosition().distance(statusMsg->getSenderPos()) <= 2000 ){

										if ( nextRepeatEmgMsgEvent->isScheduled() ){
											cancelEvent(nextRepeatEmgMsgEvent);
										}

										if ( nextForwardEmgMsgEvent->isScheduled() ){
											cancelEvent(nextForwardEmgMsgEvent);
										}
										// switch channel to SCH
										if ( mCHChanID == 0 )
											opp_error("emgPktAck: mCHChanID is 0");
										selectSCH(mCHChanID);

										// after receiving forwarded emg pkt from other CH
										// continue broadcast the emg pkt to my own CMs
										// 1st: switch channel to my Cluster channel
										if ( nextChanChangeEvent->isScheduled() )
											cancelEvent(nextChanChangeEvent);

										scheduleAt( simTime(), nextChanChangeEvent );
										if ( chris_dmmac ){
											cout << mNodeID << " CH: ChanChange: " << simTime() << ", msg Name: emgPktAck, distribute to CMs" << endl;
										}
										m_pReEmgPkt = statusMsg->dup();
										// 2nd: broadcast emg pkt.
										scheduleAt( simTime() + CHANNELSWITCHGUARD + SIFS_11P, nextDistriEmgMsgEvent );

										CMACWSM* pkt = new CMACWSM("status");
										pkt->setTTL(statusMsg->getTTL());
										pkt->setTimestamp(statusMsg->getTimestamp());
										pkt->setDestID(statusMsg->getDestID());
										pkt->setNodeID(statusMsg->getNodeID());
										pkt->setSerial(statusMsg->getSerial());
										pkt->setEmgPktSender(statusMsg->getEmgPktSender());

										sendUp(pkt);

									}
									else {
										DBG_MAC << "Emg Pkt sent to a CH with distance larger than 2km! Stop Repeat!" << endl;

										CMACWSM* pkt = new CMACWSM("status");
										pkt->setTTL(statusMsg->getTTL());
										pkt->setTimestamp(statusMsg->getTimestamp());
										pkt->setDestID(statusMsg->getDestID());
										pkt->setNodeID(statusMsg->getNodeID());
										pkt->setSerial(statusMsg->getSerial());
										pkt->setEmgPktSender(statusMsg->getEmgPktSender());
										// reach 2km, set this flag for APP layer
										pkt->setEmgReach2kFlag(true);
										sendUp(pkt);
									}
								}
								// emg pkt from other CH.
								else {
									// check if the distance is larger than 2km
									if ( mVehMob->getSumoPosition().distance(statusMsg->getSenderPos()) <= 2000 ){
										if ( nextForwardEmgMsgEvent->isScheduled() ) {
											cancelEvent(nextForwardEmgMsgEvent);
										}

										if ( nextRepeatEmgMsgEvent->isScheduled() ){
											cancelEvent(nextRepeatEmgMsgEvent);
										}

										if ( chris_dmmac ){
											cout << mNodeID << "--> CH receives emg Pkt from CH " << statusMsg->getNodeID() << endl;
											cout << "-------> emg pkt sender: "<< statusMsg->getEmgPktSender() << endl;
										}
										m_pForEmgPkt = statusMsg->dup();

										// randomize the sending time for avoiding flooding
										scheduleAt(simTime() + SIFS_11P + dblrand() * 0.0008, nextForwardEmgMsgEvent);

										CMACWSM* pkt = new CMACWSM("status");
										pkt->setTTL(statusMsg->getTTL());
										pkt->setTimestamp(statusMsg->getTimestamp());
										pkt->setDestID(statusMsg->getDestID());
										pkt->setNodeID(statusMsg->getNodeID());
										pkt->setSerial(statusMsg->getSerial());
										pkt->setEmgPktSender(statusMsg->getEmgPktSender());
										sendUp(pkt);

									}
									else {
										DBG_MAC << "Emg Pkt sent to a CH with distance larger than 2km! Stop Forward!" << endl;
										if ( chris_dmmac ){
											cout << "CH: " << mNodeID << " stops forward Emg Pkt because distance reaches 2km! " << endl;
										}
										CMACWSM* pkt = new CMACWSM("status");
										pkt->setTTL(statusMsg->getTTL());
										pkt->setTimestamp(statusMsg->getTimestamp());
										pkt->setDestID(statusMsg->getDestID());
										pkt->setNodeID(statusMsg->getNodeID());
										pkt->setSerial(statusMsg->getSerial());
										pkt->setEmgPktSender(statusMsg->getEmgPktSender());
										// reach 2km, set this flag for APP layer
										pkt->setEmgReach2kFlag(true);

										sendUp(pkt);
									}
								}

							}
							// not EMG pkt
							else {
								CMInfo entryCH;
								entryCH.beta = statusMsg->getBeta();
								entryCH.speed = statusMsg->getSpeed();
								entryCH.pos = statusMsg->getPosition();
								entryCH.acc = statusMsg->getAcc();
								entryCH.range = statusMsg->getRange();
								entryCH.CHID = statusMsg->getCHID();
								entryCH.CHBK = statusMsg->getCHBK();
								entryCH.ttl = statusMsg->getTTL();
								entryCH.timeStamp = statusMsg->getTimestamp();
								entryCH.destID = statusMsg->getDestID();

								if ( mCHMap.find(statusMsg->getNodeID()) == mCHMap.end() ){
									mCHMap.insert(std::pair<std::string, CMInfo>(statusMsg->getNodeID(), entryCH));
								}
								else{
									mCHMap.at(statusMsg->getNodeID()) = entryCH;
								}

								for ( size_t i=0; i<statusMsg->getGatherCMArraySize(); ++i ){
									mRcvdOrCMVec.push_back(statusMsg->getGatherCM(i));
									if ( strcmp(mNodeID.c_str(), statusMsg->getGatherCM(i).destID.c_str() ) == 0 ){
										CMACWSM* pkt = new CMACWSM("status");
										pkt->setTTL(statusMsg->getTTL());
										pkt->setTimestamp(statusMsg->getTimestamp());
										pkt->setDestID(statusMsg->getGatherCM(i).destID.c_str());
										pkt->setNodeID(statusMsg->getGatherCM(i).senderID.c_str());
										pkt->setSerial(statusMsg->getGatherCM(i).serialNo);
										sendUp(pkt);
									}

								}

								for ( size_t i=0; i<statusMsg->getOrCMArraySize(); ++i){
									mRcvdOrCMVec.push_back(statusMsg->getOrCM(i));
									if ( strcmp(mNodeID.c_str(), statusMsg->getOrCM(i).destID.c_str() ) == 0 ){
										CMACWSM* pkt = new CMACWSM("status");
										pkt->setTTL(statusMsg->getTTL());
										pkt->setTimestamp(statusMsg->getTimestamp());
										pkt->setDestID(statusMsg->getOrCM(i).destID.c_str());
										pkt->setNodeID(statusMsg->getOrCM(i).senderID.c_str());
										pkt->setSerial(statusMsg->getOrCM(i).serialNo);

										sendUp(pkt);
									}
								}

								if ( strcmp(mNodeID.c_str(), statusMsg->getDestID()) == 0 ){
									CMACWSM* pkt;
									pkt = statusMsg->dup();
									pkt->setName("status");
									sendUp(pkt);
								}
							}

							delete statusMsg;
						}
						else if ( mClusterRole == CM ){
							if ( strcmp(statusMsg->getDestID(), "emergency") == 0 ){
								if ( chris_dmmac ){
									cout << mNodeID << ", CM receives Emg Pkt, the last sender: " << statusMsg->getEmgPktLastSender() << endl;
									cout << mCHID << ", " << statusMsg->getNodeID() << endl;
								}
								if ( strcmp( mCHID.c_str(), statusMsg->getNodeID() ) == 0 ){
									if ( strcmp( mNodeID.c_str(), statusMsg->getEmgPktSender()) != 0){
										CMACWSM* pkt = new CMACWSM("status");
										pkt->setTTL(statusMsg->getTTL());
										pkt->setTimestamp(statusMsg->getTimestamp());
										pkt->setDestID(statusMsg->getDestID());
										pkt->setNodeID(statusMsg->getNodeID());
										pkt->setSerial(statusMsg->getSerial());
										pkt->setEmgPktSender(statusMsg->getEmgPktSender());
										sendUp(pkt);
									}
								}
							}
							delete statusMsg;
						}
					}
				}
				else if ( strcmp(statusMsg->getName(), "emergency") == 0 ){
		//			cout << mNodeID << " : CH rvcd emg 0" << endl;
					if ( mClusterRole == CH ){
						if ( chris_dmmac ){
							cout << mNodeID << " : CH rvcd emg 1" << endl;
						}
						// receive an emergency pkt from my CM
						if ( strcmp(statusMsg->getCHID(), mNodeID.c_str()) == 0 ){
							if ( chris_dmmac ){
								cout << mNodeID << " : CH rvcd emg 2" << endl;
							}
							emgPktSenderID = statusMsg->getNodeID();
		//					if ( emgReceivedReply == false ){

							// forward the same packet received from CM
							m_pForEmgPkt = statusMsg->dup();

							mUpperLayerEmgPktQueue.insert(m_pForEmgPkt);

							CMACWSM* pkt = new CMACWSM("status");
							pkt->setTTL(statusMsg->getTTL());
							pkt->setTimestamp(statusMsg->getTimestamp());
							pkt->setDestID(statusMsg->getDestID());
							pkt->setNodeID(statusMsg->getNodeID());
							pkt->setSerial(statusMsg->getSerial());
							pkt->setEmgPktSender(statusMsg->getEmgPktSender());
							sendUp(pkt);

		//					sendUp(statusMsg);
							if ( chris_dmmac ){
								cout << mNodeID << "--> CH receives emg Pkt from CM " << statusMsg->getNodeID() << endl;
							}

						}
						else {
							// receive an emergency pkt from another CH
							if ( emgReceived == false ){
								emgReceived = true;
								if ( chris_dmmac ){
									cout << mNodeID << "--> CH receives emg Pkt from another CH " << statusMsg->getNodeID() << endl;
								}
								CMACWSM* pkt = new CMACWSM("status");
								pkt->setTTL(statusMsg->getTTL());
								pkt->setTimestamp(statusMsg->getTimestamp());
								pkt->setDestID(statusMsg->getDestID());
								pkt->setNodeID(statusMsg->getNodeID());
								pkt->setSerial(statusMsg->getSerial());
								pkt->setEmgPktSender(statusMsg->getEmgPktSender());
								sendUp(pkt);

		//						sendUp(statusMsg);
		//						emgPktSenderID = statusMsg->getEmgPktSender();
		//						scheduleAt(simTime(), nextEmgMsgEvent);
								if ( emgReceivedReply == false )
									emgReceivedReply = true;
							}
						}
						delete statusMsg;
					}
					// I'm a CM
					else if (mClusterRole == CM) { // CM refrain to sending
						// CM receives EMG pkt from its CH
						if ( strcmp( mCHID.c_str(), statusMsg->getNodeID() ) == 0 ){
							if ( chris_dmmac ){
								cout << mNodeID << ", CM receives Emg Pkt from its CH, the last sender: " << statusMsg->getEmgPktLastSender() << endl;
							}
							CMACWSM* pkt = new CMACWSM("status");
							pkt->setTTL(statusMsg->getTTL());
							pkt->setTimestamp(statusMsg->getTimestamp());
							pkt->setDestID(statusMsg->getDestID());
							pkt->setNodeID(statusMsg->getNodeID());
							pkt->setSerial(statusMsg->getSerial());
							pkt->setEmgPktSender(statusMsg->getEmgPktSender());
							sendUp(pkt);
						}
						// CM receives a EMG pkt from others, a CM or a CH? refrain my sending
						else {
							if ( chris_dmmac ){
								cout << mNodeID << ", CM receives Emg Pkt from " << statusMsg->getNodeID() << endl;
							}
							CMACWSM* pkt = new CMACWSM("status");
							pkt->setTTL(statusMsg->getTTL());
							pkt->setTimestamp(statusMsg->getTimestamp());
							pkt->setDestID(statusMsg->getDestID());
							pkt->setNodeID(statusMsg->getNodeID());
							pkt->setSerial(statusMsg->getSerial());
		//					pkt->setEmgPktSender(statusMsg->getEmgPktSender());
							sendUp(pkt);

							if ( nextCMUploadEvent->isScheduled())
							    cancelEvent(nextCMUploadEvent);
						}
						delete statusMsg;
					}
				}
				// not used
				else if ( strcmp(statusMsg->getName(), "emgPktAck") == 0 ){

				}
				else {
					DBG_MAC << "Packet is not recognized, deleting..." << std::endl;
					delete statusMsg;
				}
		//		sendUp(wsm);
			}
		}
	}
	else {
		DBG_MAC << "Packet not for me, deleting..." << std::endl;
//		delete wsm;
		delete statusMsg;
	}
	delete macPkt;
}

void DMMAC::setActiveChannel(t_channel state) {
	activeChannel = state;
	assert(state == type_CCH || (useSCH && state == type_SCH));
}

void DMMAC::finish() {
	//clean up queues.

	for (std::map<t_channel,EDCA*>::iterator iter = myEDCA.begin(); iter != myEDCA.end(); iter++) {
		statsNumInternalContention += iter->second->statsNumInternalContention;
		statsNumBackoff += iter->second->statsNumBackoff;
		statsSlotsBackoff += iter->second->statsSlotsBackoff;
		iter->second->cleanUp();
		delete iter->second;
	}

	myEDCA.clear();

	if (nextMacEvent->isScheduled()) {
		cancelAndDelete(nextMacEvent);
	}
	else {
		delete nextMacEvent;
	}
	if (nextChannelSwitch && nextChannelSwitch->isScheduled())
		cancelAndDelete(nextChannelSwitch);

	if ( nextStatusMsgEvent->isScheduled() )
		cancelAndDelete(nextStatusMsgEvent);
	else
		delete nextStatusMsgEvent;

	if ( nextClusterFormEvent->isScheduled() )
		cancelAndDelete(nextClusterFormEvent);
	else
		delete nextClusterFormEvent;

	if ( nextTfEvent->isScheduled() )
		cancelAndDelete(nextTfEvent);
	else
		delete nextTfEvent;

	if ( nextCMUploadEvent->isScheduled() )
		cancelAndDelete(nextCMUploadEvent);
	else
		delete nextCMUploadEvent;



	if ( nextChanChangeEvent->isScheduled() )
		cancelAndDelete(nextChanChangeEvent);
	else
		delete nextChanChangeEvent;

	if ( nextCH1stMsgEvent->isScheduled() )
		cancelAndDelete(nextCH1stMsgEvent);
	else
		delete nextCH1stMsgEvent;

	if ( nextCH2ndMsgEvent->isScheduled() )
		cancelAndDelete(nextCH2ndMsgEvent);
	else
		delete nextCH2ndMsgEvent;

	if ( nextCH3rdMsgEvent->isScheduled() )
		cancelAndDelete(nextCH3rdMsgEvent);
	else
		delete nextCH3rdMsgEvent;


	if ( nextEmgMsgEvent->isScheduled() )
		cancelAndDelete(nextEmgMsgEvent);
	else
		delete nextEmgMsgEvent;

	if ( nextPreCH2ndMsgEvent->isScheduled() )
		cancelAndDelete(nextPreCH2ndMsgEvent);
	else
		delete nextPreCH2ndMsgEvent;

	if ( nextRepeatEmgMsgEvent->isScheduled() )
		cancelAndDelete(nextRepeatEmgMsgEvent);
	else
		delete nextRepeatEmgMsgEvent;

	if ( nextForwardEmgMsgEvent->isScheduled())
		cancelAndDelete(nextForwardEmgMsgEvent);
	else
		delete nextForwardEmgMsgEvent;

	if ( nextTestEvent->isScheduled() )
		cancelAndDelete(nextTestEvent);
	else
		delete nextTestEvent;

	if ( nextDistriEmgMsgEvent->isScheduled() )
		cancelAndDelete(nextDistriEmgMsgEvent);
	else
		delete nextDistriEmgMsgEvent;

	findHost()->unsubscribe(mobilityStateChangedSignal, this);

	//stats
	recordScalar("ReceivedUnicastPackets",statsReceivedPackets);
	recordScalar("ReceivedBroadcasts",statsReceivedBroadcasts);
	recordScalar("SentPackets",statsSentPackets);
	recordScalar("SNIRLostPackets",statsSNIRLostPackets);
	recordScalar("RXTXLostPackets",statsTXRXLostPackets);
	recordScalar("TotalLostPackets",statsSNIRLostPackets+statsTXRXLostPackets);
	recordScalar("DroppedPacketsInMac",statsDroppedPackets);
	recordScalar("TooLittleTime",statsNumTooLittleTime);
	recordScalar("TimesIntoBackoff",statsNumBackoff);
	recordScalar("SlotsBackoff",statsSlotsBackoff);
	recordScalar("NumInternalContention",statsNumInternalContention);
	recordScalar("totalBusyTime",statsTotalBusyTime.dbl());

}

void DMMAC::attachSignal(Mac80211Pkt* mac, simtime_t startTime, double frequency) {

	simtime_t duration = getFrameDuration(mac->getBitLength());

	Signal* s = createSignal(startTime, duration, txPower, bitrate, frequency);
	if ( chris_dmmac ){
//		cout << "attachSignal: txPower:" << txPower << endl;
	}

	MacToPhyControlInfo* cinfo = new MacToPhyControlInfo(s);

	mac->setControlInfo(cinfo);
}

Signal* DMMAC::createSignal(simtime_t start, simtime_t length, double power, double bitrate, double frequency) {
	simtime_t end = start + length;
	//create signal with start at current simtime and passed length
	Signal* s = new Signal(start, length);

	//create and set tx power mapping
	ConstMapping* txPowerMapping = createSingleFrequencyMapping(start, end, frequency, 5.0e6, power);
	s->setTransmissionPower(txPowerMapping);

	Mapping* bitrateMapping = MappingUtils::createMapping(DimensionSet::timeDomain, Mapping::STEPS);

	Argument pos(start);
	bitrateMapping->setValue(pos, bitrate);

	pos.setTime(phyHeaderLength / bitrate);
	bitrateMapping->setValue(pos, bitrate);

	s->setBitrate(bitrateMapping);

	return s;
}

/* checks if guard is active */
bool DMMAC::guardActive() const {
	// for DMMAC
	return false;
	if (!useSCH) return false;
	if (simTime().dbl() - nextChannelSwitch->getSendingTime() <= GUARD_INTERVAL_11P)
		return true;
	return false;
}

/* returns the time until the guard is over */
simtime_t DMMAC::timeLeftTillGuardOver() const {
	ASSERT(useSCH);
	simtime_t sTime = simTime();
	if (sTime - nextChannelSwitch->getSendingTime() <= GUARD_INTERVAL_11P) {
		return GUARD_INTERVAL_11P
		       - (sTime - nextChannelSwitch->getSendingTime());
	}
	else
		return 0;
}

/* returns the time left in this channel window */
simtime_t DMMAC::timeLeftInSlot() const {
	ASSERT(useSCH);
	return nextChannelSwitch->getArrivalTime() - simTime();
}

/* Will change the Service Channel on which the mac layer is listening and sending */
void DMMAC::changeServiceChannel(int cN) {
	ASSERT(useSCH);
	if (cN != Channels::SCH1 && cN != Channels::SCH2 && cN != Channels::SCH3 && cN != Channels::SCH4) {
		opp_error("This Service Channel doesnt exit: %d",cN);
	}

	mySCH = cN;

	if (activeChannel == type_SCH) {
		//change to new chan immediately if we are in a SCH slot,
		//otherwise it will switch to the new SCH upon next channel switch
		phy11p->changeListeningFrequency(frequency[mySCH]);
	}
}



int DMMAC::EDCA::queuePacket(t_access_category ac,CMACWSM* msg) {

	if (maxQueueSize && myQueues[ac].queue.size() >= maxQueueSize) {
		delete msg;
		return -1;
	}

	myQueues[ac].queue.push(msg);
	return myQueues[ac].queue.size();
}

int DMMAC::EDCA::createQueue(int aifsn, int cwMin, int cwMax,t_access_category ac) {

	if (myQueues.find(ac) != myQueues.end()) {
		opp_error("You can only add one queue per Access Category per EDCA subsystem");
	}

	EDCAQueue newQueue(aifsn,cwMin,cwMax,ac);
	myQueues[ac] = newQueue;

	return ++numQueues;
}

DMMAC::t_access_category DMMAC::mapPriority(int prio) {
	//dummy mapping function
	switch (prio) {
		case 0: return AC_BK;
		case 1: return AC_BE;
		case 2: return AC_VI;
		case 3: return AC_VO;
		default: opp_error("MacLayer received a packet with unknown priority"); break;
	}
	return AC_VO;
}

CMACWSM* DMMAC::EDCA::initiateTransmit(simtime_t lastIdle) {

	//iterate through the queues to return the packet we want to send
	CMACWSM* pktToSend = NULL;

	simtime_t idleTime = simTime() - lastIdle;

	DBG_MAC << "Initiating transmit at " << simTime() << ". I've been idle since " << idleTime << std::endl;

	for (std::map<t_access_category, EDCAQueue>::iterator iter = myQueues.begin(); iter != myQueues.end(); iter++) {
		if (iter->second.queue.size() != 0) {
			if (idleTime >= iter->second.aifsn* SLOTLENGTH_11P + SIFS_11P && iter->second.txOP == true) {

				DBG_MAC << "Queue " << iter->first << " is ready to send!" << std::endl;

				iter->second.txOP = false;
				//this queue is ready to send
				if (pktToSend == NULL) {
					pktToSend = iter->second.queue.front();
				}
				else {
					//there was already another packet ready. we have to go increase cw and go into backoff. It's called internal contention and its wonderful

					statsNumInternalContention++;
					iter->second.cwCur = std::min(iter->second.cwMax,iter->second.cwCur*2);
					iter->second.currentBackoff = intuniform(0,iter->second.cwCur);
					DBG_MAC << "Internal contention for queue " << iter->first  << " : "<< iter->second.currentBackoff << ". Increase cwCur to " << iter->second.cwCur << std::endl;
				}
			}
		}
	}

	if (pktToSend == NULL) {
		opp_error("No packet was ready");
	}
	return pktToSend;
}

simtime_t DMMAC::EDCA::startContent(simtime_t idleSince,bool guardActive) {

	DBG_MAC << "Restarting contention." << std::endl;

	simtime_t nextEvent = -1;

	simtime_t idleTime = SimTime().setRaw(std::max((int64_t)0,(simTime() - idleSince).raw()));;

	lastStart = idleSince;

	DBG_MAC << "Channel is already idle for:" << idleTime << " since " << idleSince << std::endl;

	//this returns the nearest possible event in this EDCA subsystem after a busy channel

	for (std::map<t_access_category, EDCAQueue>::iterator iter = myQueues.begin(); iter != myQueues.end(); iter++) {
		if (iter->second.queue.size() != 0) {

			/* 1609_4 says that when attempting to send (backoff == 0) when guard is active, a random backoff is invoked */

			if (guardActive == true && iter->second.currentBackoff == 0) {
				//cw is not increased
				iter->second.currentBackoff = intuniform(0,iter->second.cwCur);
				statsNumBackoff++;
			}

			simtime_t DIFS = iter->second.aifsn * SLOTLENGTH_11P + SIFS_11P;
			//the next possible time to send can be in the past if the channel was idle for a long time, meaning we COULD have sent earlier if we had a packet
			simtime_t possibleNextEvent = DIFS + iter->second.currentBackoff * SLOTLENGTH_11P;
//			cout << ": DIFS: " << DIFS
//					<< " aifsn: " << iter->second.aifsn
//					<< " Backoff: " << iter->second.currentBackoff << endl;

			DBG_MAC << "Waiting Time for Queue " << iter->first <<  ":" << possibleNextEvent << "=" << iter->second.aifsn << " * "  << SLOTLENGTH_11P << " + " << SIFS_11P << "+" << iter->second.currentBackoff << "*" << SLOTLENGTH_11P << "; Idle time: " << idleTime << std::endl;

			if (idleTime > possibleNextEvent) {
				DBG_MAC << "Could have already send if we had it earlier" << std::endl;
				//we could have already sent. round up to next boundary
				simtime_t base = idleSince + DIFS;
				possibleNextEvent =  simTime() - simtime_t().setRaw((simTime() - base).raw() % SLOTLENGTH_11P.raw()) + SLOTLENGTH_11P;
//				cout << simTime() << " - " << simtime_t().setRaw((simTime() - base).raw() % SLOTLENGTH_11P.raw())
//						<< " + " << SLOTLENGTH_11P << endl;
//				cout << ": base: " << base << " idleSince: " << idleSince << endl;
//				cout << " possiNextEvent: " << possibleNextEvent << endl;
			}
			else {
				//we are gonna send in the future
				DBG_MAC << "Sending in the future" << std::endl;
				possibleNextEvent =  idleSince + possibleNextEvent;
			}
			nextEvent == -1? nextEvent =  possibleNextEvent : nextEvent = std::min(nextEvent,possibleNextEvent);
		}
	}
	return nextEvent;
}

void DMMAC::EDCA::stopContent(bool allowBackoff, bool generateTxOp) {
	//update all Queues

	DBG_MAC << "Stopping Contention at " << simTime().raw() << std::endl;

	simtime_t passedTime = simTime() - lastStart;

	DBG_MAC << "Channel was idle for " << passedTime << std::endl;

	lastStart = -1; //indicate that there was no last start

	for (std::map<t_access_category, EDCAQueue>::iterator iter = myQueues.begin(); iter != myQueues.end(); iter++) {
		if (iter->second.currentBackoff != 0 || iter->second.queue.size() != 0) {
			//check how many slots we already waited until the chan became busy

			int oldBackoff = iter->second.currentBackoff;

			std::string info;
			if (passedTime < iter->second.aifsn * SLOTLENGTH_11P + SIFS_11P) {
				//we didnt even make it one DIFS :(
				info.append(" No DIFS");
			}
			else {
				//decrease the backoff by one because we made it longer than one DIFS
				iter->second.currentBackoff--;

				//check how many slots we waited after the first DIFS
				int passedSlots = (int)((passedTime - SimTime(iter->second.aifsn * SLOTLENGTH_11P + SIFS_11P)) / SLOTLENGTH_11P);

				DBG_MAC << "Passed slots after DIFS: " << passedSlots << std::endl;


				if (iter->second.queue.size() == 0) {
					//this can be below 0 because of post transmit backoff -> backoff on empty queues will not generate macevents,
					//we dont want to generate a txOP for empty queues
					iter->second.currentBackoff -= std::min(iter->second.currentBackoff,passedSlots);
					info.append(" PostCommit Over");
				}
				else {
					iter->second.currentBackoff -= passedSlots;
					if (iter->second.currentBackoff <= -1) {
						if (generateTxOp) {
							iter->second.txOP = true; info.append(" TXOP");
						}
						//else: this packet couldnt be sent because there was too little time. we could have generated a txop, but the channel switched
						iter->second.currentBackoff = 0;
					}

				}
			}
			DBG_MAC << "Updating backoff for Queue " << iter->first << ": " << oldBackoff << " -> " << iter->second.currentBackoff << info <<std::endl;
		}
	}
}
void DMMAC::EDCA::backoff(t_access_category ac) {
	myQueues[ac].currentBackoff = intuniform(0,myQueues[ac].cwCur);
	statsSlotsBackoff += myQueues[ac].currentBackoff;
	statsNumBackoff++;
	DBG_MAC << "Going into Backoff because channel was busy when new packet arrived from upperLayer" << std::endl;
}

void DMMAC::EDCA::postTransmit(t_access_category ac) {
	delete myQueues[ac].queue.front();
	myQueues[ac].queue.pop();
	myQueues[ac].cwCur = myQueues[ac].cwMin;
	//post transmit backoff
	myQueues[ac].currentBackoff = intuniform(0,myQueues[ac].cwCur);
	statsSlotsBackoff += myQueues[ac].currentBackoff;
	statsNumBackoff++;
	DBG_MAC << "Queue " << ac << " will go into post-transmit backoff for " << myQueues[ac].currentBackoff << " slots" << std::endl;
}

void DMMAC::EDCA::cleanUp() {
	for (std::map<t_access_category, EDCAQueue>::iterator iter = myQueues.begin(); iter != myQueues.end(); iter++) {
		while (iter->second.queue.size() != 0) {
			delete iter->second.queue.front();
			iter->second.queue.pop();
		}
	}
	myQueues.clear();
}

void DMMAC::EDCA::revokeTxOPs() {
	for (std::map<t_access_category, EDCAQueue>::iterator iter = myQueues.begin(); iter != myQueues.end(); iter++) {
		if (iter->second.txOP == true) {
			iter->second.txOP = false;
			iter->second.currentBackoff = 0;
		}
	}
}

void DMMAC::channelBusySelf(bool generateTxOp) {

	//the channel turned busy because we're sending. we don't want our queues to go into backoff
	//internal contention is already handled in initiateTransmission

	if (!idleChannel) return;
	idleChannel = false;
	DBG_MAC << "Channel turned busy: Switch or Self-Send" << std::endl;

	lastBusy = simTime();

	//channel turned busy
	if (nextMacEvent->isScheduled() == true) {
		cancelEvent(nextMacEvent);
	}
	else {
		//the edca subsystem was not doing anything anyway.
	}
	myEDCA[activeChannel]->stopContent(false, generateTxOp);
}

void DMMAC::channelBusy() {

	if (!idleChannel) return;

	//the channel turned busy because someone else is sending
	idleChannel = false;
	DBG_MAC << "Channel turned busy: External sender" << std::endl;
	lastBusy = simTime();

	//channel turned busy
	if (nextMacEvent->isScheduled() == true) {
		cancelEvent(nextMacEvent);
	}
	else {
		//the edca subsystem was not doing anything anyway.
	}
	myEDCA[activeChannel]->stopContent(true,false);
}

void DMMAC::channelIdle(bool afterSwitch) {

	DBG_MAC << "Channel turned idle: Switch: " << afterSwitch << std::endl;

	if (nextMacEvent->isScheduled() == true) {
		//this rare case can happen when another node's time has such a big offset that the node sent a packet although we already changed the channel
		//the workaround is not trivial and requires a lot of changes to the phy and decider
		return;
		//opp_error("channel turned idle but contention timer was scheduled!");
	}

	idleChannel = true;

	simtime_t delay = 0;

	//account for 1609.4 guards
	if (afterSwitch) {
		//	delay = GUARD_INTERVAL_11P;
	}
	if (useSCH) {
		delay += timeLeftTillGuardOver();
	}

	//channel turned idle! lets start contention!
	lastIdle = delay + simTime();
	statsTotalBusyTime += simTime() - lastBusy;

	//get next Event from current EDCA subsystem
	simtime_t nextEvent = myEDCA[activeChannel]->startContent(lastIdle,guardActive());
	if (nextEvent != -1) {
		if ((!useSCH) || (nextEvent < nextChannelSwitch->getArrivalTime())) {
//			scheduleAt(nextEvent,nextMacEvent);
			DBG_MAC << "next Event is at " << nextMacEvent->getArrivalTime().raw() << std::endl;
		}
		else {
			DBG_MAC << "Too little time in this interval. will not schedule macEvent" << std::endl;
			statsNumTooLittleTime++;
			myEDCA[activeChannel]->revokeTxOPs();
		}
	}
	else {
		DBG_MAC << "I don't have any new events in this EDCA sub system" << std::endl;
	}
}

void DMMAC::setParametersForBitrate(int bitrate) {
	for (unsigned int i = 0; i < NUM_BITRATES_80211P; i++) {
		if (bitrate == BITRATES_80211P[i]) {
			n_dbps = N_DBPS_80211P[i];
			return;
		}
	}
	opp_error("Chosen Bitrate is not valid for 802.11p: Valid rates are: 3Mbps, 4.5Mbps, 6Mbps, 9Mbps, 12Mbps, 18Mbps, 24Mbps and 27Mbps. Please adjust your omnetpp.ini file accordingly.");
}


simtime_t DMMAC::getFrameDuration(int payloadLengthBits) const {
	// calculate frame duration according to Equation (17-29) of the IEEE 802.11-2007 standard
	simtime_t duration = PHY_HDR_PREAMBLE_DURATION + PHY_HDR_PLCPSIGNAL_DURATION + T_SYM_80211P * ceil( (16 + payloadLengthBits + 6)/(n_dbps) );

	return duration;
}

void DMMAC::sendBroadcastPacket(cPacket* msg, simtime_t delay){
    //we actually came to the point where we can send a packet
    channelBusySelf(true);

//    lastAC = mapPriority(pktToSend->getPriority());
//    lastAC = 0;
//    DBG_MAC << "MacEvent received. Trying to send packet with priority" << lastAC << std::endl;

    //send the packet
    Mac80211Pkt* mac = new Mac80211Pkt(msg->getName(), msg->getKind());
    mac->setDestAddr(LAddress::L2BROADCAST);
    mac->setSrcAddr(myMacAddress);
    mac->encapsulate(msg->dup());

    simtime_t sendingDuration = delay + getFrameDuration(mac->getBitLength());
    DBG_MAC << "Sending duration will be " << sendingDuration << std::endl;
    if ( (!useSCH) || (timeLeftInSlot() > sendingDuration) ) {
//    if (true) {
//        if (useSCH) DBG_MAC << " Time in this slot left: " << timeLeftInSlot() << std::endl;
        // give time for the radio to be in Tx state before transmitting
        phy->setRadioState(Radio::TX);

        double freq = (activeChannel == type_CCH) ? frequency[Channels::CCH] : frequency[mySCH];

        attachSignal(mac, simTime()+delay, freq);
        MacToPhyControlInfo* phyInfo = dynamic_cast<MacToPhyControlInfo*>(mac->getControlInfo());
        assert(phyInfo);
        DBG_MAC << "Sending a Packet. Frequency " << freq << " Priority " << lastAC << std::endl;
        sendDelayed(mac, delay, lowerLayerOut);
        statsSentPackets++;
    }
    else {   //not enough time left now
        DBG_MAC << "Too little Time left. This packet cannot be send in this slot." << std::endl;
        statsNumTooLittleTime++;
        //revoke TXOP
//        myEDCA[activeChannel]->revokeTxOPs();
        delete mac;
//        channelIdle();
        //do nothing. contention will automatically start after channel switch
    }
}

double DMMAC::calWSFBeta(){
	double sumSpeed = 0.0;
	for (auto i:vehStatusMap){
		if ( simTime() - i.second.timeStamp <= i.second.ttl ){
			double speedDiff = fabs(i.second.speed - mVehMob->getSpeed());
			sumSpeed += speedDiff;
		}
	}

//	cout << "  sumSpeed: " << sumSpeed << endl;
	double avgSpeed;
	if ( vehStatusMap.size() != 0 )
		avgSpeed = sumSpeed/(double)vehStatusMap.size();
	else
		avgSpeed = 0.0;

//	cout << "  avgSpeed: " << avgSpeed << endl;

	double beta = std::max(1-(avgSpeed/ROADSPEEDLIMIT), 0.0);

//	cout << "  beta = " << beta << endl;

	double zeta = 0.5;

	double wBeta = zeta*beta+(1-zeta)*mPreWSF;

//	cout << "  wBeta = " << wBeta << endl;

	mPreWSF = wBeta;

	return wBeta;
}

double DMMAC::FIS(double alpha, double gamma){

	/*search front vehicle based on whole vehicle map*/
//	std::map<std::string, cModule*> vehMap = mVehMob->getManager()->getManagedHosts();
//	std::map<std::string, Veins::TraCIMobility*> vehIDMap;
//	for ( auto i:vehMap){
//		string vehID = i.second->getFullName();
//		cModule* module = i.second->getSubmodule("veinsmobility");
//		Veins::TraCIMobility* mob =(Veins::TraCIMobility*)module;
//		vehIDMap.insert(pair<std::string, Veins::TraCIMobility*>(vehID, mob));
//	}


//	Veins::TraCIMobility nearestVehMobi;

//	for ( auto i:vehIDMap){
//		if ( i.first.compare(mNodeID) != 0 ){
//			if ( i.second->commandGetLaneIndex() == mVehMob->commandGetLaneIndex() ){
//				double currentDis = fabs(i.second->commandGetOffset() - mVehMob->commandGetOffset());
//				if ( interDis == 0.0 ){
//					interDis = currentDis;
//				}
//				else if ( interDis > currentDis){
//					interDis = currentDis;
//					nearestVehMobi = i.second;
//				}
//			}
//		}
//	}

	/*
	 * inter-distance input
	 * */
	/* get the distance to the front vehicle */
	double interDis = 0.0;
	double relativeSpeed = 0.0;
	// based on received status msgs
	string frontVehID = "";
	for ( auto i:vehStatusMap ){
		if ( simTime() - i.second.timeStamp <= i.second.ttl ){
			if ( i.second.pos.y == mVehMob->getSumoPosition().y ){
				// get inter distance
				if ( i.second.pos.x > mVehMob->getSumoPosition().x ){
					double currentDis = fabs(i.second.pos.x-mVehMob->getSumoPosition().x);
					if ( interDis == 0.0 ){
						interDis = currentDis;
					}
					else if ( interDis > currentDis ){
						interDis = currentDis;
						frontVehID = i.first;
					}
				}
			}
		}
	}

	/*case of no front vehicles in the same lane*/
	if ( interDis == 0.0 ){
		interDis = -1.0;
	}
//	assert(nearVehMobi);

	fisDisInput u_d;
	double tsVj1 = TS*mVehMob->getSpeed();
	double tsVj3 = 3*TS*mVehMob->getSpeed();

	if ( interDis > 0.0 && interDis < tsVj1){
		u_d = small;
	}
	else if ( interDis >= tsVj1 && interDis < tsVj3 ){
		u_d = medium;
	}
	else if ( interDis >= tsVj3){
		u_d = large;
	}
	else {

	}

	/*
	 * relative speed input
	 * */
	if ( frontVehID != ""){
//		cout << "AT5" << endl;
		relativeSpeed = vehStatusMap.at(frontVehID).speed - mVehMob->getSpeed();
	}

	fisSpeedInput u_v;
	double alphaDTS = -alpha*interDis/(double)TS;
	double gammaDTS = gamma*interDis/(double)TS;

	if ( relativeSpeed >= gammaDTS ){
		u_v = fast;
	}
	else if ( relativeSpeed >= alphaDTS && relativeSpeed < gammaDTS){
		u_v = same;
	}
	else if ( relativeSpeed < alphaDTS){
		u_v = slow;
	}

	fisAccOuput u_acc;
	if ( u_d == small && u_v == slow){
		u_acc = same_acc;
	}
	else if ( u_d == small && u_v == same ){
		u_acc = decelerate;
	}
	else if ( u_d == small && u_v == fast ){
		u_acc = decelerate;
	}
	else if ( u_d == medium && u_v == slow ){
		u_acc = same_acc;
	}
	else if ( u_d == medium && u_v == same ){
		u_acc = accelerate;
	}
	else if ( u_d == medium && u_v == fast ){
		u_acc = decelerate;
	}
	else if ( u_d == large && u_v == slow ){
		u_acc = accelerate;
	}
	else if ( u_d == large && u_v == same ){
		u_acc = accelerate;
	}
	else if ( u_d == large && u_v == fast ){
		u_acc = same_acc;
	}

	double acc;
	if ( u_acc == accelerate ){
		acc = 1.0;
	}
	else if ( u_acc == same_acc ){
		acc = 0.0;
	}
	else if ( u_acc == decelerate ){
		acc = -1.0;
	}

//	cout << " FIS acc: " << acc << endl;
	return acc;
}

// DMMAC algorithm 1: An adaptive learning mechanism
double DMMAC::ALM(double aACT){
	double epsilon = 0.1;
	double aFIS = FIS(mAlpha, mGamma);
//	double aACT;

	// preprocess for the actual acc
	if ( aACT > 0.1 )
		aACT = 1.0;
	else if ( aACT < -0.1 )
		aACT = -1.0;
	else
		aACT = 0.0;

	// learning process
	if ( aFIS == 0.0 ) {
		if ( aACT == 1.0 )
			mAlpha = std::max((1-epsilon)*mAlpha, 0.0);
		else if ( aACT == -1.0 )
			mGamma = std::max((1-epsilon)*mGamma, 0.0);
		else {
			mAlpha = mAlpha;
			mGamma = mGamma;
		}
	}
	else if ( aFIS == 1.0 ) {
		if ( aACT == 0.0 || aACT == -1.0 )
			mAlpha = (1+epsilon)*mAlpha;
		else {
			mAlpha = mAlpha;
			mGamma = mGamma;
		}
	}
	else if ( aFIS == -1.0 ) {
		if ( aACT == 0.0 || aACT == 1.0 )
			mGamma = (1+epsilon)*mGamma;
		else {
			mAlpha = mAlpha;
			mGamma = mGamma;
		}
	}

//	cout << "ALM mAlpha: " << mAlpha << ", mGamma: " << mGamma << endl;
	return aFIS;
}

CMACWSM* DMMAC::prepareStatusMsg(	std::string name,
										int lengthBits,
										t_channel channel,
										int priority,
										int type,
										int chaID){

	CMACWSM* statusMsg  = nullptr;

	CMACWSM* cmacwsm  = nullptr;
	if ( !mUpperLayerPktQueue.empty() ){
		cmacwsm = getPktFromQ();
	}

	if ( cmacwsm != nullptr ){
		statusMsg = cmacwsm->dup();
		statusMsg->setName(name.c_str());

		statusMsg->addBitLength(headerLength);
		statusMsg->addBitLength(lengthBits);

	//	statusMsg->setPsid(0);
		statusMsg->setPriority(priority);
	//	statusMsg->setWsmVersion(1);
	//	statusMsg->setTimestamp(simTime());
	//	statusMsg->setSenderAddress(myMacAddress);
	//	statusMsg->setRecipientAddress(-1);
	//	statusMsg->setSenderPos(curPosition);
	//	statusMsg->setSerial(0);

		/*DMMAC info*/
		statusMsg->setType(type);
		statusMsg->setNodeID(mNodeID.c_str());
		// get the WSF beta value
//		mWSF = calWSFBeta();
		statusMsg->setBeta(mWSF);
		statusMsg->setSpeed(mVehMob->getSpeed());
		statusMsg->setPosition(mVehMob->getSumoPosition());

		if (mPredictAcc == -2.0)
			statusMsg->setAcc(mActAcc);
		else
			statusMsg->setAcc(mPredictAcc);

		statusMsg->setRange(cm->calcInterfDist());
		statusMsg->setCHID(mCHID.c_str());
		statusMsg->setCHBK(mCHBK.c_str());
		statusMsg->setChannelID(chaID);

		return statusMsg;
	}
	else {
		statusMsg = new CMACWSM(name.c_str());

		statusMsg->addBitLength(headerLength);
		statusMsg->addBitLength(lengthBits);
		switch (channel) {
			case type_SCH: statusMsg->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
			case type_CCH: statusMsg->setChannelNumber(Channels::CCH); break;
		}

		statusMsg->setPsid(0);
		statusMsg->setPriority(priority);
		statusMsg->setWsmVersion(1);
		statusMsg->setTimestamp(simTime());
		statusMsg->setSenderAddress(myMacAddress);
//		statusMsg->setRecipientAddress(-1);
		statusMsg->setSenderPos(mVehMob->getSumoPosition());
		statusMsg->setSerial(0);

		/*DMMAC info*/
		statusMsg->setType(type);
		statusMsg->setNodeID(mNodeID.c_str());
		// get the WSF beta value
//		mWSF = calWSFBeta();
		statusMsg->setBeta(mWSF);
		statusMsg->setSpeed(mVehMob->getSpeed());
		statusMsg->setPosition(mVehMob->getSumoPosition());

		if (mPredictAcc == -2.0)
			statusMsg->setAcc(mActAcc);
		else
			statusMsg->setAcc(mPredictAcc);

		statusMsg->setRange(cm->calcInterfDist());
		statusMsg->setCHID(mCHID.c_str());
		statusMsg->setCHBK(mCHBK.c_str());
		statusMsg->setChannelID(chaID);

		statusMsg->setDestID("");

		return statusMsg;
	}

}

CMACWSM* DMMAC::prepareCHConsoMsg(	std::string name,
										int lengthBits,
										t_channel channel,
										int priority,
										int type,
										int chaID
										){

	CMACWSM* statusMsg = nullptr;

	CMACWSM* cmacwsm = nullptr;

//	cout << "mUpperLayerPktQueue.length(): " << mUpperLayerPktQueue.length() << endl;
	if ( !mUpperLayerPktQueue.empty() ){
		cmacwsm = getPktFromQForCHDistr();
	}

	if ( cmacwsm != nullptr ){
		statusMsg = cmacwsm->dup();
		statusMsg->setName(name.c_str());

		statusMsg->addBitLength(headerLength);
		statusMsg->addBitLength(lengthBits);
		switch (channel) {
			case type_SCH: statusMsg->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
			case type_CCH: statusMsg->setChannelNumber(Channels::CCH); break;
		}
	//	statusMsg->setPsid(0);
		statusMsg->setPriority(priority);
	//	statusMsg->setWsmVersion(1);
	//	statusMsg->setTimestamp(simTime());
	//	statusMsg->setSenderAddress(myMacAddress);
	//	statusMsg->setRecipientAddress(-1);
	//	statusMsg->setSenderPos(curPosition);
	//	statusMsg->setSerial(0);

		/*DMMAC info*/
		statusMsg->setType(type);
		statusMsg->setNodeID(mNodeID.c_str());
		// get the WSF beta value
	//	mWSF = calWSFBeta();
		statusMsg->setBeta(mWSF);
		statusMsg->setSpeed(mVehMob->getSpeed());
		statusMsg->setPosition(mVehMob->getSumoPosition());

		if (mPredictAcc == -2.0)
			statusMsg->setAcc(mActAcc);
		else
			statusMsg->setAcc(mPredictAcc);

		statusMsg->setRange(cm->calcInterfDist());
		statusMsg->setCHID(mCHID.c_str());
		statusMsg->setCHBK(mCHBK.c_str());
		statusMsg->setChannelID(chaID);


		if ( emgPktSenderID.empty() ) {
			if ( mCMMap.empty() )
				opp_error("prepareCHConsoMsg: mCMMap is empty");

			statusMsg->setGatherCMArraySize(mCMMap.size());
			int index1 = 0;
			for ( std::map<string, CMInfo>::iterator it = mCMMap.begin(); it != mCMMap.end(); ++it ){
				statusMsg->setGatherCM(index1,it->second);
				++index1;
			}
		}
		else {
			statusMsg->setGatherCMArraySize(0);
		}

		return statusMsg;
	}
	else {
		statusMsg = new CMACWSM(name.c_str());

		statusMsg->addBitLength(headerLength);
		statusMsg->addBitLength(lengthBits);
		switch (channel) {
			case type_SCH: statusMsg->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
			case type_CCH: statusMsg->setChannelNumber(Channels::CCH); break;
		}

		statusMsg->setPsid(0);
		statusMsg->setPriority(priority);
		statusMsg->setWsmVersion(1);
		statusMsg->setTimestamp(simTime());
		statusMsg->setSenderAddress(myMacAddress);
//		statusMsg->setRecipientAddress(-1);
		statusMsg->setSenderPos(mVehMob->getSumoPosition());
		statusMsg->setSerial(0);

		/*DMMAC info*/
		statusMsg->setType(type);
		statusMsg->setNodeID(mNodeID.c_str());
		// get the WSF beta value
	//	mWSF = calWSFBeta();
		statusMsg->setBeta(mWSF);
		statusMsg->setSpeed(mVehMob->getSpeed());
		statusMsg->setPosition(mVehMob->getSumoPosition());

		if (mPredictAcc == -2.0)
			statusMsg->setAcc(mActAcc);
		else
			statusMsg->setAcc(mPredictAcc);

		statusMsg->setRange(cm->calcInterfDist());
		statusMsg->setCHID(mCHID.c_str());
		statusMsg->setCHBK(mCHBK.c_str());
		statusMsg->setChannelID(chaID);


		statusMsg->setDestID("");

		if ( emgPktSenderID.empty() ) {
			if ( mCMMap.empty() )
				opp_error("prepareCHConsoMsg: mCMMap is empty");

	//		cout << " prepareCHConsoMsg(): " << endl;
			statusMsg->setGatherCMArraySize(mCMMap.size());
			int index1 = 0;
			for ( std::map<string, CMInfo>::iterator it = mCMMap.begin(); it != mCMMap.end(); ++it ){
				statusMsg->setGatherCM(index1,it->second);
				++index1;
	//			cout << it->second.CMNodeID << endl;
			}
		}
		else{
			statusMsg->setGatherCMArraySize(0);
		}
		return statusMsg;
	}

}

CMACWSM* DMMAC::prepareCH2CHMsg(	std::string name,
									int lengthBits,
									t_channel channel,
									int priority,
									int type,
									int chaID
									){

	CMACWSM* statusMsg = nullptr;

	CMACWSM* cmacwsm = nullptr;

	if ( !mUpperLayerEmgPktQueue.empty() ){
		cmacwsm = getPktFromEmgQ();
	}
	else if ( !mUpperLayerPktQueue.empty() ){
		cmacwsm = getPktFromQForCH2CH();
	}

	if ( cmacwsm != nullptr ){
		statusMsg = cmacwsm->dup();
		statusMsg->setName(name.c_str());

		statusMsg->addBitLength(headerLength);
		statusMsg->addBitLength(lengthBits);

//		statusMsg->setPsid(0);
		statusMsg->setPriority(priority);
//		statusMsg->setWsmVersion(1);
//		statusMsg->setTimestamp(simTime());
//		statusMsg->setSenderAddress(myMacAddress);
//		statusMsg->setRecipientAddress(-1);
	//	statusMsg->setSenderPos(curPosition);
//		statusMsg->setSerial(0);

		/*DMMAC info*/
		statusMsg->setType(type);
		statusMsg->setNodeID(mNodeID.c_str());
		// get the WSF beta value
	//	mWSF = calWSFBeta();
		statusMsg->setBeta(mWSF);
		statusMsg->setSpeed(mVehMob->getSpeed());
		statusMsg->setPosition(mVehMob->getSumoPosition());

		if (mPredictAcc == -2.0)
			statusMsg->setAcc(mActAcc);
		else
			statusMsg->setAcc(mPredictAcc);

		statusMsg->setRange(cm->calcInterfDist());
		statusMsg->setCHID(mCHID.c_str());
		statusMsg->setCHBK(mCHBK.c_str());
		statusMsg->setChannelID(chaID);

		// my own CM information
		if ( mCMMap.empty() )
			opp_error("prepareCH2CHMsg: mCMMap is empty");
		statusMsg->setGatherCMArraySize(mCMMap.size());
		int index1 = 0;
		for ( std::map<string, CMInfo>::iterator it = mCMMap.begin(); it != mCMMap.end(); ++it ){
			statusMsg->setGatherCM(index1,it->second);
			++index1;
		}
//		mCMMap.clear();

		// include other CM information
		statusMsg->setOrCMArraySize(mRcvdOrCMVec.size());
		for ( size_t index2=0; index2<mRcvdOrCMVec.size(); ++index2 ){
			statusMsg->setOrCM(index2,mRcvdOrCMVec[index2]);
		}
		mRcvdOrCMVec.clear();

		statusMsg->setOrCMArraySize(mRcvdRandomCMVec.size());
		for ( size_t index2=0; index2<mRcvdRandomCMVec.size(); ++index2 ){
			statusMsg->setOrCM(index2,mRcvdRandomCMVec[index2]);
		}
		mRcvdRandomCMVec.clear();

		return statusMsg;
	}
	else {
		statusMsg = new CMACWSM(name.c_str());

		statusMsg->addBitLength(headerLength);
		statusMsg->addBitLength(lengthBits);
		switch (channel) {
			case type_SCH: statusMsg->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
			case type_CCH: statusMsg->setChannelNumber(Channels::CCH); break;
		}
		statusMsg->setPsid(0);
		statusMsg->setPriority(priority);
		statusMsg->setWsmVersion(1);
		statusMsg->setTimestamp(simTime());
		statusMsg->setSenderAddress(myMacAddress);
		statusMsg->setRecipientAddress(-1);
	//	statusMsg->setSenderPos(curPosition);
		statusMsg->setSerial(0);

		/*DMMAC info*/
		statusMsg->setType(type);
		statusMsg->setNodeID(mNodeID.c_str());
		// get the WSF beta value
	//	mWSF = calWSFBeta();
		statusMsg->setBeta(mWSF);
		statusMsg->setSpeed(mVehMob->getSpeed());
		statusMsg->setPosition(mVehMob->getSumoPosition());

		if (mPredictAcc == -2.0)
			statusMsg->setAcc(mActAcc);
		else
			statusMsg->setAcc(mPredictAcc);

		statusMsg->setRange(cm->calcInterfDist());
		statusMsg->setCHID(mCHID.c_str());
		statusMsg->setCHBK(mCHBK.c_str());
		statusMsg->setChannelID(chaID);

		// my own CM information
		if ( mCMMap.empty() )
			opp_error("prepareCH2CHMsg: mCMMap is empty");
		statusMsg->setGatherCMArraySize(mCMMap.size());
		int index1 = 0;
		for ( std::map<string, CMInfo>::iterator it = mCMMap.begin(); it != mCMMap.end(); ++it ){
			statusMsg->setGatherCM(index1,it->second);
			++index1;
		}
//		mCMMap.clear();
		// include other CM information
		statusMsg->setOrCMArraySize(mRcvdOrCMVec.size());
		for ( size_t index2=0; index2<mRcvdOrCMVec.size(); ++index2 ){
			statusMsg->setOrCM(index2,mRcvdOrCMVec[index2]);
		}
		mRcvdOrCMVec.clear();

		statusMsg->setOrCMArraySize(mRcvdRandomCMVec.size());
		for ( size_t index2=0; index2<mRcvdRandomCMVec.size(); ++index2 ){
			statusMsg->setOrCM(index2,mRcvdRandomCMVec[index2]);
		}
		mRcvdRandomCMVec.clear();

		statusMsg->setDestID("");

		return statusMsg;
	}
}

CMACWSM* DMMAC::prepareEmgMsg(	std::string name,
								int lengthBits,
								t_channel channel,
								int priority,
								int type,
								int chaID,
								std::string emgPktSender
								){

//	CMACWSM* statusMsg = new CMACWSM(name.c_str());
	CMACWSM* statusMsg = nullptr;

//	CMACWSM* cmacwsm = nullptr;
	if ( !mUpperLayerEmgPktQueue.empty() ){
		statusMsg = getPktFromEmgQ();
	}

	if ( statusMsg != nullptr ){
		statusMsg->setName(name.c_str());
		statusMsg->addBitLength(headerLength);
		statusMsg->addBitLength(lengthBits);
		switch (channel) {
			case type_SCH: statusMsg->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
			case type_CCH: statusMsg->setChannelNumber(Channels::CCH); break;
		}
//		statusMsg->setPsid(0);
		statusMsg->setPriority(priority);
//		statusMsg->setWsmVersion(1);
//		statusMsg->setTimestamp(simTime());
		statusMsg->setSenderAddress(myMacAddress);
//		statusMsg->setRecipientAddress(-1);
	//	statusMsg->setSenderPos(curPosition);
//		statusMsg->setSerial(0);

		/*DMMAC info*/
		statusMsg->setType(type);
		statusMsg->setNodeID(mNodeID.c_str());
		// get the WSF beta value
//		mWSF = calWSFBeta();
		statusMsg->setBeta(mWSF);
		statusMsg->setSpeed(mVehMob->getSpeed());
		statusMsg->setPosition(mVehMob->getSumoPosition());

		if (mPredictAcc == -2.0)
			statusMsg->setAcc(mActAcc);
		else
			statusMsg->setAcc(mPredictAcc);

		statusMsg->setRange(cm->calcInterfDist());
		statusMsg->setCHID(mCHID.c_str());
		statusMsg->setCHBK(mCHBK.c_str());
		statusMsg->setChannelID(chaID);

		statusMsg->setEmgPktSender(emgPktSender.c_str());

		return statusMsg;
	}
	else {
		opp_error("EMG queue is empty!");
		return nullptr;
	}

}

void DMMAC::loneTransition(){
	/*search the max WSF and the ID*/
	double maxWSFinStatusMap = 0.0;
	string vehIDMaxWSF;
	for ( auto i:vehStatusMap){
		if ( maxWSFinStatusMap == 0.0 ){
			if ( i.second.CHID.empty()
					|| strcmp(i.second.CHID.c_str(), i.second.senderID.c_str()) != 0 ){
				maxWSFinStatusMap = i.second.beta;
				vehIDMaxWSF = i.first;
			}
		}
		else if ( maxWSFinStatusMap < i.second.beta ){
			if ( i.second.CHID.empty()
					|| strcmp(i.second.CHID.c_str(), i.second.senderID.c_str()) != 0 ){
				maxWSFinStatusMap = i.second.beta;
				vehIDMaxWSF = i.first;
			}
		}
	}

	// if I has the highest WSF, becomes CH
	if ( mWSF > maxWSFinStatusMap ){
		mClusterRole = tempCH;
	}
	// if tier, larger ID is selected
	else if ( mWSF == maxWSFinStatusMap ){
		string::size_type p1_1 = mNodeID.find('[');
		string::size_type p1_2 = mNodeID.find(']');
		int myID = std::stoi(mNodeID.substr(p1_1+1, p1_2-p1_1-1));
		string::size_type p2_1 = vehIDMaxWSF.find('[');
		string::size_type p2_2 = vehIDMaxWSF.find(']');
		int otherID = std::stoi(vehIDMaxWSF.substr(p2_1+1, p2_2-p2_1-1));

		if ( myID > otherID ){
			mClusterRole = tempCH;
		}
	}
	// otherwise, I shall be CM, set CHID to the ID with highest WSF
	else {
		mClusterRole = CM;
		mCHID = vehIDMaxWSF;

		DBG_MAC << mNodeID << " is CM, my CH ID: " << mCHID << endl;
		if ( chris_dmmac ){
			cout << mNodeID << " is CM, my CH ID: " << mCHID << endl;
		}
		if ( mCHID.empty() )
			findHost()->getDisplayString().updateWith("r=6,red");
		else
			findHost()->getDisplayString().updateWith("r=6,green");
	}
}

void DMMAC::tempCHTransition(){
	bool flag1 = true;
	vector<string> neighborCHVec;
	// check if I'm in 1/2 range of another CH
	for ( auto i:vehStatusMap){
		if ( !(i.second.CHID.empty()) ){
			if ( strcmp(i.second.CHID.c_str(), i.first.c_str()) == 0 ){
				double disTwoCH = mVehMob->getSumoPosition().distance(i.second.pos);
				if ( disTwoCH < i.second.range/2.0 ){
					neighborCHVec.push_back(i.first);
					flag1 = false;
				}
			}
		}
	}

	// if I'm not in 1/2 range of another CH, I become CH
	if ( flag1 == true ){
		mClusterRole = CH;
		mCHID = mNodeID;
		findHost()->getDisplayString().updateWith("r=6,blue");
		DBG_MAC << mNodeID << " tempCH->CH, mCHID: " << mCHID << endl;
		if ( chris_dmmac ){
			cout << mNodeID << " tempCH->CH, mCHID: " << mCHID << endl;
		}
	}
	// else, merge as CM
	else {
		if ( neighborCHVec.size() == 1 ){
			mClusterRole = CM;
//			cout << "AT1" << endl;
			mCHID = neighborCHVec.at(0);
			findHost()->getDisplayString().updateWith("r=6,green");
			DBG_MAC << mNodeID << " tempCH->CM, mCHID: " << mCHID << endl;
			if ( chris_dmmac ){
				cout << mNodeID << " tempCH->CM, mCHID: " << mCHID << endl;
			}
		}
		else {
			/*search the nearest CH*/
			double maxDistToCH = 0.0;
			string maxDistVehID;
			for ( auto i: neighborCHVec ){
//				cout << "AT2" << endl;
				double dist = mVehMob->getSumoPosition().distance(vehStatusMap.at(i).pos);
				if ( maxDistToCH == 0.0 ){
					maxDistToCH = dist;
				}
				else if ( maxDistToCH < dist){
					maxDistToCH = dist;
					maxDistVehID = i;
				}
			}
			mClusterRole = CM;
			mCHID = maxDistVehID;
			findHost()->getDisplayString().updateWith("r=6,green");
			DBG_MAC << mNodeID << " tempCH->CM, mCHID: " << mCHID << endl;
			if ( chris_dmmac ){
				cout << mNodeID << " tempCH->CM, mCHID: " << mCHID << endl;
			}
		}
	}
}

void DMMAC::CHTransition(){
	/*search the max WSF and the ID*/
	double maxWSFinStatusMap = 0.0;
	string vehIDMaxWSF;
	for ( auto i:vehStatusMap){
		if ( maxWSFinStatusMap == 0.0 ){
			maxWSFinStatusMap = i.second.beta;
			vehIDMaxWSF = i.first;
		}
		else if ( maxWSFinStatusMap < i.second.beta ){
			maxWSFinStatusMap = i.second.beta;
			vehIDMaxWSF = i.first;
		}
	}
//			int myIDt = std::stoi(mNodeID);
//			int otherIDt = std::stoi(vehIDMaxWSF);
//			cout << myIDt << " : " << otherIDt << endl;
	// if I have the highest WSF, becomes CH
	if ( mWSF > maxWSFinStatusMap ){
		mClusterRole = CH;
		mCHBK = vehIDMaxWSF;
		findHost()->getDisplayString().updateWith("r=6,blue");
		DBG_MAC << mNodeID << " CH->CH, mCHBK: " << mCHBK << endl;
	}
	// if tier, larger ID is selected
	else if ( mWSF == maxWSFinStatusMap ){
		string::size_type p1_1 = mNodeID.find('[');
		string::size_type p1_2 = mNodeID.find(']');
		int myID = std::stoi(mNodeID.substr(p1_1+1, p1_2-p1_1-1));
		string::size_type p2_1 = vehIDMaxWSF.find('[');
		string::size_type p2_2 = vehIDMaxWSF.find(']');
		int otherID = std::stoi(vehIDMaxWSF.substr(p2_1+1, p2_2-p2_1-1));

		if ( myID > otherID ){
			mClusterRole = CH;
			mCHBK = vehIDMaxWSF;
			findHost()->getDisplayString().updateWith("r=6,blue");
			DBG_MAC << mNodeID << " CH->CH, mCHBK: " << mCHBK << endl;
		}
	}
	// otherwise, I shall still be CH, set CHBK to the ID with highest WSF
	else {
		// Difference: role is still CH, just set CHBK as the vehicle with max WSF
		mClusterRole = CH;
		mCHBK = vehIDMaxWSF;
		findHost()->getDisplayString().updateWith("r=6,blue");
		DBG_MAC << mNodeID << " CH->CH, mCHBK: " << mCHBK << endl;
	}
}

void DMMAC::handleMACLayerMsg(cMessage* msg) {

//	cout << mNodeID << ": handleMACLMsg: " << endl;
	CMACWSM* thisMsg;

	t_access_category ac;
	t_channel chan;
	int num;

	if ((thisMsg = dynamic_cast<CMACWSM*>(msg)) == NULL) {
		error("WaveMac only accepts WaveShortMessages");
	}

	ac = mapPriority(thisMsg->getPriority());

	DBG_MAC << "Received a message from upper layer for channel "
			<< thisMsg->getChannelNumber() << " Access Category (Priority):  "
			<< ac << std::endl;

	//rewrite SCH channel to actual SCH the Mac1609_4 is set to
	if (thisMsg->getChannelNumber() == Channels::SCH1) {
		ASSERT(useSCH);
		thisMsg->setChannelNumber(mySCH);
		chan = type_SCH;
	}
//	else if ( thisMsg->getChannelNumber() == Channels::SCH2){
//		ASSERT(useSCH);
//		thisMsg->setChannelNumber(mySCH);
//		chan = type_SCH;
//	}
//	else if ( thisMsg->getChannelNumber() == Channels::SCH3){
//		ASSERT(useSCH);
//		thisMsg->setChannelNumber(mySCH);
//		chan = type_SCH;
//	}

	//put this packet in its queue
	if (thisMsg->getChannelNumber() == Channels::CCH) {
		chan = type_CCH;
	}

//	cout << "thisMsg->getChannelNumber(): " << thisMsg->getChannelNumber() << endl;
//	cout << "chan: " << chan << endl;
//	cout << myEDCA[chan]->myQueues[ac].cwCur << endl;

	num = myEDCA[chan]->queuePacket(ac,thisMsg);

	//packet was dropped in Mac
	if (num == -1) {
		statsDroppedPackets++;
		return;
	}

	//if this packet is not at the front of a new queue we dont have to reevaluate times
	DBG_MAC << "sorted packet into queue of EDCA " << chan << " this packet is now at position: " << num << std::endl;

	if (chan == activeChannel) {
		DBG_MAC << "this packet is for the currently active channel" << std::endl;
	}
	else {
		DBG_MAC << "this packet is NOT for the currently active channel" << std::endl;
	}

	if (num == 1 && idleChannel == true && chan == activeChannel) {

		simtime_t nextEvent = myEDCA[chan]->startContent(lastIdle,guardActive());

		if (nextEvent != -1) {
			if ((!useSCH) || (nextEvent <= nextChannelSwitch->getArrivalTime())) {
				if (nextMacEvent->isScheduled()) {
					cancelEvent(nextMacEvent);
				}
				scheduleAt(nextEvent,nextMacEvent);
//				cout << mNodeID << ": nextMacEvent: " << nextEvent << endl;
				DBG_MAC << "Updated nextMacEvent:" << nextMacEvent->getArrivalTime().raw() << std::endl;
			}
			else {
				DBG_MAC << "Too little time in this interval. Will not schedule nextMacEvent" << std::endl;
				//it is possible that this queue has an txop. we have to revoke it
				myEDCA[activeChannel]->revokeTxOPs();
				statsNumTooLittleTime++;
			}
		}
		else {
			cancelEvent(nextMacEvent);
		}
	}
	if (num == 1 && idleChannel == false && myEDCA[chan]->myQueues[ac].currentBackoff == 0 && chan == activeChannel) {
		myEDCA[chan]->backoff(ac);
	}

}

void DMMAC::startTf(std::map<std::string, cModule*> allVehMap){

	std::map<std::string, cModule*>::iterator itVehsMap;

	std::vector<DMMAC*> allVehVec;
	std::vector<DMMAC*>::iterator itAllVehVec;
	allVehVec.clear();

	for (itVehsMap = allVehMap.begin(); itVehsMap != allVehMap.end(); itVehsMap++){
		cModule* macModule = (*itVehsMap).second->getModuleByPath(".nic.mac1609_4");
		DMMAC* dmmac = check_and_cast<DMMAC*>(macModule);

//		dmmac->mVehMob->getCommandInterface()->getEdgeId(dmmac->mVehMob->getExternalId());
//		dmmac->mVehMob->commandSetSpeed(12);

		// reset all
		dmmac->mCHID.clear();
		dmmac->mCHBK.clear();
		dmmac->mCHChanID = 0;
		dmmac->mCMMap.clear();
		dmmac->mCHMap.clear();
		dmmac->mClusterRole = lone;
		dmmac->mNeighborVec.clear();
		dmmac->vehStatusMap.clear();
		dmmac->mRcvdOrCMVec.clear();
		dmmac->mRcvdRandomCMVec.clear();
		dmmac->mWSF = 0.0;

		if ( getOffsetRatio(dmmac->mVehMob) < 0.98){
			if ( dmmac->mVehMob->getPlannedEdgeIdList().front().compare(dmmac->mVehMob->getCurrentEdgeId()) == 0 ){
				allVehVec.push_back(dmmac);
				(*itVehsMap).second->getDisplayString().updateWith("r=0");
			}
		}
		else {
			(*itVehsMap).second->getDisplayString().updateWith("r=0");
		}
	}

	/*
	 * find the neighborhoods
	 * */
	// clear neighbors of each vehicle
	for ( itAllVehVec = allVehVec.begin(); itAllVehVec != allVehVec.end(); ++itAllVehVec){
		(*itAllVehVec)->mNeighborVec.clear();
	}

	int maxTxDist = cm->calcInterfDist();
//	maxTxDist = 300;

	for ( itAllVehVec = allVehVec.begin(); itAllVehVec != allVehVec.end(); ++itAllVehVec){
		std::vector<DMMAC*>::iterator itAllVehVec2;
//		cout << (*itVehVec)->getFullName() << endl;
		for ( itAllVehVec2 = allVehVec.begin(); itAllVehVec2 != allVehVec.end(); ++itAllVehVec2){
			// outside
			Veins::TraCIMobility* mobility = (*itAllVehVec)->mVehMob;
			// inside
			Veins::TraCIMobility* mobility2 = (*itAllVehVec2)->mVehMob;
			// find nodes within maxTxDist
			if ( mobility->getCurrentPosition().distance(mobility2->getCurrentPosition()) < maxTxDist ){
				if ( (*itAllVehVec)->mNodeID.compare((*itAllVehVec2)->mNodeID) != 0 ){
					(*itAllVehVec)->mNeighborVec.push_back(*itAllVehVec2);
				}
			}
		}
	}

//	for ( auto i:allVehVec ){
//		cout << i->mNodeID << " : " << endl;
//		cout << "          ";
//		for ( auto j: i->mNeighborVec ){
//			cout << j->mNodeID << " ";
//		}
//		cout << endl;
//	}

//	cout 	<< setw(10) << "nodeID" << " | "
//			<< setw(9) << "betaWSF" << " | "
//			<< setw(8) << "speed" << " | "
//			<< setw(19) << "pos" << " | "
//			<< setw(8) << "acc" << " | "
//			<< setw(4) << "R"<< " | "
//			<< setw(10) << "CHID" << " | "
//			<< setw(10) << "CHBK" << " | "
//			<< endl;


	for ( itAllVehVec = allVehVec.begin(); itAllVehVec != allVehVec.end(); ++itAllVehVec){
		DMMAC* dmmac = *itAllVehVec;

        if ( mPreSpeed == 0.0 )
        	dmmac->mActAcc = 0.0;
        else
    		// update actual acc, m/s^2
    		dmmac->mActAcc = (dmmac->mVehMob->getSpeed()-dmmac->mPreSpeed)/(double)(simTime().dbl()-dmmac->mPreSpeedSampleTime.dbl());

		// calculate predicted acc
		dmmac->mPredictAcc = ALM(dmmac);

		dmmac->mWSF = calWSFBeta(dmmac);

		// regular speed sampling, every 1s
		dmmac->mPreSpeed = mVehMob->getSpeed();
		dmmac->mPreSpeedSampleTime = simTime();

//		cout 	<< setw(10) << dmmac->mNodeID << " | "
//				<< setw(9) << dmmac->mWSF << " | "
//				<< setw(8) << dmmac->mVehMob->getSpeed() << " | "
//				<< setw(2) << dmmac->mVehMob->getSumoPosition() << " | "
//				<< setw(8) << dmmac->mPredictAcc << " | "
//				<< setw(4) << dmmac->commRange << " | "
//				<< setw(10) << dmmac->mCHID << " | "
//				<< setw(10) << dmmac->mCHBK << " | "
//				<< endl;
	}
//		cout << "mPredictAcc: " << mPredictAcc << endl;
	startClustering(allVehVec);

}

void DMMAC::startTf(){
	/*purge vehStatusMap, remove expired entries*/
	vector<string> expiredEntry;
	for ( auto i:vehStatusMap ){
		if ( simTime() - i.second.timeStamp > i.second.ttl ){
			expiredEntry.push_back(i.first);
		}
	}
	for ( auto i: expiredEntry ){
		vehStatusMap.erase(i);
	}
	expiredEntry.clear();

	// clear mCMMap
	mCMMap.clear();

	// update actual acc, m/s^2
	mActAcc = (mVehMob->getSpeed()-mPreSpeed)/(double)(simTime().dbl()-mPreSpeedSampleTime.dbl());
	// calculate predicted acc
	mPredictAcc = ALM(mActAcc);
//		cout << "mPredictAcc: " << mPredictAcc << endl;
	startClustering();
//	startClustering();
}

void DMMAC::startClustering(std::vector<DMMAC*> allVehVec){

	std::vector<DMMAC*> aV4C = allVehVec;
//	cout << "BEGIN: aV4C.size(): " << aV4C.size() << endl;

	while ( aV4C.size() > 0 ){
		DMMAC* dmmacMax;
		double maxWSF = 0.0;
		std::vector<DMMAC*>::iterator it_a;
		for ( it_a = aV4C.begin(); it_a != aV4C.end(); ++it_a){
			DMMAC* dmmac = *it_a;
			if ( maxWSF == 0.0 ){
				dmmacMax = dmmac;
				maxWSF = dmmac->mWSF;
			}
			else if ( maxWSF < dmmac->mWSF ){
				maxWSF = dmmac->mWSF;
				dmmacMax = dmmac;
			}
		}
		if ( chris_dmmac_2 ){
			cout << "CH: " << dmmacMax->mNodeID << endl;
			cout << "-------> CM: ";
		}
		// remove CH from the vector
		if ( it_a != aV4C.end() ){
			aV4C.erase(it_a);
		}

		int chanID;
		// all neighbors of the maxWSF veh become its CMs
		if ( maxWSF != 0.0 ){
			dmmacMax->findHost()->getDisplayString().updateWith("r=6,green");
			dmmacMax->mClusterRole = CH;
			dmmacMax->mCHID = dmmacMax->mNodeID;
			// choose channel
			chanID = decideChannel(dmmacMax->mCHMap, dmmacMax->mVehMob);
			dmmacMax->mCHChanID = chanID;
//			dmmacMax->mCMMap.clear();
//			dmmacMax->mCHMap.clear();

			// put CH itself into CMmap;
			CMInfo CMEntry4CH;
			CMEntry4CH.beta = dmmacMax->mWSF;
			CMEntry4CH.speed = dmmacMax->mVehMob->getSpeed();
			CMEntry4CH.pos = dmmacMax->mVehMob->getSumoPosition();
			CMEntry4CH.acc = dmmacMax->mPredictAcc;
			CMEntry4CH.range = dmmacMax->commRange;
			CMEntry4CH.CHID = dmmacMax->mCHID;
			CMEntry4CH.CHBK = dmmacMax->mCHBK;
			CMEntry4CH.timeStamp = 0; // no need
			CMEntry4CH.ttl = 0; // no need
			CMEntry4CH.chanID = dmmacMax->mCHChanID;
			CMEntry4CH.destID = "";
			CMEntry4CH.senderID = dmmacMax->mNodeID;
			CMEntry4CH.CMNodeID = dmmacMax->mNodeID;

			if ( dmmacMax->mCMMap.find(dmmacMax->mNodeID) == dmmacMax->mCMMap.end() ){
				dmmacMax->mCMMap.insert(std::pair<std::string, CMInfo>(dmmacMax->mNodeID, CMEntry4CH));
//				dmmacMax->mCMMap[dmmacMax->mNodeID] = CMEntry4CH;
			}
			else {
				dmmacMax->mCMMap.at(dmmacMax->mNodeID) = CMEntry4CH;
			}

			std::vector<DMMAC*>::iterator it_b;

			for ( auto i:dmmacMax->mNeighborVec){
				if ( find(aV4C.begin(), aV4C.end(), i) != aV4C.end() ){
					i->mClusterRole = CM;
					i->mCHID = dmmacMax->mNodeID;
					i->mCHChanID = chanID;

					CMInfo CMEntry;
					CMEntry.beta = i->mWSF;
					CMEntry.speed = i->mVehMob->getSpeed();
					CMEntry.pos = i->mVehMob->getSumoPosition();
					CMEntry.acc = i->mPredictAcc;
					CMEntry.range = i->commRange;
					CMEntry.CHID = i->mCHID;
					CMEntry.CHBK = i->mCHBK;
					CMEntry.timeStamp = 0; // no need
					CMEntry.ttl = 0; // no need
					CMEntry.chanID = i->mCHChanID;
					CMEntry.destID = "";
					CMEntry.senderID = dmmacMax->mNodeID;
					CMEntry.CMNodeID = i->mNodeID;

					if ( dmmacMax->mCMMap.find(i->mNodeID) == dmmacMax->mCMMap.end() ){
						dmmacMax->mCMMap.insert(std::pair<std::string, CMInfo>(i->mNodeID, CMEntry));
					}
					else {
						dmmacMax->mCMMap.at(i->mNodeID) = CMEntry;
					}
//					dmmacMax->mCMMap[i->mNodeID] = CMEntry;

					i->findHost()->getDisplayString().updateWith("r=4,lightgreen");

					if ( chris_dmmac_2 ){
						cout << i->mNodeID << " ";
					}
				}
			}
			if ( chris_dmmac_2 ){
				cout << endl;
			}
		}
		else {
			opp_error("maxWSF is 0! ");
		}

		// remove assigned vehicles from the vector
		std::vector<DMMAC*> nVeh = dmmacMax->mNeighborVec;
		for ( it_a = aV4C.begin(); it_a != aV4C.end(); ++it_a){
			if ( !((*it_a)->mCHID.empty()) ){
				aV4C.erase(it_a);
				it_a--;
			}
		}

//		cout << "dmmacMax->mNeighborVec.size(): " << dmmacMax->mNeighborVec.size() << endl;
		if ( chris_dmmac_2 ){
			cout << "aV4C.size(): " << aV4C.size() << endl;
		}
	}

/*debug*/
//	std::vector<DMMAC*>::iterator itAllVehVec;
//	for ( itAllVehVec = allVehVec.begin(); itAllVehVec != allVehVec.end(); ++itAllVehVec){
//		DMMAC* dmmac = *itAllVehVec;
//		cout << dmmac->mNodeID << ", ";
//		cout << "mRole: " << dmmac->mClusterRole << endl;
//	}
/*debug*/
}


void DMMAC::startClustering(){
	if ( mClusterRole == lone ){
		loneTransition();
		if ( mClusterRole == tempCH ){
			tempCHTransition();
		}
/*debug*/
		if ( chris_dmmac ){
			cout << mNodeID << ", ";
			cout << "mRole: " << mClusterRole << endl;
		}
/*debug*/
	}
	else if ( mClusterRole == tempCH ){
		tempCHTransition();
	}
	else if ( mClusterRole == CH ){
		CHTransition();

		/*calculate expected position of CMs*/
		vector<string> CMwithinRange;
		for ( auto i:vehStatusMap){
			double expectMoveDis = i.second.speed*TF + 0.5*i.second.acc*pow(TF,2);
			double disToCH = mVehMob->getSumoPosition().distance(i.second.pos);
			double range = cm->calcInterfDist();
			if ( disToCH + expectMoveDis < range )
				CMwithinRange.push_back(i.first);
		}

		/*10% CMs left, handover CH role to backup CH*/
		double ratio = 1 - CMwithinRange.size()/(double)(vehStatusMap.size());
		if ( ratio > 0.1 ){
			mClusterRole = CM;
			mCHID = mCHBK;
			findHost()->getDisplayString().updateWith("r=6,green");
			DBG_MAC << mNodeID << " CH->CM due to 10% CM, mCHID = mCHBK: " << mCHID << endl;
		}

		/* a CH is in 2/3 of the neighbor CH range, handover to backup CH */
		for ( auto i:vehStatusMap ){
			// find the vehicle is CH in my statusMap
			if ( i.first.compare(i.second.CHID.c_str()) == 0 ){
				double dist = mVehMob->getSumoPosition().distance(i.second.pos);
				if ( dist/(i.second.range) < 0.6667 ){
					if ( !(mCHBK.empty()) ){
						mClusterRole = CM;
						mCHID = mCHBK;
						findHost()->getDisplayString().updateWith("r=6,green");
						DBG_MAC << mNodeID << " CH->CM due to 2/3 neighbor, mCHID = mCHBK: " << mCHID << endl;
						break;
					}
				}
			}
		}

	}
	else if ( mClusterRole == CM ){

		// a vehicle leaves a CH's range, wait 3 rounds of CCI, switch ch to ch4
		if ( mLeaveCHTime != 0.0 ){
			if ( simTime() - mLeaveCHTime > 3*CCI ){
				// switch channel to CCH

				mClusterRole = lone;
				mCHID = "";
				mCHBK = "";
				mLeaveCHTime = 0.0;
			}
		}
		else {
			for ( auto i:vehStatusMap ){
				if ( strcmp(i.first.c_str(), i.second.CHID.c_str() ) == 0  ){
					// check if a CM leaves the range of its CH
					if ( mVehMob->getSumoPosition().distance(i.second.pos)
														> commRange){
						mLeaveCHTime = simTime();
					}
					break;
				}
			}
		}
	}
	else if ( mClusterRole == backCH ){
		if ( vehStatusMap.find(mCHID) != vehStatusMap.end() ){
//			cout << "AT3" << endl;
			if ( strcmp(vehStatusMap.at(mCHID).CHID.c_str(),mNodeID.c_str()) == 0 ){
				mClusterRole = CH;
				mCHID = mNodeID;
				findHost()->getDisplayString().updateWith("r=6,blue");
				DBG_MAC << mNodeID << " backCH->CH, mCHID = mCHBK: " << mCHID << endl;
			}
		}
		else {

		}
	}
}

void DMMAC::selectSCH(int chaID){

	switch (chaID) {
		case 1: mySCH = Channels::SCH1; break;
		case 2: mySCH = Channels::SCH2; break;
		case 3: mySCH = Channels::SCH3; break;
//			case 4: mySCH = Channels::SCH4; break;
		default: opp_error("Service Channel must be between 1 and 3"); break;
	}
}

void DMMAC::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj) {
	Enter_Method_Silent();
	if (signalID == mobilityStateChangedSignal){
		mobilityUpdateTime = simTime();
/*

		FSM_Switch(fsm){
			case FSM_Exit(INIT):
				FSM_Goto(fsm, LONE);
				break;
			case FSM_Enter(LONE):
				loneTransition();

				break;
			case FSM_Exit(LONE):

				break;
			case FSM_Enter(CH):

				break;
			case FSM_Exit(CH):


				break;
			case FSM_Enter(MEMBER):

				break;
			case FSM_Exit(MEMBER):

				break;
			case FSM_Enter(BCKCH):

				break;
			case FSM_Exit(BCKCH):

				break;
			case FSM_Exit(TEMPCH):

				break;
		}
*/

	}
}

double DMMAC::calWSFBeta(DMMAC* dmmac){
	double sumSpeed = 0.0;
	for (auto i:dmmac->mNeighborVec){
//		if ( simTime() - i.second.timeStamp <= i.second.ttl ){
			double speedDiff = fabs(i->mVehMob->getSpeed() - dmmac->mVehMob->getSpeed());
			sumSpeed += speedDiff;
//		}
	}

//	cout << "  sumSpeed: " << sumSpeed << endl;
	double avgSpeed;
	if ( dmmac->mNeighborVec.size() != 0 )
		avgSpeed = sumSpeed/(double)(dmmac->mNeighborVec.size());
	else
		avgSpeed = 0.0;

//	cout << "  avgSpeed: " << avgSpeed << endl;

	double beta = std::max(1-(avgSpeed/ROADSPEEDLIMIT), 0.0);

//	cout << "  beta = " << beta << endl;

	double zeta = 0.5;

	double wBeta = zeta*beta+(1-zeta)*mPreWSF;

//	cout << "  wBeta = " << wBeta << endl;

	mPreWSF = wBeta;

	return wBeta;
}

double DMMAC::FIS(DMMAC* dmmac){

	/*search front vehicle based on whole vehicle map*/
//	std::map<std::string, cModule*> vehMap = mVehMob->getManager()->getManagedHosts();
//	std::map<std::string, Veins::TraCIMobility*> vehIDMap;
//	for ( auto i:vehMap){
//		string vehID = i.second->getFullName();
//		cModule* module = i.second->getSubmodule("veinsmobility");
//		Veins::TraCIMobility* mob =(Veins::TraCIMobility*)module;
//		vehIDMap.insert(pair<std::string, Veins::TraCIMobility*>(vehID, mob));
//	}


//	Veins::TraCIMobility nearestVehMobi;

//	for ( auto i:vehIDMap){
//		if ( i.first.compare(mNodeID) != 0 ){
//			if ( i.second->commandGetLaneIndex() == mVehMob->commandGetLaneIndex() ){
//				double currentDis = fabs(i.second->commandGetOffset() - mVehMob->commandGetOffset());
//				if ( interDis == 0.0 ){
//					interDis = currentDis;
//				}
//				else if ( interDis > currentDis){
//					interDis = currentDis;
//					nearestVehMobi = i.second;
//				}
//			}
//		}
//	}

	double alpha = dmmac->mAlpha;
	double gamma = dmmac->mGamma;
	/*
	 * inter-distance input
	 * */
	/* get the distance to the front vehicle */
	double interDis = 0.0;
	double relativeSpeed = 0.0;
	// based on received status msgs
	string frontVehID = "";
	DMMAC* frontVehPtr;
	for ( auto i:dmmac->mNeighborVec ){
		double angle = mVehMob->getAngleRad();
		// x axis moving, rad is between  (-45, 45),
		// or (135, -135)
		if ( angle < 0.785 && angle > -0.785 ) { // heading east
			if ( i->mVehMob->getSumoPosition().y == dmmac->mVehMob->getSumoPosition().y ){
				// get inter distance
				if ( i->mVehMob->getSumoPosition().x > dmmac->mVehMob->getSumoPosition().x ){
					double currentDis = fabs(i->mVehMob->getSumoPosition().x - dmmac->mVehMob->getSumoPosition().x);
					if ( interDis == 0.0 ){
						interDis = currentDis;
						frontVehID = i->mNodeID;
						frontVehPtr = i;
					}
					else if ( interDis > currentDis ){
						interDis = currentDis;
						frontVehID = i->mNodeID;
						frontVehPtr = i;
					}
				}
			}
		}
		else if ( angle > 2.356 && angle < -2.356 ){ // heading west
			if ( i->mVehMob->getSumoPosition().y == dmmac->mVehMob->getSumoPosition().y ){
				// get inter distance
				if ( i->mVehMob->getSumoPosition().x < dmmac->mVehMob->getSumoPosition().x ){
					double currentDis = fabs(i->mVehMob->getSumoPosition().x - dmmac->mVehMob->getSumoPosition().x);
					if ( interDis == 0.0 ){
						interDis = currentDis;
						frontVehID = i->mNodeID;
						frontVehPtr = i;
					}
					else if ( interDis > currentDis ){
						interDis = currentDis;
						frontVehID = i->mNodeID;
						frontVehPtr = i;
					}
				}
			}
		}
		// y axis moving, rad is between (45, 135) or (-45, -135)
		else if ( angle > -2.356 && angle < -0.785 ) { // heading south
			if ( i->mVehMob->getSumoPosition().x == dmmac->mVehMob->getSumoPosition().x ){
				// get inter distance
				if ( i->mVehMob->getSumoPosition().y < dmmac->mVehMob->getSumoPosition().y ){
					double currentDis = fabs(i->mVehMob->getSumoPosition().x - dmmac->mVehMob->getSumoPosition().x);
					if ( interDis == 0.0 ){
						interDis = currentDis;
						frontVehID = i->mNodeID;
						frontVehPtr = i;
					}
					else if ( interDis > currentDis ){
						interDis = currentDis;
						frontVehID = i->mNodeID;
						frontVehPtr = i;
					}
				}
			}
		}

		else if ( angle > 0.785 && angle < 2.356){ // heading north
			if ( i->mVehMob->getSumoPosition().x == dmmac->mVehMob->getSumoPosition().x ){
				// get inter distance
				if ( i->mVehMob->getSumoPosition().y > dmmac->mVehMob->getSumoPosition().y ){
					double currentDis = fabs(i->mVehMob->getSumoPosition().x - dmmac->mVehMob->getSumoPosition().x);
					if ( interDis == 0.0 ){
						interDis = currentDis;
						frontVehID = i->mNodeID;
						frontVehPtr = i;
					}
					else if ( interDis > currentDis ){
						interDis = currentDis;
						frontVehID = i->mNodeID;
						frontVehPtr = i;
					}
				}
			}
		}
	}

	/*case of no front vehicles in the same lane*/
	if ( interDis == 0.0 ){
		interDis = -1.0;
	}
//	assert(nearVehMobi);

	fisDisInput u_d;
	double tsVj1 = TS*dmmac->mVehMob->getSpeed();
	double tsVj3 = 3*TS*dmmac->mVehMob->getSpeed();

	if ( interDis > 0.0 && interDis < tsVj1){
		u_d = small;
	}
	else if ( interDis >= tsVj1 && interDis < tsVj3 ){
		u_d = medium;
	}
	else if ( interDis >= tsVj3){
		u_d = large;
	}
	else {

	}

	/*
	 * relative speed input
	 * */
	if ( frontVehID != ""){
		relativeSpeed = frontVehPtr->mVehMob->getSpeed() - dmmac->mVehMob->getSpeed();
	}

	fisSpeedInput u_v;
	double alphaDTS = -alpha*interDis/(double)TS;
	double gammaDTS = gamma*interDis/(double)TS;

	if ( relativeSpeed >= gammaDTS ){
		u_v = fast;
	}
	else if ( relativeSpeed >= alphaDTS && relativeSpeed < gammaDTS){
		u_v = same;
	}
	else if ( relativeSpeed < alphaDTS){
		u_v = slow;
	}

	fisAccOuput u_acc;
	if ( u_d == small && u_v == slow){
		u_acc = same_acc;
	}
	else if ( u_d == small && u_v == same ){
		u_acc = decelerate;
	}
	else if ( u_d == small && u_v == fast ){
		u_acc = decelerate;
	}
	else if ( u_d == medium && u_v == slow ){
		u_acc = same_acc;
	}
	else if ( u_d == medium && u_v == same ){
		u_acc = accelerate;
	}
	else if ( u_d == medium && u_v == fast ){
		u_acc = decelerate;
	}
	else if ( u_d == large && u_v == slow ){
		u_acc = accelerate;
	}
	else if ( u_d == large && u_v == same ){
		u_acc = accelerate;
	}
	else if ( u_d == large && u_v == fast ){
		u_acc = same_acc;
	}

	double acc;
	if ( u_acc == accelerate ){
		acc = 1.0;
	}
	else if ( u_acc == same_acc ){
		acc = 0.0;
	}
	else if ( u_acc == decelerate ){
		acc = -1.0;
	}

//	cout << " FIS acc: " << acc << endl;
	return acc;
}

// DMMAC algorithm 1: An adaptive learning mechanism
double DMMAC::ALM(DMMAC* dmmac){
	double aACT = dmmac->mActAcc;
	double epsilon = 0.1;
	double aFIS = FIS(dmmac);
//	double aACT;

	// preprocess for the actual acc
	if ( aACT > 0.1 )
		aACT = 1.0;
	else if ( aACT < -0.1 )
		aACT = -1.0;
	else
		aACT = 0.0;

	// learning process
	if ( aFIS == 0.0 ) {
		if ( aACT == 1.0 )
			dmmac->mAlpha = std::max((1-epsilon)*dmmac->mAlpha, 0.0);
		else if ( aACT == -1.0 )
			dmmac->mGamma = std::max((1-epsilon)*dmmac->mGamma, 0.0);
		else {
			dmmac->mAlpha = dmmac->mAlpha;
			dmmac->mGamma = dmmac->mGamma;
		}
	}
	else if ( aFIS == 1.0 ) {
		if ( aACT == 0.0 || aACT == -1.0 )
			dmmac->mAlpha = (1+epsilon)*dmmac->mAlpha;
		else {
			dmmac->mAlpha = dmmac->mAlpha;
			dmmac->mGamma = dmmac->mGamma;
		}
	}
	else if ( aFIS == -1.0 ) {
		if ( aACT == 0.0 || aACT == 1.0 )
			dmmac->mGamma = (1+epsilon)*dmmac->mGamma;
		else {
			dmmac->mAlpha = dmmac->mAlpha;
			dmmac->mGamma = dmmac->mGamma;
		}
	}

//	cout << "ALM mAlpha: " << mAlpha << ", mGamma: " << mGamma << endl;
	return aFIS;
}

CMACWSM* DMMAC::getPktFromEmgQ(){
	CMACWSM* tmpwsm;

	if ( mUpperLayerEmgPktQueue.empty() )
		return tmpwsm = nullptr;

	while ( !mUpperLayerEmgPktQueue.empty() ){
		tmpwsm = (CMACWSM*)mUpperLayerEmgPktQueue.pop();
		if ( simTime() < (tmpwsm->getTimestamp() + tmpwsm->getTTL()) ){
			break;
		}
		else{
//			gPeQueueExpiredPkts++;
			DBG_MAC << mNodeID << ": a Emg pkt expired and discarded in MAC layer!" << endl;
		}
	}
	return tmpwsm;
}

CMACWSM* DMMAC::getPktFromQ(){
	CMACWSM* tmpwsm;

	if ( mUpperLayerPktQueue.empty() )
		return tmpwsm = nullptr;

	while ( !mUpperLayerPktQueue.empty() ){
		tmpwsm = (CMACWSM*)mUpperLayerPktQueue.pop();
		if ( simTime() < (tmpwsm->getTimestamp() + tmpwsm->getTTL()) ){
			break;
		}
		else{
//			gPeQueueExpiredPkts++;
			DBG_MAC << mNodeID << ": a pkt expired and discarded in MAC layer!" << endl;
		}
	}
	return tmpwsm;
}

CMACWSM* DMMAC::getPktFromQForCH2CH(){
	CMACWSM* tmpwsm;

	if ( mUpperLayerPktQueue.empty() )
		return tmpwsm = nullptr;

	bool forMyCM = false;
	tmpwsm = (CMACWSM*)mUpperLayerPktQueue.front();

//	cout << myNodeID << ": " << endl;
//	for ( auto test: mClusterNeighborsVec)
//		cout << test->getFullName() << " ";
//	cout << endl;

	for ( auto i: mNeighborVec ){
		if ( i != nullptr ){
			if ( strcmp(tmpwsm->getDestID(), i->mNodeID.c_str()) == 0 ){
				forMyCM = true;
				break;
			}
		}
	}

	if ( forMyCM ){
		return tmpwsm = nullptr;
	}
	else {
		mUpperLayerPktQueue.pop();
		return tmpwsm;
	}
}


CMACWSM* DMMAC::getPktFromQForCHDistr(){
	CMACWSM* tmpwsm;

	if ( mUpperLayerPktQueue.empty() )
		return tmpwsm = nullptr;

//	std::map<std::string, cModule*> allVehMap = mVehMob->getManager()->getManagedHosts();
//	std::map<std::string, cModule*>::iterator itVehsMap;
//
//	std::vector<DMMAC*> allVehVec;
//	std::vector<DMMAC*>::iterator itAllVehVec;
//	allVehVec.clear();
//
//	for (itVehsMap = allVehMap.begin(); itVehsMap != allVehMap.end(); itVehsMap++){
//		cModule* macModule = (*itVehsMap).second->getModuleByPath(".nic.mac1609_4");
//		DMMAC* dmmac = check_and_cast<DMMAC*>(macModule);
//
//		allVehVec.push_back(dmmac);
//		(*itVehsMap).second->getDisplayString().updateWith("r=0");
//
//	}
//
//	int maxTxDist;
//	maxTxDist = 75;
//
//	mNeighborVec.clear();
//
//	for ( itAllVehVec = allVehVec.begin(); itAllVehVec != allVehVec.end(); ++itAllVehVec){
//		Veins::TraCIMobility* mobility = (*itAllVehVec)->mVehMob;
//		// inside
//		Veins::TraCIMobility* mobility2 = mVehMob;
//		// find nodes within maxTxDist
//		if ( mobility->getSumoPosition().distance(mobility2->getSumoPosition()) < maxTxDist ){
//			if ( (*itAllVehVec)->mNodeID.compare(mNodeID) != 0 ){
//				mNeighborVec.push_back(*itAllVehVec);
//			}
//		}
//	}

	tmpwsm = (CMACWSM*)mUpperLayerPktQueue.front();
	for ( auto i: mNeighborVec ){
		if ( i != nullptr ){
//			cout << tmpwsm->getDestID() << endl;
//			cout << ": " << i->mNodeID << endl;
			if ( strcmp(tmpwsm->getDestID(), i->mNodeID.c_str()) == 0 ){
				mUpperLayerPktQueue.pop();
				if ( simTime() < (tmpwsm->getTimestamp() + tmpwsm->getTTL()) ){
					return tmpwsm;
				}
				else{
//					gPeQueueExpiredPkts++;
					DBG_MAC << mNodeID << ": a pkt expired and discarded in MAC layer!" << endl;
				}
			}
		}
	}

	return tmpwsm = nullptr;
}

int DMMAC::decideChannel(std::map<std::string, CMInfo> CHMap, Veins::TraCIMobility* VehMob){
	int chanID;

	// decide channel
	if ( CHMap.size() > 0 ){
		std::map<string, CMInfo> frontCHMap;
		std::map<string, CMInfo> rearCHMap;
		// find all front and rear CHs, chose a channel different from them
		for ( auto i: CHMap){
			// x axis moving, rad is between  (-45, 45),
			// or (135, -135)
			double angle = VehMob->getAngleRad();
			if ( angle < 0.785 && angle > -0.785 ) {
				if (i.second.pos.x > VehMob->getSumoPosition().x){
					frontCHMap.insert(i);
				}
				else {
					rearCHMap.insert(i);
				}
			}
			else if ( angle > 2.356 && angle < -2.356 ){
				if (i.second.pos.x < VehMob->getSumoPosition().x){
					frontCHMap.insert(i);
				}
				else {
					rearCHMap.insert(i);
				}
			}
			// y axis moving, rad is between (45, 135) or (-45, -135)
			else if ( angle > -2.356 && angle < -0.785 ) {
				if (i.second.pos.y < VehMob->getSumoPosition().y){
					frontCHMap.insert(i);
				}
				else {
					rearCHMap.insert(i);
				}
			}

			else if ( angle > 0.785 && angle < 2.356){
				if (i.second.pos.y > VehMob->getSumoPosition().y){
					frontCHMap.insert(i);
				}
				else {
					rearCHMap.insert(i);
				}
			}
		}

		/*find the nearest front and rear CH*/
		double xMin = 0.0;
		double xMax = 0.0;
		string xMinID;
		string xMaxID;

		for ( auto j: frontCHMap ){
			if(xMin == 0.0){
				xMin = j.second.pos.x;
				xMinID = j.first;
			}
			else if (xMin > j.second.pos.x){
				xMin = j.second.pos.x;
				xMinID = j.first;
			}
		}

		for ( auto j: rearCHMap ){
			if(xMax == 0.0){
				xMax = j.second.pos.x;
				xMaxID = j.first;
			}
			else if (xMin < j.second.pos.x){
				xMax = j.second.pos.x;
				xMaxID = j.first;
			}
		}
//			cout << "AT4: " << xMinID << ", " << xMaxID <<  endl;

		if ( xMinID.empty() && xMaxID .empty() ){
			chanID = intuniform(1, 3);
		}
		else if ( xMinID.empty() ){
			do{
				chanID = intuniform(1, 3);
			}while ( chanID == CHMap.at(xMaxID).chanID );
		}
		else if ( xMaxID.empty() ){
			do{
				chanID = intuniform(1, 3);
			}while (chanID == CHMap.at(xMinID).chanID );
		}
		else {
			do{
				chanID = intuniform(1, 3);
			}while (chanID == CHMap.at(xMinID).chanID
						|| chanID == CHMap.at(xMaxID).chanID );
		}
	}
	else {
		chanID = intuniform(1, 3);
	}

	return chanID;
}

double DMMAC::getOffsetRatio(Veins::TraCIMobility* mob) {
	string nodeID = mob->getExternalId();
	string laneID = mob->getCommandInterface()->getLaneId(nodeID);
	double laneLength = mob->getCommandInterface()->getLaneLength(laneID);
	double laneOffset = mob->getCommandInterface()->getLanePosition(nodeID);

	return laneOffset/laneLength;
}