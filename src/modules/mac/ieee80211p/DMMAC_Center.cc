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

#include "DMMAC_Center.h"

#define DBG_MAC EV
//#define DBG_MAC std::cerr << "[" << simTime().raw() << "] " << myId << " "

Define_Module(DMMAC_Center);

using namespace std;
#define ROADSPEEDLIMIT 30.556 // 30.556m/s = 110 km/h = 68.35 mph
#define TS 2
#define STATUSMSGINTERVAL 0.2
#define TF 10
#define CHANNELSWITCHGUARD 0.002

const SimTime CCI = SimTime().setRaw(100000000000UL);
const SimTime TA = 6 * SLOTLENGTH_11P;

static bool gFlagClustering = false;

void DMMAC_Center::initialize(int stage) {
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

		myEDCA[type_SCH] = new EDCA(type_SCH,par("queueSize").longValue());
		myEDCA[type_SCH]->myId = myId;
		myEDCA[type_SCH]->myId.append(" SCH");
		myEDCA[type_SCH]->createQueue(2,(((CWMIN_11P+1)/4)-1),(((CWMIN_11P +1)/2)-1),AC_VO);
		myEDCA[type_SCH]->createQueue(3,(((CWMIN_11P+1)/2)-1),CWMIN_11P,AC_VI);
		myEDCA[type_SCH]->createQueue(6,CWMIN_11P,CWMAX_11P,AC_BE);
		myEDCA[type_SCH]->createQueue(9,CWMIN_11P,CWMAX_11P,AC_BK);

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
				setActiveChannel(type_SCH);
				setActiveChannel(type_CCH);
			}

			// channel switching active

			nextChannelSwitch = new cMessage("Channel Switch");
			countCCI = currenTime / switchingTime;
//			cout << "NEW! " << countCCI << endl;
			// channel switching active
			nextCCIEvent = new cMessage("CCI Interval Start Event");

			simtime_t offset = dblrand() * par("syncOffset").doubleValue();
			scheduleAt(simTime() + offset + timeToNextSwitch, nextCCIEvent);
		}
		else {
			// no channel switching
			nextChannelSwitch = 0;
			setActiveChannel(type_CCH);
		}

		// DMMAC center

		mNodeID = findHost()->getFullName();
		mVehMob = (Veins::TraCIMobility*)getOwner()->getOwner()->findObject(
						"veinsmobility", true);
        cm = (ConnectionManager*)getOwner()->getOwner()->getOwner()->findObject(
        					"connectionManager",true);
        phyLayerPtr = FindModule<PhyLayer80211p*>::findSubModule(
                        getParentModule());

//        emgPktSenderID = "";
//        emgReceived = false;
//        countCCI = 1;
//        mCHID = "";
//        mCHBK = "";
        mClusterRole = lone;
        commRange = 150;

        mWSF = 0.0;
        mPreWSF = 0.0;
        mPreSpeed = 0.0;
        mActAcc = 0.0;
        // initial value is -2 because the predicted acc can be -1, 0, 1
        mPredictAcc = -2.0;
        mAlpha = 1.0;
        mGamma = 1.0;
        mLeaveCHTime = 0.0;
//        mCHChanID = 0;
//        mCHWaitTime = 0.0;


		// DMMAC
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

void DMMAC_Center::handleSelfMsg(cMessage* msg) {
	if (msg == nextChannelSwitch) {
		ASSERT(useSCH);

		scheduleAt(simTime() + SWITCHING_INTERVAL_11P, nextChannelSwitch);

		switch (activeChannel) {
			case type_CCH:
				DBG_MAC << "CCH --> SCH" << std::endl;
				channelBusySelf(false);
				setActiveChannel(type_SCH);
				channelIdle(true);
				phy11p->changeListeningFrequency(frequency[mySCH]);
				break;
			case type_SCH:
				DBG_MAC << "SCH --> CCH" << std::endl;
				channelBusySelf(false);
				setActiveChannel(type_CCH);
				channelIdle(true);
				phy11p->changeListeningFrequency(frequency[Channels::CCH]);
				break;
		}
		//schedule next channel switch in 50ms

	}
	else if (msg ==  nextMacEvent) {

		//we actually came to the point where we can send a packet
		channelBusySelf(true);
		CMACWSM* pktToSend = myEDCA[activeChannel]->initiateTransmit(lastIdle);

		lastAC = mapPriority(pktToSend->getPriority());

		DBG_MAC << "MacEvent received. Trying to send packet with priority" << lastAC << std::endl;

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
			DBG_MAC << "Sending a Packet. Frequency " << freq << " Priority" << lastAC << std::endl;
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
	else if (msg == nextCCIEvent){
		ASSERT(useSCH);

		/* channel switching modified for DMMAC */
//		scheduleAt(simTime() + SWITCHING_INTERVAL_11P, nextChannelSwitch);
		scheduleAt(simTime() + CCI, nextCCIEvent);
		countCCI++;
		DBG_MAC << " CCI Count: " << countCCI << endl;

		if ( countCCI >= 10 ){
			// every 1s, do Tf and cluster maintenance
			if ( countCCI % 10 == 0 ){
				if ( gFlagClustering == false ){
					std::map<std::string, cModule*> allVehMap = mVehMob->getManager()->getManagedHosts();
//					int randId = intuniform(0, allVehMap.size());
//	//				cout << randId << endl;
//					string randIdString = std::to_string(randId);

//					std::map<std::string, cModule*>::iterator it;
//					it = allVehMap.find(randIdString);
//					if ( it != allVehMap.end() ){
//						cModule* randModule = it->second;
//						cout << randModule->getFullName() << endl;
//						if ( mNodeID == randModule->getFullName() ){
							startClustering( allVehMap );

//						}
//					}

					gFlagClustering = true;
				}
			}

			if ( countCCI % 7 == 0 ){
				gFlagClustering = false;
			}
		}

		if ( countCCI >= 20 ){
			cout << simTime() << " countCCI: " << countCCI << endl;
//			mbCCIDuration = true;

			// cancel status message event;
//			if ( nextStatusMsgEvent->isScheduled())
//				cancelEvent(nextStatusMsgEvent);

//			cout << myNodeID << ": " << mClusterRole << endl;
//			if ( mClusterRole == CH ){
//				scheduleAt( simTime(), nextCH1stMsgEvent );
//			}
		}
	}
}

void DMMAC_Center::handleUpperControl(cMessage* msg) {
	assert(false);
}

void DMMAC_Center::handleUpperMsg(cMessage* msg) {

	CMACWSM* thisMsg;
	if ((thisMsg = dynamic_cast<CMACWSM*>(msg)) == NULL) {
		error("WaveMac only accepts WaveShortMessages");
	}

	t_access_category ac = mapPriority(thisMsg->getPriority());

	DBG_MAC << "Received a message from upper layer for channel "
	        << thisMsg->getChannelNumber() << " Access Category (Priority):  "
	        << ac << std::endl;

	t_channel chan;

	//rewrite SCH channel to actual SCH the Mac1609_4 is set to
	if (thisMsg->getChannelNumber() == Channels::SCH1) {
		ASSERT(useSCH);
		thisMsg->setChannelNumber(mySCH);
		chan = type_SCH;
	}


	//put this packet in its queue
	if (thisMsg->getChannelNumber() == Channels::CCH) {
		chan = type_CCH;
	}

	int num = myEDCA[chan]->queuePacket(ac,thisMsg);

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

void DMMAC_Center::handleLowerControl(cMessage* msg) {
	if (msg->getKind() == MacToPhyInterface::TX_OVER) {

		DBG_MAC << "Successfully transmitted a packet on " << lastAC << std::endl;

		phy->setRadioState(Radio::RX);

		//message was sent
		//update EDCA queue. go into post-transmit backoff and set cwCur to cwMin
		myEDCA[activeChannel]->postTransmit(lastAC);
		//channel just turned idle.
		//don't set the chan to idle. the PHY layer decides, not us.

		if (guardActive()) {
			opp_error("We shouldnt have sent a packet in guard!");
		}
	}
	else if (msg->getKind() == Mac80211pToPhy11pInterface::CHANNEL_BUSY) {
		channelBusy();
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

void DMMAC_Center::setActiveChannel(t_channel state) {
	activeChannel = state;
	assert(state == type_CCH || (useSCH && state == type_SCH));
}

void DMMAC_Center::finish() {
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

void DMMAC_Center::attachSignal(Mac80211Pkt* mac, simtime_t startTime, double frequency) {

	simtime_t duration = getFrameDuration(mac->getBitLength());

	Signal* s = createSignal(startTime, duration, txPower, bitrate, frequency);
	MacToPhyControlInfo* cinfo = new MacToPhyControlInfo(s);

	mac->setControlInfo(cinfo);
}

Signal* DMMAC_Center::createSignal(simtime_t start, simtime_t length, double power, double bitrate, double frequency) {
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
bool DMMAC_Center::guardActive() const {
	if (!useSCH) return false;
	if (simTime().dbl() - nextChannelSwitch->getSendingTime() <= GUARD_INTERVAL_11P)
		return true;
	return false;
}

/* returns the time until the guard is over */
simtime_t DMMAC_Center::timeLeftTillGuardOver() const {
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
simtime_t DMMAC_Center::timeLeftInSlot() const {
	ASSERT(useSCH);
	return nextChannelSwitch->getArrivalTime() - simTime();
}

/* Will change the Service Channel on which the mac layer is listening and sending */
void DMMAC_Center::changeServiceChannel(int cN) {
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

void DMMAC_Center::handleLowerMsg(cMessage* msg) {
	Mac80211Pkt* macPkt = static_cast<Mac80211Pkt*>(msg);
	ASSERT(macPkt);

	CMACWSM*  wsm =  dynamic_cast<CMACWSM*>(macPkt->decapsulate());

	long dest = macPkt->getDestAddr();

	DBG_MAC << "Received frame name= " << macPkt->getName()
	        << ", myState=" << " src=" << macPkt->getSrcAddr()
	        << " dst=" << macPkt->getDestAddr() << " myAddr="
	        << myMacAddress << std::endl;

	if (macPkt->getDestAddr() == myMacAddress) {
		DBG_MAC << "Received a data packet addressed to me." << std::endl;
		statsReceivedPackets++;
		sendUp(wsm);
	}
	else if (dest == LAddress::L2BROADCAST) {
		statsReceivedBroadcasts++;
		sendUp(wsm);
	}
	else {
		DBG_MAC << "Packet not for me, deleting..." << std::endl;
		delete wsm;
	}
	delete macPkt;
}

int DMMAC_Center::EDCA::queuePacket(t_access_category ac,CMACWSM* msg) {

	if (maxQueueSize && myQueues[ac].queue.size() >= maxQueueSize) {
		delete msg;
		return -1;
	}
	myQueues[ac].queue.push(msg);
	return myQueues[ac].queue.size();
}

int DMMAC_Center::EDCA::createQueue(int aifsn, int cwMin, int cwMax,t_access_category ac) {

	if (myQueues.find(ac) != myQueues.end()) {
		opp_error("You can only add one queue per Access Category per EDCA subsystem");
	}

	EDCAQueue newQueue(aifsn,cwMin,cwMax,ac);
	myQueues[ac] = newQueue;

	return ++numQueues;
}

DMMAC_Center::t_access_category DMMAC_Center::mapPriority(int prio) {
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

CMACWSM* DMMAC_Center::EDCA::initiateTransmit(simtime_t lastIdle) {

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

simtime_t DMMAC_Center::EDCA::startContent(simtime_t idleSince,bool guardActive) {

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


			DBG_MAC << "Waiting Time for Queue " << iter->first <<  ":" << possibleNextEvent << "=" << iter->second.aifsn << " * "  << SLOTLENGTH_11P << " + " << SIFS_11P << "+" << iter->second.currentBackoff << "*" << SLOTLENGTH_11P << "; Idle time: " << idleTime << std::endl;

			if (idleTime > possibleNextEvent) {
				DBG_MAC << "Could have already send if we had it earlier" << std::endl;
				//we could have already sent. round up to next boundary
				simtime_t base = idleSince + DIFS;
				possibleNextEvent =  simTime() - simtime_t().setRaw((simTime() - base).raw() % SLOTLENGTH_11P.raw()) + SLOTLENGTH_11P;
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

void DMMAC_Center::EDCA::stopContent(bool allowBackoff, bool generateTxOp) {
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
void DMMAC_Center::EDCA::backoff(t_access_category ac) {
	myQueues[ac].currentBackoff = intuniform(0,myQueues[ac].cwCur);
	statsSlotsBackoff += myQueues[ac].currentBackoff;
	statsNumBackoff++;
	DBG_MAC << "Going into Backoff because channel was busy when new packet arrived from upperLayer" << std::endl;
}

void DMMAC_Center::EDCA::postTransmit(t_access_category ac) {
	delete myQueues[ac].queue.front();
	myQueues[ac].queue.pop();
	myQueues[ac].cwCur = myQueues[ac].cwMin;
	//post transmit backoff
	myQueues[ac].currentBackoff = intuniform(0,myQueues[ac].cwCur);
	statsSlotsBackoff += myQueues[ac].currentBackoff;
	statsNumBackoff++;
	DBG_MAC << "Queue " << ac << " will go into post-transmit backoff for " << myQueues[ac].currentBackoff << " slots" << std::endl;
}

void DMMAC_Center::EDCA::cleanUp() {
	for (std::map<t_access_category, EDCAQueue>::iterator iter = myQueues.begin(); iter != myQueues.end(); iter++) {
		while (iter->second.queue.size() != 0) {
			delete iter->second.queue.front();
			iter->second.queue.pop();
		}
	}
	myQueues.clear();
}

void DMMAC_Center::EDCA::revokeTxOPs() {
	for (std::map<t_access_category, EDCAQueue>::iterator iter = myQueues.begin(); iter != myQueues.end(); iter++) {
		if (iter->second.txOP == true) {
			iter->second.txOP = false;
			iter->second.currentBackoff = 0;
		}
	}
}

void DMMAC_Center::channelBusySelf(bool generateTxOp) {

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

void DMMAC_Center::channelBusy() {

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

void DMMAC_Center::channelIdle(bool afterSwitch) {

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
			scheduleAt(nextEvent,nextMacEvent);
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

void DMMAC_Center::setParametersForBitrate(int bitrate) {
	for (unsigned int i = 0; i < NUM_BITRATES_80211P; i++) {
		if (bitrate == BITRATES_80211P[i]) {
			n_dbps = N_DBPS_80211P[i];
			return;
		}
	}
	opp_error("Chosen Bitrate is not valid for 802.11p: Valid rates are: 3Mbps, 4.5Mbps, 6Mbps, 9Mbps, 12Mbps, 18Mbps, 24Mbps and 27Mbps. Please adjust your omnetpp.ini file accordingly.");
}


simtime_t DMMAC_Center::getFrameDuration(int payloadLengthBits) const {
	// calculate frame duration according to Equation (17-29) of the IEEE 802.11-2007 standard
	simtime_t duration = PHY_HDR_PREAMBLE_DURATION + PHY_HDR_PLCPSIGNAL_DURATION + T_SYM_80211P * ceil( (16 + payloadLengthBits + 6)/(n_dbps) );

	return duration;
}

void DMMAC_Center::startClustering(std::map<std::string, cModule*> allVehMap){

	std::map<std::string, cModule*>::iterator itVehsMap;

	std::vector<DMMAC_Center*> allVehVec;
	std::vector<DMMAC_Center*>::iterator itAllVehVec;
	allVehVec.clear();

	for (itVehsMap = allVehMap.begin(); itVehsMap != allVehMap.end(); itVehsMap++){
		Veins::TraCIMobility* i_Mob;
		i_Mob = (Veins::TraCIMobility*)itVehsMap->second->getSubmodule("veinsmobility");

		cModule* macModule = (*itVehsMap).second->getModuleByPath(".nic.mac1609_4");
		DMMAC_Center* dmmac = check_and_cast<DMMAC_Center*>(macModule);
		i_Mob = dmmac->mVehMob;
		/* do not cluster vehicles going to disappear,
		 * at the end of a road segment
		 * */
		if ( getOffsetRatio(i_Mob) <= 0.98 ) {
			allVehVec.push_back(dmmac);
			(*itVehsMap).second->getDisplayString().updateWith("r=0");
//			dmmac->mClusterRole = lone;
		}
		else {

			dmmac->mClusterRole = idle;
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
	maxTxDist = 150;

	for ( itAllVehVec = allVehVec.begin(); itAllVehVec != allVehVec.end(); ++itAllVehVec){
		std::vector<DMMAC_Center*>::iterator itAllVehVec2;
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

	for ( auto i:allVehVec ){
		cout << i->mNodeID << " : " << endl;
		cout << "          ";
		for ( auto j: i->mNeighborVec ){
			cout << j->mNodeID << " ";
		}
		cout << endl;
	}

	/*purge vehStatusMap, remove expired entries*/
//	vector<string> expiredEntry;
//	for ( auto i:vehStatusMap ){
//		if ( simTime() - i.second.timeStamp > i.second.ttl ){
//			expiredEntry.push_back(i.first);
//		}
//	}
//	for ( auto i: expiredEntry ){
//		vehStatusMap.erase(i);
//	}
//	expiredEntry.clear();

	// clear mCMMap
//	mCMMap.clear();

	cout 	<< setw(10) << "nodeID" << " | "
			<< setw(9) << "betaWSF" << " | "
			<< setw(8) << "speed" << " | "
			<< setw(19) << "pos" << " | "
			<< setw(8) << "acc" << " | "
			<< setw(4) << "R"<< " | "
			<< setw(10) << "CHID" << " | "
			<< setw(10) << "CHBK" << " | "
//				<< setw(1) << "cID" << " | "b
//				<< setw(10) << "destID"<< " | "
//				<< setw(10) << "senderID"<< " | "
			<< endl;


	for ( itAllVehVec = allVehVec.begin(); itAllVehVec != allVehVec.end(); ++itAllVehVec){
		DMMAC_Center* dmmac = *itAllVehVec;
		// update actual acc, m/s^2
		dmmac->mActAcc = (dmmac->mVehMob->getSpeed()-dmmac->mPreSpeed)/(double)(simTime().dbl()-dmmac->mPreSpeedSampleTime.dbl());
		// calculate predicted acc
		dmmac->mPredictAcc = ALM(dmmac);

		dmmac->mWSF = calWSFBeta(dmmac);

//		cout << dmmac->mNodeID << endl;
//		cout << "| WSF: " << dmmac->mWSF <<" | mActAcc: " << dmmac->mActAcc
//				<< " | mPredictAcc: " << dmmac->mPredictAcc << endl;
		cout 	<< setw(10) << dmmac->mNodeID << " | "
				<< setw(9) << dmmac->mWSF << " | "
				<< setw(8) << dmmac->mVehMob->getSpeed() << " | "
				<< setw(2) << dmmac->mVehMob->getSumoPosition() << " | "
				<< setw(8) << dmmac->mPredictAcc << " | "
				<< setw(4) << dmmac->commRange << " | "
				<< setw(10) << dmmac->mCHID << " | "
				<< setw(10) << dmmac->mCHBK << " | "
//				<< setw(3) << i.second.chanID << " | "
//				<< setw(10) << i.second.destID << " | "
//				<< setw(10) << i.second.senderID << " | "
				<< endl;
	}

	std::vector<DMMAC_Center*> aV4C = allVehVec;
	cout << "BEGIN: aV4C.size(): " << aV4C.size() << endl;
	while ( aV4C.size() > 0 ){
		DMMAC_Center* dmmacMax;
		double maxWSF = 0.0;
		std::vector<DMMAC_Center*>::iterator it_a;
		for ( it_a = aV4C.begin(); it_a != aV4C.end(); ++it_a){
			DMMAC_Center* dmmac = *it_a;
			if ( maxWSF == 0.0 ){
				dmmacMax = dmmac;
				maxWSF = dmmac->mWSF;
			}
			else if ( maxWSF < dmmac->mWSF ){
				maxWSF = dmmac->mWSF;
				dmmacMax = dmmac;
			}
		}

		cout << "CH: " << dmmacMax->mNodeID << endl;
		cout << "-------> CM: ";
		// remove CH from the vector
		if ( it_a != aV4C.end() ){
			aV4C.erase(it_a);
		}

		// all neighbors of the maxWSF veh become its CMs
		if ( maxWSF != 0.0 ){
			dmmacMax->mClusterRole = CH;
			dmmacMax->mCHID = dmmacMax->mNodeID;

			std::vector<DMMAC_Center*>::iterator it_b;

			for ( auto i:dmmacMax->mNeighborVec){
				if ( find(aV4C.begin(), aV4C.end(), i) != aV4C.end() ){
					i->mClusterRole = CM;
					i->mCHID = dmmacMax->mNodeID;
					cout << i->mNodeID << " ";
				}
			}
			cout << endl;
		}
		else {
			opp_error("maxWSF is 0! ");
		}

		// remove assigned vehicles from the vector
		std::vector<DMMAC_Center*> nVeh = dmmacMax->mNeighborVec;
		for ( it_a = aV4C.begin(); it_a != aV4C.end(); ++it_a){
			if ( !((*it_a)->mCHID.empty()) ){
				aV4C.erase(it_a);
				it_a--;
			}
		}

		cout << "dmmacMax->mNeighborVec.size(): " << dmmacMax->mNeighborVec.size() << endl;
		cout << "aV4C.size(): " << aV4C.size() << endl;

	}

/*
	// clustering in distributed way
	for ( itAllVehVec = allVehVec.begin(); itAllVehVec != allVehVec.end(); ++itAllVehVec){
		DMMAC_Center* dmmac = *itAllVehVec;
		// Clustering
		if ( dmmac->mClusterRole == lone ){
			loneTransition(dmmac);
			if ( dmmac->mClusterRole == tempCH ){
				tempCHTransition(dmmac);
			}

		}
		else if ( dmmac->mClusterRole == tempCH ){
			tempCHTransition(dmmac);
		}
		else if ( dmmac->mClusterRole == CH ){
			CHTransition(dmmac);

			// calculate expected position of CMs
			vector<string> CMwithinRange;
			for ( auto i:dmmac->mNeighborVec){
				double expectMoveDis = i->mVehMob->getSpeed()*TF + 0.5*i->mPredictAcc*pow(TF,2);
				double disToCH = dmmac->mVehMob->getSumoPosition().distance(i->mVehMob->getSumoPosition());
				double range = dmmac->commRange;
				if ( disToCH + expectMoveDis < range )
					CMwithinRange.push_back(i->mNodeID);
			}

			// 10% CMs left, handover CH role to backup CH
			double ratio = 1 - CMwithinRange.size()/(double)(dmmac->mNeighborVec.size());
			if ( ratio > 0.1 ){
				dmmac->mClusterRole = CM;
				dmmac->mCHID = mCHBK;
//				findHost()->getDisplayString().updateWith("r=6,green");
				DBG_MAC << dmmac->mNodeID << " CH->CM due to 10% CM, mCHID = mCHBK: " << dmmac->mCHID << endl;
			}

			// a CH is in 2/3 of the neighbor CH range, handover to backup CH
			for ( auto i:dmmac->mNeighborVec ){
				// find the vehicle is CH in my statusMap
				if ( i->mNodeID.compare(i->mCHID) == 0 ){
					double dist = dmmac->mVehMob->getSumoPosition().distance(i->mVehMob->getSumoPosition());
					if ( dist/(i->commRange) < 0.6667 ){
						if ( !(dmmac->mCHBK.empty()) ){
							dmmac->mClusterRole = CM;
							dmmac->mCHID = mCHBK;
//							findHost()->getDisplayString().updateWith("r=6,green");
							DBG_MAC << dmmac->mNodeID << " CH->CM due to 2/3 neighbor, mCHID = mCHBK: " << dmmac->mCHID << endl;
							break;
						}
					}
				}
			}

		}
		else if ( dmmac->mClusterRole == CM ){
			// a vehicle leaves a CH's range, wait 3 rounds of CCI, switch ch to ch4

			if ( dmmac->mLeaveCHTime != 0.0 ){
				if ( simTime() - dmmac->mLeaveCHTime > 3*CCI ){
					// switch channel to CCH

					dmmac->mClusterRole = lone;
					dmmac->mCHID = "";
					dmmac->mCHBK = "";
					dmmac->mLeaveCHTime = 0.0;
				}
			}
			else {
				for ( auto i:dmmac->mNeighborVec ){
					if ( i->mClusterRole == CH  ){
						// check if a CM leaves the range of its CH
						if ( dmmac->mVehMob->getSumoPosition().distance(i->mVehMob->getSumoPosition())
															> i->commRange){
							dmmac->mLeaveCHTime = simTime();
						}
						break;
					}
				}
			}
		}
		else if ( dmmac->mClusterRole == backCH ){

			for ( auto i:dmmac->mNeighborVec ){
				if ( i->mClusterRole == CH  ){
					// if the current CH transit to CM, the backCH becomes CH
					if ( i->mCHID.compare(dmmac->mNodeID) == 0 ){
						dmmac->mClusterRole = CH;
						dmmac->mCHID = mNodeID;
						DBG_MAC << dmmac->mNodeID << " backCH->CH, mCHID = mCHBK: " << dmmac->mCHID << endl;
					}
					break;
				}
			}
		}
	}
*/

/*debug*/
	for ( itAllVehVec = allVehVec.begin(); itAllVehVec != allVehVec.end(); ++itAllVehVec){
		DMMAC_Center* dmmac = *itAllVehVec;
		cout << dmmac->mNodeID << ", ";
		cout << "mRole: " << dmmac->mClusterRole << endl;
	}
/*debug*/
}

double DMMAC_Center::getOffsetRatio(Veins::TraCIMobility* mob) {
	string nodeID = mob->getExternalId();
	string laneID = mob->getCommandInterface()->getLaneId(nodeID);
	double laneLength = mob->getCommandInterface()->getLaneLength(laneID);
	double laneOffset = mob->getCommandInterface()->getLanePosition(nodeID);

	return laneOffset/laneLength;
}

double DMMAC_Center::calWSFBeta(DMMAC_Center* dmmac){
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

double DMMAC_Center::FIS(DMMAC_Center* dmmac){

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
//		if ( i.first.compare(myNodeID) != 0 ){
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
	DMMAC_Center* frontVehPtr;
	for ( auto i:dmmac->mNeighborVec ){
//		if ( simTime() - i.second.timeStamp <= i.second.ttl ){
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
//		}
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
double DMMAC_Center::ALM(DMMAC_Center* dmmac){
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

void DMMAC_Center::loneTransition(DMMAC_Center* dmmac){
	/*search the max WSF and the ID*/
	double maxWSFinStatusMap = 0.0;
	string vehIDMaxWSF;
	for ( auto i:dmmac->mNeighborVec){
		if ( maxWSFinStatusMap == 0.0 ){
//			if ( i->mCHID.empty() || i->mCHID.compare(i->mNodeID) != 0 ){
				maxWSFinStatusMap = i->mWSF;
				vehIDMaxWSF = i->mNodeID;
//			}
		}
		else if ( maxWSFinStatusMap < i->mWSF ){
//			if ( i->mCHID.empty() /*|| i->mCHID.compare(i->mNodeID) != 0*/ ){
				maxWSFinStatusMap = i->mWSF;
				vehIDMaxWSF = i->mNodeID;
//			}
		}
	}

	// if I has the highest WSF, becomes CH
	if ( dmmac->mWSF > maxWSFinStatusMap ){
		dmmac->mClusterRole = tempCH;
		dmmac->mCHID = dmmac->mNodeID;
	}
	// if tier, larger ID is selected
	else if ( dmmac->mWSF == maxWSFinStatusMap ){
		string::size_type p1_1 = dmmac->mNodeID.find('[');
		string::size_type p1_2 = dmmac->mNodeID.find(']');
		int myID = std::stoi(dmmac->mNodeID.substr(p1_1+1, p1_2-p1_1-1));
		string::size_type p2_1 = vehIDMaxWSF.find('[');
		string::size_type p2_2 = vehIDMaxWSF.find(']');
		int otherID = std::stoi(vehIDMaxWSF.substr(p2_1+1, p2_2-p2_1-1));

		if ( myID > otherID ){
			dmmac->mClusterRole = tempCH;
			dmmac->mCHID = dmmac->mNodeID;
		}
	}
	// otherwise, I shall be CM, set CHID to the ID with highest WSF
	else {
		dmmac->mClusterRole = CM;
		dmmac->mCHID = vehIDMaxWSF;

		DBG_MAC << dmmac->mNodeID << " is CM, my CH ID: " << dmmac->mCHID << endl;
		cout << dmmac->mNodeID << " is CM, my CH ID: " << dmmac->mCHID << endl;
//		if ( dmmac->mCHID.empty() )
//			findHost()->getDisplayString().updateWith("r=6,red");
//		else
//			findHost()->getDisplayString().updateWith("r=6,green");
	}
}

void DMMAC_Center::tempCHTransition(DMMAC_Center* dmmac){
	bool flag1 = true;
	vector<DMMAC_Center*> neighborCHVec;
	// check if I'm in 1/2 range of another CH
	for ( auto i:dmmac->mNeighborVec){
		if ( !(i->mCHID.empty()) ){
			if ( i->mCHID.compare(i->mNodeID) == 0 ){
				double disTwoCH = dmmac->mVehMob->getSumoPosition().distance(i->mVehMob->getSumoPosition() );
				if ( disTwoCH < i->commRange/2.0 ){
					neighborCHVec.push_back(i);
					flag1 = false;
				}
			}
		}
	}

	// if I'm not in 1/2 range of another CH, I become CH
	if ( flag1 == true ){
		dmmac->mClusterRole = CH;
		dmmac->mCHID = dmmac->mNodeID;
//		findHost()->getDisplayString().updateWith("r=6,blue");
		DBG_MAC << dmmac->mNodeID << " tempCH->CH, mCHID: " << dmmac->mCHID << endl;
		cout << dmmac->mNodeID << " tempCH->CH, mCHID: " << dmmac->mCHID << endl;
	}
	// else, merge as CM
	else {
		if ( neighborCHVec.size() == 1 ){
			dmmac->mClusterRole = CM;
			dmmac->mCHID = neighborCHVec.at(0)->mNodeID;
//			findHost()->getDisplayString().updateWith("r=6,green");
			DBG_MAC << dmmac->mNodeID << " tempCH->CM, mCHID: " << dmmac->mCHID << endl;
			cout << dmmac->mNodeID << " tempCH->CM, mCHID: " << dmmac->mCHID << endl;
		}
		else {
			/*search the nearest CH*/
			double minDistToCH = 0.0;
			string minDistVehID;
			for ( auto i: neighborCHVec ){
				double dist = dmmac->mVehMob->getSumoPosition().distance(i->mVehMob->getSumoPosition());
				if ( minDistToCH == 0.0 ){
					minDistToCH = dist;
					minDistVehID = i->mNodeID;
				}
				else if ( minDistToCH > dist){
					minDistToCH = dist;
					minDistVehID = i->mNodeID;
				}
			}
			dmmac->mClusterRole = CM;
			dmmac->mCHID = minDistVehID;
//			findHost()->getDisplayString().updateWith("r=6,green");
			DBG_MAC << dmmac->mNodeID << " tempCH->CM, mCHID: " << dmmac->mCHID << endl;
			cout << dmmac->mNodeID << " tempCH->CM, mCHID: " << dmmac->mCHID << endl;
		}
	}
}

void DMMAC_Center::CHTransition(DMMAC_Center* dmmac){
	/*search the max WSF and the ID*/
	double maxWSFinStatusMap = 0.0;
	string vehIDMaxWSF;
	for ( auto i:dmmac->mNeighborVec){
		if ( maxWSFinStatusMap == 0.0 ){
			maxWSFinStatusMap = i->mWSF;
			vehIDMaxWSF = i->mNodeID;
		}
		else if ( maxWSFinStatusMap < i->mWSF ){
			maxWSFinStatusMap = i->mWSF;
			vehIDMaxWSF = i->mNodeID;
		}
	}
//			int myIDt = std::stoi(myNodeID);
//			int otherIDt = std::stoi(vehIDMaxWSF);
//			cout << myIDt << " : " << otherIDt << endl;
	// if I have the highest WSF, becomes CH
	if ( dmmac->mWSF > maxWSFinStatusMap ){
		dmmac->mClusterRole = CH;
		dmmac->mCHBK = vehIDMaxWSF;
//		findHost()->getDisplayString().updateWith("r=6,blue");
		DBG_MAC << dmmac->mNodeID << " CH->CH, mCHBK: " << dmmac->mCHBK << endl;
	}
	// if tier, larger ID is selected
	else if ( dmmac->mWSF == maxWSFinStatusMap ){
		string::size_type p1_1 = dmmac->mNodeID.find('[');
		string::size_type p1_2 = dmmac->mNodeID.find(']');
		int myID = std::stoi(dmmac->mNodeID.substr(p1_1+1, p1_2-p1_1-1));
		string::size_type p2_1 = vehIDMaxWSF.find('[');
		string::size_type p2_2 = vehIDMaxWSF.find(']');
		int otherID = std::stoi(vehIDMaxWSF.substr(p2_1+1, p2_2-p2_1-1));

		if ( myID > otherID ){
			dmmac->mClusterRole = CH;
			dmmac->mCHBK = vehIDMaxWSF;
//			findHost()->getDisplayString().updateWith("r=6,blue");
			DBG_MAC << dmmac->mNodeID << " CH->CH, mCHBK: " << dmmac->mCHBK << endl;
		}
	}
	// otherwise, I shall still be CH, set CHBK to the ID with highest WSF
	else {
		// Difference: role is still CH, just set CHBK as the vehicle with max WSF
		dmmac->mClusterRole = CH;
		dmmac->mCHBK = vehIDMaxWSF;
//		findHost()->getDisplayString().updateWith("r=6,blue");
		DBG_MAC << dmmac->mNodeID << " CH->CH, mCHBK: " << dmmac->mCHBK << endl;
	}
}

void DMMAC_Center::handleMACLayerMsg(cMessage* msg) {

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

	//put this packet in its queue
	if (thisMsg->getChannelNumber() == Channels::CCH) {
		chan = type_CCH;
	}

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
//				cout << myNodeID << ": nextMacEvent: " << nextEvent << endl;
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
