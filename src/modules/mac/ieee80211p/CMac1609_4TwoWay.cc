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

#include "CMac1609_4TwoWay.h"

#define DBG_MAC EV
//#define DBG_MAC std::cerr << "[" << simTime().raw() << "] " << myId << " "

// Chris
//typedef tuple<cModule*, int, int> t_Tuple;

//const simsignalwrap_t CMac1609_4TwoWay::roundCountSignal = simsignalwrap_t(MIXIM_SIGNAL_ROUND);
const simsignalwrap_t CMac1609_4TwoWay::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);



#include "modules/mobility/traci/TraCIScenarioManagerLaunchd.h"
#include "modules/application/traci/CMACApp11pTwoWay.h"

using Veins::TraCIScenarioManagerLaunchd;


Define_Module(CMac1609_4TwoWay);

//static bool debugMAC = true;

// control flag
//static bool g_bCMUploadingInSequence =true;
//static bool g_bSignal = false;
static bool g_bNewVeh = false;
static double gdLastClusTime = 0.0;
static double gdClusInterval = 0.5;
static double roundEnd = 0.0;
static bool gMeasureMACDelay = false;

static int gPeNumVeh = 0;
static int gPeClusterSize = 0;
static int gPeClusterLayers = 0;
static int gPeClusterNum = 0;

static long gPeQueuePkts;
static long gPeQueueDelPkts;
static long gPeQueueExpiredPkts;
static long gPeBitErrPkts;
static long gPePhyDropPkts;
static long gPeTxWhileSendPkts;

static std::vector<int> g_vSlotCountByTSAlgo;
static std::vector<int> g_vSlotCountOptimal;

static std::string gLatestNodeID;

static std::vector<int> g_twoWayChannelList = {2, 4};

//static bool g_bSaveDataToFile = false;
// Chris ends

void CMac1609_4TwoWay::initialize(int stage) {
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

//        myEDCA[type_CCH] = new EDCA(type_CCH,par("queueSize").longValue());
//        myEDCA[type_CCH]->myId = myId;
//        myEDCA[type_CCH]->myId.append(" CCH");
//
//        myEDCA[type_CCH]->createQueue(2,(((CWMIN_11P+1)/4)-1),(((CWMIN_11P +1)/2)-1),AC_VO);
//        myEDCA[type_CCH]->createQueue(3,(((CWMIN_11P+1)/2)-1),CWMIN_11P,AC_VI);
//        myEDCA[type_CCH]->createQueue(6,CWMIN_11P,CWMAX_11P,AC_BE);
//        myEDCA[type_CCH]->createQueue(9,CWMIN_11P,CWMAX_11P,AC_BK);
//
//        myEDCA[type_SCH] = new EDCA(type_SCH,par("queueSize").longValue());
//        myEDCA[type_SCH]->myId = myId;
//        myEDCA[type_SCH]->myId.append(" SCH");
//
//        myEDCA[type_SCH]->createQueue(2,(((CWMIN_11P+1)/4)-1),(((CWMIN_11P +1)/2)-1),AC_VO);
//        myEDCA[type_SCH]->createQueue(3,(((CWMIN_11P+1)/2)-1),CWMIN_11P,AC_VI);
//        myEDCA[type_SCH]->createQueue(6,CWMIN_11P,CWMAX_11P,AC_BE);
//        myEDCA[type_SCH]->createQueue(9,CWMIN_11P,CWMAX_11P,AC_BK);

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
//            uint64_t currenTime = simTime().raw();
//            uint64_t switchingTime = SWITCHING_INTERVAL_11P.raw();
//            double timeToNextSwitch = (double)(switchingTime
//                    - (currenTime % switchingTime)) / simTime().getScale();
//            if ((currenTime / switchingTime) % 2 == 0) {
//                setActiveChannel(type_CCH);
//            }
//            else {
//                setActiveChannel(type_SCH);
//            }
//
//            // channel switching active
//            nextChannelSwitch = new cMessage("Channel Switch");
//            simtime_t offset = dblrand() * par("syncOffset").doubleValue();
//            scheduleAt(simTime() + offset + timeToNextSwitch, nextChannelSwitch);
        }
        else {
            // no channel switching
            nextChannelSwitch = 0;
            setActiveChannel(type_CCH);
        }

        // Chris
        if (FindModule<TraCIScenarioManagerLaunchd*>::findGlobalModule()){
            TraCIScenarioManagerLaunchd* managerL = FindModule<TraCIScenarioManagerLaunchd*>::findGlobalModule();
            seed = managerL->par("seed");
            std::string fileN = managerL->par("launchConfig").xmlValue()->detailedInfo();
            std::size_t pos1 = fileN.find("sumo.");
            if (pos1!=std::string::npos){
                std::size_t pos2 = fileN.find(".cfg");
                if (pos2!=std::string::npos){
                    density = std::stoi(fileN.substr(pos1+5, pos2-(pos1+5)));
//                     std::cout << density << endl;
//                     std::cout << fileN.substr(pos1+5, pos2-(pos1+5)) << endl;
                }
            }
            else{
                error("no density!");
            }
        }
        else{
            error("can't find module!");
        }

        if (FindModule<CMACApp11pTwoWay*>::findGlobalModule()){
            CMACApp11pTwoWay* appL = FindModule<CMACApp11pTwoWay*>::findGlobalModule();
            d_bgTrafficInterval = appL->par("appBGTrafficInterval").doubleValue();
        }
        else{
            error("can't find module!");
        }

        schemeName = "ECMAC";
//        gLatestNodeID = findHost()->getFullName();
        /*initialization*/
        debugMAC = par("debugMAC").boolValue();
        macSaveToFile = par("macSaveToFile").boolValue();

        CMUploadByTimeslotAlgo = par("CMUploadByTimeslotAlgo").boolValue();
        CHDistriBySequence = par("CHDistriBySequence").boolValue();

        vehMobility = NULL;
        vehMobility = (Veins::TraCIMobility*)getOwner()->getOwner()->findObject(
        					"veinsmobility", true);
        m_activeVehCount = 0;

        myOptimalLayer = 0;
        myOptimalClusterSize = 0;

        mClusterHead = nullptr;
        mCH = "";
        mCHMacAdd = 0;

        mNeighborVec.clear();
        timeSlotAlloVec.clear();
        timeSlotAlloStrVec.clear();

        mRoleInCluster = IDLE;
        clusterIndex = -1;
        mChannelID = -1;
        mClusterChannelID = -1;

        myNodeID = findHost()->getFullName();

        chSwitchGuardInterval = 0.002;
        myMaxTimeSlotSize = -1;
        myNumCluster = -1;

        myCMInfoMap.clear();
        orCMInfoMap.clear();
        myUpperLayerPktQueue.clear();

        round = 0;
        numRounds = 10;

        findHost()->subscribe(mobilityStateChangedSignal, this);
//        findHost()->subscribe(roundCountSignal, this);

        /*set communication range*/

        cm = (ConnectionManager*)getOwner()->getOwner()->getOwner()->findObject(
        					"connectionManager",true);
//        cm->parPMax = 20;

		// change tx power
		par("txPower").setDoubleValue(5);
		txPower = par("txPower");
		// connection manager
		cModule* cmModule = check_and_cast<cModule*>(cm);
		cmModule->par("pMax").setDoubleValue(5);

        maxTxDist = cm->calcInterfDist();

        phyLayerPtr = FindModule<PhyLayer80211p*>::findSubModule(
                        getParentModule());

        phyLayerPtr->par("maxTXPower").setDoubleValue(2.5);
        phyLayerPtr->setMaxTXPower(2.5);
/*
 * 		cout << "1 phyLayerPtr: "<< phyLayerPtr->par("maxTXPower").doubleValue() << " "
        		<< phyLayerPtr->getMaxTXPower()<< endl;
        phyLayerPtr->par("maxTXPower").setDoubleValue(2.5);
        phyLayerPtr->setMaxTXPower(2.5);
        cout << "2 phyLayerPtr: "<< phyLayerPtr->par("maxTXPower").doubleValue() << " "
        		<< phyLayerPtr->getMaxTXPower()<< endl;
*/

        nextClusteringEvent = new cMessage("next Cluster Event");
        nextTAFEvent = new cMessage("next TAF Event");
        nextCheckEvent = new cMessage("next Check Event");
//        nextACKEvent = new cMessage("next ACK Event");
        nextChannelAssignment = new cMessage("next Channel Assignment");

        startEvent = new cMessage("start Event");
        nextPrepareUploadEvent = new cMessage("next Prepare Upload Event");
        nextUploadEvent = new cMessage("next Upload Event");
        nextPrepareCHsCommuEvent = new cMessage("next Prepare CHs Communication Event");
        nextCHsCommuEvent = new cMessage("next CHs Communication Event");
        nextPrepareCHsDistrEvent = new cMessage("next Prepare CHs Distribution Event");
        nextCHsDistrEvent = new cMessage("next CHs Distribution Event");
        nextConnectEvent = new cMessage("next Connect Event");

        if (strcmp(findHost()->getName(), "node") == 0  ){
        	// the initial vehicles
        	if ( simTime() < 0.1001){
            	simtime_t offset = dblrand() * par("syncOffset").doubleValue();
            	scheduleAt( simTime() + offset, startEvent );
            	scheduleAt( simTime() + chSwitchGuardInterval, nextCheckEvent );
        	}
        	// new joined vehicles
        	else {
        		g_bNewVeh = true;
//        		gFlag_initiation = false;

        		if ( debugMAC ){
                    cout << "New Ini!: " << endl;
                    cout << roundEnd << "   " << simTime().dbl() << endl;
        		}

        		if ( roundEnd > simTime().dbl() ){
        			scheduleAt( roundEnd, nextConnectEvent );
        		}
        	}

        }

        /*set channel to CCH*/
        setActiveChannel(type_CCH);
        phy11p->changeListeningFrequency(frequency[Channels::CCH]);

//        if ( strcmp(findHost()->getFullName(), "node[0]") == 0 ){
//        	mySCH = Channels::SCH1;
//
////        	setActiveChannel(type_CCH);
//        	setActiveChannel(type_SCH);
//            phy11p->changeListeningFrequency(frequency[mySCH]);
//            scheduleAt(simTime() + 0.105, nextUploadEvent);
//        }
//
//		if ( strcmp(findHost()->getFullName(), "node[1]") == 0 ){
//			mySCH = Channels::SCH1;
//
////			setActiveChannel(type_CCH);
//			setActiveChannel(type_SCH);
//			phy11p->changeListeningFrequency(frequency[mySCH]);
//		}
//
//		if ( strcmp(findHost()->getFullName(), "node[2]") == 0 ){
//			mySCH = Channels::SCH2;
//			setActiveChannel(type_SCH);
//			phy11p->changeListeningFrequency(frequency[mySCH]);
//            scheduleAt(simTime() + 0.105064, nextUploadEvent);
//		}
        // Chris end

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

void CMac1609_4TwoWay::handleSelfMsg(cMessage* msg) {
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

    } else if (msg ==  nextMacEvent) {
        //we actually came to the point where we can send a packet
        channelBusySelf(true);
        CMACWSM* pktToSend = myEDCA[activeChannel]->initiateTransmit(lastIdle);

        lastAC = mapPriority(pktToSend->getPriority());

        DBG_MAC << "MacEvent received. Trying to send packet with priority" << lastAC << std::endl;

        //send the packet
        CMac80211Pkt* mac = new CMac80211Pkt(pktToSend->getName(), pktToSend->getKind());
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
    else if ( msg == startEvent ){

    	/*find the vehicle with the smallest offset*/
		std::map<std::string, cModule*> vehs_Map = vehMobility->getManager()->getManagedHosts();


        if (gPeNumVeh == 0)
            gPeNumVeh = vehs_Map.size();
//        std::cout << "startEvent: TwoWay!" << endl;
		// for twoway case
		std::vector<std::string> vehMinOffsetVec;
		vehMapEdgeId.clear();

        for (auto& i:vehs_Map){
            Veins::TraCIMobility* iMob;
            iMob = (Veins::TraCIMobility*)i.second->getSubmodule("veinsmobility");
            std::string edgeID = iMob->getCommandInterface()->getEdgeId(iMob->getExternalId());
            if (getOffsetRatio(iMob) < 0.98){
                if (vehMapEdgeId.find(edgeID) != vehMapEdgeId.end()){

                    vehMapEdgeId.at(edgeID).push_back(i.second);

                }
                else{
                    // insert a key and a value
                    vehMapEdgeId[edgeID].push_back(i.second);
                }

                if (edgeIdChannel.find(edgeID) == edgeIdChannel.end() ){
                    edgeIdChannel[edgeID] = -1;
                }
            }
        }

        // initialize edge id and channel id map
        int index = 0;
        for (auto& i:edgeIdChannel){
            i.second = g_twoWayChannelList[index];
            ++index;
        }

        for (auto& i:vehMapEdgeId){
            double minOffset = 0.0;
            std::string vehMin = "";
            for (auto& j:i.second){
                Veins::TraCIMobility* jMob;
                jMob = (Veins::TraCIMobility*)j->getSubmodule("veinsmobility");
                double offset = jMob->getCommandInterface()->getLanePosition(jMob->getExternalId());
                if ( minOffset == 0.0 ){
                    minOffset = offset;
                    vehMin = j->getFullName();
                }
                else if ( minOffset > offset ){
                    minOffset = offset;
                    vehMin = j->getFullName();
                }
            }

            if (!vehMin.empty()){
                vehMinOffsetVec.push_back(vehMin);
            }

        }

        if (debugMAC){
//            for (auto& i:vehMapEdgeId){
//                cout << i.first << ": " << endl;
//                for (auto& j:i.second){
//                    cout << " "<< j->getFullName();
//                }
//                cout << endl;
//            }
        }


        if (gLatestNodeID.empty()){
            gLatestNodeID = vehMinOffsetVec[0];
        }
		/*new vehicle join reclustering process*/
    	if ( g_bNewVeh ){
			scheduleAt(simTime(), nextClusteringEvent);
			gdLastClusTime = simTime().dbl();
			g_bNewVeh = false;
    	}
    	/*initial clustering and periodic clustering*/
    	else {
    	    for (auto& i: vehMinOffsetVec){
    	        if ( myNodeID.compare(i) == 0 ){

                    scheduleAt(simTime(), nextClusteringEvent);

                    DBG_MAC << "Time: "<< simTime().dbl() << " LastClusTime: " << gdLastClusTime
                            << " Difference: " << simTime().dbl() - gdLastClusTime << endl;
        //          cout << "Time: "<< simTime().dbl() << " LastClusTime: " << gdLastClusTime
        //                  << " Difference: " << simTime().dbl() - gdLastClusTime << endl;
                    gdLastClusTime = simTime().dbl();
                }
    	    }
    	}

    }
    else if ( msg ==  nextClusteringEvent ){

/*debugging*/
/*printing all existing vehicles from vehs_Map*/
//		for (std::map<std::string, cModule*>::iterator it = vehs_Map.begin(); it != vehs_Map.end(); it++){
//			std::cout << (*it).first << ": "<< (*it).second->getFullName() << std::endl;
//		}
//		0: node[0]
/*debugging*/

//		std::vector<cModule*> vehs_Vec;
        std::vector<cModule*>::iterator itVehVec;
        vehs_Vec.clear();

		string myEdgeId = vehMobility->getCommandInterface()->getEdgeId(vehMobility->getExternalId());
		if (vehMapEdgeId.find(myEdgeId) != vehMapEdgeId.end()){
		    vehs_Vec = vehMapEdgeId.at(myEdgeId);
		}
		else{
		    cout << "nextClusteringEvent: no find edge Id" << endl;
		}

//		for (itVehsMap = vehs_Map.begin(); itVehsMap != vehs_Map.end(); itVehsMap++){
//			Veins::TraCIMobility* i_Mob;
//			i_Mob = (Veins::TraCIMobility*)itVehsMap->second->getSubmodule("veinsmobility");
//
//			/* do not cluster vehicles going to disappear,
//			 * at the end of a road segment
//			 * */
//			if ( getOffsetRatio(i_Mob) <= 0.98 ) {
//				vehs_Vec.push_back((*itVehsMap).second);
//				(*itVehsMap).second->getDisplayString().updateWith("r=0");
//				cModule* macModule = (*itVehsMap).second->getModuleByPath(".nic.mac1609_4");
//				CMac1609_4TwoWay* cmac1609_4 = check_and_cast<CMac1609_4TwoWay*>(macModule);
//				cmac1609_4->mRoleInCluster = CM;
//			}
//			else {
//				cModule* macModule = (*itVehsMap).second->getModuleByPath(".nic.mac1609_4");
//				CMac1609_4TwoWay* cmac1609_4 = check_and_cast<CMac1609_4TwoWay*>(macModule);
//				cmac1609_4->mRoleInCluster = IDLE;
//			}
//		}

		/*
		 * find the neighborhoods
		 * */
		// clear neighbors of each vehicle
		for ( itVehVec = vehs_Vec.begin(); itVehVec != vehs_Vec.end(); ++itVehVec){
			cModule* macModule = (*itVehVec)->getModuleByPath(".nic.mac1609_4");
			CMac1609_4TwoWay* cmac1609_4 = check_and_cast<CMac1609_4TwoWay*>(macModule);
			cmac1609_4->mNeighborVec.clear();
			cmac1609_4->mRoleInCluster = CM;
		}

		maxTxDist = cm->calcInterfDist();

		for ( itVehVec = vehs_Vec.begin(); itVehVec != vehs_Vec.end(); ++itVehVec){
			std::vector<cModule*>::iterator itVehVec2;
//				cout << (*itVehVec)->getFullName() << endl;
			for ( itVehVec2 = vehs_Vec.begin(); itVehVec2 != vehs_Vec.end(); ++itVehVec2){
				// outside
				cModule* macModule = (*itVehVec)->getModuleByPath(".nic.mac1609_4");
				cModule* mobModule = (*itVehVec)->getModuleByPath(".veinsmobility");
				CMac1609_4TwoWay* cmac1609_4 = check_and_cast<CMac1609_4TwoWay*>(macModule);
				Veins::TraCIMobility* mobility = check_and_cast<Veins::TraCIMobility*>(mobModule);

				// inside
				cModule* mobModule2 = (*itVehVec2)->getModuleByPath(".veinsmobility");
				Veins::TraCIMobility* mobility2 = check_and_cast<Veins::TraCIMobility*>(mobModule2);
				// find nodes within maxTxDist
				if ( mobility->getCurrentPosition().distance(mobility2->getCurrentPosition()) < maxTxDist ){
					if (strcmp( (*itVehVec)->getFullName(), (*itVehVec2)->getFullName()) != 0 ){
						cmac1609_4->mNeighborVec.push_back(*itVehVec2);
					}
				}
			}
		}

/*debugging*/
/*printing all neighbors of each vehicle*/
//			for (itVehVec = vehs_Vec.begin(); itVehVec != vehs_Vec.end(); itVehVec++){
//				cModule* macModule = (*itVehVec)->getModuleByPath(".nic.mac1609_4");
//				CMac1609_4TwoWay* cmac1609_4 = check_and_cast<CMac1609_4TwoWay*>(macModule);
//				std::cout << (*itVehVec)->getFullName() << " neighbors: " << std::endl;
//
//				std::vector<cModule*> neighborVehVec = cmac1609_4->mNeighborVec;
//				for (std::vector<cModule*>::iterator it = neighborVehVec.begin(); it != neighborVehVec.end(); it++){
//					std::cout << "    " << (*it)->getFullName();
//				}
//				std::cout << std::endl;
//			}
/*debugging end*/

		/*
		 * clustering procedure
		 * */
		// sort based on x coordinates
		std::sort( vehs_Vec.begin(), vehs_Vec.end(), cmpFunction_cmactwoway );

/*debugging*/
		/*printing sorted vehicle list*/
        if (debugMAC){
            for (itVehVec = vehs_Vec.begin(); itVehVec != vehs_Vec.end(); itVehVec++ ){
                Veins::TraCIMobility* test = (Veins::TraCIMobility*)(*itVehVec)->getSubmodule("veinsmobility");
                std::cout << test->commandGetOffset() << std::endl;
            }
        }
/*debugging end*/

		// divide vehicles into clusters by giving clusterIndex
		// myOptimalLayer = optTreeDepth(vehs_Vec.size()) + 1; // depth + 1
        int myOptimalLayer_ori = optTreeDepth(vehs_Vec.size()) + 1; // depth + 1

		myOptimalLayer = 3;
		myOptimalClusterSize = optNumChildren(vehs_Vec.size(), myOptimalLayer - 1); // depth = layer-1

		if (gPeClusterSize == 0)
			gPeClusterSize = myOptimalClusterSize;
		if (gPeClusterLayers ==0 )
			// gPeClusterLayers = myOptimalLayer;
            gPeClusterLayers = myOptimalLayer_ori;
/*debugging*/
		if ( debugMAC ){
            cout << "Layer: " << myOptimalLayer << ", ClusterSize:" << myOptimalClusterSize
                    <<", Vehicle size: " << vehs_Vec.size()<< endl;
		}
/*debugging end*/

		// assign cluster index for each vehicle
		int numCM = myOptimalClusterSize - 1;
		int indexCluster = 0;

		for (itVehVec = vehs_Vec.begin(); itVehVec != vehs_Vec.end(); itVehVec++ ){
			cModule* macModule = (*itVehVec)->getModuleByPath(".nic.mac1609_4");
			CMac1609_4TwoWay* cmac1609_4 = check_and_cast<CMac1609_4TwoWay*>(macModule);
			if ( numCM !=0 ){
				cmac1609_4->clusterIndex = indexCluster;
				numCM--;
			}
			else{
				cmac1609_4->clusterIndex = indexCluster;
				numCM = myOptimalClusterSize - 1;
				indexCluster++;
			}
		}

		int numCluster = indexCluster;
		myNumCluster = numCluster + 1;

		if (gPeClusterNum == 0)
			gPeClusterNum = myNumCluster;
/*debugging*/
		/*printing each vehicle's cluster index*/
//			for (itVehVec = vehs_Vec.begin(); itVehVec != vehs_Vec.end(); itVehVec++ ){
//				cModule* macModule = (*itVehVec)->getModuleByPath(".nic.mac1609_4");
//				CMac1609_4TwoWay* cmac1609_4 = check_and_cast<CMac1609_4TwoWay*>(macModule);
//				cout << (*itVehVec)->getFullName()<< ": " << cmac1609_4->clusterIndex << endl;
//			}
/*debugging end*/

		// let each vehicle get neighbors in its own cluster
		for (itVehVec = vehs_Vec.begin(); itVehVec != vehs_Vec.end(); itVehVec++ ){
			std::vector<cModule*>::iterator itVehVec2;
			cModule* macModule = (*itVehVec)->getModuleByPath(".nic.mac1609_4");
			CMac1609_4TwoWay* cmac1609_4 = check_and_cast<CMac1609_4TwoWay*>(macModule);
			cmac1609_4->mClusterNeighborsVec.clear();
			for (itVehVec2 = vehs_Vec.begin(); itVehVec2 != vehs_Vec.end(); itVehVec2++ ){
				cModule* macModuel2 = (*itVehVec2)->getModuleByPath(".nic.mac1609_4");
				CMac1609_4TwoWay* cmac1609_4_2 = check_and_cast<CMac1609_4TwoWay*>(macModuel2);

				if ( strcmp( (*itVehVec)->getFullName(), (*itVehVec2)->getFullName() ) != 0 ){
					if ( cmac1609_4->clusterIndex == cmac1609_4_2->clusterIndex ){
						cmac1609_4->mClusterNeighborsVec.push_back(*itVehVec2);
					}
				}
			}
		}

		// select CH based on the smallest average distance in one cluster
		float minAveDis = 0.0;
		cModule* moduleMinAveDis = nullptr;
		int currentClusterIndex = 0;
		for (itVehVec = vehs_Vec.begin(); itVehVec != vehs_Vec.end(); itVehVec++ ){
			cModule* macModule = (*itVehVec)->getModuleByPath(".nic.mac1609_4");
			CMac1609_4TwoWay* cmac1609_4 = check_and_cast<CMac1609_4TwoWay*>(macModule);

			// same cluster
			if ( currentClusterIndex == cmac1609_4->clusterIndex ){

//				moduleMinAveDis = findCenter(*itVehVec, cmac1609_4, minAveDis, moduleMinAveDis );
				float sumDistance = 0.0;
				float averageDistance = 0.0;
				vector<cModule*>::iterator itClusterNeighbors;
				for (itClusterNeighbors = cmac1609_4->mClusterNeighborsVec.begin();
						itClusterNeighbors != cmac1609_4->mClusterNeighborsVec.end();
						itClusterNeighbors++ ){
					cModule* mobModule = (*itVehVec)->getModuleByPath(".veinsmobility");
					Veins::TraCIMobility* mobility = check_and_cast<Veins::TraCIMobility*>(mobModule);
					cModule* mobNeiModule = (*itClusterNeighbors)->getModuleByPath(".veinsmobility");
					Veins::TraCIMobility* mobilityNei = check_and_cast<Veins::TraCIMobility*>(mobNeiModule);

					sumDistance = sumDistance + mobility->getCurrentPosition().distance(mobilityNei->getCurrentPosition());
					averageDistance = sumDistance / ( cmac1609_4->mClusterNeighborsVec.size() );
				}

				if (minAveDis == 0.0){
					minAveDis = averageDistance;
					moduleMinAveDis = (*itVehVec);
				}
				else if ( minAveDis > averageDistance ){
					minAveDis = averageDistance;
					moduleMinAveDis = (*itVehVec);
				}

			}
			// next new cluster
			else {
				// set previous cluster
				// already find the center vehicle for each cluster
				cModule* macModuleCH = moduleMinAveDis->getModuleByPath(".nic.mac1609_4");
//				cModule* mobModuleCH = moduleMinAveDis->getModuleByPath(".veinsmobility");
				CMac1609_4TwoWay* cmac1609_4CH = check_and_cast<CMac1609_4TwoWay*>(macModuleCH);
//				Veins::TraCIMobility* mobi = check_and_cast<Veins::TraCIMobility*>(mobModuleCH);
				moduleMinAveDis->getDisplayString().updateWith("r=4,yellow");

				// set this center vehicle as CH
				cmac1609_4CH->mRoleInCluster = CH;
				// set CH as itself
				cmac1609_4CH->mCH = moduleMinAveDis->getFullName();
				cmac1609_4CH->mCHMacAdd = cmac1609_4CH->myMacAddress;

/*debugging*/
				/*print CH and CMs*/
				if ( debugMAC ){
                    cout << "CH: " << moduleMinAveDis->getFullName() << endl;
                    cout << "CM: " << endl;
                    for (unsigned int i = 0; i < cmac1609_4CH->mClusterNeighborsVec.size(); i++){
                        cout << cmac1609_4CH->mClusterNeighborsVec[i]->getFullName() << " ";
                        if ( (i+1) % 4 == 0)
                            cout << endl;
                    }
                    cout << endl;
				}
/*debugging end*/

				// set all other vehicles as CM from CH in each cluster
				for (unsigned int i = 0; i < cmac1609_4CH->mClusterNeighborsVec.size(); i++){
					cModule* macModuleCM = cmac1609_4CH->mClusterNeighborsVec[i]->getModuleByPath(".nic.mac1609_4");
					CMac1609_4TwoWay* cmac1609_4CM = check_and_cast<CMac1609_4TwoWay*>(macModuleCM);
					cmac1609_4CM->mRoleInCluster = CM;
					cmac1609_4CM->mClusterHead = cmac1609_4CH;
					cmac1609_4CM->mCH = moduleMinAveDis->getFullName();
					cmac1609_4CM->mCHMacAdd = cmac1609_4CH->myMacAddress;
				}

				// this new cluster
				// find the CH by finding the most center one within a cluster
				minAveDis = 0.0;
				moduleMinAveDis = nullptr;
//				moduleMinAveDis = findCenter(*itVehVec, cmac1609_4, minAveDis, moduleMinAveDis );

				float sumDistance = 0.0;
				float averageDistance = 0.0;
				vector<cModule*>::iterator itClusterNeighbors;
				for (itClusterNeighbors = cmac1609_4->mClusterNeighborsVec.begin();
						itClusterNeighbors != cmac1609_4->mClusterNeighborsVec.end();
						itClusterNeighbors++ ){
					cModule* mobModule = (*itVehVec)->getModuleByPath(".veinsmobility");
					Veins::TraCIMobility* mobility = check_and_cast<Veins::TraCIMobility*>(mobModule);
					cModule* mobNeiModule = (*itClusterNeighbors)->getModuleByPath(".veinsmobility");
					Veins::TraCIMobility* mobilityNei = check_and_cast<Veins::TraCIMobility*>(mobNeiModule);

					sumDistance = sumDistance + mobility->getCurrentPosition().distance(mobilityNei->getCurrentPosition());
					averageDistance = sumDistance / ( cmac1609_4->mClusterNeighborsVec.size() );
				}

				if (minAveDis == 0.0){
					minAveDis = averageDistance;
					moduleMinAveDis = (*itVehVec);
				}
				else if ( minAveDis > averageDistance ){
					minAveDis = averageDistance;
					moduleMinAveDis = (*itVehVec);
				}

			}
			// reach the last element
			if ( itVehVec + 1 == vehs_Vec.end() ){
				// already find the center vehicle for each cluster
				cModule* macModuleCH = moduleMinAveDis->getModuleByPath(".nic.mac1609_4");
//				cModule* mobModuleCH = moduleMinAveDis->getModuleByPath(".veinsmobility");
				CMac1609_4TwoWay* cmac1609_4CH = check_and_cast<CMac1609_4TwoWay*>(macModuleCH);
//				Veins::TraCIMobility* mobi = check_and_cast<Veins::TraCIMobility*>(mobModuleCH);
				moduleMinAveDis->getDisplayString().updateWith("r=4,yellow");

				// set this center vehicle as CH
				cmac1609_4CH->mRoleInCluster = CH;
				cmac1609_4CH->mCHMacAdd = cmac1609_4CH->myMacAddress;

/*debugging*/
				/*print CH and CMs*/
				if ( debugMAC ){
                    cout << "CH: " << moduleMinAveDis->getFullName() << endl;
                    cout << "CM: " << endl;
                    for (unsigned int i = 0; i < cmac1609_4CH->mClusterNeighborsVec.size(); i++){
                        cout << cmac1609_4CH->mClusterNeighborsVec[i]->getFullName() << " ";
                        if ( (i+1) % 4 == 0)
                            cout << endl;
                    }
                    cout << endl;
				}
/*debugging end*/

				// set all other vehicles as CM from CH for each cluster
				for (unsigned int i = 0; i < cmac1609_4CH->mClusterNeighborsVec.size(); i++){
					cModule* macModuleCM = cmac1609_4CH->mClusterNeighborsVec[i]->getModuleByPath(".nic.mac1609_4");
					CMac1609_4TwoWay* cmac1609_4CM = check_and_cast<CMac1609_4TwoWay*>(macModuleCM);
					cmac1609_4CM->mRoleInCluster = CM;
					cmac1609_4CM->mClusterHead = cmac1609_4CH;
					cmac1609_4CM->mCH = moduleMinAveDis->getFullName();
					cmac1609_4CM->mCHMacAdd = cmac1609_4CH->myMacAddress;
				}
			}

			currentClusterIndex = cmac1609_4->clusterIndex;
		}

/*debug*/
//		cout << simTime() << endl;
//		for (itVehVec = vehs_Vec.begin(); itVehVec != vehs_Vec.end(); itVehVec++ ){
//			cModule* macModule = (*itVehVec)->getModuleByPath(".nic.mac1609_4");
//			CMac1609_4TwoWay* cmac1609_4 = check_and_cast<CMac1609_4TwoWay*>(macModule);
//			cout << (*itVehVec)->getFullName() << " " << cmac1609_4->mRoleInCluster << endl;
//		}
/*debug*/
		/*
		 * select MCH
		 * */

		/*among CHs find MCH with the most center*/
		cModule* moduleMCH;
		unsigned int maxNeighborVehSize = 0;
		int mostCenter = vehs_Vec.size();
		if ( myOptimalLayer > 2){
			for (unsigned int i = 0; i < vehs_Vec.size(); i++ ){
				cModule* macModule = (vehs_Vec[i])->getModuleByPath(".nic.mac1609_4");
				CMac1609_4TwoWay* cmac1609_4 = check_and_cast<CMac1609_4TwoWay*>(macModule);
				if ( cmac1609_4->mRoleInCluster == CH ){
					if ( maxNeighborVehSize <  cmac1609_4->mNeighborVec.size() ){
						// find a CH with the smallest distance to the center of all vehicles
						if ( mostCenter > abs( i-vehs_Vec.size()/2 ) ) {
							maxNeighborVehSize = cmac1609_4->mNeighborVec.size();
							moduleMCH = vehs_Vec[i];
							mostCenter = abs( i-vehs_Vec.size()/2 );
						}
					}
				}
			}

			cModule* macModuleMCH = moduleMCH->getModuleByPath(".nic.mac1609_4");
			CMac1609_4TwoWay* cmac1609_4MCH = check_and_cast<CMac1609_4TwoWay*>(macModuleMCH);
			cmac1609_4MCH->mRoleInCluster = MCH;
			cmac1609_4MCH->mClusterHead = cmac1609_4MCH;
	//		cmac1609_4MCH->mCH = moduleMCH->getFullName();
	//		cmac1609_4MCH->mCHMacAdd = cmac1609_4MCH->myMacAddress;
			moduleMCH->getDisplayString().updateWith("r=4,white");

	/*debug*/
	//		for (itVehVec = vehs_Vec.begin(); itVehVec != vehs_Vec.end(); itVehVec++ ){
	//			cModule* macModule = (*itVehVec)->getModuleByPath(".nic.mac1609_4");
	//			CMac1609_4TwoWay* cmac1609_4 = check_and_cast<CMac1609_4TwoWay*>(macModule);
	//			cout << (*itVehVec)->getFullName() << " " << cmac1609_4->mRoleInCluster << endl;
	//		}
	/*debug*/

			// save CHs into MCH
			cmac1609_4MCH->mCoveredCHVec.clear();
//			for (unsigned int i = 0; i < cmac1609_4MCH->mNeighborVec.size(); i ++ ){
//				cModule* macModuleCoveredCH = cmac1609_4MCH->mNeighborVec[i]->getModuleByPath(".nic.mac1609_4");
//				CMac1609_4TwoWay* cmac1609_4CoveredCH = check_and_cast<CMac1609_4TwoWay*>(macModuleCoveredCH);
//				if ( cmac1609_4CoveredCH->mRoleInCluster == CH ){
//					cmac1609_4MCH->mCoveredCHVec.push_back(cmac1609_4MCH->mNeighborVec[i]);
//					cmac1609_4CoveredCH->mClusterHead = moduleMCH;
//					cmac1609_4CoveredCH->mCH = moduleMCH->getFullName();
//				}
//			}

			for (itVehVec = vehs_Vec.begin(); itVehVec != vehs_Vec.end(); itVehVec++ ){
				cModule* macModule = (*itVehVec)->getModuleByPath(".nic.mac1609_4");
				CMac1609_4TwoWay* cmac1609_4 = check_and_cast<CMac1609_4TwoWay*>(macModule);
				if ( cmac1609_4->mRoleInCluster == CH ){
					cmac1609_4MCH->mCoveredCHVec.push_back(*itVehVec);
					cmac1609_4->mClusterHead = cmac1609_4MCH;
					cmac1609_4->mCH = moduleMCH->getFullName();
				}
			}

/*debugging*/
			if ( debugMAC ){
                cout << "MCH: " << moduleMCH->getFullName() << endl;
                cout << "MCH's CHs: " << endl;
                for (unsigned int i = 0; i < cmac1609_4MCH->mCoveredCHVec.size(); i ++ ){
                    cout << cmac1609_4MCH->mCoveredCHVec[i]->getFullName() << " ";
                    if ( (i+1) % 4 == 0)
                        cout << endl;
                }
                cout << endl;
                cout << "-----------------------------------------------" << endl;
			}
/*debugging end*/
		}

		/*channel assignment part*/
//		clusterChannelAssign( vehs_Vec, vehs_Vec.size(), 6, numCluster + 1);
		clusterChannelAssign( vehs_Vec, 6, numCluster + 1);

		/*time slot assignment part*/
		vector< vector<cModule*> > timeSlotAll;

		if ( CMUploadByTimeslotAlgo ){
		    timeSlotAll = timeSlotAllocation( vehs_Vec, numCluster + 1, myOptimalClusterSize);
		}
		else {
		    timeSlotAll = timeSlotSequence( vehs_Vec, numCluster + 1, myOptimalClusterSize);
		}

		g_vSlotCountByTSAlgo.push_back(myMaxTimeSlotSize);
		g_vSlotCountOptimal.push_back(myOptimalClusterSize);

		// let CH know the time slot assignment
		for ( unsigned int i = 0; i < vehs_Vec.size(); i++){
			cModule* module_i = vehs_Vec[i]->getModuleByPath(".nic.mac1609_4");
			CMac1609_4TwoWay* mac_i = check_and_cast<CMac1609_4TwoWay*>(module_i);

			mac_i->timeSlotAlloVec = timeSlotAll[mac_i->clusterIndex];

			vector<string> tempStr;
			tempStr.clear();
			for (unsigned int t2 = 0; t2 < timeSlotAll[mac_i->clusterIndex].size(); t2++ ){
				if ( timeSlotAll[mac_i->clusterIndex][t2] != nullptr){
					tempStr.push_back(timeSlotAll[mac_i->clusterIndex][t2]->getFullName());
				}
				else{
					tempStr.push_back("");
				}
			}

			if (debugMAC){
//			    for ( auto test:tempStr){
//			        cout << test << "  ";
//			    }
//			    cout << endl;
			}

			mac_i->timeSlotAlloStrVec = tempStr;

			if ( myMaxTimeSlotSize > 0 )
				mac_i->myMaxTimeSlotSize = myMaxTimeSlotSize;
			else
				error("myMaxTimeSlotSize is below 0!");

			if ( myNumCluster > 0 )
				mac_i->myNumCluster = myNumCluster;
			else
				error("myNumCluster is below 0!");

			if (myOptimalClusterSize > 0 )
				mac_i->myOptimalClusterSize = myOptimalClusterSize;

			if (myOptimalLayer > 0)
				mac_i->myOptimalLayer = myOptimalLayer;
		}

    } else if (msg == nextTAFEvent){

    	sendTAF(prepareTAF());

    	/*CH channel switch*/
    	/*channel should be SCH, i.e., > 0*/
    	if ( mChannelID != -1 && mChannelID > 0){
    		simtime_t timeCS = 0.002;
    		scheduleAt(simTime() + timeCS, nextChannelAssignment);
    	}
    	else {
    		opp_error("My Channel ID is -1, or is CCH...");
    	}

    	/*CH schedule communication among CHs*/

    } else if (msg == nextConnectEvent ){
    	if ( getOffsetRatio() <= 0.98 ){
    		if ( g_bNewVeh == true
    				|| (simTime().dbl() - gdLastClusTime >= gdClusInterval)
    				|| m_activeVehCount != vehMobility->getManager()->getActiveVehicleCount()){
    			if ( startEvent->isScheduled() == true ){
    			    if ( debugMAC ){
                        cout << myNodeID << " startEvent scheduled: "
                                << startEvent->getTimestamp() << endl;
    			    }
    				cancelEvent(startEvent);
    			}

    			scheduleAt(simTime(), startEvent);
    			scheduleAt(simTime() + 0.000000001, nextCheckEvent);
			} else {
				scheduleAt(simTime(), nextCheckEvent);
			}
    	} else {
			findHost()->getDisplayString().updateWith("r=0,light green");
    	}

    } else if (msg == nextCheckEvent ){
    	if ( mRoleInCluster == MCH){
    	    if ( debugMAC ){
    	        cout << simTime() << ": nextCheckEvt called! " << endl;
    	    }

    	    // update cluster neighbors for special cases when a vehicle gonna disappear
            std::map<std::string, cModule*> vehs_Map = vehMobility->getManager()->getManagedHosts();
            // for twoway case
            vehMapEdgeId.clear();

            for (auto& i:vehs_Map){
                Veins::TraCIMobility* iMob;
                iMob = (Veins::TraCIMobility*)i.second->getSubmodule("veinsmobility");
                std::string edgeID = iMob->getCommandInterface()->getEdgeId(iMob->getExternalId());
                if (getOffsetRatio(iMob) < 0.98){
                    if (vehMapEdgeId.find(edgeID) != vehMapEdgeId.end()){

                        vehMapEdgeId.at(edgeID).push_back(i.second);
                    }
                    else{
                        // insert a key and a value
                        vehMapEdgeId[edgeID].push_back(i.second);
                    }
                }
            }

            string myEdgeId = vehMobility->getCommandInterface()->getEdgeId(vehMobility->getExternalId());
            if (vehMapEdgeId.find(myEdgeId) != vehMapEdgeId.end()){
                vehs_Vec = vehMapEdgeId.at(myEdgeId);
            }
            else{
                cout << "nextCheckEvent: no find edge Id" << endl;
            }

            std::vector<cModule*>::iterator itVehVec;

//            for (itVehVec = vehs_Vec.begin(); itVehVec != vehs_Vec.end(); itVehVec++ ){
//                cModule* macModule = (*itVehVec)->getModuleByPath(".nic.mac1609_4");
//                if (macModule == nullptr){
//                    vehs_Vec.erase(itVehVec);
//                }
//            }

            for (itVehVec = vehs_Vec.begin(); itVehVec != vehs_Vec.end(); itVehVec++ ){
                std::vector<cModule*>::iterator itVehVec2;
                cModule* macModule = (*itVehVec)->getModuleByPath(".nic.mac1609_4");
                CMac1609_4TwoWay* cmac1609_4 = check_and_cast<CMac1609_4TwoWay*>(macModule);
                cmac1609_4->mClusterNeighborsVec.clear();
                for (itVehVec2 = vehs_Vec.begin(); itVehVec2 != vehs_Vec.end(); itVehVec2++ ){
                    cModule* macModule2 = (*itVehVec2)->getModuleByPath(".nic.mac1609_4");
                    CMac1609_4TwoWay* cmac1609_4_2 = check_and_cast<CMac1609_4TwoWay*>(macModule2);
                    if ( getOffsetRatio(cmac1609_4_2->vehMobility) < 0.98){
                        if ( strcmp( (*itVehVec)->getFullName(), (*itVehVec2)->getFullName() ) != 0 ){
                            if ( cmac1609_4->clusterIndex == cmac1609_4_2->clusterIndex ){
                                cmac1609_4->mClusterNeighborsVec.push_back(*itVehVec2);
                            }
                        }
                    }
                }
            }

            if (debugMAC){
//                for (itVehVec = vehs_Vec.begin(); itVehVec != vehs_Vec.end(); itVehVec++ ){
//                    cModule* macModule = (*itVehVec)->getModuleByPath(".nic.mac1609_4");
//                    CMac1609_4TwoWay* cmac1609_4 = check_and_cast<CMac1609_4TwoWay*>(macModule);
//                    std::cout << cmac1609_4->myNodeID << " CN: " << endl;
//                    for(auto& i:cmac1609_4->mClusterNeighborsVec){
//                        std::cout << i->getFullName() << " ";
//                    }
//                    std::cout << endl;
//                }
            }
    	}

        m_activeVehCount = vehMobility->getManager()->getActiveVehicleCount();



		/****************
		 * emit signal to notify new joined vehicle to start nextCheckEvent
		 * **************/
//    	if ( round >= 10){
//    		if (g_bSignal == false){
//    			emit(roundCountSignal, this);
//    			g_bSignal = true;
//    			cout << myNodeID << " emits Signal." << endl;
//    		}
//    	}
//
//    	g_bSignal = false;

		/*all nodes switch channel*/
		mChannelID = mClusterChannelID;
		if ( mChannelID != -1 && mChannelID > 0){
			scheduleAt(simTime(), nextChannelAssignment);
		}
		else {
//			opp_error("nextCheckEvent: Channel ID is invalid! -1 or 0");
			DBG_MAC << "nextCheckEvent: Channel ID is invalid! -1 or 0" << endl;
		}

		/* CMs schedule prepare upload event */
		if ( mRoleInCluster == CM ){
			if ( getOffsetRatio() <= 0.98 ){
				// upload event after CSGI
				scheduleAt( simTime() + chSwitchGuardInterval, nextPrepareUploadEvent);

				findHost()->getDisplayString().updateWith("r=0");

				simtime_t offset;
				if ( CHDistriBySequence ){
					// duration: CMs uploads + CHs commu + CHs distributes
					offset = chSwitchGuardInterval
								+ myMaxTimeSlotSize * ( getFrameDuration(400)
															+ RADIODELAY_11P
															+ SLOTLENGTH_11P)
								+ chSwitchGuardInterval
									+ myNumCluster * ( getFrameDuration(4000)
															+ RADIODELAY_11P
															+ SLOTLENGTH_11P )
								+ chSwitchGuardInterval
									+ myNumCluster * ( getFrameDuration(8000)
															+ RADIODELAY_11P
															+ SLOTLENGTH_11P );

				}
				// CHs transmit in the same time slot
				else {
					offset = chSwitchGuardInterval
							+ myMaxTimeSlotSize * ( getFrameDuration(400)
														+ RADIODELAY_11P
														+ SLOTLENGTH_11P)
							+ chSwitchGuardInterval
								+ myNumCluster * ( getFrameDuration(4000)
														+ RADIODELAY_11P
														+ SLOTLENGTH_11P )
							 + chSwitchGuardInterval
							 	 + ( getFrameDuration(8000)
							 			+ RADIODELAY_11P
							 			+ SLOTLENGTH_11P );
				}

				if (nextConnectEvent->isScheduled() == true)
					cancelEvent(nextConnectEvent);
				scheduleAt(simTime() + offset, nextConnectEvent);

				roundEnd = simTime().dbl() + offset.dbl();
			}
		}
		/* CHs and MCH schedule CHs communication event */
		else if ( mRoleInCluster == CH || mRoleInCluster == MCH ){
			if ( myMaxTimeSlotSize > 0 ){
				simtime_t offset = chSwitchGuardInterval
									+ myMaxTimeSlotSize * ( getFrameDuration(400)
															+ RADIODELAY_11P
															+ SLOTLENGTH_11P);
				scheduleAt( simTime() + offset, nextPrepareCHsCommuEvent);
			}
			else{
				opp_error ("nextCheckEvent: myMaxTimeSlotSize is 0!");
			}
		}
    }
    else if ( msg == nextPrepareUploadEvent){
    	if ( getOffsetRatio() <= 0.98 ){

    	    // upload by time-slot allocation algorithm
//    	    if ( CMUploadByTimeslotAlgo ){
                for ( size_t i = 0; i < timeSlotAlloStrVec.size(); i++ ){
                    if ( timeSlotAlloStrVec[i].compare("") != 0 ){
                        if ( myNodeID.compare(timeSlotAlloStrVec[i]) == 0 ){
                            simtime_t offset = i * ( getFrameDuration(400)
                                                        + RADIODELAY_11P
                                                        + SLOTLENGTH_11P );
                            scheduleAt( simTime() + offset, nextUploadEvent );
                            break;
                        }
                    }
                }
//    	    }
    	    // upload by sequential order
//    	    else{
//    	        // minus twice 1 is for index 0 and CH not joining
//                for ( int i = 0; i < myOptimalClusterSize - 1 - 1; i++ ){
//
//                    simtime_t offset = i * ( getFrameDuration(400)
//                                                + RADIODELAY_11P
//                                                + SLOTLENGTH_11P );
//                    scheduleAt( simTime() + offset, nextUploadEvent );
//                }
//    	    }
    	}
    }
    else if (msg == nextUploadEvent ){
    	if( mCHMacAdd != 0 ){
    		if ( getOffsetRatio() <= 0.98 ){
				findHost()->getDisplayString().updateWith("r=4,green");

/*debug*/
//				cout << myNodeID << ": " << endl;
//				cout << "txPower: " << par("txPower").doubleValue() << endl;
//				cout << "cm: " << cm->calcInterfDist()
//						<< " parPMax: " << cm->parPMax
//						<< ", connects: " << cm->getGateList(getParentModule()->getId() ).size() << endl;
/*debug*/
				cModule* cmModule = check_and_cast<cModule*>(cm);

				if ( myOptimalLayer > 2){
					// change tx power
					par("txPower").setDoubleValue(20);
					txPower = par("txPower");
					// connection manager
					cmModule->par("pMax").setDoubleValue(20);

					phyLayerPtr->par("maxTXPower").setDoubleValue(10);
					phyLayerPtr->setMaxTXPower(10);
				}
				else {
					// change tx power
					par("txPower").setDoubleValue(200);
					txPower = par("txPower");
					// connection manager
					cmModule->par("pMax").setDoubleValue(200);

					phyLayerPtr->par("maxTXPower").setDoubleValue(100);
					phyLayerPtr->setMaxTXPower(100);
				}

		        // update position to change connections
		        if ( mobilityUpdateTime != simTime() ){
			        mobilityUpdateTime = simTime();
					vehMobility->changePosition();
		        }

				DBG_MAC << "CM: " << myNodeID << " alters TX power: 5, Inter: "
						<< cm->calcInterfDist() << ", Connect Size: "
						<< cm->getGateList(getParentModule()->getId() ).size() << endl;
				DBG_MAC << "Phy: " << myNodeID << " alters Phy TX power: "
						<< phyLayerPtr->getMaxTXPower() << endl;
/*debug*/
//				cout << "txPower: " << par("txPower").doubleValue() << endl;
//				cout << "cm: " << cm->calcInterfDist()
//						<< " parPMax: " << cm->parPMax
//						<< ", connects: " << cm->getGateList(getParentModule()->getId() ).size() << endl;
//				cout << "--------------------------------" << endl;
/*debug*/

//				if ( myNodeID.compare("node[75]") == 0 )
//					cout << myNodeID << " upload " << simTime() << endl;

				// broadcast pkt, if the queue is not empty
				if ( !myUpperLayerPktQueue.empty() || !myUpperLayerEmgPktQueue.empty()){
					sendBroadcastPacket(prepareUpload());
					if (debugMAC){
						cout << "MAC: nextUploadEvent" << endl;
						cout << myNodeID << ": sending Upload message" << endl;
					}
				}

				par("txPower").setDoubleValue(20);
				txPower = par("txPower");
				// connection manager
				cmModule->par("pMax").setDoubleValue(20);

		        phyLayerPtr->par("maxTXPower").setDoubleValue(10);
		        phyLayerPtr->setMaxTXPower(10);
				// update position to change connections
//				vehMobility->changePosition();
				DBG_MAC << "CM: " << myNodeID << " alters TX power: 20, Inter: "
						<< cm->calcInterfDist() << ", Connect Size: "
						<< cm->getGateList(getParentModule()->getId() ).size() << endl;
				DBG_MAC << "Phy: " << myNodeID << " alters Phy TX power: "
						<< phyLayerPtr->getMaxTXPower() << endl;
    		} else{
				findHost()->getDisplayString().updateWith("r=4,dark gray");
    		}

    	}

// test
//    	sendBroadcastPacket(prepareUpload());
    }
    else if ( msg == nextPrepareCHsCommuEvent ){
    	/*switch to CCH*/
		if ( mChannelID != -1 && mChannelID > 0){
			mChannelID = 0;

			// two channel assignment, each direction uses one SCH

			std::string mEdgeID = vehMobility->getCommandInterface()->getEdgeId(vehMobility->getExternalId());

			if (edgeIdChannel.find(mEdgeID) != edgeIdChannel.end()){
			    mChannelID = edgeIdChannel.at(mEdgeID);
			}

			if (debugMAC){
			    cout << myNodeID << ": CH switch to ch " << mChannelID << endl;
			}

			scheduleAt(simTime(), nextChannelAssignment);
		}
		else {
			DBG_MAC << "nextPrepareCHsCommuEven: Channel ID is invalid! -1 or 0" << endl;
		}

		/* CHs broadcast sequence based on cluster index
		 * MCH shall broadcast at the last.
		 * */
		/*TODO: frame duration*/

		if ( myOptimalLayer > 2 ) {
            if ( mRoleInCluster == CH || mRoleInCluster == MCH) {

//			if ( mRoleInCluster == CH ) {
//				if ( clusterIndex > mClusterHead->clusterIndex){
//				    int index = clusterIndex-1;
//				    // reverse index from largest to smallest
//				    index = abs(index-myNumCluster+1+1);
//
//					simtime_t offset1 = chSwitchGuardInterval
//											+ index * ( getFrameDuration(4000)
//																+ RADIODELAY_11P
//																+ SLOTLENGTH_11P );
//					scheduleAt( simTime() + offset1, nextCHsCommuEvent);
//				}
//				else {
//				    int index = clusterIndex;
//				    // reverse index from largest to smallest.
//				    index = abs(index-myNumCluster+1+1);
//					simtime_t offset1 = chSwitchGuardInterval
//											+ index * ( getFrameDuration(4000)
//																+ RADIODELAY_11P
//																+ SLOTLENGTH_11P );
//					scheduleAt( simTime() + offset1, nextCHsCommuEvent);
//				}

                int index = clusterIndex;
                // reverse index from largest to smallest
                index = abs(index-myNumCluster+1);

                if (debugMAC){
                    std::cout << "CH2CH send order: " << myNodeID << ": " << index << endl;
                }

                simtime_t offset1 = chSwitchGuardInterval
                                        + index * ( getFrameDuration(4000)
                                                            + RADIODELAY_11P
                                                            + SLOTLENGTH_11P );
                scheduleAt( simTime() + offset1, nextCHsCommuEvent);

			}
			else if (mRoleInCluster == MCH) {

//				simtime_t offset1 = chSwitchGuardInterval
//										+ (myNumCluster - 1) * ( getFrameDuration(4000)
//															+ RADIODELAY_11P
//															+ SLOTLENGTH_11P );
//                simtime_t offset1 = chSwitchGuardInterval
//                                        + myNumCluster * ( getFrameDuration(4000)
//                                                            + RADIODELAY_11P
//                                                            + SLOTLENGTH_11P );
//				scheduleAt( simTime() + offset1, nextCHsCommuEvent);
			}
		}
		else {
			if ( mRoleInCluster == CH ) {
				simtime_t offset1 = chSwitchGuardInterval
										+ clusterIndex * ( getFrameDuration(4000)
															+ RADIODELAY_11P
															+ SLOTLENGTH_11P );
				scheduleAt( simTime() + offset1, nextCHsCommuEvent);
			}
		}

		/* next CHs distribute received other CMs information to itself's
		 * myNumCluster is the number of all clusters.
		 * */
		/*TODO: frame duration*/
		simtime_t offset2 = chSwitchGuardInterval
								+ myNumCluster * ( getFrameDuration(4000)
													+ RADIODELAY_11P
													+ SLOTLENGTH_11P );
		scheduleAt( simTime() + offset2, nextPrepareCHsDistrEvent);

    }

    else if ( msg == nextCHsCommuEvent ){

    	findHost()->getDisplayString().updateWith("r=4,blue");

    	// change tx power
		par("txPower").setDoubleValue(250);
		txPower = par("txPower");
		// connection manager
		cModule* cmModule = check_and_cast<cModule*>(cm);
		cmModule->par("pMax").setDoubleValue(250);

        phyLayerPtr->par("maxTXPower").setDoubleValue(125);
        phyLayerPtr->setMaxTXPower(125);

//        cout << findHost()->getFullName();
//    	cout << "  1_CMAC1609: " << mobilityUpdateTime << " " << simTime() << endl;
		// update position to change connections
        if ( mobilityUpdateTime != simTime() ){
	        mobilityUpdateTime = simTime();
			vehMobility->changePosition();
        }

		DBG_MAC << "CH: " << myNodeID << " alters TX power: 250, Inter: "
				<< cm->calcInterfDist() << ", Connect Size: "
				<< cm->getGateList(getParentModule()->getId() ).size() << endl;
		DBG_MAC << "Phy: " << myNodeID << " alters Phy TX power: "
				<< phyLayerPtr->getMaxTXPower() << endl;

    	sendBroadcastPacket(prepareCHtoCH());

    	// change tx power
		par("txPower").setDoubleValue(20);
		txPower = par("txPower");
		// connection manager
		cmModule->par("pMax").setDoubleValue(20);

        phyLayerPtr->par("maxTXPower").setDoubleValue(10);
        phyLayerPtr->setMaxTXPower(10);

		// update position to change connections
//		vehMobility->changePosition();
		DBG_MAC << "CH: " << myNodeID << " alters TX power: 20, Inter: "
				<< cm->calcInterfDist() << ", Connect Size: "
				<< cm->getGateList(getParentModule()->getId() ).size() << endl;

		DBG_MAC << "Phy: " << myNodeID << " alters Phy TX power: "
				<< phyLayerPtr->getMaxTXPower() << endl;

		DBG_MAC << "CH: " << myNodeID << " myCMInfoMap size: "
				<< myCMInfoMap.size() << ", orCMInfoMap size: "
				<< orCMInfoMap.size() << endl;

    	/*after sending, clear*/
    	myCMInfoMap.clear();
    }

    else if ( msg == nextPrepareCHsDistrEvent ){
    	/*switch to SCH#*/
    	mChannelID = mClusterChannelID;
		if ( mChannelID != -1 && mChannelID > 0){
			scheduleAt(simTime(), nextChannelAssignment);
//			cout << myNodeID << ": switch ch to SCH" << mChannelID << endl;
		}
		else {
			DBG_MAC << "nextPrepareCHsDistrEvent: Channel ID is invalid! -1 or 0" << endl;
		}

		/*CHs distribute sequence based on cluster index*/
		/*TODO: frame duration*/
		simtime_t offset1;
		if ( CHDistriBySequence ){
			offset1 = chSwitchGuardInterval
							+ clusterIndex * ( getFrameDuration(8000)
													+ RADIODELAY_11P
													+ SLOTLENGTH_11P );
		}
		else {
			// CHs transmit in the same time slot
			offset1 = chSwitchGuardInterval;
		}
		scheduleAt( simTime() + offset1, nextCHsDistrEvent);

		/*
		 * restart from beginning, connect event
		 * */
		simtime_t offset2;
		if ( CHDistriBySequence ){
			offset2 = chSwitchGuardInterval
								+ myNumCluster * ( getFrameDuration(8000)
													+ RADIODELAY_11P
													+ SLOTLENGTH_11P );
		}
		else {
			// CHs transmit in the same time slot
			offset2 = chSwitchGuardInterval
						+ getFrameDuration(8000)
								+ RADIODELAY_11P
								+ SLOTLENGTH_11P;
		}
		scheduleAt( simTime() + offset2, nextConnectEvent);
    }
    else if ( msg == nextCHsDistrEvent){

    	findHost()->getDisplayString().updateWith("r=4,blue");

    	/* power control
    	 * control Mac layer transmission power
    	 * and connection manager transmission power
		 */
		cModule* cmModule = check_and_cast<cModule*>(cm);
    	if ( myOptimalLayer <= 2){
			par("txPower").setDoubleValue(200);
			txPower = par("txPower");
			// connection manager
			cmModule->par("pMax").setDoubleValue(200);

			phyLayerPtr->par("maxTXPower").setDoubleValue(100);
			phyLayerPtr->setMaxTXPower(100);
    	}
    	else {
			par("txPower").setDoubleValue(5);
			txPower = par("txPower");
			// connection manager
			cmModule->par("pMax").setDoubleValue(5);

			phyLayerPtr->par("maxTXPower").setDoubleValue(2.5);
			phyLayerPtr->setMaxTXPower(2.5);
    	}

//    	cout << "2_CMAC1609: " << mobilityUpdateTime << " " << simTime() << endl;
		// update position to change connections
        if ( mobilityUpdateTime != simTime() ){
	        mobilityUpdateTime = simTime();
			vehMobility->changePosition();
        }

		DBG_MAC << "CH: " << myNodeID << " alters TX power: 5, Inter: "
				<< cm->calcInterfDist() << ", Connect Size: "
				<< cm->getGateList(getParentModule()->getId() ).size() << endl;

		DBG_MAC << "Phy: " << myNodeID << " alters Phy TX power: "
				<< phyLayerPtr->getMaxTXPower() << endl;

		/*broadcast*/
    	sendBroadcastPacket(prepareCHDistr());

    	/*
    	 * power control
    	 * control Mac layer transmission power
    	 * and connection manager transmission power
		 */
		par("txPower").setDoubleValue(5);
		txPower = par("txPower");
		// connection manager
		cmModule->par("pMax").setDoubleValue(5);

        phyLayerPtr->par("maxTXPower").setDoubleValue(2.5);
        phyLayerPtr->setMaxTXPower(2.5);
		// update position to change connections
//		vehMobility->changePosition();
		DBG_MAC << "CH: " << myNodeID << " alters TX power: 5, Inter: "
				<< cm->calcInterfDist() << ", Connect Size: "
				<< cm->getGateList(getParentModule()->getId() ).size() << endl;

		DBG_MAC << "Phy: " << myNodeID << " alters Phy TX power: "
				<< phyLayerPtr->getMaxTXPower() << endl;

		DBG_MAC << "CH: " << myNodeID << " myCMInfoMap size: "
				<< myCMInfoMap.size() << ", orCMInfoMap size: "
				<< orCMInfoMap.size() << endl;

    	/*after sending, clear*/
    	orCMInfoMap.clear();
    	myCMInfoMap.clear();
    }

    else if (msg == nextChannelAssignment){

    	switch(mChannelID) {
    	case 0:
            DBG_MAC << myNodeID << ": Channel --> CCH" << std::endl;
            channelBusySelf(false);
            setActiveChannel(type_CCH);
            channelIdle(true);
            mySCH = Channels::CCH;
            phy11p->changeListeningFrequency(frequency[mySCH]);
            break;
    	case 1:
            DBG_MAC << myNodeID << " Channel --> CRIT_SOL" << std::endl;
            channelBusySelf(false);
            setActiveChannel(type_SCH);
            channelIdle(true);
            mySCH = Channels::CRIT_SOL;
            phy11p->changeListeningFrequency(frequency[mySCH]);
            break;
    	case 2:
            DBG_MAC << myNodeID << " Channel --> SCH1" << std::endl;
            channelBusySelf(false);
            setActiveChannel(type_SCH);
            channelIdle(true);
            mySCH = Channels::SCH1;
            phy11p->changeListeningFrequency(frequency[mySCH]);
            break;
    	case 3:
            DBG_MAC << myNodeID << " Channel --> SCH2" << std::endl;
            channelBusySelf(false);
            setActiveChannel(type_SCH);
            channelIdle(true);
            mySCH = Channels::SCH2;
            phy11p->changeListeningFrequency(frequency[mySCH]);
            break;
    	case 4:
            DBG_MAC << myNodeID << " Channel --> SCH3" << std::endl;
            channelBusySelf(false);
            setActiveChannel(type_SCH);
            channelIdle(true);
            mySCH = Channels::SCH3;
            phy11p->changeListeningFrequency(frequency[mySCH]);
            break;
    	case 5:
    		DBG_MAC << myNodeID << " Channel --> SCH4" << std::endl;
            channelBusySelf(false);
			setActiveChannel(type_SCH);
			channelIdle(true);
			mySCH = Channels::SCH4;
			phy11p->changeListeningFrequency(frequency[mySCH]);
			break;
    	case 6:
    		DBG_MAC << myNodeID << " Channel --> HPPS" << std::endl;
            channelBusySelf(false);
			setActiveChannel(type_SCH);
			channelIdle(true);
			mySCH = Channels::HPPS;
			phy11p->changeListeningFrequency(frequency[mySCH]);
			break;
    	}

    }
}

void CMac1609_4TwoWay::handleUpperControl(cMessage* msg) {
    assert(false);
}

void CMac1609_4TwoWay::handleUpperMsg(cMessage* msg) {

    CMACWSM* thisMsg;
    if ((thisMsg = dynamic_cast<CMACWSM*>(msg)) == NULL) {
        error("WaveMac only accepts WaveShortMessages");
    }

//    t_access_category ac = mapPriority(thisMsg->getPriority());

//    DBG_MAC << "Received a message from upper layer for channel "
//            << thisMsg->getChannelNumber() << " Access Category (Priority):  "
//            << ac << std::endl;
//
//    t_channel chan;
//
//    //rewrite SCH channel to actual SCH the CMac1609_4TwoWay is set to
//    if (thisMsg->getChannelNumber() == Channels::SCH1) {
//        ASSERT(useSCH);
//        thisMsg->setChannelNumber(mySCH);
//        chan = type_SCH;
//    }
//
//    //put this packet in its queue
//    if (thisMsg->getChannelNumber() == Channels::CCH) {
//        chan = type_CCH;
//    }

//    int num = myEDCA[chan]->queuePacket(ac,thisMsg);



    // Priority queue for emergency pkts
    if (strcmp(thisMsg->getName(), "data" ) == 0){
        if (myUpperLayerEmgPktQueue.length() < 1024){
            myUpperLayerEmgPktQueue.insert(thisMsg);
            DBG_MAC << "Received a emg data message from upper layer, Emg Q Length: "
                    << myUpperLayerEmgPktQueue.length() << endl;
        }
        else{
            gPeQueueDelPkts++;
            delete(thisMsg);
            cout << myNodeID
                 << ": A data pkt rvced from upper is deleted!" << endl;

            DBG_MAC << "Queue is full, delete the pkt, Q Length: "
                    << myUpperLayerEmgPktQueue.length() << endl;
        }
    }
    else {
        if ( myUpperLayerPktQueue.length() < 1024){
            myUpperLayerPktQueue.insert(thisMsg);
            DBG_MAC << "Received a wsm data message from upper layer, Q Length: "
                    << myUpperLayerPktQueue.length() << endl;
        }
        else{
            gPeQueueDelPkts++;
            delete(thisMsg);
            cout << myNodeID
                 << ": A data pkt rvced from upper is deleted!" << endl;

            DBG_MAC << "Queue is full, delete the pkt, Q Length: "
                    << myUpperLayerPktQueue.length() << endl;
        }
    }

    // ECMAC-1.0 ######################################################
    /*limit the size of pkt queue*/
//    if ( myUpperLayerPktQueue.length() < 1024){
//    	myUpperLayerPktQueue.insert(thisMsg);
//        DBG_MAC << "Received a data message from upper layer, Q Length: "
//        		<< myUpperLayerPktQueue.length() << endl;
//    }
//    else {
//
////    	for ( cPacketQueue::Iterator it(myUpperLayerPktQueue, 1); !it.end(); ++it){
////
////    		CMACWSM* msgInQ = (CMACWSM*) it();
////    		if ( simTime() > (msgInQ->getTimestamp() + msgInQ->getTTL()) ){
////
////    		}
////    	}
//
//    	gPeQueueDelPkts++;
//    	delete(thisMsg);
//    	cout << myNodeID
//    		 << ": A data pkt rvced from upper is deleted!" << endl;
//
//        DBG_MAC << "Queue is full, delete the pkt, Q Length: "
//        		<< myUpperLayerPktQueue.length() << endl;
//    }

    // ECMAC-1.0 ######################################################

//    //packet was dropped in Mac
//    if (num == -1) {
//        statsDroppedPackets++;
//        return;
//    }

//    //if this packet is not at the front of a new queue we dont have to reevaluate times
//    DBG_MAC << "sorted packet into queue of EDCA " << chan << " this packet is now at position: " << num << std::endl;
//
//    if (chan == activeChannel) {
//        DBG_MAC << "this packet is for the currently active channel" << std::endl;
//    }
//    else {
//        DBG_MAC << "this packet is NOT for the currently active channel" << std::endl;
//    }
//
//    if (num == 1 && idleChannel == true && chan == activeChannel) {
//
//        simtime_t nextEvent = myEDCA[chan]->startContent(lastIdle,guardActive());
//
//        if (nextEvent != -1) {
//            if ((!useSCH) || (nextEvent <= nextChannelSwitch->getArrivalTime())) {
//                if (nextMacEvent->isScheduled()) {
//                    cancelEvent(nextMacEvent);
//                }
//                scheduleAt(nextEvent,nextMacEvent);
//                DBG_MAC << "Updated nextMacEvent:" << nextMacEvent->getArrivalTime().raw() << std::endl;
//            }
//            else {
//                DBG_MAC << "Too little time in this interval. Will not schedule nextMacEvent" << std::endl;
//                //it is possible that this queue has an txop. we have to revoke it
//                myEDCA[activeChannel]->revokeTxOPs();
//                statsNumTooLittleTime++;
//            }
//        }
//        else {
//            cancelEvent(nextMacEvent);
//        }
//    }
//    if (num == 1 && idleChannel == false && myEDCA[chan]->myQueues[ac].currentBackoff == 0 && chan == activeChannel) {
//        myEDCA[chan]->backoff(ac);
//    }

}

void CMac1609_4TwoWay::handleLowerControl(cMessage* msg) {
    if (msg->getKind() == MacToPhyInterface::TX_OVER) {

        DBG_MAC << "Successfully transmitted a packet on " << lastAC << std::endl;

        phy->setRadioState(Radio::RX);

        //message was sent
        //update EDCA queue. go into post-transmit backoff and set cwCur to cwMin
//        myEDCA[activeChannel]->postTransmit(lastAC);
        //channel just turned idle.
        //don't set the chan to idle. the PHY layer decides, not us.

//        if (guardActive()) {
//            opp_error("We shouldnt have sent a packet in guard!");
//        }
    }
    else if (msg->getKind() == Mac80211pToPhy11pInterface::CHANNEL_BUSY) {
        channelBusy();
    }
    else if (msg->getKind() == Mac80211pToPhy11pInterface::CHANNEL_IDLE) {
        channelIdle();
    }
    else if (msg->getKind() == Decider80211p::BITERROR) {
        statsSNIRLostPackets++;
        if ( debugMAC ){
            cout << myNodeID << " pkt bit errors! " << simTime() << endl;
        }
		findHost()->getDisplayString().updateWith("r=1,red");
		gPeBitErrPkts++;

        DBG_MAC << "A packet was not received due to biterrors" << std::endl;
    }
    else if (msg->getKind() == Decider80211p::RECWHILESEND) {
        statsTXRXLostPackets++;
        if ( debugMAC ){
            cout << myNodeID << " RECWHILESEND! " << simTime() << endl;
        }
        findHost()->getDisplayString().updateWith("r=1,red");
        gPeTxWhileSendPkts++;
        DBG_MAC << "A packet was not received because we were sending while receiving" << std::endl;
    }
    else if (msg->getKind() == MacToPhyInterface::RADIO_SWITCHING_OVER) {
        DBG_MAC << "Phylayer said radio switching is done" << std::endl;
    }
    else if (msg->getKind() == BaseDecider::PACKET_DROPPED) {
        phy->setRadioState(Radio::RX);
        if ( debugMAC ){
            cout << myNodeID << " Phy pkt dropped! " << simTime() << endl;
        }
        gPePhyDropPkts++;
        DBG_MAC << "Phylayer said packet was dropped" << std::endl;
    }
    else {
        DBG_MAC << "Invalid control message type (type=NOTHING) : name=" << msg->getName() << " modulesrc=" << msg->getSenderModule()->getFullPath() << "." << std::endl;
        assert(false);
    }
    delete msg;
}

void CMac1609_4TwoWay::handleLowerMsg(cMessage* msg) {
    CMac80211Pkt* macPkt = static_cast<CMac80211Pkt*>(msg);
    ASSERT(macPkt);

//    CMACWSM*  wsm =  dynamic_cast<CMACWSM*>(macPkt->decapsulate());
//    ASSERT(wsm);

    long dest = macPkt->getDestAddr();

    DBG_MAC << "Received frame name= " << macPkt->getName()
	                << ", myState:" << " src=" << macPkt->getSrcAddr()
	                << " dst=" << macPkt->getDestAddr() << " myAddr="
	                << myMacAddress << std::endl;

    /*receive unicast pkt*/
    /*NOT USED*/                
    if (macPkt->getDestAddr() == myMacAddress) {
        DBG_MAC << "Received a data packet addressed to me." << std::endl;
        statsReceivedPackets++;

        if ( strcmp(macPkt->getName(), "data") == 0){
        	if ( mRoleInCluster == CH || mRoleInCluster == MCH ){
        		CMACWSM* up = dynamic_cast<CMACWSM*>(macPkt->decapsulate());

        		CMSafetyInfo CMInfo;
        		CMInfo.nodeID = up->getNodeID();
        		CMInfo.speed = up->getSpeed();
        		CMInfo.currentCoord = up->getPosition();
        		CMInfo.destID = up->getDestID();
        		CMInfo.ttl = up->getTTL();
        		CMInfo.timeStamp = up->getTimestamp();

        		std::pair<std::map<std::string, CMSafetyInfo>::iterator, bool> ret;
        		ret = myCMInfoMap.insert(std::pair<std::string, CMSafetyInfo>( up->getNodeID(),
        																		CMInfo ) );
        		// if existed, then update
        		if (ret.second == false) {
        			ret.first->second.nodeID = up->getNodeID();
        			ret.first->second.currentCoord = up->getPosition();
        			ret.first->second.speed = up->getSpeed();
        			ret.first->second.destID = up->getDestID();
        			ret.first->second.ttl = up->getTTL();
        			ret.first->second.timeStamp = up->getTimestamp();
        		}

//        		cout << myNodeID << " CM size " << myCMInfoMap.size()
//        				<< " : receive Upload pkt from " << up->getNodeID() << endl;

        		sendUp(up);
        	}
        }
//        sendUp(wsm);
    }
    /*receive broadcast pkt*/
    else if (dest == LAddress::L2BROADCAST) {
        statsReceivedBroadcasts++;

        /*Chris*/
        if ( strcmp(macPkt->getName(), "data") == 0 ){
    		CMACWSM* up = dynamic_cast<CMACWSM*>(macPkt->decapsulate());

//    		if ( clusterIndex == 0)
//    			cout << myNodeID << ": |"
//    					<< up->getDestID() << "| " << simTime() << endl;

//    		if ( strcmp(up->getDestID(), "emergency" ) == 0 )
//				cout << myNodeID << " 0 EMG pkt " << simTime() << endl;

        	if ( mRoleInCluster == CH || mRoleInCluster == MCH ){
        		if (strcmp( up->getDestID(), "" ) != 0 ){
					CMSafetyInfo CMInfo;
					CMInfo.nodeID = up->getNodeID();
					CMInfo.speed = up->getSpeed();
					CMInfo.currentCoord = up->getPosition();
					CMInfo.destID = up->getDestID();
					CMInfo.ttl = up->getTTL();
					CMInfo.timeStamp = up->getTimestamp();
					CMInfo.serial = up->getSerial();

					std::pair<std::map<std::string, CMSafetyInfo>::iterator, bool> ret;
					ret = myCMInfoMap.insert(std::pair<std::string, CMSafetyInfo>( up->getNodeID(),
																					CMInfo ) );
					// if existed, then update
					if (ret.second == false) {
						ret.first->second.nodeID = up->getNodeID();
						ret.first->second.currentCoord = up->getPosition();
						ret.first->second.speed = up->getSpeed();
						ret.first->second.destID = up->getDestID();
						ret.first->second.ttl = up->getTTL();
						ret.first->second.timeStamp = up->getTimestamp();
						ret.first->second.serial = up->getSerial();
					}

//					cout << myNodeID << " CM size " << myCMInfoMap.size()
//							<< " : receive Upload pkt from " << up->getNodeID() << endl;

					DBG_MAC << myNodeID << " receives CM data from " << up->getNodeID()
							<<  " myCMInfoMap size: " << myCMInfoMap.size() << endl;

//					if ( strcmp(up->getDestID(), "emergency" ) == 0 )
//						cout << myNodeID << " 1 EMG pkt " << simTime() << endl;
					// sendup if the packet is for me
					sendUp(up);
        		}
        		else {
        			DBG_MAC << myNodeID << " drops an empty pkt from "
        					<< up->getNodeID() << endl;
        		}
        	}
        	else if ( mRoleInCluster == CM ){

//				if ( strcmp(up->getDestID(), "emergency" ) == 0 )
//					cout << myNodeID << " 2 EMG pkt " << simTime() << endl;

        		sendUp(up);
        	}
        }
        else if ( strcmp(macPkt->getName(), "CH2CH") == 0){
        	if ( mRoleInCluster == CH || mRoleInCluster == MCH ){
        		CHtoCH* CH2CH = dynamic_cast<CHtoCH*>(macPkt->decapsulate());

        		/*sender's own CM list*/
        		for ( size_t i = 0; i < CH2CH->getCMListArraySize(); ++i ){
            		/*1. check if each item of this pkt is for me*/
            		CMACWSM* cwsm = new CMACWSM("data");
            		cwsm->setDestID(CH2CH->getCMList(i).destID.c_str());
            		cwsm->setTTL(CH2CH->getCMList(i).ttl);
            		cwsm->setTimestamp(CH2CH->getCMList(i).timeStamp);
            		cwsm->setSerial(CH2CH->getCMList(i).serial);
            		cwsm->setNodeID(CH2CH->getCMList(i).nodeID.c_str());

            		// if there is empty pkt, drop
            		if ( strcmp( cwsm->getDestID(), "" ) == 0 )
            			continue;

//					if ( strcmp(cwsm->getDestID(), "emergency" ) == 0 )
//						cout << myNodeID << " 3 EMG pkt " << simTime() << endl;

            		// send to upper layer to check dest
            		sendUp(cwsm);

            		// pkt is sent to me, skip
            		if ( myNodeID.compare(CH2CH->getCMList(i).destID.c_str()) == 0 ){
            			continue;
            		}

            		/*2. cache CH2CH packet*/
        			CMSafetyInfo orCMInfo;

            		orCMInfo.nodeID = CH2CH->getCMList(i).nodeID;
            		orCMInfo.speed = CH2CH->getCMList(i).speed;
            		orCMInfo.currentCoord = CH2CH->getCMList(i).currentCoord;
            		orCMInfo.destID = CH2CH->getCMList(i).destID;
            		orCMInfo.ttl = CH2CH->getCMList(i).ttl;
            		orCMInfo.timeStamp = CH2CH->getCMList(i).timeStamp;
            		orCMInfo.serial = CH2CH->getCMList(i).serial;

            		std::pair<std::map<std::string, CMSafetyInfo>::iterator, bool> ret;
            		ret = orCMInfoMap.insert(std::pair<std::string, CMSafetyInfo>( CH2CH->getCMList(i).nodeID.c_str(),
            																		orCMInfo ) );
            		// if existed, then update
//            		if (ret.second == false) {
//            			ret.first->second.nodeID = CH2CH->getCMList(i).nodeID;
//            			ret.first->second.currentCoord = CH2CH->getCMList(i).currentCoord;
//            			ret.first->second.speed = CH2CH->getCMList(i).speed;
//            			ret.first->second.destID = CH2CH->getCMList(i).destID;
//            			ret.first->second.ttl = CH2CH->getCMList(i).ttl;
//            			ret.first->second.timeStamp = CH2CH->getCMList(i).timeStamp;
//            			ret.first->second.serial = CH2CH->getCMList(i).serial;
//            		}
        		}

        		/*sender's other CMs list*/
        		for ( size_t i = 0; i < CH2CH->getOrCMListArraySize(); ++i ){
					/*1. check if each item of this pkt is for me*/
					CMACWSM* cwsm = new CMACWSM("data");

					cwsm->setDestID(CH2CH->getOrCMList(i).destID.c_str());
					cwsm->setTTL(CH2CH->getOrCMList(i).ttl);
					cwsm->setTimestamp(CH2CH->getOrCMList(i).timeStamp);
					cwsm->setSerial(CH2CH->getOrCMList(i).serial);
					cwsm->setNodeID(CH2CH->getOrCMList(i).nodeID.c_str());

					// if there is empty pkt, drop
					if ( strcmp( cwsm->getDestID(), "" ) == 0 )
						continue;

//					if ( strcmp(cwsm->getDestID(), "emergency" ) == 0 )
//						cout << myNodeID << " 3 EMG pkt " << simTime() << endl;

					// send to upper layer to check dest
					sendUp(cwsm);

					// pkt is sent to me, jump to next item
					if ( myNodeID.compare(CH2CH->getOrCMList(i).destID.c_str()) == 0 ){
						continue;
					}

					/*2. cache CH2CH packet*/
					CMSafetyInfo orCMInfo;

					orCMInfo.nodeID = CH2CH->getOrCMList(i).nodeID;
					orCMInfo.speed = CH2CH->getOrCMList(i).speed;
					orCMInfo.currentCoord = CH2CH->getOrCMList(i).currentCoord;
					orCMInfo.destID = CH2CH->getOrCMList(i).destID;
					orCMInfo.ttl = CH2CH->getOrCMList(i).ttl;
					orCMInfo.timeStamp = CH2CH->getOrCMList(i).timeStamp;
					orCMInfo.serial = CH2CH->getOrCMList(i).serial;

					std::pair<std::map<std::string, CMSafetyInfo>::iterator, bool> ret;
					ret = orCMInfoMap.insert(std::pair<std::string, CMSafetyInfo>( CH2CH->getOrCMList(i).nodeID.c_str(),
																					orCMInfo ) );
					// if existed, then update
					// no need to update
//					if (ret.second == false) {
//						ret.first->second.nodeID = CH2CH->getOrCMList(i).nodeID;
//						ret.first->second.currentCoord = CH2CH->getOrCMList(i).currentCoord;
//						ret.first->second.speed = CH2CH->getOrCMList(i).speed;
//						ret.first->second.destID = CH2CH->getOrCMList(i).destID;
//						ret.first->second.ttl = CH2CH->getOrCMList(i).ttl;
//						ret.first->second.timeStamp = CH2CH->getOrCMList(i).timeStamp;
//						ret.first->second.serial = CH2CH->getOrCMList(i).serial;
//					}

				}

        		DBG_MAC << myNodeID << " orCM size " << orCMInfoMap.size()
        				<< " : receive CH2CH pkt from " << CH2CH->getNodeID()
        				<< ", myCMSize: " << CH2CH->getCMListArraySize()
        				<< ", orCMSize: " << CH2CH->getOrCMListArraySize()
        				<< endl;

//        		cout << myNodeID << " orCM size " << orCMInfoMap.size()
//        				<< " : receive CH2CH pkt from " << CH2CH->getNodeID() << endl;

        		delete(CH2CH);
        	}
            // not useful because this is for CH -> CH (MCH)
            else {
            
            }
        }
        else if ( strcmp(macPkt->getName(), "CHDistr") == 0 ){
        	if ( mRoleInCluster == CM ){
        		CHDistr* CHDistrPkt = dynamic_cast<CHDistr*>(macPkt->decapsulate());

        		for ( size_t i = 0; i < CHDistrPkt->getOrCMListArraySize(); ++i ){
            		/*1. check each item of this pkt is if for me*/
            		CMACWSM* cwsm = new CMACWSM("data");
            		cwsm->setDestID(CHDistrPkt->getOrCMList(i).destID.c_str());
            		cwsm->setTTL(CHDistrPkt->getOrCMList(i).ttl);
            		cwsm->setTimestamp(CHDistrPkt->getOrCMList(i).timeStamp);
            		cwsm->setSerial(CHDistrPkt->getOrCMList(i).serial);
            		cwsm->setNodeID(CHDistrPkt->getOrCMList(i).nodeID.c_str());

            		// if there is empty pkt, drop
            		if ( strcmp( cwsm->getDestID(), "" ) == 0 )
            			continue;

//					if ( strcmp(cwsm->getDestID(), "emergency" ) == 0 )
//						cout << myNodeID << " 4 EMG pkt " << simTime() << endl;
            		// send to upper layer to check dest
            		sendUp(cwsm);

            		// pkt is sent to me, jump to next item
            		if ( myNodeID.compare(CHDistrPkt->getOrCMList(i).destID.c_str()) == 0 ){
            			continue;
            		}
        		}

        		DBG_MAC << myNodeID << " orCM size " << orCMInfoMap.size()
        				<< " : receive CHDist pkt from " << CHDistrPkt->getNodeID() << endl;

//        		cout << myNodeID << " orCM size " << orCMInfoMap.size()
//        				<< " : receive CHDistrPkt pkt from " << CHDistrPkt->getNodeID() << endl;

        		delete(CHDistrPkt);
        	}
            // not useful because this is for CH -> CMss
            else{

            }

        }
        /*Chris end*/
//        sendUp(wsm);
    }
    else {
        DBG_MAC << "Packet not for me, deleting..." << std::endl;
//        delete wsm;
    }
    delete macPkt;
}

void CMac1609_4TwoWay::setActiveChannel(t_channel state) {
    activeChannel = state;
    assert(state == type_CCH || (useSCH && state == type_SCH));
}

void CMac1609_4TwoWay::finish() {
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

    // Chris

    if (nextClusteringEvent->isScheduled())
    	cancelAndDelete(nextClusteringEvent);
    else
    	delete nextClusteringEvent;

    if (nextTAFEvent->isScheduled())
    	cancelAndDelete(nextTAFEvent);
    else
    	delete nextTAFEvent;

    if (nextCheckEvent->isScheduled())
    	cancelAndDelete(nextCheckEvent);
    else
    	delete nextCheckEvent;

    if (nextChannelAssignment->isScheduled())
    	cancelAndDelete(nextChannelAssignment);
    else
    	delete nextChannelAssignment;

    if (startEvent->isScheduled())
    	cancelAndDelete(startEvent);
    else
    	delete startEvent;

    if (nextPrepareCHsCommuEvent->isScheduled())
    	cancelAndDelete(nextPrepareCHsCommuEvent);
    else
    	delete nextPrepareCHsCommuEvent;

    if (nextPrepareCHsDistrEvent->isScheduled())
    	cancelAndDelete(nextPrepareCHsDistrEvent);
    else
    	delete nextPrepareCHsDistrEvent;

    if (nextPrepareUploadEvent->isScheduled())
    	cancelAndDelete(nextPrepareUploadEvent);
    else
    	delete nextPrepareUploadEvent;

    if (nextUploadEvent->isScheduled())
    	cancelAndDelete(nextUploadEvent);
    else
    	delete nextUploadEvent;

    if (nextCHsCommuEvent->isScheduled())
    	cancelAndDelete(nextCHsCommuEvent);
    else
    	delete nextCHsCommuEvent;

    if (nextCHsDistrEvent->isScheduled())
    	cancelAndDelete(nextCHsDistrEvent);
    else
    	delete nextCHsDistrEvent;

    if (nextConnectEvent->isScheduled())
    	cancelAndDelete(nextConnectEvent);
    else
    	delete nextConnectEvent;

    // PE
    gPeQueuePkts += myUpperLayerPktQueue.length();

    if ( strcmp(findHost()->getFullName(), gLatestNodeID.c_str()) == 0 ){
		cout << "--------------- MAC INFO --------------------" << endl;
		cout << myNodeID << " --> finished " << endl;
		cout << "NumVeh: " << gPeNumVeh << endl;
		cout << "ClusterLayers: " << gPeClusterLayers << endl;
		cout << "ClusterSize: "<< gPeClusterSize << endl;
		cout << "ClusterNum: "<< gPeClusterNum << endl<<endl;
		cout << "gPeQueuePkts: " << gPeQueuePkts << endl;
		cout << "gPeQueueDelPkts: " << gPeQueueDelPkts << endl;
		cout << "gPeQueueExpiredPkts: " << gPeQueueExpiredPkts << endl;
		cout << "gPeBitErrPkts: " << gPeBitErrPkts << endl;
		cout << "gPeTxWhileSendPkts: " << gPeTxWhileSendPkts << endl;
		cout << "gPePhyDropPkts: " << gPePhyDropPkts << endl;

		double meanSlotCountByTSAlgo;
		double meanSlotCountOptimal;
		meanSlotCountByTSAlgo = std::accumulate(g_vSlotCountByTSAlgo.begin(),
		                                            g_vSlotCountByTSAlgo.end(),
		                                            0)/(double)g_vSlotCountByTSAlgo.size();
		meanSlotCountOptimal = std::accumulate(g_vSlotCountOptimal.begin(),
		                                         g_vSlotCountOptimal.end(),
		                                         0)/(double)g_vSlotCountOptimal.size();
		cout << "meanSlotCountByTSAlgo: " << meanSlotCountByTSAlgo << endl;
		cout << "meanSlotCountOptimal: " << meanSlotCountOptimal << endl;

		if ( macSaveToFile ){
            char fileName[100];

            if (d_bgTrafficInterval > 0){
                sprintf(fileName, "result/ECMAC-%s-Mac-d-%d-bg-%3f-s-%d.csv", schemeName.c_str(), density, d_bgTrafficInterval, seed);
            }
            else{
                sprintf(fileName, "result/ECMAC-%s-Mac-d-%d-s-%d.csv", schemeName.c_str(), density, seed);
            }
            std::ofstream data;
            data.open(fileName);
            if ( data.is_open()){
                data << "scheme," << schemeName << endl;
                data << "BGTI," << d_bgTrafficInterval << endl;
                data << "seed," << seed << endl;
                data << "density," << density << endl;

                data << "NumVeh," << gPeNumVeh << endl;
                data << "ClusterLayers," << gPeClusterLayers << endl;
                data << "ClusterSize,"<< gPeClusterSize << endl;
                data << "ClusterNum,"<< gPeClusterNum << endl;
                data << "meanSlotCountByTSAlgo," << meanSlotCountByTSAlgo << endl;
                data << "meanSlotCountOptimal," << meanSlotCountOptimal << endl;

                data << "gPeQueuePkts,"<< gPeQueuePkts << endl;
                data << "gPeQueueDelPkts,"<<gPeQueueDelPkts << endl;
                data << "gPeQueueExpiredPkts,"<< gPeQueueExpiredPkts << endl;
                data << "gPeBitErrPkts,"<< gPeBitErrPkts << endl;
                data << "gPeTxWhileSendPkts,"<<gPeTxWhileSendPkts<<endl;
                data << "gPePhyDropPkts,"<<gPePhyDropPkts << endl;
                data << endl;

                data.close();
            }
            else{
                error("can't open result file!");
            }
		}
    }

    myUpperLayerPktQueue.clear();
    myUpperLayerEmgPktQueue.clear();
    mNeighborVec.clear();
    mClusterNeighborsVec.clear();
    mCoveredCHVec.clear();
    timeSlotAlloStrVec.clear();
    timeSlotAlloVec.clear();
    myCMInfoMap.clear();
    orCMInfoMap.clear();
    // Chris ends

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

void CMac1609_4TwoWay::attachSignal(CMac80211Pkt* mac, simtime_t startTime, double frequency) {

    simtime_t duration = getFrameDuration(mac->getBitLength());

    Signal* s = createSignal(startTime, duration, txPower, bitrate, frequency);
    MacToPhyControlInfo* cinfo = new MacToPhyControlInfo(s);

    mac->setControlInfo(cinfo);
}

Signal* CMac1609_4TwoWay::createSignal(simtime_t start, simtime_t length, double power, double bitrate, double frequency) {
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
bool CMac1609_4TwoWay::guardActive() const {
    if (!useSCH) return false;
    if (simTime().dbl() - nextChannelSwitch->getSendingTime() <= GUARD_INTERVAL_11P)
        return true;
    return false;
}

/* returns the time until the guard is over */
simtime_t CMac1609_4TwoWay::timeLeftTillGuardOver() const {
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
simtime_t CMac1609_4TwoWay::timeLeftInSlot() const {
    ASSERT(useSCH);
    return nextChannelSwitch->getArrivalTime() - simTime();
}

/* Will change the Service Channel on which the mac layer is listening and sending */
void CMac1609_4TwoWay::changeServiceChannel(int cN) {
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



int CMac1609_4TwoWay::EDCA::queuePacket(t_access_category ac,CMACWSM* msg) {

    if (maxQueueSize && myQueues[ac].queue.size() >= maxQueueSize) {
        delete msg;
        return -1;
    }
    myQueues[ac].queue.push(msg);
    return myQueues[ac].queue.size();
}

int CMac1609_4TwoWay::EDCA::createQueue(int aifsn, int cwMin, int cwMax,t_access_category ac) {

    if (myQueues.find(ac) != myQueues.end()) {
        opp_error("You can only add one queue per Access Category per EDCA subsystem");
    }

    EDCAQueue newQueue(aifsn,cwMin,cwMax,ac);
    myQueues[ac] = newQueue;

    return ++numQueues;
}

CMac1609_4TwoWay::t_access_category CMac1609_4TwoWay::mapPriority(int prio) {
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

CMACWSM* CMac1609_4TwoWay::EDCA::initiateTransmit(simtime_t lastIdle) {

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

simtime_t CMac1609_4TwoWay::EDCA::startContent(simtime_t idleSince,bool guardActive) {

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

void CMac1609_4TwoWay::EDCA::stopContent(bool allowBackoff, bool generateTxOp) {
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
void CMac1609_4TwoWay::EDCA::backoff(t_access_category ac) {
    myQueues[ac].currentBackoff = intuniform(0,myQueues[ac].cwCur);
    statsSlotsBackoff += myQueues[ac].currentBackoff;
    statsNumBackoff++;
    DBG_MAC << "Going into Backoff because channel was busy when new packet arrived from upperLayer" << std::endl;
}

void CMac1609_4TwoWay::EDCA::postTransmit(t_access_category ac) {
    delete myQueues[ac].queue.front();
    myQueues[ac].queue.pop();
    myQueues[ac].cwCur = myQueues[ac].cwMin;
    //post transmit backoff
    myQueues[ac].currentBackoff = intuniform(0,myQueues[ac].cwCur);
    statsSlotsBackoff += myQueues[ac].currentBackoff;
    statsNumBackoff++;
    DBG_MAC << "Queue " << ac << " will go into post-transmit backoff for " << myQueues[ac].currentBackoff << " slots" << std::endl;
}

void CMac1609_4TwoWay::EDCA::cleanUp() {
    for (std::map<t_access_category, EDCAQueue>::iterator iter = myQueues.begin(); iter != myQueues.end(); iter++) {
        while (iter->second.queue.size() != 0) {
            delete iter->second.queue.front();
            iter->second.queue.pop();
        }
    }
    myQueues.clear();
}

void CMac1609_4TwoWay::EDCA::revokeTxOPs() {
    for (std::map<t_access_category, EDCAQueue>::iterator iter = myQueues.begin(); iter != myQueues.end(); iter++) {
        if (iter->second.txOP == true) {
            iter->second.txOP = false;
            iter->second.currentBackoff = 0;
        }
    }
}

void CMac1609_4TwoWay::channelBusySelf(bool generateTxOp) {

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
//    myEDCA[activeChannel]->stopContent(false, generateTxOp);
}

void CMac1609_4TwoWay::channelBusy() {

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
//    myEDCA[activeChannel]->stopContent(true,false);
}

void CMac1609_4TwoWay::channelIdle(bool afterSwitch) {

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
        delay = GUARD_INTERVAL_11P;
    }
    if (useSCH) {
//        delay += timeLeftTillGuardOver();
    }

    //channel turned idle! lets start contention!
    lastIdle = delay + simTime();
    statsTotalBusyTime += simTime() - lastBusy;

    //get next Event from current EDCA subsystem
//    simtime_t nextEvent = myEDCA[activeChannel]->startContent(lastIdle,guardActive());
//    if (nextEvent != -1) {
//        if ((!useSCH) || (nextEvent < nextChannelSwitch->getArrivalTime())) {
//            scheduleAt(nextEvent,nextMacEvent);
//            DBG_MAC << "next Event is at " << nextMacEvent->getArrivalTime().raw() << std::endl;
//        }
//        else {
//            DBG_MAC << "Too little time in this interval. will not schedule macEvent" << std::endl;
//            statsNumTooLittleTime++;
//            myEDCA[activeChannel]->revokeTxOPs();
//        }
//    }
//    else {
//        DBG_MAC << "I don't have any new events in this EDCA sub system" << std::endl;
//    }
}

void CMac1609_4TwoWay::setParametersForBitrate(int bitrate) {
    for (unsigned int i = 0; i < NUM_BITRATES_80211P; i++) {
        if (bitrate == BITRATES_80211P[i]) {
            n_dbps = N_DBPS_80211P[i];
            return;
        }
    }
    opp_error("Chosen Bitrate is not valid for 802.11p: Valid rates are: 3Mbps, 4.5Mbps, 6Mbps, 9Mbps, 12Mbps, 18Mbps, 24Mbps and 27Mbps. Please adjust your omnetpp.ini file accordingly.");
}


simtime_t CMac1609_4TwoWay::getFrameDuration(int payloadLengthBits) const {
    // calculate frame duration according to Equation (17-29) of the IEEE 802.11-2007 standard
    simtime_t duration = PHY_HDR_PREAMBLE_DURATION + PHY_HDR_PLCPSIGNAL_DURATION + T_SYM_80211P * ceil( (16 + payloadLengthBits + 6)/(n_dbps) );

    return duration;
}

bool cmpFunction_cmactwoway(cModule* i, cModule* j){
	Veins::TraCIMobility* i_Mob;
	Veins::TraCIMobility* j_Mob;
	i_Mob = (Veins::TraCIMobility*)i->getSubmodule("veinsmobility");
	j_Mob = (Veins::TraCIMobility*)j->getSubmodule("veinsmobility");

//	return (i_Mob->getCurrentPosition().x) < (j_Mob->getCurrentPosition().x);
    return (i_Mob->commandGetOffset()) < (j_Mob->commandGetOffset());

}

int CMac1609_4TwoWay::optTreeDepth( int numNodes ){
    int optd;
    double lowBound = log(numNodes)/2;
    double upperBound = (log(numNodes)+1)/2;

//    lowBount =

    int roundupLowBound = int(lowBound);
    int rounddownUpperBound = int(upperBound);

    optd = roundupLowBound;
    if (roundupLowBound - rounddownUpperBound >= 1){
        cout << "LowBound is " << roundupLowBound << " upperBound is " << rounddownUpperBound  << endl;
    }
    return optd;
}

int CMac1609_4TwoWay::optNumChildren( int numNodes, int optd){
//    return (int)(pow(numNodes,(1/(double)optd))+1);
    return (int)(pow(numNodes,(1/(double)optd)));
}

/*this function is not used*/
cModule* CMac1609_4TwoWay::findCenter(cModule* currentVeh,
								CMac1609_4TwoWay* cmac1609_4,
								float& minAveDis,
								cModule* &modMinAveDis){
	float sumDistance = 0.0;
	float averageDistance = 0.0;
	vector<cModule*>::iterator itClusterNeighbors;
	for (itClusterNeighbors = cmac1609_4->mClusterNeighborsVec.begin();
			itClusterNeighbors != cmac1609_4->mClusterNeighborsVec.end();
			itClusterNeighbors++ ){
		cModule* mobModule = currentVeh->getModuleByPath(".veinsmobility");
		Veins::TraCIMobility* mobility = check_and_cast<Veins::TraCIMobility*>(mobModule);
		cModule* mobNeiModule = (*itClusterNeighbors)->getModuleByPath(".veinsmobility");
		Veins::TraCIMobility* mobilityNei = check_and_cast<Veins::TraCIMobility*>(mobNeiModule);

		sumDistance = sumDistance + mobility->getCurrentPosition().distance(mobilityNei->getCurrentPosition());
		averageDistance = sumDistance / ( cmac1609_4->mClusterNeighborsVec.size() );
	}

	if (minAveDis == 0.0){
		minAveDis = averageDistance;
		modMinAveDis = currentVeh;
	}
	else if ( minAveDis > averageDistance ){
		minAveDis = averageDistance;
		modMinAveDis = currentVeh;
	}

	return currentVeh;
}

TAF* CMac1609_4TwoWay::prepareTAF() {
	// Timing Advertisement Frame
	TAF* taf = new TAF("TAF");

	taf->setNodeID(findHost()->getFullName());
	taf->setCMArraySize( timeSlotAlloVec.size() );

	for (unsigned int i = 0; i < timeSlotAlloVec.size(); i++ ){
		if (timeSlotAlloVec[i] != nullptr){
			cModule* macModuleCM = timeSlotAlloVec[i]->getModuleByPath(".nic.mac1609_4");
			CMac1609_4TwoWay* macCM = check_and_cast<CMac1609_4TwoWay*>(macModuleCM);
			taf->setCM(i, macCM->myMacAddress);
		}
		else {
			taf->setCM(i, 0);
		}
	}

	// CMs' Mac address, int type = 4 bytes * 8 bit = 32 bit
	taf->addBitLength( mClusterNeighborsVec.size() * 4 * 8);
	return taf;
}

void CMac1609_4TwoWay::sendTAF(TAF* taf) {
    //we actually came to the point where we can send a packet
    channelBusySelf(true);
//    CMACWSM* pktToSend = myEDCA[activeChannel]->initiateTransmit(lastIdle);

//    lastAC = mapPriority(pktToSend->getPriority());

//    DBG_MAC << "MacEvent received. Trying to send packet with priority" << lastAC << std::endl;

    //send the packet
    CMac80211Pkt* mac = new CMac80211Pkt(taf->getName(), taf->getKind());
    mac->setDestAddr(LAddress::L2BROADCAST);
    mac->setSrcAddr(myMacAddress);
    mac->encapsulate(taf->dup());

    simtime_t sendingDuration = RADIODELAY_11P + getFrameDuration(mac->getBitLength());
    DBG_MAC << "Sending duration will be" << sendingDuration << std::endl;
//    if ( (!useSCH) || (timeLeftInSlot() > sendingDuration) ) {
    if (true) {
//        if (useSCH) DBG_MAC << " Time in this slot left: " << timeLeftInSlot() << std::endl;
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
//        myEDCA[activeChannel]->revokeTxOPs();
        delete mac;
//        channelIdle();
        //do nothing. contention will automatically start after channel switch
    }
}

void CMac1609_4TwoWay::sendUnicastPacket(cPacket* msg, int destAddr){
    //we actually came to the point where we can send a packet
    channelBusySelf(true);
//    CMACWSM* pktToSend = myEDCA[activeChannel]->initiateTransmit(lastIdle);

//    lastAC = mapPriority(pktToSend->getPriority());
//    DBG_MAC << "MacEvent received. Trying to send packet with priority" << lastAC << std::endl;

    //send the packet
    CMac80211Pkt* mac = new CMac80211Pkt(msg->getName(), msg->getKind());
    mac->setDestAddr(destAddr);
    mac->setSrcAddr(myMacAddress);
    mac->encapsulate(msg->dup());

    simtime_t sendingDuration = RADIODELAY_11P + getFrameDuration(mac->getBitLength());
    DBG_MAC << "Sending duration will be" << sendingDuration << std::endl;
//    if ( (!useSCH) || (timeLeftInSlot() > sendingDuration) ) {
    if (true) {
//        if (useSCH) DBG_MAC << " Time in this slot left: " << timeLeftInSlot() << std::endl;
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
//        myEDCA[activeChannel]->revokeTxOPs();
        delete mac;
//        channelIdle();
        //do nothing. contention will automatically start after channel switch
    }
}

void CMac1609_4TwoWay::sendBroadcastPacket(cPacket* msg){
    //we actually came to the point where we can send a packet
    channelBusySelf(true);

//    lastAC = mapPriority(pktToSend->getPriority());
//    lastAC = 0;
//    DBG_MAC << "MacEvent received. Trying to send packet with priority" << lastAC << std::endl;

    //send the packet
    CMac80211Pkt* mac = new CMac80211Pkt(msg->getName(), msg->getKind());
    mac->setDestAddr(LAddress::L2BROADCAST);
    mac->setSrcAddr(myMacAddress);
    mac->encapsulate(msg->dup());

    simtime_t sendingDuration = RADIODELAY_11P + getFrameDuration(mac->getBitLength());
    DBG_MAC << "Sending duration will be " << sendingDuration << std::endl;
//    if ( (!useSCH) || (timeLeftInSlot() > sendingDuration) ) {
    if (true) {
//        if (useSCH) DBG_MAC << " Time in this slot left: " << timeLeftInSlot() << std::endl;
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
//        myEDCA[activeChannel]->revokeTxOPs();
        delete mac;
//        channelIdle();
        //do nothing. contention will automatically start after channel switch
    }
}


CMACWSM* CMac1609_4TwoWay::prepareUpload(){

	// get SUMO coordinates
	Veins::TraCICoord mySumoPosition = vehMobility->getManager()->omnet2traci( vehMobility->getCurrentPosition() );
	Coord myPosition_S;
	myPosition_S.x = mySumoPosition.x;
	myPosition_S.y = mySumoPosition.y;

	CMACWSM* cmacwsm = getPktFromQ();

	if (cmacwsm != nullptr){
		cmacwsm->setNodeID(myNodeID.c_str());
		cmacwsm->setSpeed(vehMobility->getSpeed());
		cmacwsm->setPosition(myPosition_S);

		if ( gMeasureMACDelay )
			cmacwsm->setTimestamp(simTime());

		if (debugMAC){
			cout << "MAC: Upload Pkt" << endl;
			cout << myNodeID << ": get a message from Q" << endl;
			cout << "DestID: "<< cmacwsm->getDestID()
					<< ", Serial: "<< cmacwsm->getSerial()<< endl;
		}
	}
	else {
		CMACWSM* cmacwsm_empty = new CMACWSM("data");
		cmacwsm_empty->setDestID("");
		cmacwsm_empty->setTTL(120);
		cmacwsm_empty->setTimestamp(simTime());
		cmacwsm_empty->setNodeID(myNodeID.c_str());
		cmacwsm_empty->setSpeed(vehMobility->getSpeed());
		cmacwsm_empty->setPosition(myPosition_S);
		cmacwsm_empty->addBitLength(400);
		return cmacwsm_empty;
	}

	cmacwsm->addBitLength(400);

	return cmacwsm;
}

ACK* CMac1609_4TwoWay::prepareACK(){
	ACK* ack = new ACK("ACK");

	ack->setNodeID( findHost()->getFullName() );

	ack->addBitLength(128);
	return ack;
}

CHtoCH* CMac1609_4TwoWay::prepareCHtoCH(){
	CHtoCH* CH2CH = new CHtoCH("CH2CH");

	CH2CH->setNodeID( myNodeID.c_str() );

	// +1 means its own info
	CH2CH->setCMListArraySize(myCMInfoMap.size() + 1);

	int index = 0;
	std::map<std::string, CMSafetyInfo>::iterator it;
	for (it = myCMInfoMap.begin(); it != myCMInfoMap.end(); ++it){
		CH2CH->setCMList(index, it->second);
		++index;
	}

	/*include itself's info*/
	// get SUMO coordinates
	Veins::TraCICoord mySumoPosition = vehMobility->getManager()->omnet2traci( vehMobility->getCurrentPosition() );
	Coord myPosition_S;
	myPosition_S.x = mySumoPosition.x;
	myPosition_S.y = mySumoPosition.y;

	CMSafetyInfo myInfo;
	myInfo.currentCoord = myPosition_S;
//	myInfo.nodeID = myNodeID;
	myInfo.speed = vehMobility->getSpeed();

	// get pkt from upper layer
	CMACWSM* cmacwsm = getPktFromQForCH2CH();

	if (cmacwsm != nullptr){
//		cout << cmacwsm->getDestID() << endl;
		myInfo.nodeID = cmacwsm->getNodeID();
		myInfo.destID = cmacwsm->getDestID();
		myInfo.ttl = cmacwsm->getTTL();
		myInfo.timeStamp = cmacwsm->getTimestamp();
		myInfo.serial = cmacwsm->getSerial();

		if ( gMeasureMACDelay )
			myInfo.timeStamp = simTime();

		if (debugMAC){
			cout << "MAC: CH2CH pkt" << endl;
			cout << myNodeID << ": get a message from Q" << endl;
			cout << "DestID: "<< cmacwsm->getDestID()
					<< ", Serial: "<< cmacwsm->getSerial()<< endl;
		}
	}
	else {
		myInfo.destID = "";
		myInfo.timeStamp = simTime();
		myInfo.ttl = 120;
		myInfo.serial = -1;
	}

	// the last index is the size of the map
	CH2CH->setCMList(myCMInfoMap.size(), myInfo);


	/*other CMs information*/
	CH2CH->setOrCMListArraySize(orCMInfoMap.size());
	index = 0;
	for (it = orCMInfoMap.begin(); it != orCMInfoMap.end(); ++it){
		CH2CH->setOrCMList(index, it->second);
		++index;
	}

	/*
	 * TODO:
	 * packet size*/
	// 50 bytes: 400 bits * 10 = 4000 bit (500 bytes)
	CH2CH->addBitLength(4000);

	delete cmacwsm;
	return CH2CH;
}

CHDistr* CMac1609_4TwoWay::prepareCHDistr(){
	CHDistr* CHDistPkt = new CHDistr("CHDistr");

	CHDistPkt->setNodeID( myNodeID.c_str() );

	// CM pkts from other clusters
	CHDistPkt->setOrCMListArraySize(orCMInfoMap.size() + 1);

	int index1 = 0;
	std::map<std::string, CMSafetyInfo>::iterator it1;
	for (it1 = orCMInfoMap.begin(); it1 != orCMInfoMap.end(); ++it1){
		CMPktInfo tmp;

		tmp.currentCoord = it1->second.currentCoord;
		tmp.destID = it1->second.destID;
		tmp.nodeID = it1->second.nodeID;
		tmp.speed = it1->second.speed;
		tmp.timeStamp = it1->second.timeStamp;
		tmp.ttl = it1->second.ttl;
		tmp.serial = it1->second.serial;

		CHDistPkt->setOrCMList(index1, tmp);
		++index1;
	}

	// CM pkts from my cluster
	CHDistPkt->setMyCMListArraySize(myCMInfoMap.size());

	int index2 = 0;
	std::map<std::string, CMSafetyInfo>::iterator it2;
	for (it2 = myCMInfoMap.begin(); it2 != myCMInfoMap.end(); ++it2){
		CMPktInfo tmp;

		tmp.currentCoord = it2->second.currentCoord;
		tmp.destID = it2->second.destID;
		tmp.nodeID = it2->second.nodeID;
		tmp.speed = it2->second.speed;
		tmp.timeStamp = it2->second.timeStamp;
		tmp.ttl = it2->second.ttl;
		tmp.serial = it2->second.serial;

		CHDistPkt->setMyCMList(index2, tmp);
		++index2;
	}


	/*need to include itself's info*/

	// get SUMO coordinates
	Veins::TraCICoord mySumoPosition = vehMobility->getManager()->omnet2traci( vehMobility->getCurrentPosition() );
	Coord myPosition_S;
	myPosition_S.x = mySumoPosition.x;
	myPosition_S.y = mySumoPosition.y;

	CMPktInfo myInfo;
	myInfo.currentCoord = myPosition_S;
//	myInfo.nodeID = myNodeID;
	myInfo.speed = vehMobility->getSpeed();


	// get pkt from upper layer
	CMACWSM* cmacwsm = getPktFromQForCHDistr();

	if (cmacwsm != nullptr){
//		cout << cmacwsm->getDestID() << endl;
/*
		for ( auto i: mClusterNeighborsVec ){
			if ( strcmp(cmacwsm->getDestID(), i->getFullName()) == 0 ){

			}

		for ( auto it = mClusterNeighborsVec.begin();
				it != mClusterNeighborsVec.end(); ++it)
				mClusterNeighborsVec;
*/
		myInfo.nodeID = cmacwsm->getNodeID();
		myInfo.destID = cmacwsm->getDestID();
		myInfo.ttl = cmacwsm->getTTL();
		myInfo.timeStamp = cmacwsm->getTimestamp();
		myInfo.serial = cmacwsm->getSerial();

		if ( gMeasureMACDelay )
			myInfo.timeStamp = simTime();

	}
	else {
		myInfo.destID = "";
		myInfo.timeStamp = simTime();
		myInfo.ttl = 120;
		myInfo.serial = -1;
	}

	// the last index is the size of the map
	CHDistPkt->setOrCMList(orCMInfoMap.size(), myInfo);

	/*
	 * TODO:
	 * packet size*/
	// 50 bytes: 400 bits * 20 = 8000 bit (1000 bytes)
	CHDistPkt->addBitLength(8000);

	delete cmacwsm;
	return CHDistPkt;
}

CMACWSM* CMac1609_4TwoWay::getPktFromQ(){
	CMACWSM* tmpwsm = nullptr;

	if (myUpperLayerEmgPktQueue.empty()){

        if ( myUpperLayerPktQueue.empty() )
            return tmpwsm = nullptr;

        while ( !myUpperLayerPktQueue.empty() ){
            tmpwsm = (CMACWSM*)myUpperLayerPktQueue.pop();
            if ( simTime() < (tmpwsm->getTimestamp() + tmpwsm->getTTL()) ){
                return tmpwsm;
            }
            else{
                gPeQueueExpiredPkts++;
                delete tmpwsm;
                DBG_MAC << myNodeID << ": a pkt expired and discarded in MAC layer!" << endl;
            }
        }
	}
	else {

        while ( !myUpperLayerEmgPktQueue.empty() ){
            tmpwsm = (CMACWSM*)myUpperLayerEmgPktQueue.pop();
            if ( simTime() < (tmpwsm->getTimestamp() + tmpwsm->getTTL()) ){
                return tmpwsm;
            }
            else{
                gPeQueueExpiredPkts++;
                delete tmpwsm;
                DBG_MAC << myNodeID << ": a pkt expired and discarded in MAC layer!" << endl;
            }
        }
	}
	return tmpwsm;

}

CMACWSM* CMac1609_4TwoWay::getPktFromQForCH2CH(){
	CMACWSM* tmpwsm = nullptr;

	if (myUpperLayerEmgPktQueue.empty()){
	    if ( myUpperLayerPktQueue.empty() )
	        return tmpwsm = nullptr;

	    bool forMyCM = false;
	    tmpwsm = (CMACWSM*)myUpperLayerPktQueue.front();

	//  cout << myNodeID << ": " << endl;
	//  for ( auto test: mClusterNeighborsVec)
	//      cout << test->getFullName() << " ";
	//  cout << endl;

	    for ( auto i: mClusterNeighborsVec ){
	        if ( i != nullptr ){
	            if ( strcmp(tmpwsm->getDestID(), i->getFullName()) == 0 ){
	                forMyCM = true;
	                break;
	            }
	        }
	    }

	    if ( forMyCM ){
	        return tmpwsm = nullptr;
	    }
	    else {
	        myUpperLayerPktQueue.pop();
	        return tmpwsm;
	    }
	}
	else {

        bool forMyCM = false;
        tmpwsm = (CMACWSM*)myUpperLayerEmgPktQueue.front();

    //  cout << myNodeID << ": " << endl;
    //  for ( auto test: mClusterNeighborsVec)
    //      cout << test->getFullName() << " ";
    //  cout << endl;

        for ( auto i: mClusterNeighborsVec ){
            if ( i != nullptr ){
                if ( strcmp(tmpwsm->getDestID(), i->getFullName()) == 0 ){
                    forMyCM = true;
                    break;
                }
            }
        }

        if ( forMyCM ){
            return tmpwsm = nullptr;
        }
        else {
            myUpperLayerEmgPktQueue.pop();
            return tmpwsm;
        }

	}

}


CMACWSM* CMac1609_4TwoWay::getPktFromQForCHDistr(){
	CMACWSM* tmpwsm;

    if (myUpperLayerEmgPktQueue.empty()){
        if ( myUpperLayerPktQueue.empty() )
            return tmpwsm = nullptr;

        tmpwsm = (CMACWSM*)myUpperLayerPktQueue.front();
        for ( auto i: mClusterNeighborsVec ){
            if ( i != nullptr ){
                if ( strcmp(tmpwsm->getDestID(), i->getFullName()) == 0 ){
                    myUpperLayerPktQueue.pop();
                    if ( simTime() < (tmpwsm->getTimestamp() + tmpwsm->getTTL()) ){
                        return tmpwsm;
                    }
                    else{
                        gPeQueueExpiredPkts++;
                        DBG_MAC << myNodeID << ": a pkt expired and discarded in MAC layer!" << endl;
                    }
                }
            }
        }

        return tmpwsm = nullptr;
    }
    else {
        tmpwsm = (CMACWSM*)myUpperLayerEmgPktQueue.front();
        for ( auto i: mClusterNeighborsVec ){
            if ( i != nullptr ){
                if ( strcmp(tmpwsm->getDestID(), i->getFullName()) == 0 ){
                    myUpperLayerEmgPktQueue.pop();
                    if ( simTime() < (tmpwsm->getTimestamp() + tmpwsm->getTTL()) ){
                        return tmpwsm;
                    }
                    else{
                        gPeQueueExpiredPkts++;
                        delete tmpwsm;
                        DBG_MAC << myNodeID << ": a pkt expired and discarded in MAC layer!" << endl;
                    }
                }
            }
        }

        return tmpwsm = nullptr;
    }
}

void CMac1609_4TwoWay::clusterChannelAssign(std::vector<cModule*> nodeSet,
														int numNodes,
														int numAvailChannels,
														int numClusters){
    // real number of cluster
//	int numClusters = numCluster;
	// cGraph: intercluster interference
	// tGraph: same
    int cGraph[numClusters][numClusters];
    int tGraph[numClusters][numClusters];
    cout << "++++++++++numCluster++++++ " << numClusters << endl;
    cout << "++++++++++numNodes++++++ " << numNodes << endl;
    for (int i=0; i < numClusters ; i++){
       for (int j=0; j < numClusters ; j++){
           cGraph[i][j]=0;
           tGraph[i][j]=0;
       }
    }
    cout << "++++++++++cGraph++++++ " << endl;
    // calculate interference

    for (int i=0; i < numNodes ; i++){
    	cModule* module_i = nodeSet[i]->getModuleByPath(".nic.mac1609_4");
    	CMac1609_4TwoWay* mac_i = check_and_cast<CMac1609_4TwoWay*>(module_i);
        for (int j=0; j < numNodes ; j++){
        	cModule* module_j = nodeSet[j]->getModuleByPath(".nic.mac1609_4");
        	CMac1609_4TwoWay* mac_j = check_and_cast<CMac1609_4TwoWay*>(module_j);
        	// if it is interference
//            if (nodeSet[i].neighTable[j]==1){
        	if ( std::find(mac_i->mNeighborVec.begin(), mac_i->mNeighborVec.end(), nodeSet[j]) != mac_i->mNeighborVec.end() ){
            	// if it is the same cluster
//                if (nodeSet[i].getClusterId() != nodeSet[j].getClusterId()){
        		if (mac_i->clusterIndex != mac_j->clusterIndex){
                	// if it has parent
//                    if (nodeSet[i].getParentId() > -1 && nodeSet[j].getParentId() > -1){
        			if (mac_i->mCHMacAdd != 0 && mac_j->mCHMacAdd !=0 ){
//                        cGraph[nodeSet[i].getClusterId()][nodeSet[j].getClusterId()]++;
                    	cGraph[mac_i->clusterIndex][mac_j->clusterIndex]++;
                        tGraph[mac_i->clusterIndex][mac_j->clusterIndex]++;
                    }else{
                        //This space is for handling a node without a reach of any parent node
                        //Left empty until we find a solution for them.
                        //cout << "i:" << i << " ,j:" << j << "::" << nodeSet[i].getParentId() << "," << nodeSet[j].getParentId() << endl;
                    }
                }
            }
        }
    }

    int sum_i[numClusters];
    // aver interference, high average has real interference
    double avr_interfere[numClusters];
    // number of interference.
    int count_interfere[numClusters];

    int remain_cluster = numClusters;
    int ClusterChannel[numClusters];
    int remain_channel = numAvailChannels;

    vector<int> remain_channel_vec;

    for ( int j=0; j< numClusters; ++j){
		for (int i=1; i<=numAvailChannels; ++i){
			if ( i % 2 != 0)
				remain_channel_vec.push_back(i);
		}
    }


    queue<int> remain_channel_q;

    for (int k=0; k < numClusters ; k++){
        ClusterChannel[k]=0;
    }
    while (remain_cluster > 0){

//        for (int i=0; i < numClusters ; i++){
//           for (int j=0; j < numClusters ; j++){
//               cout << setw(4) << cGraph[i][j];
//           }
//           cout << endl;
//        }

        //0. calculate average inter-cluster interferences.
        for (int i=0; i < numClusters ; i++){
            //cout <<  i << "::";
            sum_i[i] = 0;
            avr_interfere[i] = 0;
            count_interfere[i] = 0;
            for (int j=0; j < numClusters ; j++){
                //cout  << cGraph[i][j] << ' ';
                sum_i[i] = sum_i[i] + cGraph[i][j];
                if (cGraph[i][j] > 0){
                    count_interfere[i]++;
                }
            }
            if (sum_i[i] > 0){
                avr_interfere[i] = (double) (sum_i[i]/count_interfere[i]);
            }
            cout << avr_interfere[i] << " = "
            		<< sum_i[i] << "/" << count_interfere[i] << endl;
        }
        //1. find max average interference
        int max_cl=0, max_avr = 0;
        for (int k=0; k < numClusters ; k++){
            if (avr_interfere[k] > max_avr){
                max_cl = k;
                max_avr = avr_interfere[k];
            }
        }
        if (max_avr==0){//this mean all conflicting clusters are resolved
            break;
        }
        //2. find max edge in max_cl from cGraph by
        int max_edge=0, max_interfere = 0;
        for (int l=0; l < numClusters ; l++){
            if (cGraph[max_cl][l] > max_interfere){
                max_edge = l;
                max_interfere = cGraph[max_cl][l];
            }
        }
        cout << remain_cluster << "-max_cl: " << max_cl << ",max_edge: " << max_edge << endl;
        cout << "max_cl_avr: " << max_avr << ",max_edge_weight: " << max_interfere << endl;
        //3. assigning different channels to the nodes on that edge.
        // and remove it from array
        if (remain_channel <= 0){//if all available channels are used then return back to first used channel
            remain_channel = numAvailChannels;
        }

        // Chris
//        if ( remain_channel_vec.size() == 0){
//            for (int i=1; i<=numAvailChannels; ++i){
//            	remain_channel_vec.push_back(i);
//            }
//        }
//        remain_channel_q.
        if ( remain_channel_q.empty() ){
//        	for ( int i=1; i<=numAvailChannels; ++i){
//        		remain_channel_q.push(i);
//        	}
        }
        // end

        cout << remain_cluster << endl;
        if (ClusterChannel[max_cl] == 0){
        	cout << " 1 ClusterChannel[max_cl]: " << ClusterChannel[max_cl] << endl;
//            ClusterChannel[max_cl] = remain_channel;
        	searchChannel(	numClusters,
        					numAvailChannels,
        					max_cl,
        					remain_channel_q,
        					ClusterChannel);

            cout << " 2 ClusterChannel[max_cl]: " << ClusterChannel[max_cl] << endl;
            remain_cluster--;
            remain_channel--;
            //update other interfering clusters who has different channel to this
            for (int i=0; i < numClusters; i++){
//                if (ClusterChannel[i] != 0){
                    cGraph[i][max_cl] = 0;
                    cGraph[max_cl][i] = 0;
//                }
            }
        }
        if (ClusterChannel[max_edge] == 0){
        	cout << " 1 ClusterChannel[max_edge]: " << ClusterChannel[max_edge] << endl;
        	searchChannel(	numClusters,
        					numAvailChannels,
        					max_edge,
        					remain_channel_q,
        					ClusterChannel);
        	cout << " 2 ClusterChannel[max_edge]: " << ClusterChannel[max_edge] << endl;
            remain_cluster--;
            remain_channel--;
//            cGraph[max_cl][max_edge] = 0;
//            cGraph[max_edge][max_cl] = 0;
            for (int i=0; i < numClusters; i++){
                cGraph[i][max_edge] = 0;
                cGraph[max_edge][i] = 0;
            }
        }else{
            cout << "ERROR" << endl;
        }

        cout << ClusterChannel[max_cl] << " : " << ClusterChannel[max_edge] << endl;
//        for (int i=0; i < numClusters; i++){
//            cout << i << "::" <<  avr_interfere[i] << endl;
//        }
//        cout << "remain_cluster" << remain_cluster << endl;
    }

//    ClusterChannel[0] = 1;
//    ClusterChannel[1] = 3;
//    ClusterChannel[2] = 5;
//    ClusterChannel[3] = 2;
//    ClusterChannel[4] = 4;
//    ClusterChannel[5] = 1;
//    ClusterChannel[6] = 3;
//    ClusterChannel[7] = 5;
//    ClusterChannel[8] = 2;
//    ClusterChannel[9] = 4;

	int cID = -1;
	for (int i=0; i<numClusters; ++i){
		if ( cID < 5 ){
			cID += 2;
			ClusterChannel[i] = cID;
		}
		else {
			cID = 1;
			ClusterChannel[i] = cID;
		}
	}

    cout << "channels:";
    for (int i=0; i < numClusters; i++){
        cout <<  ClusterChannel[i] << " ";
    }
    cout << endl;

    // assign channel to each node
    for (int i=0; i < numNodes ; i++){
    	cModule* module = nodeSet[i]->getModuleByPath(".nic.mac1609_4");
    	CMac1609_4TwoWay* mac = check_and_cast<CMac1609_4TwoWay*>(module);
//        nodeSet[i].setChannel(ClusterChannel[nodeSet[i].getClusterId()]);
    	mac->mChannelID = ClusterChannel[mac->clusterIndex];
    	mac->mClusterChannelID = ClusterChannel[mac->clusterIndex];
    }
    int avr_itf[numClusters];
    int itr_itf = 0;
    cout << "avr_itf_tgraph:";
    // check again the left interference, for future time slot assignment
    for (int j=0; j < numClusters; j++){
        avr_itf[j] = 0;
        itr_itf = 0;
        for (int k=0; k < numClusters; k++){
            if (ClusterChannel[j] == ClusterChannel[k]){
                avr_itf[j] = avr_itf[j] + tGraph[j][k];
                itr_itf++;
            }
        }
        cout << (int)avr_itf[j]/itr_itf << " ";
    }
    cout << endl;
}

void CMac1609_4TwoWay::clusterChannelAssign(std::vector<cModule*> nodeSet,
														int numAvailChannels,
														int numClusters){
    unsigned numNodes = nodeSet.size();

    int ClusterChannel[numClusters];

//	int cID = -1;
//	for (int i=0; i<numClusters; ++i){
//		if (cID < 0){
//            cID += 3;
//            ClusterChannel[i] = cID;
//		}
//		else if ( cID < 5 ){
//			cID += 1;
//			ClusterChannel[i] = cID;
//		}
//		else {
//			cID = 2;
//			ClusterChannel[i] = cID;
//		}
//	}

//    int chSize = 4;
    int cIndex[] = {2,4,3,5};
    int j = -1;
    for (int i=0; i<numClusters; ++i){
        ++j;
        ClusterChannel[i] = cIndex[j];
        if (j == 3)
            j=-1;
    }

	if ( debugMAC ){
        cout << "channels:";
        for (int i=0; i < numClusters; i++){
            cout <<  ClusterChannel[i] << " ";
        }
        cout << endl;
	}
	// assign channel to each node
    for (unsigned i=0; i < numNodes ; i++){
    	cModule* module = nodeSet[i]->getModuleByPath(".nic.mac1609_4");
    	CMac1609_4TwoWay* mac = check_and_cast<CMac1609_4TwoWay*>(module);
//        nodeSet[i].setChannel(ClusterChannel[nodeSet[i].getClusterId()]);
    	mac->mChannelID = ClusterChannel[mac->clusterIndex];
    	mac->mClusterChannelID = ClusterChannel[mac->clusterIndex];
    }
}


vector< vector<cModule*> >
CMac1609_4TwoWay::timeSlotAllocation(std::vector<cModule*> nodeSet,
                                                 int clusterNum,
                                                 int optimalClusterSize){
	// copy the nodeSet to nodesSet, because we need to reduce the elements of nodesSet
    std::vector<cModule*> nodesSet = nodeSet;
    // the output, time slot assignment
    vector< vector<cModule*> > timeSlotAssign(clusterNum, vector<cModule*>( nodesSet.size() ));
//    vector< vector<cModule*> > timeSlotAssign( clusterNum );
    // count of time slot
    int currentTimeSlot = 0;

    while ( nodesSet.size() != 0){

/*debugging*/
//		cout << "~~~~~~~~~~~~~~~~~~~~~~"<< endl;
//		cout << nodesSet.size() << endl;
//		cout << currentTimeSlot << endl;
/*debugging*/

        vector< vector<int> > deg(nodesSet.size(), vector<int>(clusterNum));
        /*for each node compute degree to each cluster */
        for (unsigned int i = 0; i < nodesSet.size(); i++){
            cModule* module_i = nodesSet[i]->getModuleByPath(".nic.mac1609_4");
            CMac1609_4TwoWay* mac_i = check_and_cast<CMac1609_4TwoWay*>(module_i);
            for (int j = 0; j < clusterNum; j++ ){
                for (unsigned int k = 0; k < mac_i->mNeighborVec.size(); k++){
                    cModule* module_k = mac_i->mNeighborVec[k]->getModuleByPath(".nic.mac1609_4");
                    CMac1609_4TwoWay* mac_k = check_and_cast<CMac1609_4TwoWay*>(module_k);
                    if ( mac_i->mCHMacAdd != 0 ){
                        if ( mac_k->clusterIndex == j){
                            deg[i][j]++;
                        }
                    }
                }
            }
        }

/*debugging*/
//		cout << "Debugging:" << endl;
//		cout << "deg:" << endl;
//		for ( unsigned int t1 = 0; t1 < deg.size(); t1++ ){
//			for ( unsigned int t2 = 0; t2 < deg[t1].size(); t2++ ){
//				cout << deg[t1][t2] << " ";
//			}
//			cout << endl;
//		}
//		cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
/*debugging*/

        /*remove CH and MCH from time slot assignment*/
        for (vector<cModule*>::iterator it = nodesSet.begin(); it != nodesSet.end(); it++){
            cModule* module_i = (*it)->getModuleByPath(".nic.mac1609_4");
            CMac1609_4TwoWay* mac_i = check_and_cast<CMac1609_4TwoWay*>(module_i);
            if (mac_i->mRoleInCluster == CH || mac_i->mRoleInCluster == MCH){
    //				cout << "erase " << (*it)->getFullName() << endl;
                nodesSet.erase( it );
                it--;
            }
        }

/*debugging*/
//		for (vector<cModule*>::iterator it = nodesSet.begin(); it != nodesSet.end(); it++){
//			cout << (*it)->getFullName() << endl;
//		}
/*debugging*/

        /*save maximum degree and cluster index for each node */
        /*maxDegree_Cluster: id (module*), max degree, cluster ID*/
        vector< std::tuple<cModule*, int, int> > maxDegree_Cluster_t;
        maxDegree_Cluster_t.clear();

        for ( unsigned int i = 0; i < deg.size(); i++ ){
            vector<int>::iterator iteMax = std::max_element( deg[i].begin(), deg[i].end() );
            cModule* module = nodesSet[i]->getModuleByPath(".nic.mac1609_4");
            CMac1609_4TwoWay* mac = check_and_cast<CMac1609_4TwoWay*>(module);
            std::tuple<cModule*, int, int> entry (nodesSet[i], *iteMax, mac->clusterIndex);

            maxDegree_Cluster_t.push_back(entry);
        }

/*debugging*/
//		for ( unsigned int t1 = 0; t1 < maxDegree_Cluster_t.size(); t1++ ){
//			cout << std::get<0>(maxDegree_Cluster_t[t1])->getFullName() << " ";
//			cout << std::get<1>(maxDegree_Cluster_t[t1]) << " ";
//			cout << std::get<2>(maxDegree_Cluster_t[t1]) << endl;
//		}
/*debugging*/

        /*sort based on degree*/
        sort(maxDegree_Cluster_t.begin(), maxDegree_Cluster_t.end(),
            [](const tuple<cModule*, int, int>& i, const tuple<cModule*, int, int>& j) -> bool{return std::get<1>(i) > std::get<1>(j); } );

/*debugging*/
//		cout << "Sort --------------------------------------" << endl;
//		for ( unsigned int t1 = 0; t1 < maxDegree_Cluster_t.size(); t1++ ){
//			cout << std::get<0>(maxDegree_Cluster_t[t1])->getFullName() << " ";
//			cout << std::get<1>(maxDegree_Cluster_t[t1]) << " ";
//			cout << std::get<2>(maxDegree_Cluster_t[t1]) << endl;
//		}
/*debugging*/

        /*assign time slot*/
        vector< int > assignedClusterID;
        vector< cModule* > assignedNode;
        for( unsigned int i = 0; i < maxDegree_Cluster_t.size(); i++ ){
            if ( i == 0){
    //				if ( )
                // assign 1st entry of maxDegree_Cluster time slot. timeSlotAssign[cluster Index][time slot]
                timeSlotAssign[std::get<2>(maxDegree_Cluster_t[i])][currentTimeSlot] = std::get<0>(maxDegree_Cluster_t[i]);
    //				timeSlotAssign[std::get<2>(maxDegree_Cluster_t[i])].push_back( std::get<0>(maxDegree_Cluster_t[i]) );
                assignedClusterID.push_back( std::get<2>(maxDegree_Cluster_t[i]) );
                assignedNode.push_back(std::get<0>(maxDegree_Cluster_t[i]));
                cModule* assignedNodeModule = std::get<0>(maxDegree_Cluster_t[i]);
                vector<cModule*>::iterator erasePos = std::find(nodesSet.begin(), nodesSet.end(), assignedNodeModule );
                if ( erasePos != nodesSet.end()){
                    nodesSet.erase( erasePos );
                } else {
                    cout << "erase process! could not find the cModule, need to check!" << endl;
                }
            } else{
                // 1st condition: if the new node is in the same cluster of any allocated node.
                if ( std::find(	assignedClusterID.begin(), assignedClusterID.end(), std::get<2>(maxDegree_Cluster_t[i]) ) == assignedClusterID.end() ) {
                    bool flag = false;
                    for ( unsigned int j = 0; j < assignedNode.size(); j++){
                        cModule* module_j = assignedNode[j]->getModuleByPath(".nic.mac1609_4");
                        CMac1609_4TwoWay* mac_j = check_and_cast<CMac1609_4TwoWay*>(module_j);
                        // 2nd condition: if the new node has connections with any allocated nodes
                        if ( std::find( mac_j->mNeighborVec.begin(), mac_j->mNeighborVec.end(), std::get<0>(maxDegree_Cluster_t[i]) ) != mac_j->mNeighborVec.end() ){
                            flag = true;
                            break;
                        }
                    }
                    if (flag == false){
                        timeSlotAssign[std::get<2>(maxDegree_Cluster_t[i])][currentTimeSlot] = std::get<0>(maxDegree_Cluster_t[i]);
    //						timeSlotAssign[std::get<2>(maxDegree_Cluster_t[i])].push_back( std::get<0>(maxDegree_Cluster_t[i]) );
                        assignedClusterID.push_back( std::get<2>(maxDegree_Cluster_t[i]));
                        assignedNode.push_back(std::get<0>(maxDegree_Cluster_t[i]));

                        cModule* assignedNodeModule = std::get<0>(maxDegree_Cluster_t[i]);
                        vector<cModule*>::iterator erasePos = std::find(nodesSet.begin(), nodesSet.end(), assignedNodeModule );
                        if ( erasePos != nodesSet.end()){
                            nodesSet.erase( erasePos );
                        } else {
                            cout << "erase process! could not find the cModule, need to check!" << endl;
                        }
                    }
                }
            }
        }
        currentTimeSlot++;

//		cout << "remaining node Size: " << nodesSet.size() << endl;
//		cout << "currentTimeSlot: "<< currentTimeSlot << endl;
/*debugging*/
//		cout << "Inside: time slot assignment: " << endl;
//		for ( unsigned int t1 = 0; t1 < timeSlotAssign.size(); t1++ ){
//			for (unsigned int t2 = 0; t2 < timeSlotAssign[t1].size(); t2++ ){
//				if ( timeSlotAssign[t1][t2] != nullptr){
//					cout << timeSlotAssign[t1][t2]->getFullName() << " ";
//				} else {
//					cout << "         ";
//				}
//			}
//			cout << endl;
//		}
/*debugging*/
    }

    /*resize the vector to reduce the waste space*/
    for ( unsigned int i = 0; i < timeSlotAssign.size(); i++ ){
    	timeSlotAssign[i].resize( currentTimeSlot + 1 );
    }

/*debugging*/

    if ( debugMAC ){
        cout << "MaxTimeSlotSize: " << currentTimeSlot << endl;
        cout << "time slot assignment: " << endl;
        for ( unsigned int t1 = 0; t1 < timeSlotAssign.size(); t1++ ){
            for (unsigned int t2 = 0; t2 < timeSlotAssign[t1].size(); t2++ ){
                if ( timeSlotAssign[t1][t2] != nullptr){
                    cout << setw(10) << timeSlotAssign[t1][t2]->getFullName();
                } else{
                    cout << "          ";
                }
            }
            cout << endl;
        }
    }
/*debugging*/

    myMaxTimeSlotSize = currentTimeSlot;
    return timeSlotAssign;
}

vector< vector<cModule*> >
CMac1609_4TwoWay::timeSlotSequence(std::vector<cModule*> nodeSet,
                                                 int clusterNum,
                                                 int optimalClusterSize){

    // copy the nodeSet to nodesSet, because we need to reduce the elements of nodesSet
    std::vector<cModule*> nodesSet = nodeSet;
    // the output, time slot assignment
    vector< vector<cModule*> > timeSlotAssign(clusterNum, vector<cModule*>(0) );

    /*remove CH and MCH from time slot assignment*/
    for (vector<cModule*>::iterator it = nodesSet.begin(); it != nodesSet.end(); it++){
        cModule* module_i = (*it)->getModuleByPath(".nic.mac1609_4");
        CMac1609_4TwoWay* mac_i = check_and_cast<CMac1609_4TwoWay*>(module_i);
        if (mac_i->mRoleInCluster == CH || mac_i->mRoleInCluster == MCH){
//              cout << "erase " << (*it)->getFullName() << endl;
            nodesSet.erase( it );
            it--;
        }
    }

    if ( debugMAC){
        cout << "nodesSet size: " << nodesSet.size() << endl;
    }

    for (vector<cModule*>::iterator it = nodesSet.begin(); it != nodesSet.end(); it++){
        cModule* module_i = (*it)->getModuleByPath(".nic.mac1609_4");
        CMac1609_4TwoWay* mac_i = check_and_cast<CMac1609_4TwoWay*>(module_i);
        if (mac_i->mRoleInCluster == CM ){
            timeSlotAssign[mac_i->clusterIndex].push_back(*it);
        }
    }

    if ( debugMAC ){
//        cout << "MaxTimeSlotSize: " << currentTimeSlot << endl;
        cout << "time slot assignment: " << endl;
        for ( unsigned int t1 = 0; t1 < timeSlotAssign.size(); t1++ ){
            for (unsigned int t2 = 0; t2 < timeSlotAssign[t1].size(); t2++ ){
                if ( timeSlotAssign[t1][t2] != nullptr){
                    cout << setw(10) << timeSlotAssign[t1][t2]->getFullName();
                } else{
                    cout << "          ";
                }
            }
            cout << endl;
        }
    }

    myMaxTimeSlotSize = optimalClusterSize - 1;
    return timeSlotAssign;
}

void CMac1609_4TwoWay::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj) {
	Enter_Method_Silent();
//	cout << "signalID: " << signalID << endl;
//	if (signalID == roundCountSignal) {
//		cout << " ###### Receive roundCountSignal" << endl;
		/*
		 * immediately restart from beginning, check event
		 * */
//		if ( mRoleInCluster == IDLE ){
//			if ( nextCheckEvent->isScheduled()){
//				if ( nextCheckEvent->getArrivalTime() != simTime() ) {
//					cancelEvent(nextCheckEvent);
//					scheduleAt( simTime(), nextCheckEvent);
//					cout << myNodeID
//						 << ": (CancelAndSched)NEW vehicle start check event." << endl;
//				}
//			}
//			else{
//				scheduleAt( simTime(), nextCheckEvent);
//				cout << myNodeID << ": NEW vehicle start check event." << endl;
//			}
//		}
//	}
	if (signalID == mobilityStateChangedSignal){
		mobilityUpdateTime = simTime();
//		cout << "mobilityStateChangedSignal" << endl;
	}
}

double CMac1609_4TwoWay::getOffsetRatio() {
	string nodeID = vehMobility->getExternalId();
	string laneID = vehMobility->getCommandInterface()->getLaneId(nodeID);
	double laneLength = vehMobility->getCommandInterface()->getLaneLength(laneID);
	double laneOffset = vehMobility->getCommandInterface()->getLanePosition(nodeID);

	return laneOffset/laneLength;
}

double CMac1609_4TwoWay::getOffsetRatio(Veins::TraCIMobility* mob) {
	string nodeID = mob->getExternalId();
	string laneID = mob->getCommandInterface()->getLaneId(nodeID);
	double laneLength = mob->getCommandInterface()->getLaneLength(laneID);
	double laneOffset = mob->getCommandInterface()->getLanePosition(nodeID);

	return laneOffset/laneLength;
}


void CMac1609_4TwoWay::searchChannel(	int numClusters,
								int numAvailChannels,
								int max_cl,
								queue<int>& remain_channel_q,
								int ClusterChannel[]){
    // check the front and after cluster channel
    bool flag = true;
    unsigned count = 0;
    while (flag){
    	count++;
//    	if ( count > 5){
//    		while ( !remain_channel_q.empty() )
//    			remain_channel_q.pop();
//
//        	for ( int i=1; i<=numAvailChannels; ++i){
//        		remain_channel_q.push(i);
//        	}
//        	count = 0;
//    	}
    	ClusterChannel[max_cl] = remain_channel_q.front();
    	remain_channel_q.pop();
    	flag = false;
    	// not the head or tail cluster index
		if ( max_cl!=0 && max_cl!=(numClusters-1)){
			if ( ClusterChannel[max_cl]!= 1 && ClusterChannel[max_cl]!=numAvailChannels){
				// check the cluster before max_cl cluster
				if ( ClusterChannel[max_cl-1] == (ClusterChannel[max_cl]-1)
						|| ClusterChannel[max_cl-1] == (ClusterChannel[max_cl]+1)
						|| ClusterChannel[max_cl-1] == ClusterChannel[max_cl] ){
					flag = true;
					remain_channel_q.push(ClusterChannel[max_cl]);
					continue;
				}
				// cluster after max_cl cluster
				else if ( ClusterChannel[max_cl+1] == (ClusterChannel[max_cl]-1)
							|| ClusterChannel[max_cl+1] == (ClusterChannel[max_cl]+1)
							|| ClusterChannel[max_cl+1] == ClusterChannel[max_cl]){
					flag = true;
					remain_channel_q.push(ClusterChannel[max_cl]);
					continue;
				}
			}
			else if (ClusterChannel[max_cl] == 1){
				// check the cluster before max_cl cluster
				if ( ClusterChannel[max_cl-1] == (ClusterChannel[max_cl]+1)
						|| ClusterChannel[max_cl-1] == ClusterChannel[max_cl]){
					flag = true;
					remain_channel_q.push(ClusterChannel[max_cl]);
					continue;
				}
				// cluster after max_cl cluster
				else if ( ClusterChannel[max_cl+1] == (ClusterChannel[max_cl]+1)
							|| ClusterChannel[max_cl+1] == ClusterChannel[max_cl]){
					flag = true;
					remain_channel_q.push(ClusterChannel[max_cl]);
					continue;
				}
			}
			else if (ClusterChannel[max_cl] == numAvailChannels){
				// check the cluster before max_cl cluster
				if ( ClusterChannel[max_cl-1] == (ClusterChannel[max_cl]-1)
						|| ClusterChannel[max_cl-1] == ClusterChannel[max_cl]){
					flag = true;
					remain_channel_q.push(ClusterChannel[max_cl]);
					continue;
				}
				// cluster after max_cl cluster
				else if ( ClusterChannel[max_cl+1] == (ClusterChannel[max_cl]-1)
							|| ClusterChannel[max_cl+1] == ClusterChannel[max_cl]){
					flag = true;
					remain_channel_q.push(ClusterChannel[max_cl]);
					continue;
				}
			}
		}
		// tail cluster
		else if ( max_cl==(numClusters-1) ){
			if ( ClusterChannel[max_cl]!= 1 && ClusterChannel[max_cl]!=numAvailChannels){
				// only check the cluster before max_cl cluster
				if ( ClusterChannel[max_cl-1] == (ClusterChannel[max_cl]-1)
						|| ClusterChannel[max_cl-1] == (ClusterChannel[max_cl]+1)
						|| ClusterChannel[max_cl-1] == ClusterChannel[max_cl]){
					flag = true;
					remain_channel_q.push(ClusterChannel[max_cl]);
					continue;
				}
			}
			else if (ClusterChannel[max_cl]== 1){
				// only check the cluster before max_cl cluster
				if ( ClusterChannel[max_cl-1] == (ClusterChannel[max_cl]+1)
						|| ClusterChannel[max_cl-1] == ClusterChannel[max_cl]){
					flag = true;
					remain_channel_q.push(ClusterChannel[max_cl]);
					continue;
				}
			}
			else if (ClusterChannel[max_cl]== numAvailChannels){
				// only check the cluster before max_cl cluster
				if ( ClusterChannel[max_cl-1] == (ClusterChannel[max_cl]-1)
						|| ClusterChannel[max_cl-1] == ClusterChannel[max_cl]){
					flag = true;
					remain_channel_q.push(ClusterChannel[max_cl]);
					continue;
				}
			}
		}
		// head cluster
		else if ( max_cl==0 ){
			if ( ClusterChannel[max_cl]!= 1 && ClusterChannel[max_cl]!=numAvailChannels){
				// only cluster after max_cl cluster
				if ( ClusterChannel[max_cl+1] == (ClusterChannel[max_cl]-1)
						|| ClusterChannel[max_cl+1] == (ClusterChannel[max_cl]+1)
						|| ClusterChannel[max_cl+1] == ClusterChannel[max_cl] ){
					flag = true;
					remain_channel_q.push(ClusterChannel[max_cl]);
					continue;
				}
			}
			else if (ClusterChannel[max_cl]== 1){
				// only cluster after max_cl cluster
				if ( ClusterChannel[max_cl+1] == (ClusterChannel[max_cl]+1)
						|| ClusterChannel[max_cl+1] == ClusterChannel[max_cl]){
					flag = true;
					remain_channel_q.push(ClusterChannel[max_cl]);
					continue;
				}
			}
			else if (ClusterChannel[max_cl]== numAvailChannels){
				// only cluster after max_cl cluster
				if ( ClusterChannel[max_cl+1] == (ClusterChannel[max_cl]-1)
						|| ClusterChannel[max_cl+1] == ClusterChannel[max_cl]){
					flag = true;
					remain_channel_q.push(ClusterChannel[max_cl]);
					continue;
				}
			}
		}
    }
}

//std::vector<std::string> CMac1609_4TwoWay::updateHosts(){
//
//    std::map<std::string, cModule*> vehs_Map = vehMobility->getManager()->getManagedHosts();
//
//    std::vector<std::string> vehMinOffsetVec;
//    vehMapEdgeId.clear();
//
//    for (auto& i:vehs_Map){
//        Veins::TraCIMobility* iMob;
//        iMob = (Veins::TraCIMobility*)i.second->getSubmodule("veinsmobility");
//        std::string edgeID = iMob->getCommandInterface()->getEdgeId(iMob->getExternalId());
//        if (getOffsetRatio(iMob) < 0.98){
//            if (vehMapEdgeId.find(edgeID) != vehMapEdgeId.end()){
//                vehMapEdgeId.at(edgeID).push_back(i.second);
//            }
//            else{
//                // insert a key and a value
//                vehMapEdgeId[edgeID].push_back(i.second);
//            }
//        }
//    }
//
//    for (auto& i:vehMapEdgeId){
//        double minOffset = 0.0;
//        std::string vehMin = "";
//        for (auto& j:i.second){
//            Veins::TraCIMobility* jMob;
//            jMob = (Veins::TraCIMobility*)j->getSubmodule("veinsmobility");
//            double offset = jMob->getCommandInterface()->getLanePosition(jMob->getExternalId());
//            if ( minOffset == 0.0 ){
//                minOffset = offset;
//                vehMin = j->getFullName();
//            }
//            else if ( minOffset > offset ){
//                minOffset = offset;
//                vehMin = j->getFullName();
//            }
//        }
//
//        if (!vehMin.empty()){
//            vehMinOffsetVec.push_back(vehMin);
//        }
//
//    }
//
//    if (debugMAC){
//        for (auto& i:vehMapEdgeId){
//            cout << i.first << ": " << endl;
//            for (auto& j:i.second){
//                cout << " "<< j->getFullName();
//            }
//            cout << endl;
//        }
//    }
//
//    return vehMinOffsetVec;
//}
