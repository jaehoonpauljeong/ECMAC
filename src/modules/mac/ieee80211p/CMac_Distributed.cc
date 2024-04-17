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

#include "CMac_Distributed.h"

#define DBG_MAC EV
//#define DBG_MAC std::cerr << "[" << simTime().raw() << "] " << myId << " "
Define_Module(CMac_Distributed);

static bool flag_initiation = false;

void CMac_Distributed::initialize(int stage) {
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
//        frequency.insert(std::pair<int, double>(Channels::CRIT_SOL, 5.86e9));
        frequency.insert(std::pair<int, double>(Channels::SCH1, 5.87e9));
        frequency.insert(std::pair<int, double>(Channels::SCH2, 5.88e9));
        frequency.insert(std::pair<int, double>(Channels::CCH, 5.89e9));
        frequency.insert(std::pair<int, double>(Channels::SCH3, 5.90e9));
        frequency.insert(std::pair<int, double>(Channels::SCH4, 5.91e9));
//        frequency.insert(std::pair<int, double>(Channels::HPPS, 5.92e9));

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

        /*initialization*/
        vehMobility = nullptr;
        vehMobility = (Veins::TraCIMobility*)getOwner()->getOwner()->findObject("veinsmobility", true);

        mClusterIndex = -1;

        mClusterHead = nullptr;
        mCH = "";
        mCHMacAdd = -1;

        mNeighborVec.clear();
        timeSlotAlloVec.clear();

        mRoleInCluster = CM;
        clusterIndex = 0;

        mChannelID = -1;
        mNextChanID = -1;
        mCMMobilityMap.clear();

        /*set communication range*/
        cm = (ConnectionManager*)getOwner()->getOwner()->getOwner()->findObject("connectionManager",true);
//        cModule* rsumodule;
//        rsumodule = (cModule*)getOwner()->getOwner()->getOwner()->findObject("rsu",true);

//        rsumodule->findSubmodule("mac1609_4");

        maxTxDist = cm->calcInterfDist();

        nextInitialEvent = new cMessage("next Initialization Event");
        nextInitiationEvent = new cMessage("next Initiation Event");
        nextFormClusterEvent = new cMessage("next Form Cluster Event");
        nextDeclareCHEvent = new cMessage("next Declare Cluster Head Event");
        nextJoinEvent = new cMessage("next Join Event");

        nextClusteringEvent = new cMessage("next Cluster Event");
        nextTAFEvent = new cMessage("next TAF Event");
        nextCheckEvent = new cMessage("next Check Event");
        nextUploadEvent = new cMessage("next Upload Event");
        nextACKEvent = new cMessage("next ACK Event");
        nextChannelAssignment = new cMessage("next Channel Assignment");
        nextCMComfirm = new cMessage("next Cluster Member Confirm");

        myNodeID = findHost()->getFullName();

//        if (strcmp(findHost()->getFullName(), "node[49]") == 0 ){
            simtime_t offset = dblrand() * par("syncOffset").doubleValue();
            scheduleAt(simTime() + offset, nextInitialEvent);
//        }

//        if (strcmp(findHost()->getName(), "node") == 0  ){
////        	cout << findHost()->getFullName() << "..... nextCheckEvent!" << endl;
//        	scheduleAt( simTime() + 0.05, nextCheckEvent );
//        }

        /*set channel to CCH*/
        setActiveChannel(type_CCH);
        phy11p->changeListeningFrequency(frequency[Channels::CCH]);
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

void CMac_Distributed::handleSelfMsg(cMessage* msg) {

	if (msg == nextInitialEvent) {

    	std::map<std::string, cModule*> vehs_Map = vehMobility->getManager()->getManagedHosts();
		std::map<std::string, cModule*>::iterator itVehsMap;

		std::vector<cModule*> vehs_Vec;
		std::vector<cModule*>::iterator itVehVec;
		vehs_Vec.clear();

		for (itVehsMap = vehs_Map.begin(); itVehsMap != vehs_Map.end(); itVehsMap++){
			vehs_Vec.push_back((*itVehsMap).second);
//			(*itVehsMap).second->getDisplayString().updateWith("r=0");
		}

		// sort based on x coordinates
		std::sort( vehs_Vec.begin(), vehs_Vec.end(), cmpFunction );

		if ( flag_initiation == false /*&& vehs_Vec.size() > 200*/){
			if ( strcmp( myNodeID.c_str(), vehs_Vec[0]->getFullName() ) == 0 ){
				scheduleAt(simTime(), nextInitiationEvent);
				mClusterIndex = 0;
				flag_initiation = true;
			}
		}
	}

	/*
	 * most entering vehicle initiates the clustering
	 * */
	else if (msg == nextInitiationEvent){

		int numVeh = vehMobility->getManager()->getManagedHosts().size();
		optimalLayer = optTreeDepth( numVeh ) + 1; // depth + 1
		optimalClusterSize = optNumChildren(numVeh, optimalLayer - 1); // depth = layer-1

/*debugging*/
		cout << myNodeID << endl;
		cout << optimalLayer << ", " << optimalClusterSize <<", " << numVeh << endl;
/*debugging*/

		std::map<std::string, cModule*> vehs_Map = vehMobility->getManager()->getManagedHosts();
		std::map<std::string, cModule*>::iterator itVehsMap;

		double yCoor;

		/*find the y coordinates by searching vehicle that is in the middle lane with index 1*/
		for (itVehsMap = vehs_Map.begin(); itVehsMap != vehs_Map.end(); itVehsMap++){
			cModule* mobModule = (*itVehsMap).second->getModuleByPath(".veinsmobility");

//			cModule* rsuMacModule = ptr->getModuleByPath(".nic.mac1609_4");
//			CMac_Distributed* rsuMac = check_and_cast<CMac_Distributed*>(rsuMacModule);

			Veins::TraCIMobility* mobility = check_and_cast<Veins::TraCIMobility*>(mobModule);
			if ( mobility->getCommandInterface()->getLaneIndex(mobility->getExternalId()) == 1 ){

				yCoor = mobility->getManager()->omnet2traci(mobility->getCurrentPosition()).y;
				break;
			}
		}
/*debugging*/
		cout << "yCoor: " << yCoor << endl;
/*debugging*/

		int numCluster = numVeh / optimalClusterSize;
		int roadLength = vehMobility->getCommandInterface()->getLaneLength(vehMobility->commandGetLaneId());
		int partition = roadLength / numCluster;
		int halfPartition = partition / 2;

/*debugging*/
		cout << "roadLength = " << roadLength << " numCluster = " << numCluster << endl;
		cout << "partition: " << partition << " half: " << halfPartition << endl;
/*debugging*/

		std::list<std::string> junList = vehMobility->getCommandInterface()->getJunctionIds();

/*debugging*/
//		for (std::list<std::string>::iterator i = junList.begin(); i != junList.end();i++ ){
//			cout << (*i) << endl;
//		}
/*debugging*/

		// get the start position of this road segment based on the start junction
		std::list<std::string>::iterator it = find(junList.begin(), junList.end(), "0/1");

		cout << "reference Point: " << endl;
		Veins::TraCICoord sumoCoord;
		if (it != junList.end() ){
			sumoCoord = vehMobility->getCommandInterface()->getJunctionPosition(*it);
		}

		// reference coordinates and distance for each partition
		std::vector<struct ClusterRef> reference;
//		std::vector<GeoRef*> reference;

		int length = roadLength;
		while (length > 0){
			struct ClusterRef clusterRef;
//			GeoRef* clusterRef = new GeoRef;

			if (reference.size() == 0){
				clusterRef.center.x = sumoCoord.x + 10 + halfPartition;
				clusterRef.center.y = yCoor;
				clusterRef.distance= halfPartition;
				reference.push_back(clusterRef);
				length = length - partition;
			}
			else {
				clusterRef.center.x = reference.back().center.x + partition;
				clusterRef.center.y = yCoor;
				clusterRef.distance = halfPartition;
				reference.push_back(clusterRef);
				length = length - partition;
			}

		}

/*debugging*/
		for (unsigned int i = 0; i < reference.size(); i++){
			cout << reference[i].center.x << ", " << reference[i].center.y << ", " << reference[i].distance << endl;
		}
/*debugging*/

		/*
		 * TODO: Schedule time slot for each CH
		 * */

		/*
		 * Channel Assignment
		 * */
		// channel ID, 1 means SCH1 ..
		int chanID = 1;
		for (unsigned int i = 0; i < reference.size(); i++){

			reference[i].channel = chanID;
			chanID++;

			if ( chanID > 4 ){
				chanID = 1;
			}
		}

		/* fill initiating packet to send */
		Initiating* init = new Initiating("Initiating");

		init->setReferArraySize(reference.size());
		for ( unsigned int i = 0; i < init->getReferArraySize(); i++){
			init->setRefer(i, reference[i]);
		}

		init->setDistance(halfPartition);
		init->setSenderCoord(vehMobility->getCurrentPosition());

		// cluster size: number of CMs in each cluster
		init->setOptimalClusterSize(optimalClusterSize);

		init->setSendID(findHost()->getFullName());

		/*
		 * packet size, see the InitiatingPkt.msg
		 * ( x,y + distance + channel ID ) * ini->getReferSize + cluster size + x,y
		 * (float * 2 + int + int) * refersize + int + float * 2
		 * ( 32 * 2 + 32 + 32 ) * ReferSize + 32 + 32 * 2
		 */
		init->addBitLength( (32 * 2 + 32 * 2) * init->getReferArraySize() + 32 + 32 * 2 );

		sendPacket(init);

	}

	else if (msg == nextDeclareCHEvent){
		DeclareCH* decCH = new DeclareCH("Declare CH");

		decCH->setSenderAddress(myMacAddress);
		decCH->setClusterIndex(mClusterIndex);

		if ( clusterRef.size() != 0 ){
			decCH->setReferArraySize(clusterRef.size());
			for (unsigned int i = 0; i < decCH->getReferArraySize(); i++ ){
				decCH->setRefer( i, clusterRef[i] );
			}
		}
		else {
			opp_error("cluster Ref vector is empty!");
		}

//		char disp[100];
//      sprintf(disp, "Declare CH in Cluster %d", mClusterIndex );
//		findHost()->bubble(disp);
		decCH->setDistance(clusterRef[0].distance);

		Veins::TraCICoord mySumoPos = vehMobility->getManager()->omnet2traci( vehMobility->getCurrentPosition() );
		Coord mySumoPos_C;
		mySumoPos_C.x = mySumoPos.x;
		mySumoPos_C.y = mySumoPos.y;

		decCH->setSenderCoord(mySumoPos_C);
		decCH->setSequence(1);
		decCH->setSendID(findHost()->getFullName());
		/*
		 * packet size, see the DeclareCHPkt.msg, which is extended from initiatingPkt.msg
		 * ( x,y + distance + channel ID ) * ini->getReferSize + cluster size + x,y + senderAddress + clusterIndex
		 * ( float * 2 + int + int ) * refersize + int + float * 2 + int + int
		 * ( 32 * 2 + 32 + 32 ) * ReferSize + 32 + 32 * 2 + 32 + 32
		 */
		decCH->addBitLength( (32 * 2 + 32 * 2) * decCH->getReferArraySize() + 32 + 32 * 2 + 32 * 2 );

		// sending CH declaring
		simtime_t sendingDuration = sendPacket(decCH);

		// set my role
		mRoleInCluster = CH;

		// after sending decCH message, switch channel
		if ( mChannelID >= 0 ){
			scheduleAt( simTime() + sendingDuration * 1.001, nextChannelAssignment );

		}

		Veins::TraCICoord coord_Sumo = vehMobility->getManager()->omnet2traci(vehMobility->getCurrentPosition());

		findHost()->getDisplayString().updateWith("r=6,green");
		cout << findHost()->getFullName() << ": send declare CH packet! " << mClusterIndex << endl;
		cout << coord_Sumo.x << endl;

		// after 15ms to start CM confirmation process
		// 15ms is the longest time for all vehicles sending join message in a partition
		scheduleAt(simTime() + 0.015 + intrand(10) * 0.001, nextCMComfirm);
	}
	/* vehicles received DeclareCH packet, send unicast to join */
	else if (msg == nextJoinEvent){
		Join* joinPkt = new Join("Join");

		joinPkt->setSenderAddress(myMacAddress);
		joinPkt->setReceiverAddress(mCHMacAdd);
		joinPkt->setClusterIndex(mClusterIndex);
		joinPkt->setCHCoord(mCHCoord);
		joinPkt->setSendID(findHost()->getFullName());
		// write mobility information
		CMMobility mobiJoin;
		Veins::TraCICoord mSumoPos = vehMobility->getManager()->omnet2traci(vehMobility->getCurrentPosition());
		Coord mSumoPosC;
		mSumoPosC.x = mSumoPos.x;
		mSumoPosC.y = mSumoPos.y;

		mobiJoin.currentCoord = mSumoPosC;
		mobiJoin.speed = vehMobility->getSpeed();

		joinPkt->setSenderMobility(mobiJoin);

		// power control
		// control Mac layer transmission power
		// and connection manager transmission power
		par("txPower").setDoubleValue(2);
		txPower = par("txPower");
		// connection manager
		cModule* cmModule = check_and_cast<cModule*>(cm);
		cmModule->par("pMax").setDoubleValue(2);
		// update position to change connections
		vehMobility->changePosition();
//		cm->calcInterfDist();
// debug
//		cout << txPower << ", " << cmModule->par("pMax").doubleValue() << ", " << findHost()->getFullName() << endl;
// debug

		/*TODO:
		 * write the length of pkt
		 * */
		sendPacket(joinPkt);

	}
	else if (msg == nextCMComfirm){
		CMConfirm* cmConfirm = new CMConfirm("CMConfirm");

		// fill the packet
		cmConfirm->setCMListArraySize(mCMMobilityMap.size());
		std::map<int, CMMobility>::iterator it;
		int ii = 0;
		for ( it = mCMMobilityMap.begin(); it != mCMMobilityMap.end(); ++it){
			cmConfirm->setCMList(ii, it->first);
//			cout << it->first << "  " << ii << endl;
			++ii;
		}

		// cluster index
		cmConfirm->setClusterIndex(mClusterIndex);

		if (clusterRef.size() != 0 ){
			cmConfirm->setReferArraySize(clusterRef.size());

			for ( size_t i = 0; i < clusterRef.size(); ++i){
				cmConfirm->setRefer(i, clusterRef[i]);
			}
		}
		else {
			opp_error("my cluster reference is 0 !!");
		}

		cmConfirm->setSenderAddress(myMacAddress);
		cmConfirm->setClusterIndex(mClusterIndex);

		Veins::TraCICoord mySumoPos = vehMobility->getManager()->omnet2traci( vehMobility->getCurrentPosition() );
		Coord mySumoPos_C;
		mySumoPos_C.x = mySumoPos.x;
		mySumoPos_C.y = mySumoPos.y;

		cmConfirm->setSenderCoord(mySumoPos_C);

		cmConfirm->setDistance(clusterRef[0].distance);
		cmConfirm->setSendID(findHost()->getFullName());
		/*TODO:
		 * write the length of pkt
		 * */
		sendPacket(cmConfirm);

	}

	else if (msg == nextChannelSwitch) {
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

    /* centralized way */
    }else if (msg ==  nextClusteringEvent){
        if (strcmp(getOwner()->getOwner()->getName(), "node") == 0){

        	scheduleAt(simTime() + SWITCHING_CLS_11P * 2, nextClusteringEvent);

        	// get all running vehicles
        	std::map<std::string, cModule*> vehs_Map = vehMobility->getManager()->getManagedHosts();
			std::map<std::string, cModule*>::iterator itVehsMap;

			/*debugging*/
			/*printing all existing vehicles from vehs_Map*/
//			for (std::map<std::string, cModule*>::iterator it = test.begin(); it != test.end(); it++){
//				std::cout << (*it).first << ": "<< (*it).second->getFullName() << std::endl;
//			}
			/*debugging end*/

			/*
			 * find the neighborhoods
			 * */
			// clear neighbors of each vehicle
			for (itVehsMap = vehs_Map.begin(); itVehsMap != vehs_Map.end(); itVehsMap++ ){
				cModule* macModule = itVehsMap->second->getModuleByPath(".nic.mac1609_4");
				CMac_Distributed* cmac1609_4 = check_and_cast<CMac_Distributed*>(macModule);
				cmac1609_4->mNeighborVec.clear();
			}

			for (itVehsMap = vehs_Map.begin(); itVehsMap != vehs_Map.end(); itVehsMap++){
				std::map<std::string, cModule*>::iterator itVehsMap2;
				for (itVehsMap2 = vehs_Map.begin(); itVehsMap2 != vehs_Map.end(); itVehsMap2++){
					// outside
					cModule* macModule = itVehsMap->second->getModuleByPath(".nic.mac1609_4");
					cModule* mobModule = itVehsMap->second->getModuleByPath(".veinsmobility");
					CMac_Distributed* cmac1609_4 = check_and_cast<CMac_Distributed*>(macModule);
					Veins::TraCIMobility* mobility = check_and_cast<Veins::TraCIMobility*>(mobModule);

					// inside
					cModule* mobModule2 = itVehsMap2->second->getModuleByPath(".veinsmobility");
					Veins::TraCIMobility* mobility2 = check_and_cast<Veins::TraCIMobility*>(mobModule2);
					// find nodes within maxTxDist
					if ( mobility->getCurrentPosition().distance(mobility2->getCurrentPosition()) < maxTxDist ){
						if (itVehsMap->first != itVehsMap2->first){
							cmac1609_4->mNeighborVec.push_back(itVehsMap2->second);
						}
					}
				}
			}

			/*debugging*/
			/*printing all neighbors of each vehicle*/
//			for (itVehsMap = vehs_Map.begin(); itVehsMap != vehs_Map.end(); itVehsMap++){
//				cModule* macModule = itVehsMap->second->getModuleByPath(".nic.mac1609_4");
//				CMac1609_4* cmac1609_4 = check_and_cast<CMac1609_4*>(macModule);
//				std::cout << itVehsMap->second->getFullName() << "   neighbors: " << std::endl;
//
//				std::vector<cModule*> neighborVehVec = cmac1609_4->mNeighborVec;
//				for (std::vector<cModule*>::iterator it = neighborVehVec.begin(); it != neighborVehVec.end(); it++){
//					std::cout << (*it)->getFullName() << " ";
//				}
//				std::cout << std::endl;
//			}
			/*debugging end*/

			/*
			 * clustering algorithm
			 * */
			std::vector<cModule*> vehs_Vec;
			std::vector<cModule*>::iterator itVehVec;
			vehs_Vec.clear();

			for (itVehsMap = vehs_Map.begin(); itVehsMap != vehs_Map.end(); itVehsMap++){
				vehs_Vec.push_back((*itVehsMap).second);
				(*itVehsMap).second->getDisplayString().updateWith("r=0");
			}

			// sort based on x coordinates
			std::sort( vehs_Vec.begin(), vehs_Vec.end(), cmpFunction );

			/*debugging*/
			/*printing sorted vehicle list*/
//			for (itVehVec = vehs_Vec.begin(); itVehVec != vehs_Vec.end(); itVehVec++ ){
//				Veins::TraCIMobility* test = (Veins::TraCIMobility*)(*itVehVec)->getSubmodule("veinsmobility");
//				std::cout << test->getCurrentPosition().x << ", " << test->getCurrentPosition().y << std::endl;
//			}
			/*debugging end*/

			// divide vehicles into clusters by giving clusterIndex
			optimalLayer = optTreeDepth(vehs_Vec.size()) + 1; // depth + 1
			optimalClusterSize = optNumChildren(vehs_Vec.size(), optimalLayer - 1); // depth = layer-1

			/*debugging*/
			cout << optimalLayer << ", " << optimalClusterSize <<", " << vehs_Vec.size()<< endl;
			/*debugging end*/

			// assign cluster index for each vehicle
			int numCM = optimalClusterSize - 1;
			int indexCluster = 0;

			for (itVehVec = vehs_Vec.begin(); itVehVec != vehs_Vec.end(); itVehVec++ ){
				cModule* macModule = (*itVehVec)->getModuleByPath(".nic.mac1609_4");
				CMac_Distributed* cmac1609_4 = check_and_cast<CMac_Distributed*>(macModule);
				if ( numCM !=0 ){
					cmac1609_4->clusterIndex = indexCluster;
					numCM--;
				}
				else{
					cmac1609_4->clusterIndex = indexCluster;
					numCM = optimalClusterSize - 1;
					indexCluster++;
				}
			}

			int numCluster = indexCluster;

			/*debugging*/
			/*printing each vehicle's cluster index*/
//			for (itVehVec = vehs_Vec.begin(); itVehVec != vehs_Vec.end(); itVehVec++ ){
//				cModule* macModule = (*itVehVec)->getModuleByPath(".nic.mac1609_4");
//				CMac1609_4* cmac1609_4 = check_and_cast<CMac1609_4*>(macModule);
//				cout << (*itVehVec)->getFullName()<< ": " << cmac1609_4->clusterIndex << endl;
//			}
			/*debugging end*/

			// let each vehicle get neighbors in its own cluster
			for (itVehVec = vehs_Vec.begin(); itVehVec != vehs_Vec.end(); itVehVec++ ){
				std::vector<cModule*>::iterator itVehVec2;
				cModule* macModule = (*itVehVec)->getModuleByPath(".nic.mac1609_4");
				CMac_Distributed* cmac1609_4 = check_and_cast<CMac_Distributed*>(macModule);
				cmac1609_4->mClusterNeighborsVec.clear();
				for (itVehVec2 = vehs_Vec.begin(); itVehVec2 != vehs_Vec.end(); itVehVec2++ ){
					cModule* macModuel2 = (*itVehVec2)->getModuleByPath(".nic.mac1609_4");
					CMac_Distributed* cmac1609_4_2 = check_and_cast<CMac_Distributed*>(macModuel2);

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
				CMac_Distributed* cmac1609_4 = check_and_cast<CMac_Distributed*>(macModule);

				// same cluster
				if ( currentClusterIndex == cmac1609_4->clusterIndex ){

					/*moduleMinAveDis = findCenter(*itVehVec, cmac1609_4, minAveDis );*/

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
					// already find the center vehicle for each cluster
					cModule* macModuleCH = moduleMinAveDis->getModuleByPath(".nic.mac1609_4");
//					cModule* mobModuleCH = moduleMinAveDis->getModuleByPath(".veinsmobility");
					CMac_Distributed* cmac1609_4CH = check_and_cast<CMac_Distributed*>(macModuleCH);
//					Veins::TraCIMobility* mobi = check_and_cast<Veins::TraCIMobility*>(mobModuleCH);
//					mobi->commandSetColor( Veins::TraCIColor::fromTkColor("red") );
					moduleMinAveDis->getDisplayString().updateWith("r=4,yellow");

					// set this center vehicle as CH
					cmac1609_4CH->mRoleInCluster = CH;
					// set CH as itself
					cmac1609_4CH->mCH = moduleMinAveDis->getFullName();
					cmac1609_4CH->mCHMacAdd = cmac1609_4CH->myMacAddress;

					/*debugging*/
					/*print CH and CMs*/
					cout << "CH: " << moduleMinAveDis->getFullName() << endl;
					cout << "CM: " << endl;
					for (unsigned int i = 0; i < cmac1609_4CH->mClusterNeighborsVec.size(); i++){
						cout << cmac1609_4CH->mClusterNeighborsVec[i]->getFullName() << " ";
						if ( (i+1) % 4 == 0)
							cout << endl;
					}
					cout << endl;
					/*debugging end*/

					// set all other vehicles as CM from CH in each cluster
					for (unsigned int i = 0; i < cmac1609_4CH->mClusterNeighborsVec.size(); i++){
						cModule* macModuleCM = cmac1609_4CH->mClusterNeighborsVec[i]->getModuleByPath(".nic.mac1609_4");
						CMac_Distributed* cmac1609_4CM = check_and_cast<CMac_Distributed*>(macModuleCM);
						cmac1609_4CM->setRoleInCluster(CM);
						cmac1609_4CM->mClusterHead = moduleMinAveDis;
						cmac1609_4CM->mCH = moduleMinAveDis->getFullName();
						cmac1609_4CM->mCHMacAdd = cmac1609_4CH->myMacAddress;
					}

					// find the CH by finding the most center one within a cluster
					minAveDis = 0.0;
					moduleMinAveDis = nullptr;
					/*moduleMinAveDis = findCenter(*itVehVec, cmac1609_4, minAveDis );*/

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
//					cModule* mobModuleCH = moduleMinAveDis->getModuleByPath(".veinsmobility");
					CMac_Distributed* cmac1609_4CH = check_and_cast<CMac_Distributed*>(macModuleCH);
//					Veins::TraCIMobility* mobi = check_and_cast<Veins::TraCIMobility*>(mobModuleCH);
//					mobi->commandSetColor( Veins::TraCIColor::fromTkColor("red") );
					moduleMinAveDis->getDisplayString().updateWith("r=4,yellow");

					// set this center vehicle as CH
					cmac1609_4CH->mRoleInCluster = CH;
					cmac1609_4CH->mCHMacAdd = cmac1609_4CH->myMacAddress;

					/*debugging*/
					/*print CH and CMs*/
					cout << "CH: " << moduleMinAveDis->getFullName() << endl;
					cout << "CM: " << endl;
					for (unsigned int i = 0; i < cmac1609_4CH->mClusterNeighborsVec.size(); i++){
						cout << cmac1609_4CH->mClusterNeighborsVec[i]->getFullName() << " ";
						if ( (i+1) % 4 == 0)
							cout << endl;
					}
					cout << endl;
					/*debugging end*/

					// set all other vehicles as CM from CH for each cluster
					for (unsigned int i = 0; i < cmac1609_4CH->mClusterNeighborsVec.size(); i++){
						cModule* macModuleCM = cmac1609_4CH->mClusterNeighborsVec[i]->getModuleByPath(".nic.mac1609_4");
						CMac_Distributed* cmac1609_4CM = check_and_cast<CMac_Distributed*>(macModuleCM);
						cmac1609_4CM->setRoleInCluster(CM);
						cmac1609_4CM->mClusterHead = moduleMinAveDis;
						cmac1609_4CM->mCH = moduleMinAveDis->getFullName();
						cmac1609_4CM->mCHMacAdd = cmac1609_4CH->myMacAddress;
					}
				}

				currentClusterIndex = cmac1609_4->clusterIndex;
			}

			/*
			 * select MCH
			 * */
			/*among CHs find MCH with the most center*/
			cModule* moduleMCH;
			unsigned int maxNeighborVehSize = 0;
			int mostCenter = vehs_Vec.size();
			for (unsigned int i = 0; i < vehs_Vec.size(); i++ ){
				cModule* macModule = (vehs_Vec[i])->getModuleByPath(".nic.mac1609_4");
				CMac_Distributed* cmac1609_4 = check_and_cast<CMac_Distributed*>(macModule);
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
			CMac_Distributed* cmac1609_4MCH = check_and_cast<CMac_Distributed*>(macModuleMCH);
			cmac1609_4MCH->mRoleInCluster = MCH;
			cmac1609_4MCH->mClusterHead = moduleMCH;
//			cmac1609_4MCH->mCH = moduleMCH->getFullName();
			cmac1609_4MCH->mCHMacAdd = cmac1609_4MCH->myMacAddress;
			moduleMCH->getDisplayString().updateWith("r=4,white");

			// save CHs into MCH
			cmac1609_4MCH->mCoveredCHVec.clear();
			for (unsigned int i = 0; i < cmac1609_4MCH->mNeighborVec.size(); i ++ ){
				cModule* macModuleCoveredCH = cmac1609_4MCH->mNeighborVec[i]->getModuleByPath(".nic.mac1609_4");
				CMac_Distributed* cmac1609_4CoveredCH = check_and_cast<CMac_Distributed*>(macModuleCoveredCH);
				if ( cmac1609_4CoveredCH->mRoleInCluster == CH ){
					cmac1609_4MCH->mCoveredCHVec.push_back(cmac1609_4MCH->mNeighborVec[i]);
					cmac1609_4CoveredCH->mClusterHead = moduleMCH;
					cmac1609_4CoveredCH->mCH = moduleMCH->getFullName();
				}
			}

			/*debugging*/
			cout << "MCH: " << moduleMCH->getFullName() << endl;
			cout << "MCH's CHs: " << endl;
			for (unsigned int i = 0; i < cmac1609_4MCH->mCoveredCHVec.size(); i ++ ){
				cout << cmac1609_4MCH->mCoveredCHVec[i]->getFullName() << " ";
				if ( (i+1) % 4 == 0)
					cout << endl;
			}
			cout << endl;
			cout << "-----------------------------------------------" << endl;
			/*debugging end*/

			/*channel assignment part*/
			clusterChannelAssign( vehs_Vec, vehs_Vec.size(), 4, numCluster + 1);

			/*time slot assignment part*/
			vector< vector<cModule*> > timeSlotAll = timeSlotAllocation( vehs_Vec, numCluster + 1, optimalClusterSize);

			// let CH know the time slot assignment
			for ( unsigned int i = 0; i < vehs_Vec.size(); i++){
				cModule* module_i = vehs_Vec[i]->getModuleByPath(".nic.mac1609_4");
				CMac_Distributed* mac_i = check_and_cast<CMac_Distributed*>(module_i);
				if ( mac_i->mRoleInCluster == CH ){
					mac_i->timeSlotAlloVec = timeSlotAll[mac_i->clusterIndex];
				}
			}
        }

    } else if (msg == nextTAFEvent){

    	sendTAF(prepareTAF());

    } else if (msg == nextCheckEvent){

    	simtime_t offset = dblrand() * par("syncOffset").doubleValue() * 100;
    	if ( mRoleInCluster == CH || mRoleInCluster == MCH ){

    		scheduleAt(simTime() + offset + 0.0001, nextTAFEvent);

    		if (nextCheckEvent->isScheduled() == true)
        		cancelEvent(nextCheckEvent);
    		scheduleAt(simTime() + 0.1, nextCheckEvent);
    	} else {

        	if (nextCheckEvent->isScheduled() == true)
        		cancelEvent(nextCheckEvent);
    		scheduleAt(simTime() + 0.1, nextCheckEvent);
    	}

//    	if ( mChannelID >= 0 ){
//        	scheduleAt(simTime() + offset, nextChannelAssignment);
//
//        	cout << findHost()->getFullName()<< "--> scheduled CA!" << endl;
//        	if (nextCheckEvent->isScheduled() == true)
//        		cancelEvent(nextCheckEvent);
//    		scheduleAt(simTime() + 0.04, nextCheckEvent);
//    	} else {
//        	if (nextCheckEvent->isScheduled() == true)
//        		cancelEvent(nextCheckEvent);
//    		scheduleAt(simTime() + 0.04, nextCheckEvent);
//    	}

    } else if (msg == nextUploadEvent ){

    	sendPacket(prepareUpload());

    } else if (msg == nextACKEvent){

    	sendPacket(prepareACK());

    } else if (msg == nextChannelAssignment){

    	cout << findHost()->getFullName() << " --> " << "switch Channel: " << mChannelID << endl;

    	switch(mChannelID) {
    	case 0:
            DBG_MAC << "Channel --> CCH" << std::endl;
//            channelBusySelf(false);
            setActiveChannel(type_CCH);
            channelIdle(true);
            mySCH = Channels::CCH;
            phy11p->changeListeningFrequency(frequency[mySCH]);
            break;
    	case 1:
            DBG_MAC << "Channel --> SCH1" << std::endl;
//            channelBusySelf(false);
            setActiveChannel(type_SCH);
            channelIdle(true);
            mySCH = Channels::SCH1;
            phy11p->changeListeningFrequency(frequency[mySCH]);
            break;
    	case 2:
            DBG_MAC << "Channel --> SCH2" << std::endl;
//            channelBusySelf(false);
            setActiveChannel(type_SCH);
            channelIdle(true);
            mySCH = Channels::SCH2;
            phy11p->changeListeningFrequency(frequency[mySCH]);
            break;
    	case 3:
            DBG_MAC << "Channel --> SCH3" << std::endl;
//            channelBusySelf(false);
            setActiveChannel(type_SCH);
            channelIdle(true);
            mySCH = Channels::SCH3;
            phy11p->changeListeningFrequency(frequency[mySCH]);
            break;
    	case 4:
            DBG_MAC << "Channel --> SCH4" << std::endl;
//            channelBusySelf(false);
            setActiveChannel(type_SCH);
            channelIdle(true);
            mySCH = Channels::SCH4;
            phy11p->changeListeningFrequency(frequency[mySCH]);
            break;
    	}


    } else if (msg == nextTimeSlotAssignment){

    }

}

void CMac_Distributed::handleUpperControl(cMessage* msg) {
    assert(false);
}

void CMac_Distributed::handleUpperMsg(cMessage* msg) {

    CMACWSM* thisMsg;
    if ((thisMsg = dynamic_cast<CMACWSM*>(msg)) == NULL) {
        error("WaveMac only accepts WaveShortMessages");
    }

    t_access_category ac = mapPriority(thisMsg->getPriority());

    DBG_MAC << "Received a message from upper layer for channel "
            << thisMsg->getChannelNumber() << " Access Category (Priority):  "
            << ac << std::endl;

    t_channel chan;

    //rewrite SCH channel to actual SCH the CMac1609_4 is set to
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

void CMac_Distributed::handleLowerControl(cMessage* msg) {
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

void CMac_Distributed::handleLowerMsg(cMessage* msg) {
    CMac80211Pkt* macPkt = static_cast<CMac80211Pkt*>(msg);
    ASSERT(macPkt);

//    CMACWSM*  wsm =  dynamic_cast<CMACWSM*>(macPkt->decapsulate());
//    ASSERT(wsm);

    long dest = macPkt->getDestAddr();

    DBG_MAC << "Received frame name= " << macPkt->getName()
	                << ", myState:" << " src=" << macPkt->getSrcAddr()
	                << " dst=" << macPkt->getDestAddr() << " myAddr="
	                << myMacAddress
//	                << macPkt->getSenderModule()
	                << std::endl;

//    cout << "HLM: " << macPkt->getSenderModule()->getOwner()->getOwner()->getFullName() << endl;

    if (macPkt->getDestAddr() == myMacAddress) {
        DBG_MAC << "Received a data packet addressed to me." << std::endl;
        statsReceivedPackets++;

        cout << findHost()->getFullName() << ": " << "received a packet to me! " << endl;
//        sendUp(wsm);
    }
    else if (dest == LAddress::L2BROADCAST) {
        statsReceivedBroadcasts++;

        /*Chris*/
        if (strcmp(macPkt->getName(), "TAF") == 0 ){
        	TAF* taf = dynamic_cast<TAF*>(macPkt->decapsulate());
        	cout << findHost()->getFullName() << " --->" << endl;
        	cout << mCHMacAdd << " : " << macPkt->getSrcAddr() << endl;
			if ( mCHMacAdd == macPkt->getSrcAddr() ){
				cout << findHost()->getFullName() << ": received TAF packet from CH <" << taf->getNodeID() << ">" << endl;
				/*1st switch channel*/
				simtime_t timeCS = 0.004;
				scheduleAt(simTime() + timeCS, nextChannelAssignment);
				for ( unsigned int i = 0; i < taf->getCMArraySize(); i++ ){
					if ( myMacAddress == taf->getCM(i) ){
						cout << findHost()->getFullName() << ": schedule Upload message! " << endl;
						findHost()->getDisplayString().updateWith("r=4,green");
						simtime_t offset = SIFS_11P + i * ( SIFS_11P + getFrameDuration(128) + RADIODELAY_11P );
						/*2nd schedule upload event*/
						scheduleAt(simTime() + timeCS + offset, nextUploadEvent);
					}
				}
			} else {
//					cout << findHost()->getFullName() << ": TAF not for me!" << endl;
			}

            delete taf;
        }

        else if (strcmp(macPkt->getName(), "Upload") == 0 ){
        	// CH receives Upload message from CM;
        	if ( mRoleInCluster == CH || mRoleInCluster == MCH ){
            	Upload* up = dynamic_cast<Upload*>(macPkt->decapsulate());
            	bool flag = false;
            	for ( unsigned int i = 0; i < mClusterNeighborsVec.size(); i ++){
            		cModule* moduleCN = mClusterNeighborsVec[i]->getModuleByPath(".nic.mac1609_4");
            		CMac_Distributed* macModule = check_and_cast<CMac_Distributed*>(moduleCN);
            		if ( macPkt->getSrcAddr() == macModule->myMacAddress ){
            			flag = true;
            			break;
            		}
            	}
    			if ( flag ){
//    				scheduleAt(simTime() + SIFS_11P, nextACKEvent);
    				findHost()->bubble("recvUpload!");
    				cout << "CH: " << findHost()->getFullName() << endl;
    				cout << "received #Upload# message from CM <" << up->getNodeID() << ">" << endl;
    			} else {
    //        		cout << findHost()->getFullName() << ": Upload not for me!" << endl;
    			}

    			delete up;
        	}

        }
        else if ( strcmp(macPkt->getName(), "ACK") == 0){
        	// CMs receive ACK message for the Upload message
//        	if ( mRoleInCluster == CM ){
//            	ACK* ack = dynamic_cast<ACK*>(macPkt->decapsulate());
//
//    			if ( mCHMacAdd == macPkt->getSrcAddr() ){
//    				findHost()->getDisplayString().updateWith("r=0");
//    				cout << findHost()->getFullName() << ": received ACK from my CH <" << ack->getNodeID() << ">" << endl;
//    			} else {
//
//    			}
//    //        	}
//            	delete ack;
//        	}
        }
        else if ( strcmp(macPkt->getName(), "Initiating") ==0 ){


        	Initiating* ini = dynamic_cast<Initiating*>(macPkt->decapsulate());

        	DBG_MAC << "Received Initiating Packet from "<< ini->getSendID() << endl;


        	for (unsigned int i = 0; i < ini->getReferArraySize(); i++ ){
        		clusterRef.push_back( ini->getRefer(i) );
        	}

        	cout << findHost()->getFullName() << ": received Ini Packet from " << ini->getSendID() << endl;

        	Veins::TraCICoord mySumoPosition = vehMobility->getManager()->omnet2traci( vehMobility->getCurrentPosition() );
			Coord myPosition_S;
			myPosition_S.x = mySumoPosition.x;
			myPosition_S.y = mySumoPosition.y;

			// calculate distance to sender, need to be careful about two coordinates system (SUMO, OMNeT).
//			Veins::TraCICoord senderPosition_Sumo = vehMobility->getManager()->omnet2traci( ini->getSenderCoord() );
//			Coord senderPosition_S;
//			senderPosition_S.x = senderPosition_Sumo.x;
//			senderPosition_S.y = senderPosition_Sumo.y;

//			int distance2Sender = myPosition_S.distance( senderPosition_S );

			/*1. decide partition*/
			vector<double> borderX;

			for (unsigned int i = 0; i < ini->getReferArraySize(); i++){
				borderX.push_back(ini->getRefer(i).center.x + ini->getRefer(i).distance);
			}

			for (unsigned int i = 0; i < borderX.size(); i++ ){
				if ( mySumoPosition.x < borderX[i] ){
					mClusterIndex = i;
					break;
				}
			}

//			cout << mClusterIndex << endl;
			int distance2SenderCenter = myPosition_S.distance( ini->getRefer(0).center );
			cout << distance2SenderCenter << endl;
			/* limit sending of vehicls within 3 times radius of partition */
        	if ( distance2SenderCenter < ( ini->getDistance() * 3) ){

//	        	cout << mClusterIndex << endl;

				/*2. distance-based backoff to compete CH*/

				mChannelID = ini->getRefer(mClusterIndex).channel;
				int distance2Center = myPosition_S.distance( ini->getRefer(mClusterIndex).center );
//	        	int backoffCH = intuniform(0, distance2Center * 10);
				int backoffCH = distance2Center * 10;

/*debug*/
//				cout << "distance2Center: " << distance2Center << endl;
//        		cout << "Dist2Sender: " << distance2Sender << endl;
//        		cout << "sender: " << senderPosition_S.x << ", " << senderPosition_S.y << endl;
/*debug*/
				/*
				 * TODO: various Cluster Head selection schemes?
				 * */
				simtime_t time2Send = backoffCH * SLOTLENGTH_11P;

				scheduleAt( simTime() + time2Send, nextDeclareCHEvent );
        	}
        	else {
        		mClusterIndex = -1;
        	}

        	delete ini;
        }

        else if ( strcmp(macPkt->getName(), "Declare CH") == 0){

        	DeclareCH* decCH = dynamic_cast<DeclareCH*>(macPkt->decapsulate());

        	DBG_MAC << "Received CH Declaring Packet from " << decCH->getSendID() << endl;


        	/* 1. relay partition information */
        	if ( mClusterIndex < 0){
        		scheduleCHDecl(decCH);
        	}

        	/* 2. within the same partition
        	 * cancel my CH declaration message sending process
        	 * schedule join message
        	 * */
        	if ( decCH->getClusterIndex() == mClusterIndex ){
        		cout << "DCH from " << decCH->getSendID() << endl;
        		/*
        		 * a case that CMs couldn't send join message to CH (due to interference),
        		 * but send declaring CH message
        		 * */
        		if ( mRoleInCluster == CH ) {
        			cout << "mRoleInCluster == CH!" << endl;
        			if ( clusterRef.size() != 0 ){
//        				Veins::TraCICoord senderPosition_Sumo = vehMobility->getManager()->omnet2traci( decCH->getSenderCoord() );
//        				Coord senderPosition_S;
//        				senderPosition_S.x = senderPosition_Sumo.x;
//        				senderPosition_S.y = senderPosition_Sumo.y;

        				Veins::TraCICoord myPosition_Sumo = vehMobility->getManager()->omnet2traci( vehMobility->getCurrentPosition() );
        				Coord myPosition_S;
        				myPosition_S.x = myPosition_Sumo.x;
        				myPosition_S.y = myPosition_Sumo.y;

        				/*measure the CM*/
        				float my2Center = myPosition_S.distance(clusterRef[mClusterIndex].center);
        				float sender2Center = decCH->getSenderCoord().distance(clusterRef[mClusterIndex].center);

        				if ( my2Center < sender2Center ){
        					scheduleAt(simTime() + SIFS_11P, nextDeclareCHEvent);
        				}
        				else {
        					opp_error("CH role needs to be given to another node!");
        				}
        			}

        		}
        		else if ( mRoleInCluster == CM ) {
            		findHost()->getDisplayString().updateWith("r=4,lime");

            		if ( nextDeclareCHEvent->isScheduled() == true ){
            			cancelEvent(nextDeclareCHEvent);
            			cout << findHost()->getFullName() << ": cancel declare event! " << mClusterIndex << endl;
            		}

            		// set my role in this cluster
            		mRoleInCluster = CM;

            		// get channel ID
            		mChannelID = decCH->getRefer(mClusterIndex).channel;

            		mCHMacAdd = decCH->getSenderAddress();

            		mCHCoord = decCH->getSenderCoord();

                	/* distance-based backoff to join cluster*/
                	Veins::TraCICoord mySumoPosition = vehMobility->getManager()->omnet2traci( vehMobility->getCurrentPosition() );
                	Coord mySumoPositionC;
                	mySumoPositionC.x = mySumoPosition.x;
                	mySumoPositionC.y = mySumoPosition.y;

                	/*
                	 * TODO:
                	 * Other backoff functions to scale the time point of sending
                	 * */

                	int distance2Center = mySumoPositionC.distance( decCH->getRefer(decCH->getClusterIndex()).center );
                	int backoffCH = intuniform(0, distance2Center * 10);

                	simtime_t time2Send = backoffCH * SLOTLENGTH_11P;

                	if ( nextJoinEvent->isScheduled() == true){
                		cancelEvent(nextJoinEvent);
                	}
                	scheduleAt(simTime() + time2Send, nextJoinEvent);

            		if ( mChannelID >= 0 ){
            			scheduleAt(simTime(), nextChannelAssignment);
            		}
        		}
        	}

        	/* 3. save cluster partition information*/
        	if ( clusterRef.size() == 0 ){
            	for (unsigned int i = 0; i < decCH->getReferArraySize(); i++ ){
            		clusterRef.push_back( decCH->getRefer(i) );
            	}
        	}

        	delete decCH;
        }
        //  CH receive Join message from cluster member
        else if (strcmp(macPkt->getName(), "Join") == 0){

        	Join* join = dynamic_cast<Join*>( macPkt->decapsulate() );



        	if ( join->getClusterIndex() == mClusterIndex ){
            	if ( join->getReceiverAddress() == myMacAddress ){
            		DBG_MAC << "Received Join Packet for me. Save it to my CMMobilityMap" << endl;

            		// define return value of map insert
            		std::pair<std::map<int, CMMobility>::iterator, bool> ret;
            		ret = mCMMobilityMap.insert(std::pair<int, CMMobility>( join->getSenderAddress(),
            																join->getSenderMobility() ) );
            		// if existed, then update
            		if (ret.second == false) {
            			ret.first->second.currentCoord = join->getSenderMobility().currentCoord;
            			ret.first->second.speed = join->getSenderMobility().speed;
            		}
/*debug*/
            		cout << myNodeID << ", myCM size: " << mCMMobilityMap.size() << endl;

            		cout << "| MacAddr" << setw(10) << " | " << " Coordinates " << setw(11) << " | " << " Speed " << " | " << endl;
            		for (std::map<int, CMMobility>::iterator it = mCMMobilityMap.begin(); it != mCMMobilityMap.end(); it++ ){
            			cout <<"| " << it->first << setw(6) << " | " << it->second.currentCoord << setw(4) << " |   " << it->second.speed << "   | " << endl;
            		}
            		cout << endl;
/*debug*/
            		// timer for multiple joining
            		cout << findHost()->getFullName()<< " CM size + 1:" << mCMMobilityMap.size() << endl;
            	}
//            	else if ( mRoleInCluster == CH ){
//            		// deal with multiple CH declaring.
//            		// switch its role in cluster to CM
//            		mRoleInCluster = CM;
//            		// schedule sending join message
//            		mCHMacAdd = join->getReceiverAddress();
//            		// get CH's coordinates
//            		mCHCoord = join->getCHCoord();
//
//            		DBG_MAC << "Received Join Packet, change role CH -> CM." << endl;
//
//                	/* distance-based backoff to join cluster*/
//                	Veins::TraCICoord mySumoPosition = vehMobility->getManager()->omnet2traci( vehMobility->getCurrentPosition() );
//                	Coord mySumoPositionC;
//                	mySumoPositionC.x = mySumoPosition.x;
//                	mySumoPositionC.y = mySumoPosition.y;
//
//                	Veins::TraCICoord CHSumoPosition = vehMobility->getManager()->omnet2traci( mCHCoord );
//                	Coord CHSumoPositionC;
//                	CHSumoPositionC.x = CHSumoPosition.x;
//                	CHSumoPositionC.y = CHSumoPosition.y;
//
//                	/*
//                	 * TODO:
//                	 * Other backoff functions to scale the time point of sending
//                	 * */
//
//                	int distance2Center = mySumoPositionC.distance( CHSumoPositionC );
//                	int backoffCH = intuniform(0, distance2Center * 10);
//
//                	simtime_t time2Send = backoffCH * SLOTLENGTH_11P;
//            		if ( nextJoinEvent->isScheduled() == false ){
//            			scheduleAt(simTime() + time2Send, nextJoinEvent);
//            		}
//
//            		findHost()->getDisplayString().updateWith("r=4,lime");
//            	}
            	else{
                    DBG_MAC << "Packet not for me, deleting..." << std::endl;
            	}
        	}

        	delete join;
        }
        else if ( strcmp(macPkt->getName(), "CMConfirm") == 0 ){

        	CMConfirm* cmConfirm = dynamic_cast<CMConfirm*>( macPkt->decapsulate() );

        	if ( cmConfirm->getClusterIndex() == mClusterIndex ){
        		DBG_MAC << "Received Cluster Member Confirm Packet." << endl;

        		if ( mRoleInCluster == CM){
					bool inList = false;
					for ( size_t i = 0; i < cmConfirm->getCMListArraySize(); ++i ){
//						cout << cmConfirm->getCMList(i) << " $$ " << myMacAddress << endl;
						if ( cmConfirm->getCMList(i) == myMacAddress ){
							inList = true;
//							cout << findHost()->getFullName() << " : in List!" << endl;
							break;
						}
					}

					if ( inList == false ){
	            		// get channel ID
	            		mChannelID = cmConfirm->getRefer(mClusterIndex).channel;

	            		mCHMacAdd = cmConfirm->getSenderAddress();

	            		mCHCoord = cmConfirm->getSenderCoord();

	                	/* distance-based backoff to join cluster*/
	                	Veins::TraCICoord mySumoPosition = vehMobility->getManager()->omnet2traci( vehMobility->getCurrentPosition() );
	                	Coord mySumoPositionC;
	                	mySumoPositionC.x = mySumoPosition.x;
	                	mySumoPositionC.y = mySumoPosition.y;

	                	/*
	                	 * TODO:
	                	 * Other backoff functions to scale the time point of sending
	                	 * */

	                	int distance2Center = mySumoPositionC.distance( cmConfirm->getRefer(mClusterIndex).center );
	                	int backoffCH = intuniform(0, distance2Center * 10);

	                	simtime_t time2Send = backoffCH * SLOTLENGTH_11P;

	                	if ( nextJoinEvent->isScheduled() == true){
	                		cancelEvent(nextJoinEvent);
	                	}

	                	cout << findHost()->getFullName() << ": NOT in CM list, to send join!" << endl;
	                	scheduleAt(simTime() + time2Send, nextJoinEvent);

	            		findHost()->getDisplayString().updateWith("r=4,lime");

					}
        		}
        		else if (mRoleInCluster == CH){
                	if ( nextCMComfirm->isScheduled() == true){
                		cancelEvent(nextCMComfirm);
                	}

            		// get channel ID
            		mChannelID = cmConfirm->getRefer(mClusterIndex).channel;

            		mCHMacAdd = cmConfirm->getSenderAddress();

            		mCHCoord = cmConfirm->getSenderCoord();

            		mRoleInCluster = CM;
                	/* distance-based backoff to join cluster*/
                	Veins::TraCICoord mySumoPosition = vehMobility->getManager()->omnet2traci( vehMobility->getCurrentPosition() );
                	Coord mySumoPositionC;
                	mySumoPositionC.x = mySumoPosition.x;
                	mySumoPositionC.y = mySumoPosition.y;

                	/*
                	 * TODO:
                	 * Other backoff functions to scale the time point of sending
                	 * */

                	int distance2Center = mySumoPositionC.distance( cmConfirm->getRefer(mClusterIndex).center );
                	int backoffCH = intuniform(0, distance2Center * 10);

                	simtime_t time2Send = backoffCH * SLOTLENGTH_11P;

                	if ( nextJoinEvent->isScheduled() == true){
                		cancelEvent(nextJoinEvent);
                	}
                	scheduleAt(simTime() + time2Send, nextJoinEvent);

            		findHost()->getDisplayString().updateWith("r=4,lime");

        		}
        	}

        	delete cmConfirm;
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

void CMac_Distributed::setActiveChannel(t_channel state) {
    activeChannel = state;
    assert(state == type_CCH || (useSCH && state == type_SCH));
}

void CMac_Distributed::finish() {
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

void CMac_Distributed::attachSignal(CMac80211Pkt* mac, simtime_t startTime, double frequency) {

    simtime_t duration = getFrameDuration(mac->getBitLength());

    Signal* s = createSignal(startTime, duration, txPower, bitrate, frequency);
    MacToPhyControlInfo* cinfo = new MacToPhyControlInfo(s);

    mac->setControlInfo(cinfo);
}

Signal* CMac_Distributed::createSignal(simtime_t start, simtime_t length, double power, double bitrate, double frequency) {
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
bool CMac_Distributed::guardActive() const {
    if (!useSCH) return false;
    if (simTime().dbl() - nextChannelSwitch->getSendingTime() <= GUARD_INTERVAL_11P)
        return true;
    return false;
}

/* returns the time until the guard is over */
simtime_t CMac_Distributed::timeLeftTillGuardOver() const {
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
simtime_t CMac_Distributed::timeLeftInSlot() const {
    ASSERT(useSCH);
    return nextChannelSwitch->getArrivalTime() - simTime();
}

/* Will change the Service Channel on which the mac layer is listening and sending */
void CMac_Distributed::changeServiceChannel(int cN) {
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



int CMac_Distributed::EDCA::queuePacket(t_access_category ac,CMACWSM* msg) {

    if (maxQueueSize && myQueues[ac].queue.size() >= maxQueueSize) {
        delete msg;
        return -1;
    }
    myQueues[ac].queue.push(msg);
    return myQueues[ac].queue.size();
}

int CMac_Distributed::EDCA::createQueue(int aifsn, int cwMin, int cwMax,t_access_category ac) {

    if (myQueues.find(ac) != myQueues.end()) {
        opp_error("You can only add one queue per Access Category per EDCA subsystem");
    }

    EDCAQueue newQueue(aifsn,cwMin,cwMax,ac);
    myQueues[ac] = newQueue;

    return ++numQueues;
}

CMac_Distributed::t_access_category CMac_Distributed::mapPriority(int prio) {
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

CMACWSM* CMac_Distributed::EDCA::initiateTransmit(simtime_t lastIdle) {

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

simtime_t CMac_Distributed::EDCA::startContent(simtime_t idleSince,bool guardActive) {

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

void CMac_Distributed::EDCA::stopContent(bool allowBackoff, bool generateTxOp) {
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
void CMac_Distributed::EDCA::backoff(t_access_category ac) {
    myQueues[ac].currentBackoff = intuniform(0,myQueues[ac].cwCur);
    statsSlotsBackoff += myQueues[ac].currentBackoff;
    statsNumBackoff++;
    DBG_MAC << "Going into Backoff because channel was busy when new packet arrived from upperLayer" << std::endl;
}

void CMac_Distributed::EDCA::postTransmit(t_access_category ac) {
    delete myQueues[ac].queue.front();
    myQueues[ac].queue.pop();
    myQueues[ac].cwCur = myQueues[ac].cwMin;
    //post transmit backoff
    myQueues[ac].currentBackoff = intuniform(0,myQueues[ac].cwCur);
    statsSlotsBackoff += myQueues[ac].currentBackoff;
    statsNumBackoff++;
    DBG_MAC << "Queue " << ac << " will go into post-transmit backoff for " << myQueues[ac].currentBackoff << " slots" << std::endl;
}

void CMac_Distributed::EDCA::cleanUp() {
    for (std::map<t_access_category, EDCAQueue>::iterator iter = myQueues.begin(); iter != myQueues.end(); iter++) {
        while (iter->second.queue.size() != 0) {
            delete iter->second.queue.front();
            iter->second.queue.pop();
        }
    }
    myQueues.clear();
}

void CMac_Distributed::EDCA::revokeTxOPs() {
    for (std::map<t_access_category, EDCAQueue>::iterator iter = myQueues.begin(); iter != myQueues.end(); iter++) {
        if (iter->second.txOP == true) {
            iter->second.txOP = false;
            iter->second.currentBackoff = 0;
        }
    }
}

void CMac_Distributed::channelBusySelf(bool generateTxOp) {

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

void CMac_Distributed::channelBusy() {

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

    /*Chris*/
    /*CH declared by others, cancel my declaretion*/
//    if (nextDeclareCHEvent->isScheduled() == true) {
//    	cancelEvent(nextDeclareCHEvent);
//    }
//    else{
//
//    }

    /*Chris end*/
//    myEDCA[activeChannel]->stopContent(true,false);
}

void CMac_Distributed::channelIdle(bool afterSwitch) {

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

void CMac_Distributed::setParametersForBitrate(int bitrate) {
    for (unsigned int i = 0; i < NUM_BITRATES_80211P; i++) {
        if (bitrate == BITRATES_80211P[i]) {
            n_dbps = N_DBPS_80211P[i];
            return;
        }
    }
    opp_error("Chosen Bitrate is not valid for 802.11p: Valid rates are: 3Mbps, 4.5Mbps, 6Mbps, 9Mbps, 12Mbps, 18Mbps, 24Mbps and 27Mbps. Please adjust your omnetpp.ini file accordingly.");
}


simtime_t CMac_Distributed::getFrameDuration(int payloadLengthBits) const {
    // calculate frame duration according to Equation (17-29) of the IEEE 802.11-2007 standard
    simtime_t duration = PHY_HDR_PREAMBLE_DURATION + PHY_HDR_PLCPSIGNAL_DURATION + T_SYM_80211P * ceil( (16 + payloadLengthBits + 6)/(n_dbps) );

    return duration;
}

bool cmpFunction(cModule* i, cModule* j){
	Veins::TraCIMobility* i_Mob;
	Veins::TraCIMobility* j_Mob;
	i_Mob = (Veins::TraCIMobility*)i->getSubmodule("veinsmobility");
	j_Mob = (Veins::TraCIMobility*)j->getSubmodule("veinsmobility");

	return (i_Mob->getCurrentPosition().x) < (j_Mob->getCurrentPosition().x);
}

int CMac_Distributed::optTreeDepth( int numNodes ){
    int optd;
    double lowBound = log(numNodes)/2;
    double upperBound = (log(numNodes)+1)/2;

    int roundupLowBound = int(lowBound);
    int rounddownUpperBound = int(upperBound);

    optd = roundupLowBound;
    if (roundupLowBound - rounddownUpperBound >= 1){
        cout << "LowBound is " << roundupLowBound << " upperBound is " << rounddownUpperBound  << endl;
    }
    return optd;
}

int CMac_Distributed::optNumChildren( int numNodes, int optd){
    return (int)(pow(numNodes,(1/(double)optd))+1);
}

void CMac_Distributed::setRoleInCluster(t_role_in_cluster r){
	mRoleInCluster = r;
}

/*this function is not used*/
cModule* CMac_Distributed::findCenter(cModule* currentVeh, CMac_Distributed* cmac1609_4, float& minAveDis ){
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
		/*moduleMinAveDis = currentVeh;*/
	}
	else if ( minAveDis > averageDistance ){
		minAveDis = averageDistance;
		/*moduleMinAveDis = currentVeh;*/
	}

	return currentVeh;
}

TAF* CMac_Distributed::prepareTAF() {
	// Timing Advertisement Frame
	TAF* taf = new TAF("TAF");

	taf->setNodeID(findHost()->getFullName());
	taf->setCMArraySize( timeSlotAlloVec.size() );

	for (unsigned int i = 0; i < timeSlotAlloVec.size(); i++ ){
		if (timeSlotAlloVec[i] != nullptr){
			cModule* macModuleCM = timeSlotAlloVec[i]->getModuleByPath(".nic.mac1609_4");
			CMac_Distributed* macCM = check_and_cast<CMac_Distributed*>(macModuleCM);
			taf->setCM(i, macCM->myMacAddress);
		} else {
			taf->setCM(i, 0);
		}

	}

	// CMs' Mac address, int type = 4 bytes * 8 bit = 32 bit
	taf->addBitLength( taf->getCMArraySize() * 4 * 8);
	return taf;
}

void CMac_Distributed::sendTAF(TAF* taf) {
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
        DBG_MAC << "Sending a Packet. Frequency " << freq << " Priority" << lastAC << std::endl;
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

simtime_t CMac_Distributed::sendPacket(cPacket* msg){
    //we actually came to the point where we can send a packet
    channelBusySelf(true);
//    CMACWSM* pktToSend = myEDCA[activeChannel]->initiateTransmit(lastIdle);

//    lastAC = mapPriority(pktToSend->getPriority());

//    DBG_MAC << "MacEvent received. Trying to send packet with priority" << lastAC << std::endl;

    //send the packet
    CMac80211Pkt* mac = new CMac80211Pkt(msg->getName(), msg->getKind());
    mac->setDestAddr(LAddress::L2BROADCAST);
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
        DBG_MAC << "Sending a Packet. Frequency " << freq << " Priority" << lastAC << std::endl;
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

    return sendingDuration;
}

simtime_t CMac_Distributed::sendPacketUnicast(cPacket* msg, int destAddr){
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
        DBG_MAC << "Sending a Packet. Frequency " << freq << " Priority" << lastAC << std::endl;
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
    return sendingDuration;
}

Upload* CMac_Distributed::prepareUpload(){
	Upload* up = new Upload("Upload");

	up->setNodeID( findHost()->getFullName() );

	up->addBitLength(128);
	return up;
}

ACK* CMac_Distributed::prepareACK(){
	ACK* ack = new ACK("ACK");

	ack->setNodeID( findHost()->getFullName() );

	ack->addBitLength(128);
	return ack;
}

void CMac_Distributed::clusterChannelAssign(std::vector<cModule*> nodeSet,
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
    	CMac_Distributed* mac_i = check_and_cast<CMac_Distributed*>(module_i);
        for (int j=0; j < numNodes ; j++){
        	cModule* module_j = nodeSet[j]->getModuleByPath(".nic.mac1609_4");
        	CMac_Distributed* mac_j = check_and_cast<CMac_Distributed*>(module_j);
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
    for (int k=0; k < numClusters ; k++){
        ClusterChannel[k]=0;
    }
    while (remain_cluster > 0){
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
            //cout << avr_interfere[i] << endl;
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
        if (ClusterChannel[max_cl] == 0){
            ClusterChannel[max_cl] = remain_channel;
            remain_cluster--;
            remain_channel--;
            //update other interfering clusters who has different channel to this
            for (int i=0; i < numClusters; i++){
                if (ClusterChannel[i] != 0){
                    cGraph[i][max_cl] = 0;
                    cGraph[max_cl][i] = 0;
                }

            }
        }
        if (ClusterChannel[max_edge] == 0){
            ClusterChannel[max_edge] = remain_channel;
            remain_cluster--;
            remain_channel--;
            cGraph[max_cl][max_edge] = 0;
            cGraph[max_edge][max_cl] = 0;
            for (int i=0; i < numClusters; i++){
                cGraph[i][max_edge] = 0;
                cGraph[max_edge][i] = 0;
            }
        }else{
            cout << "ERROR" << endl;
        }

        //for (int i=0; i < numClusters; i++){
        //    cout << i << "::" <<  avr_interfere[i] << endl;
        //}
        //cout << "remain_cluster" << remain_cluster << endl;
    }
    cout << "channels:";
    for (int i=0; i < numClusters; i++){
        cout <<  ClusterChannel[i] << " ";
    }
    cout << endl;
    // assign channel to each node
    for (int i=0; i < numNodes ; i++){
    	cModule* module = nodeSet[i]->getModuleByPath(".nic.mac1609_4");
    	CMac_Distributed* mac = check_and_cast<CMac_Distributed*>(module);
//        nodeSet[i].setChannel(ClusterChannel[nodeSet[i].getClusterId()]);
    	mac->mChannelID = ClusterChannel[mac->clusterIndex];
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

vector< vector<cModule*> >
CMac_Distributed::timeSlotAllocation(std::vector<cModule*> nodeSet,
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
		cout << "~~~~~~~~~~~~~~~~~~~~~~"<< endl;
		cout << nodesSet.size() << endl;
		cout << currentTimeSlot << endl;
		/*debugging*/

		vector< vector<int> > deg(nodesSet.size(), vector<int>(clusterNum));
		/*for each node compute degree in each cluster */
		for (unsigned int i = 0; i < nodesSet.size(); i++){
			cModule* module_i = nodesSet[i]->getModuleByPath(".nic.mac1609_4");
			CMac_Distributed* mac_i = check_and_cast<CMac_Distributed*>(module_i);
			for (int j = 0; j < clusterNum; j++ ){
				for (unsigned int k = 0; k < mac_i->mNeighborVec.size(); k++){
					cModule* module_k = mac_i->mNeighborVec[k]->getModuleByPath(".nic.mac1609_4");
					CMac_Distributed* mac_k = check_and_cast<CMac_Distributed*>(module_k);
					if (mac_i->mCHMacAdd != 0){
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
			CMac_Distributed* mac_i = check_and_cast<CMac_Distributed*>(module_i);
			if (mac_i->mRoleInCluster == CH || mac_i->mRoleInCluster == MCH){
				cout << "erase " << (*it)->getFullName() << endl;
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
		/*maxDegree_Cluster: id, degree, cluster ID*/
		vector< std::tuple<cModule*, int, int> > maxDegree_Cluster_t;
		maxDegree_Cluster_t.clear();

		for ( unsigned int i = 0; i < deg.size(); i++ ){
			vector<int>::iterator iteMax = std::max_element( deg[i].begin(), deg[i].end() );
			cModule* module = nodesSet[i]->getModuleByPath(".nic.mac1609_4");
			CMac_Distributed* mac = check_and_cast<CMac_Distributed*>(module);
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
						CMac_Distributed* mac_j = check_and_cast<CMac_Distributed*>(module_j);
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

		cout << "remaining node Size: " << nodesSet.size() << endl;
		cout << "currentTimeSlot: "<< currentTimeSlot << endl;
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
    cout << "time slot assignment: " << endl;
    for ( unsigned int t1 = 0; t1 < timeSlotAssign.size(); t1++ ){
    	for (unsigned int t2 = 0; t2 < timeSlotAssign[t1].size(); t2++ ){
    		if ( timeSlotAssign[t1][t2] != nullptr){
    			cout << timeSlotAssign[t1][t2]->getFullName() << " ";
    		} else{
    			cout << "        ";
    		}
    	}
    	cout << endl;
    }
    /*debugging*/

    return timeSlotAssign;
}

void
CMac_Distributed::scheduleCHDecl(DeclareCH* decCH){


	Veins::TraCICoord mySumoPosition = vehMobility->getManager()->omnet2traci( vehMobility->getCurrentPosition() );

	Coord mySumoPositionC;
	mySumoPositionC.x = mySumoPosition.x;
	mySumoPositionC.y = mySumoPosition.y;

//	Veins::TraCICoord senderPosition_Sumo = vehMobility->getManager()->omnet2traci( decCH->getSenderCoord() );
//	Coord senderPosition_S;
//	senderPosition_S.x = senderPosition_Sumo.x;
//	senderPosition_S.y = senderPosition_Sumo.y;
//
//	double distance2Sender = mySumoPositionC.distance( senderPosition_S );

	vector<double> borderX;

	/*1. decide partition*/
	for (unsigned int i = 0; i < decCH->getReferArraySize(); i++){
		borderX.push_back(decCH->getRefer(i).center.x + decCH->getRefer(i).distance);
	}

	for (unsigned int i = 0; i < borderX.size(); i++ ){
		if ( mySumoPosition.x < borderX[i] ){
			mClusterIndex = i;
			cout << findHost()->getFullName() << ": CI " << mClusterIndex << endl;
			break;
		}
	}

	double distance2SenderCenter = mySumoPositionC.distance( decCH->getRefer(decCH->getClusterIndex()).center );

	if ( distance2SenderCenter < (decCH->getDistance() * 3) ){
		// calculate distance to sender, need to be careful two coordinates system (SUMO, OMNeT).

		/* 2. distance-based backoff to compete CH */

		mChannelID = decCH->getRefer(mClusterIndex).channel;

		/*
    	 * TODO:
    	 * Other backoff functions to scale the time point of sending
    	 * */

		double distance2Center = mySumoPositionC.distance( decCH->getRefer(mClusterIndex).center );
	//	int backoffCH = intuniform(0, distance2Center * 10);
		int backoffCH = (int)distance2Center * 10;


		/*
		 * TODO: various Cluster Head selection schemes?
		 * */
		simtime_t time2Send = backoffCH * SLOTLENGTH_11P;

		// only when distance between sender and receiver is less than 2 times of radius of a partition

		if ( nextDeclareCHEvent->isScheduled() == false ){
			scheduleAt(simTime() + time2Send, nextDeclareCHEvent);
			cout << "Relay! scheduleCHDecl!" << endl;
		}
	}
	else {
		mClusterIndex = -1;
	}

//	delete decCH;
}


typedef tuple<cModule*, int, int> t_Tuple;

//bool cmpFunction2(const t_Tuple &i, const t_Tuple &j){
//    return (std::get<1>(i) > std::get<1>(j) );
//}
