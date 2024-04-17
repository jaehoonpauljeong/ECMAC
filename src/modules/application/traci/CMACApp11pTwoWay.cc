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

#include "application/traci/CMACApp11pTwoWay.h"

using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

const simsignalwrap_t CMACApp11pTwoWay::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

Define_Module(CMACApp11pTwoWay);

static long gPeTxPackets;
static long gPeRxPackets;
static long gPeRxDropPkts;
static long gPeRxDuplPkts;
static long gPeRxExpiPkts;
static long gPeTotalRxPkts;

static std::vector<double> gPeE2EDelay;
static std::vector<double> g_vPeEmg2kDelay;

static double gPeEmgPktSentTime = 0.0;
static double gPeEmgPktRecvTime = 0.0;
static std::vector<long> gPeEmgRxPktVec;

static std::string gLatestNodeId;

static bool gEmgPktSentFlag = false;
static int g_iEmgPktCount = 0;
static int g_iEmgPktSendPlan = 10;

static long g_lBGPktsCount;

//static int g_iTotalVehCount = 0;

void CMACApp11pTwoWay::initialize(int stage) {
	CMacBaseWaveApplLayer::initialize(stage);
	if (stage == 0) {
		traci = TraCIMobilityAccess().get(getParentModule());
		annotations = AnnotationManagerAccess().getIfExists();
		ASSERT(annotations);

		sentMessage = false;
		lastDroveAt = simTime();
		findHost()->subscribe(parkingStateChangedSignal, this);
		isParking = false;
		sendWhileParking = par("sendWhileParking").boolValue();

		// Chris
		// gLatestNodeId = findHost()->getFullName();

		debugApp = par("debugApp").boolValue();
		appSaveToFile = par("appSaveToFile").boolValue();

		ecmacApp=par("ecmacApp").boolValue();
		dmmacApp=par("dmmacApp").boolValue();
		waveApp=par("waveApp").boolValue();

		schemeName = "ECMAC";
        myNodeID = findHost()->getFullName();

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

		if ( ecmacApp || dmmacApp)
			dataLengthBits = 0;
		else if ( waveApp )
			dataLengthBits = 144; // 400-256(headbits)
		
		b_backgroundTraffic = par("appBGTraffic").boolValue();

		if (b_backgroundTraffic){
		    d_bgTrafficInterval = par("appBGTrafficInterval").doubleValue();
		}
		else {
		    d_bgTrafficInterval = 0.0;
		}

		schemeName = "ECMAC";

		if (FindModule<TraCIScenarioManagerLaunchd*>::findGlobalModule()){
		    TraCIScenarioManagerLaunchd* managerL = FindModule<TraCIScenarioManagerLaunchd*>::findGlobalModule();
		    seed = managerL->par("seed");
		}
		else{
		    error("can't find module!");
		}

		eventSendWSM = new cMessage("next Send WSM Packet Event");

		// back ground
		if ( b_backgroundTraffic ) {
		    scheduleAt(simTime() + 0.1 + dblrand() * 0.001, eventSendWSM);
		}

		sendPkt = par("sendPkt").boolValue();
		sendRandPkt=par("sendRandPkt").boolValue();
		pktSendEndTime = par("pktSendEndTime").doubleValue();
		sendInterval = par("pktSendInterval").doubleValue();

		sendDataEvt = new cMessage("data", SEND_DATA_EVT);
		transitEvt = new cMessage("transit event");

		double maxOffset = par("maxOffset").doubleValue();
		individualOffset = dblrand() * maxOffset;

		peTxPackets = 0;
		peRxPackets = 0;

		// individualOffset is 0.005s
		if ( sendPkt ){
//			if (strcmp(findHost()->getFullName(),"node[91]") == 0)
//			scheduleAt(simTime() + sendInterval + individualOffset, sendDataEvt);
//			std::cout << simTime() + 0.2 + sendInterval + individualOffset << " APP: sendData " << endl;
			scheduleAt(simTime() + 0.2 + individualOffset, transitEvt);
		}
		else if ( sendRandPkt){
		    scheduleAt(simTime() + 0.2 + individualOffset, sendDataEvt);
		}

		pktSerial = 0;
		rxSerMap.clear();

		sendEmgPkt = par("sendEmgPkt").boolValue();
		emgPktCount = 0;
		sendEmgInterval = 10 * dblrand();

		peEmgPktRxTime = 0.0;
		peEmgRxPackets = 0;
		peEmgTXPackets = 0;
		peRxDropPkts = 0;

		sendFloodEvt = new cMessage("flood", SEND_FLOOD_EVT);
		decideVehEvt = new cMessage("decide");

		/*sendEmgPkt is not used here, for ECMAC, sendPkt is used above, which is unicast*/
		if ( sendEmgPkt )
			scheduleAt(simTime() + 1.1 + dblrand() * 0.000001, decideVehEvt);

		// end Chris
	}
}

void CMACApp11pTwoWay::handleSelfMsg(cMessage* msg){
	switch (msg->getKind()) {
		case SEND_BEACON_EVT: {
			sendWSM(prepareWSM("beacon", beaconLengthBits, type_CCH, beaconPriority, 0, -1));
			scheduleAt(simTime() + par("beaconInterval").doubleValue(), sendBeaconEvt);
			break;
		}
		case SEND_DATA_EVT: {
			if ( getOffsetRatio(traci) <= 0.98 ){
				if ( simTime() < pktSendEndTime){
//					sendWSM(prepareWSM("data",
//										dataLengthBits,
//										type_CCH,
//										dataPriority,
//										0,
//										++pktSerial));
				    if (ecmacApp){
				        sendWSM(prepareWSM("data",
                                            dataLengthBits,
                                            type_SCH,
                                            1,
                                            0,
                                            ++pktSerial));
				    }
				    else if ( dmmacApp ){
				        sendWSM(prepareWSM("status",
                                           dataLengthBits,
                                           type_SCH,
                                           1,
                                           0,
                                           ++pktSerial));
				    }


					if ( debugApp ){
						std::cout << "......................................" << endl;
						std::cout << "APPL: SEND_DATA_EVT" << endl;
						std::cout << simTime() << " " << findHost()->getFullName()
								<< " sending a pkt to " << destIDdebug <<  endl;
						findHost()->getDisplayString().updateWith("r=8,deep pink");

					}

					peTxPackets++;
					gPeTxPackets++;

					if ( sendRandPkt){
					    scheduleAt(simTime() + sendInterval + dblrand() * 0.01, sendDataEvt);
					}
				}
			}
			break;
		}
		case SEND_FLOOD_EVT: {

			g_iEmgPktCount++;

			findHost()->getDisplayString().updateWith("r=8,red");

			sendWSM(prepareWSM("data",
								dataLengthBits,
								type_CCH,
								dataPriority,
								0,
								g_iEmgPktCount));

            if ( debugApp ){
                std::cout << "......................................" << endl;
                std::cout << "APPL: SEND_FLOOD_EVT" << endl;
                std::cout << findHost()->getFullName()
                        << " sending an emergency pkt to " << destIDdebug <<  endl;
                findHost()->getDisplayString().updateWith("r=8,deep pink");

            }

            if ( debugApp ){
                std::cout << findHost()->getFullName()
                        << " sends a EMG pkt. " << simTime() << endl;
            }

			peEmgTXPackets++;
			gPeTxPackets++;

			if ( gPeEmgPktSentTime == 0.0 )
				gPeEmgPktSentTime = simTime().dbl();

			gEmgPktSentFlag = false;

//			if ( emgPktCount < 10 ){
//				scheduleAt(simTime() + sendEmgInterval, sendFloodEvt);
//
//			}

			break;

		}
		default: {
			if ( msg == decideVehEvt ){

				// for debug, set a particular vehicle send emg pkt
//				if ( strcmp(findHost()->getFullName(), "node[1]") == 0 ){
//					scheduleAt(simTime() + sendEmgInterval, sendFloodEvt);
//				}

				// set
				if ( !gEmgPktSentFlag ){
					if ( getOffsetRatio(traci) > 0.875 && getOffsetRatio(traci) < 0.98){
						if ( traci->getPlannedEdgeIdList().front().compare(traci->getCurrentEdgeId()) == 0 ){
							// generate a random number ranging [0.1, 0.2)
							double randNum = dblrand() * 0.1 + 0.1;

							scheduleAt(simTime() + randNum, sendFloodEvt);

							if ( debugApp ){
								std::cout << "randNum: " << randNum << endl;
								std::cout << findHost()->getFullName() << " schedule a sendFloodEvt at " << simTime() + randNum << endl;
							}

							gEmgPktSentFlag = true;
						}
					}
				}

				// periodic schedule emergency msg
				if ( g_iEmgPktCount < g_iEmgPktSendPlan ){
					scheduleAt(simTime() + 1.0 + dblrand(), decideVehEvt);
					if ( debugApp ){
						std::cout << "findHost()->getFullName(): decideEvt: " << decideVehEvt->getArrivalTime() << endl;
					}
				}

			}
			else if ( msg == transitEvt ){

                if ( getOffsetRatio(traci) < 0.98 && getOffsetRatio(traci) > 0.85){
                    std::map<std::string, cModule*> allVeh = traci->getManager()->getManagedHosts();
                    std::map<std::string, cModule*>::iterator it;


                    // for twoway case
                    std::vector<std::string> vehFarthestVec;
                    vehMapEdgeId.clear();

                    for (auto& i:allVeh){
                        Veins::TraCIMobility* iMob;
                        iMob = (Veins::TraCIMobility*)i.second->getSubmodule("veinsmobility");
                        std::string edgeID = iMob->getCommandInterface()->getEdgeId(iMob->getExternalId());
                        if (getOffsetRatio(iMob) <= 0.98 && getOffsetRatio(iMob) >= 0.02){
                            if (vehMapEdgeId.find(edgeID) != vehMapEdgeId.end()){

                                vehMapEdgeId.at(edgeID).push_back(i.second);
                            }
                            else{
                                // insert a key and a value
                                vehMapEdgeId[edgeID].push_back(i.second);
                            }
                        }
                    }

                    for (auto& i:vehMapEdgeId){
                        double farthestNodeX = 0.0;
                        std::string farthestNodeId;
                        for (auto& j:i.second){
                            Veins::TraCIMobility* jMob;
                            jMob = (Veins::TraCIMobility*)j->getSubmodule("veinsmobility");
                            double offset = jMob->getCommandInterface()->getLanePosition(jMob->getExternalId());
                            if ( farthestNodeX == 0.0 ){
                                farthestNodeX = offset;
                                farthestNodeId = j->getFullName();
                            }
                            else if ( farthestNodeX < offset ){
                                farthestNodeX = offset;
                                farthestNodeId = j->getFullName();
                            }

                            if (gLatestNodeId.empty()){
                                if (getOffsetRatio(jMob) < 0.3){
                                    gLatestNodeId = j->getFullName();
                                }
                            }
                        }

                        if (!farthestNodeId.empty()){
                            vehFarthestVec.push_back(farthestNodeId);
                        }


                    }

//                    for (it=allVeh.begin(); it!=allVeh.end(); ++it ){
//                        Veins::TraCIMobility* i_Mob;
//                        i_Mob = (Veins::TraCIMobility*)it->second->getSubmodule("veinsmobility");
//
//                        if ( getOffsetRatio(i_Mob) < 0.98 && getOffsetRatio(i_Mob) > 0.85){
//                            if ( farthestNodeX == 0.0){
//                                farthestNodeX = i_Mob->getCurrentPosition().x;
//                                farthestNodeId = it->second->getFullName();
//                            }
//                            else if ( farthestNodeX < i_Mob->getCurrentPosition().x){
//                                farthestNodeX = i_Mob->getCurrentPosition().x;
//                                farthestNodeId = it->second->getFullName();
//                            }
//                        }
//
//
//
//                    }

//                    std::cout << "farthestNodeId: " << farthestNodeId << endl;
                    for (auto& i: vehFarthestVec){
                        if ( myNodeID.compare(i) == 0 ){
                            scheduleAt(simTime()+ dblrand() * 0.001, sendDataEvt);
                        }
                    }

//                    if ( vehFarthestVec.compare(findHost()->getFullName()) == 0){
////                        std::cout << "sendDataEvt scheduled: " << farthestNodeId << endl;
//                        scheduleAt(simTime()+ dblrand() * 0.001, sendDataEvt);
//                    }
                }

                scheduleAt(simTime()+ sendInterval + dblrand() * 0.001, transitEvt);
			}
			else if (msg == eventSendWSM){
			    CMACWSM* wsmPkt = new CMACWSM("wsm");

		        wsmPkt->setChannelNumber(Channels::SCH1);
		        wsmPkt->setDataRate(18);
		        wsmPkt->setPsid(0);
		        wsmPkt->setPriority(1);
		        wsmPkt->setWsmVersion(1);
		        wsmPkt->setTimestamp(simTime());
		        wsmPkt->setSenderAddress(myId);
		        wsmPkt->setRecipientAddress(0);
		        wsmPkt->setSenderPos(curPosition);
		        wsmPkt->setSerial(-1);

		        wsmPkt->setDestID("");
		        // Time to live in second
		        wsmPkt->setTTL(120);
		        // Sender ID
		        wsmPkt->setNodeID(findHost()->getFullName());

		        sendDown(wsmPkt);

//		        std::cout << "APP: sending a BG packet! " << findHost()->getFullName() << endl;

		        g_lBGPktsCount++;

		        scheduleAt(simTime() + d_bgTrafficInterval + dblrand() * 0.001 , eventSendWSM);
			}			
			else if (msg)
				DBG << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
			break;
		}
	}
}

void CMACApp11pTwoWay::handleLowerMsg(cMessage* msg){

	CMACWSM* wsm = dynamic_cast<CMACWSM*>(msg);
	ASSERT(wsm);

	if (std::string(wsm->getName()) == "beacon") {
		onBeacon(wsm);
	}
	else if (std::string(wsm->getName()) == "data") {
		onData(wsm);
		if ( debugApp){
//		    std::cout << "APP:handleLowerMsg, onData()" << endl;
		}
	}
	else if ( std::string(wsm->getName()) == "status"){
		onStatus(wsm);
	}
	else {
		DBG << "unknown message (" << wsm->getName() << ")  received\n";
		delete(msg);
	}
}

void CMACApp11pTwoWay::onBeacon(CMACWSM* wsm) {
	delete(wsm);
}

void CMACApp11pTwoWay::onData(CMACWSM* wsm) {
//	findHost()->getDisplayString().updateWith("r=8,green");
//	annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), traci->getPositionAt(simTime()), "blue"));
	gPeTotalRxPkts++;
	if ( simTime() < (wsm->getTimestamp() + wsm->getTTL()) ){
		if ( strcmp(wsm->getDestID(), findHost()->getFullName()) == 0 ){
			// check rxSerMap if I had the sender's pkt
			if ( rxSerMap.find(wsm->getNodeID()) == rxSerMap.end() ){
				findHost()->getDisplayString().updateWith("r=8,LightBlue");

				rxSerMap.insert(std::pair<std::string, int>(wsm->getNodeID(), wsm->getSerial()));
				peRxPackets++;
				gPeRxPackets++;

				simtime_t e2eDelay = simTime() - wsm->getTimestamp();
				gPeE2EDelay.push_back(e2eDelay.dbl());

				DBG <<  findHost()->getFullName()
						<< " APPL: successfully receive a new pkt, total: " << peRxPackets
						<< " from "<< wsm->getNodeID() << endl;

				if ( debugApp ){
	                std::cout <<  simTime() << " " << findHost()->getFullName()
	                        << " APPL: successfully receive a new pkt, total: " << peRxPackets
	                        << " from "<< wsm->getNodeID() << endl;
				}

			} else {
				// check if the sender's serial is larger than the existing one, if so, update
				if (rxSerMap.at(wsm->getNodeID()) < wsm->getSerial()){
					findHost()->getDisplayString().updateWith("r=8,LightBlue");

					rxSerMap.at(wsm->getNodeID()) = wsm->getSerial();
					peRxPackets++;
					gPeRxPackets++;

					simtime_t e2eDelay = simTime() - wsm->getTimestamp();
					gPeE2EDelay.push_back(e2eDelay.dbl());

					DBG <<  findHost()->getFullName()
							<< " APPL: successfully receive a pkt, total: " << peRxPackets
							<< " from "<< wsm->getNodeID() << endl;

	                if ( debugApp ){
	                    std::cout <<  simTime() << " " << findHost()->getFullName()
	                            << " APPL: successfully receive a pkt, total: " << peRxPackets
	                            << " from "<< wsm->getNodeID() << endl;
	                }
				}
				// if not, it is an old or duplicate pkt, ignore it.
				else {
					gPeRxDuplPkts++;
				}
			}

			if (debugApp){
				std::cout << findHost()->getFullName() << ", rxSerMap: " << endl;
				for (auto i:rxSerMap){
					std::cout << i.first << ": " << i.second << endl;
				}
				std::cout << endl;
			}


		} else if (strcmp(wsm->getDestID(), "emergency" ) == 0){

			if ( rxSerMap.find(wsm->getNodeID()) == rxSerMap.end() ){
				findHost()->getDisplayString().updateWith("r=8,LightBlue");

				rxSerMap.insert(std::pair<std::string, int>(wsm->getNodeID(), wsm->getSerial()));
                
                simtime_t e2eDelay = simTime() - wsm->getTimestamp();
                if ( wsm->getEmgReach2kFlag() ){
                    g_vPeEmg2kDelay.push_back(e2eDelay.dbl());
                }

				peEmgRxPackets++;
				gPeRxPackets++;

				if (debugApp){
					std::cout << "APPL: " << findHost()->getFullName()
							<< " recvd EMG pkt. " << simTime() << endl;
				}

				DBG <<  findHost()->getFullName()
						<< " APPL: successfully receives a EMG pkt, "
						<< " from "<< wsm->getNodeID()
						<< ", total: " << peEmgRxPackets << endl;
			}
			else {
				// check if the sender's serial is larger than the existing one, if so, update
				if (rxSerMap.at(wsm->getNodeID()) < wsm->getSerial()){
					findHost()->getDisplayString().updateWith("r=8,LightBlue");

					rxSerMap.at(wsm->getNodeID()) = wsm->getSerial();
					
					simtime_t e2eDelay = simTime() - wsm->getTimestamp();
                    if ( wsm->getEmgReach2kFlag() ){
                        g_vPeEmg2kDelay.push_back(e2eDelay.dbl());
                    }

					peEmgRxPackets++;
					gPeRxPackets++;

					if (debugApp){
						std::cout << "APPL: " << findHost()->getFullName()
								<< " recvd EMG pkt. " << simTime() << endl;
					}

					DBG <<  findHost()->getFullName()
							<< " APPL: successfully receives a EMG pkt, "
							<< " from "<< wsm->getNodeID()
							<< ", total: " << peEmgRxPackets << endl;
				}
				else{
					gPeRxDuplPkts++;
				}
			}
			if ( peEmgPktRxTime == 0.0 )
				peEmgPktRxTime = simTime().dbl();

		} else {
			peRxDropPkts++;
			gPeRxDropPkts++;

			DBG << "APPL: wsm pkt is not for me, delete... " << peRxDropPkts << endl;
		}
	} else {
		DBG << "APPL: wsm pkt expired" << endl;
		gPeRxExpiPkts++;
	}
	delete(wsm);
}

void CMACApp11pTwoWay::onStatus(CMACWSM* wsm) {
//	findHost()->getDisplayString().updateWith("r=8,green");
//	annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), traci->getPositionAt(simTime()), "blue"));
	gPeTotalRxPkts++;
	if ( simTime() < (wsm->getTimestamp() + wsm->getTTL()) ){
		if ( strcmp(wsm->getDestID(), findHost()->getFullName()) == 0 ){
			// check rxSerMap if I had the sender's pkt
			if ( rxSerMap.find(wsm->getNodeID()) == rxSerMap.end() ){
				findHost()->getDisplayString().updateWith("r=8,LightBlue");

				rxSerMap.insert(std::pair<std::string, int>(wsm->getNodeID(), wsm->getSerial()));
				peRxPackets++;
				gPeRxPackets++;

				simtime_t e2eDelay = simTime() - wsm->getTimestamp();
				gPeE2EDelay.push_back(e2eDelay.dbl());

				DBG <<  findHost()->getFullName()
						<< " APPL: successfully receive a pkt, total: " << peRxPackets
						<< " from "<< wsm->getNodeID() << endl;
				if (debugApp){
					std::cout <<  findHost()->getFullName()
							<< " APPL: successfully receive a pkt, total: " << peRxPackets
							<< " from "<< wsm->getNodeID() << endl;
				}

			} else {
				// check if the sender's serial is larger than the existing one, if so, update
				if (rxSerMap.at(wsm->getNodeID()) < wsm->getSerial()){
					findHost()->getDisplayString().updateWith("r=8,LightBlue");

					rxSerMap.at(wsm->getNodeID()) = wsm->getSerial();
					peRxPackets++;
					gPeRxPackets++;

					simtime_t e2eDelay = simTime() - wsm->getTimestamp();
					gPeE2EDelay.push_back(e2eDelay.dbl());


					DBG <<  findHost()->getFullName()
							<< " APPL: successfully receive a pkt, total: " << peRxPackets
							<< " from "<< wsm->getNodeID() << endl;
					if (debugApp){
						std::cout <<  findHost()->getFullName()
								<< " APPL: successfully receive a pkt, total: " << peRxPackets
								<< " from "<< wsm->getNodeID() << endl;
					}
				} else {
					// if not, it is an old or duplicate pkt, ignore it.
					gPeRxDuplPkts++;
				}
			}

			if (debugApp){
				std::cout << findHost()->getFullName() << ", rxSerMap: " << endl;
				for (auto i:rxSerMap){
					std::cout << i.first << ": " << i.second << endl;
				}
				std::cout << endl;
			}


		}
		else if (strcmp(wsm->getDestID(), "emergency" ) == 0){

			if ( rxSerMap.find(wsm->getEmgPktSender()) == rxSerMap.end() ){
				findHost()->getDisplayString().updateWith("r=8,LightBlue");

				rxSerMap.insert(std::pair<std::string, int>(wsm->getEmgPktSender(), wsm->getSerial()));

				simtime_t e2eDelay = simTime() - wsm->getTimestamp();
				if ( wsm->getEmgReach2kFlag() ){
					g_vPeEmg2kDelay.push_back(e2eDelay.dbl());
				}

				peEmgRxPackets++;
				gPeRxPackets++;

				if (debugApp){
					std::cout << "APPL: " << findHost()->getFullName()
							<< " recvd EMG pkt. " << simTime() << endl;
				}

				DBG <<  findHost()->getFullName()
						<< " APPL: successfully receives a EMG pkt, "
						<< " from "<< wsm->getEmgPktSender()
						<< ", total: " << peEmgRxPackets << endl;
			} else {
				// check if the sender's serial is larger than the existing one, if so, update
				if (rxSerMap.at(wsm->getEmgPktSender()) < wsm->getSerial()){
					findHost()->getDisplayString().updateWith("r=8,LightBlue");

					rxSerMap.at(wsm->getEmgPktSender()) = wsm->getSerial();

					simtime_t e2eDelay = simTime() - wsm->getTimestamp();
					if ( wsm->getEmgReach2kFlag() ){
						g_vPeEmg2kDelay.push_back(e2eDelay.dbl());
					}

					peEmgRxPackets++;
					gPeRxPackets++;

					if (debugApp){
						std::cout << "APPL: " << findHost()->getFullName()
								<< " recvd EMG pkt. " << simTime() << endl;
					}

					DBG <<  findHost()->getFullName()
							<< " APPL: successfully receives a EMG pkt, "
							<< " from "<< wsm->getEmgPktSender()
							<< ", total: " << peEmgRxPackets << endl;
				} else{
					gPeRxDuplPkts++;
				}
			}
			if ( peEmgPktRxTime == 0.0 )
				peEmgPktRxTime = simTime().dbl();

		} else {
			peRxDropPkts++;
			gPeRxDropPkts++;

			DBG << "APPL: wsm pkt is not for me, delete... " << peRxDropPkts << endl;
		}
	} else {
		DBG << "APPL: wsm pkt expired" << endl;
		gPeRxExpiPkts++;
	}
	delete(wsm);
}

void CMACApp11pTwoWay::sendMessage(std::string blockedRoadId) {
	sentMessage = true;

	t_channel channel = dataOnSch ? type_SCH : type_CCH;
	CMACWSM* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
	wsm->setWsmData(blockedRoadId.c_str());

	// SCMAC
//	std::map<std::string, cModule*> allVehMap = traci->getManager()->getManagedHosts();

//	wsm->setDestID();

//	sendWSM(wsm);
}
void CMACApp11pTwoWay::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj) {
	Enter_Method_Silent();
	if (signalID == mobilityStateChangedSignal) {
		handlePositionUpdate(obj);
	}
	else if (signalID == parkingStateChangedSignal) {
//		handleParkingUpdate(obj);
	}
}
void CMACApp11pTwoWay::handleParkingUpdate(cObject* obj) {
	isParking = traci->getParkingState();
	if (sendWhileParking == false) {
		if (isParking == true) {
			(FindModule<BaseConnectionManager*>::findGlobalModule())->unregisterNic(this->getParentModule()->getSubmodule("nic"));
		} else {
			Coord pos = traci->getCurrentPosition();
			(FindModule<BaseConnectionManager*>::findGlobalModule())->registerNic(this->getParentModule()->getSubmodule("nic"), (ChannelAccess*) this->getParentModule()->getSubmodule("nic")->getSubmodule("phy80211p"), &pos);
		}
	}
}
void CMACApp11pTwoWay::handlePositionUpdate(cObject* obj) {
	CMacBaseWaveApplLayer::handlePositionUpdate(obj);

	// stopped for for at least 10s?
//	if (traci->getSpeed() < 1) {
//		if (simTime() - lastDroveAt >= 10) {
//			findHost()->getDisplayString().updateWith("r=16,red");
//			if (!sentMessage) sendMessage(traci->getRoadId());
//		}
//	}
//	else {
//		lastDroveAt = simTime();
//	}
}
void CMACApp11pTwoWay::sendWSM(CMACWSM* wsm) {
//	if (isParking && !sendWhileParking) return;
	sendDown(wsm);
}

CMACWSM* CMACApp11pTwoWay::prepareWSM(std::string name,
								int lengthBits,
								t_channel channel,
								int priority,
								int rcvId,
								int serial) {

	CMACWSM* wsm = new CMACWSM(name.c_str());

	if ( waveApp ){
		wsm->addBitLength(headerLength);
		wsm->addBitLength(lengthBits);
	}

	switch (channel) {
		case type_SCH: wsm->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
		case type_CCH: wsm->setChannelNumber(Channels::CCH); break;
	}
	wsm->setPsid(0);
	wsm->setPriority(priority);
	wsm->setWsmVersion(1);
	wsm->setTimestamp(simTime());
	wsm->setSenderAddress(myId);
	wsm->setRecipientAddress(rcvId);
	wsm->setSenderPos(curPosition);
	wsm->setSerial(serial);

	// SCMAC
//	std::map<std::string, cModule*> allVehMap = traci->getManager()->getManagedHosts();
//	std::map<std::string, cModule*>::iterator it;


    // for twoway case
	// select the newest car

    double newestNodePos = 0.0;
    std::string newestNodeId;
    std::vector<cModule*> allVehNode;
	std::string myCurrentEdgeID = traci->getCommandInterface()->getEdgeId(traci->getExternalId());

	if (vehMapEdgeId.find(myCurrentEdgeID) != vehMapEdgeId.end()){
	    allVehNode = vehMapEdgeId.at(myCurrentEdgeID);
        for (auto& j:allVehNode){
            Veins::TraCIMobility* jMob;
            jMob = (Veins::TraCIMobility*)j->getSubmodule("veinsmobility");
            double offset = jMob->getCommandInterface()->getLanePosition(jMob->getExternalId());
            if ( newestNodePos == 0.0 ){
                newestNodePos = offset;
                newestNodeId = j->getFullName();
            }
            else if ( newestNodePos > offset ){
                newestNodePos = offset;
                newestNodeId = j->getFullName();
            }
        }
	}
	else{
	    error("prepareWSM(): can't find the current edgeid");
	}


//	std::vector<cModule*> allVehNode;
//	for ( it = allVehMap.begin(); it != allVehMap.end(); ++it){
//		Veins::TraCIMobility* i_Mob;
//		i_Mob = (Veins::TraCIMobility*)it->second->getSubmodule("veinsmobility");
//
//		/* do not include vehicles going to disappear,
//		 * at the end of a road segment
//		 * */
//		if ( getOffsetRatio(i_Mob) <= 0.98 && getOffsetRatio(i_Mob) >= 0.02) {
//			allVehNode.push_back(it->second);
////			it->second->getDisplayString().updateWith("r=0");
//		}
//	}

/*debug*/
//	for ( size_t tt = 0; tt < allVehNodeID.size(); ++tt)
//		std::cout << allVehNodeID[tt] << " ";
//	std::cout << std::endl;
/*debug*/


	/*
	do {
		randVeh = allVehNode[intuniform(0, allVehNode.size()-1)];
		cModule* randVehMobi = randVeh->getModuleByPath(".veinsmobility");
		Veins::TraCIMobility* mobility = check_and_cast<Veins::TraCIMobility*>(randVehMobi);
		dist = mobility->getCurrentPosition().distance(curPosition);

	}while(dist > 500);
	*/

//	std::string vehIdNewest;
//	double vehNewestX = 0.0;
//	// search the newest vehicle entering current edge
//	for ( std::vector<cModule*>::iterator itt = allVehNode.begin(); itt != allVehNode.end(); ++itt ){
//
//        cModule* appModule = (*itt)->getModuleByPath(".appl");
//        CMACApp11pTwoWay* cmacApp = check_and_cast<CMACApp11pTwoWay*>(appModule);
//        double currentX = cmacApp->traci->getCurrentPosition().x;
//        if ( vehNewestX == 0.0 ){
//            vehNewestX = currentX;
//            vehIdNewest = cmacApp->findHost()->getFullName();
//        }
//        else if ( vehNewestX >= currentX ){
//            vehNewestX = currentX;
//            vehIdNewest = cmacApp->findHost()->getFullName();
//        }
//	}

    // dist limit 500 meter
    cModule* randVeh;
    double dist = 0.0;

//	std::cout << "vehIdNewest: " << vehIdNewest << endl;
	// no dist limit
	do{
		randVeh = allVehNode[intuniform(0, allVehNode.size()-1)];
	}while (strcmp(randVeh->getFullName(), findHost()->getFullName()) == 0);
//	std::cout << "Dest DIST: " << dist << std::endl;

	std::string destID = randVeh->getFullName();

	if ( !newestNodeId.empty() )
	    destID = newestNodeId;
	else
	    opp_error("vehIdNewest is empty!");

//	destID = "node[17]";
	if (sendEmgPkt){
		destID = "emergency";
	}
	else if ( sendRandPkt){
	    destID = randVeh->getFullName();
	}

	destIDdebug = destID;

	DBG << findHost()->getFullName() << " generates a packet to dest: "
			<< destID << ", dist: " << dist << std::endl;

	wsm->setDestID(destID.c_str());
	// Time to live in second
	wsm->setTTL(120);
	// Sender ID
	wsm->setNodeID(findHost()->getFullName());

	if (name == "beacon") {
		DBG << "Creating Beacon with Priority " << priority << " at Applayer at " << wsm->getTimestamp() << std::endl;
	}
	if (name == "data") {
		DBG << "Creating Data with Priority " << priority << " at Applayer at " << wsm->getTimestamp() << std::endl;
	}

	return wsm;
}

void CMACApp11pTwoWay::finish() {
	if (sendBeaconEvt->isScheduled()) {
		cancelAndDelete(sendBeaconEvt);
	} else {
		delete sendBeaconEvt;
	}

	if (sendDataEvt->isScheduled()) {
		cancelAndDelete(sendDataEvt);
	} else {
		delete sendDataEvt;
	}

	if (sendFloodEvt->isScheduled()) {
		cancelAndDelete(sendFloodEvt);
	} else {
		delete sendFloodEvt;
	}

	if (decideVehEvt->isScheduled()) {
		cancelAndDelete(decideVehEvt);
	} else {
		delete decideVehEvt;
	}

	if (transitEvt->isScheduled() ){
		cancelAndDelete(transitEvt);
	}else {
		delete transitEvt;
	}

    if (eventSendWSM->isScheduled() ){
        cancelAndDelete(eventSendWSM);
    }else {
        delete eventSendWSM;
    }

	findHost()->unsubscribe(mobilityStateChangedSignal, this);

	// PE
	/*
	 * print out results
	 * */
	if ( sendPkt || sendRandPkt ){
		if ( strcmp(findHost()->getFullName(), gLatestNodeId.c_str()) == 0 ){
			std::cout << findHost()->getFullName() << "APP: finish!" << endl;
			std::cout << "==============================================="<< endl;
			std::cout << "gPeTxPackets:" << gPeTxPackets << endl;
			std::cout << "gPeRxPackets:" << gPeRxPackets << endl;
			std::cout << "gPeRxDropPkts:" << gPeRxDropPkts << endl;
			std::cout << "gPeRxDuplPkts:" << gPeRxDuplPkts << endl;
			std::cout << "gPeTotalRxPkts:" << gPeTotalRxPkts << endl;
			std::cout << "gPeRxExpiPkts:" << gPeRxExpiPkts << endl;
			std::cout << "Successful Ratio:"
					<< (double)gPeRxPackets/(double)gPeTxPackets << endl;

			double max=0.0;
			double mean=0.0;
			if ( !gPeE2EDelay.empty() ){
			    auto maxResult = std::max_element(gPeE2EDelay.begin(), gPeE2EDelay.end());
                std::cout << "MaxE2Edelay:" << *maxResult << endl;
                max = *maxResult;

                double sum = 0.0;
                for (auto i:gPeE2EDelay){
                    sum += i;
                }
                mean = sum/gPeE2EDelay.size();
                std::cout << "MeanE2Edelay:" << mean << endl;
			}
			else {
			    std::cout << "gPeE2EDelay is empty! no successful packet delivery." << endl;
			}
//			std::cout << "PktE2Edelay: " << endl;
//			for ( size_t i = 0; i < gPeE2EDelay.size(); ++i ){
//				std::cout << gPeE2EDelay[i] << " ";
//				if ( (i+1) % 10 == 0)
//					std::cout << endl;
//			}
//			std::cout << endl
//					<< "===============================================" << endl;

//			gPeE2EDelay.clear();
			if ( appSaveToFile ){
                char str[100];
                if ( ecmacApp ){
                    if (b_backgroundTraffic){
                        sprintf(str, "result/ECMAC-%s-App-d-%d-bg-%.3f-s-%d.csv", schemeName.c_str(), density, d_bgTrafficInterval, seed);
                    }
                    else{
                        sprintf(str, "result/ECMAC-%s-App-d-%d-s-%d.csv", schemeName.c_str(), density, seed);
                    }
                }
                else if ( waveApp ){
                    sprintf(str, "result/WAVE_App_Result.csv");
                }
                else if ( dmmacApp ){
                    sprintf(str, "result/DMMACstmac/DMMAC_App_Result.csv");
                }

                std::ofstream data;
                data.open(str);
                if ( data.is_open()){
                    data << "scheme," << schemeName << endl;
                    data << "BGTI," << d_bgTrafficInterval << endl;
                    data << "seed," << seed << endl;
                    data << "density," << density << endl;
                    data << "gPeTxPackets," << gPeTxPackets << endl;
                    data << "gPeRxPackets," << gPeRxPackets << endl;
                    data << "gPeRxDropPkts," << gPeRxDropPkts << endl;
                    data << "PDR," << gPeRxPackets/(double)gPeTxPackets << endl;
                    data << "maxE2EDelay," << max << endl;
                    data << "meanE2EDelay," << mean << endl;
                    data << endl;

                    data.close();
                }
                else{
                    error("APP: can't open result file!");
                }
			}
		}
	}

	if ( sendEmgPkt ){

		if ( gPeEmgPktRecvTime < peEmgPktRxTime )
			gPeEmgPktRecvTime = peEmgPktRxTime;

		gPeEmgRxPktVec.push_back(peEmgRxPackets);

		if ( strcmp(findHost()->getFullName(), gLatestNodeId.c_str()) == 0 ){



			for ( size_t i = 0; i < gPeEmgRxPktVec.size(); ++i ){
				std::cout << gPeEmgRxPktVec[i] << " ";
				if ( (i+1) % 10 == 0)
					std::cout << endl;
			}

			std::cout << endl << "==============================================="<< endl;

			double mean;
			double maxResultD;

			if ( g_vPeEmg2kDelay.empty() ){
				maxResultD = 0.0;
				mean = 0.0;
			}
			else{
				auto maxResult = std::max_element(g_vPeEmg2kDelay.begin(), g_vPeEmg2kDelay.end());
				maxResultD = *maxResult;

				double sum = 0.0;
				for (auto i:g_vPeEmg2kDelay){
					sum += i;
				}
				mean = sum/g_vPeEmg2kDelay.size();
			}
			std::cout << "MaxEmgE2Edelay: " << maxResultD << endl;

			std::cout << "MeanEmgE2Edelay: " << mean << endl;

			std::cout << findHost()->getFullName() << " APP: finish!" << endl;
			std::cout << "==============================================="<< endl;
			std::cout << "gPeTxPackets: " << gPeTxPackets << endl;
			std::cout << "gPeRxPackets: " << gPeRxPackets << endl;
			std::cout << "peEmgTXPackets: " << peEmgTXPackets << endl;
			std::cout << "peEmgRxPackets: " << peEmgRxPackets << endl;
			// std::cout << "gPeEmgPktSentTime: " << gPeEmgPktSentTime << endl;
			// std::cout << "gPeEmgPktRecvTime: " << gPeEmgPktRecvTime << endl;
			// std::cout << "Flood E2E delay: " << gPeEmgPktRecvTime - gPeEmgPktSentTime << endl;
			std::cout << "EmgPktReach2kmRatio: " << g_vPeEmg2kDelay.size()/(double)g_iEmgPktSendPlan << endl;
			std::cout << "EmgPktReach2kmDelay," << mean << endl;
			std::cout << "EmgPktReach2kmMaxDelay," << maxResultD << endl;
			// std::cout << "Num of Rx Emg Pkt in each vehicle: " << endl;

			if ( appSaveToFile ){
                char str[100];
                if ( ecmacApp ){
                    sprintf(str, "result/ECMAC-%s-App-d-%d-s-%d.csv", schemeName.c_str(), density, seed);
                }
                else if ( waveApp ){
                    sprintf(str, "result/WAVE_App_Flood_Result.csv");
                }
                else if ( dmmacApp ){
    //				sprintf(str, "result/DMMAC_App_Flood_Result.csv");
                    sprintf(str, "result/DMMAC_App_2KM_Result.csv");
				}

                std::ofstream data;
                data.open(str);
                data << "EmgPktSentTime,"<< gPeEmgPktSentTime << endl;
                data << "EmgPktRecvTime,"<<gPeEmgPktRecvTime << endl;
                data << "EmgPktE2EDelay,"<< gPeEmgPktRecvTime - gPeEmgPktSentTime << endl;
                data << "schemeName," << schemeName << endl;
                data << "seed," << seed << endl;
                data << "density," << density << endl;
                data << "PDR," << g_vPeEmg2kDelay.size()/(double)g_iEmgPktSendPlan << endl;
                data << "meanE2EDelay," << mean << endl;
                data << "maxE2EDelay," << maxResultD << endl;

    //			for ( size_t i = 0; i < gPeEmgRxPktVec.size(); ++i ){
    //				data << gPeEmgRxPktVec[i] << std::endl;
    //			}

                data.close();
			}
		}
	}

}

double CMACApp11pTwoWay::getOffsetRatio(Veins::TraCIMobility* vehMob) {
	std::string nodeID = vehMob->getExternalId();
	std::string laneID = vehMob->getCommandInterface()->getLaneId(nodeID);
	double laneLength = vehMob->getCommandInterface()->getLaneLength(laneID);
	double laneOffset = vehMob->getCommandInterface()->getLanePosition(nodeID);

	return laneOffset/laneLength;
}
