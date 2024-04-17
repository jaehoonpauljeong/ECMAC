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

#ifndef CMacRSUAppl11p_H
#define CMacRSUAppl11p_H

#include "CMacBaseWaveApplLayer.h"
#include "modules/world/annotations/AnnotationManager.h"

//#include "modules/mobility/traci/TraCIMobility.h"

//using Veins::TraCIMobility;
using Veins::AnnotationManager;

/**
 * Small RSU Demo using 11p
 */
class CMacRSUAppl11p : public CMacBaseWaveApplLayer {
	public:
		virtual void initialize(int stage);
	protected:
		AnnotationManager* annotations;
		BaseMobility* mobi;
//		TraCIMobility* mobi;
		bool sentMessage;

		// Chris
		cMessage* sendDataEvt;

		// Chris end

	protected:
		virtual void onBeacon(CMACWSM* wsm);
		virtual void onData(CMACWSM* wsm);
		void sendMessage(std::string blockedRoadId);
		virtual void sendWSM(CMACWSM* wsm);

		// Chris
		virtual void handleSelfMsg(cMessage* msg);

		// Chris end
};

#endif
