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

package org.car2x.veins.modules.application.traci;
import org.car2x.veins.modules.application.ieee80211p.CMacBaseWaveApplLayer;

simple CMACApp11p extends CMacBaseWaveApplLayer
{
    @class(CMACApp11p);
    @display("i=block/app2");
    bool sendWhileParking  = default(false); //send messages when the car is parked
    
    // SCMAC
    bool appSaveToFile = default(true);
    bool debugApp = default(false);
    
    bool ecmacApp = default(true);
    bool dmmacApp = default(false);
    bool waveApp=default(false);
    
    bool sendPkt = default(false);
    bool sendRandPkt = default(false);
    bool sendEmgPkt = default(false);
    double pktSendEndTime = default(0);
    double pktSendInterval = default(1);
    
    bool appBGTraffic = default(false);
    double appBGTrafficInterval = default(0.1);
}
