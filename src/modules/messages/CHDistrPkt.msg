//**************************************************************************
// * file:        CMac80211Pkt.msg
// *
// * author:      Marc L�bbers
// * copyright:   (c) by 'the Glimmer Twins'
// *              Telecommunication Networks Group
// *              TU-Berlin
// * email:       loebbers@tkn.tu-berlin.de
// **************************************************************************
// * part of:     framework implementation developed by tkn
// *************************************************************************/

cplusplus {{
#include "Coord.h"
//#include "CHtoCHPkt_m.h"
}}

//class CHtoCH;
class noncobject Coord;

// same as CMSafetyInfo
struct CMPktInfo{
    string nodeID;
	Coord currentCoord;
	double speed;
	
	simtime_t ttl;
	string destID;
	simtime_t timeStamp;
	int serial;
};

packet CHDistr
{
    // SCMAC specifics
    string nodeID;
    CMPktInfo myCMList[];
    CMPktInfo orCMList[];
}
