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
}}

class noncobject Coord;

struct CMMobility{
	Coord currentCoord;
	int speed;
};

packet Join
{
    int senderAddress = 0;
    int receiverAddress = 0;
    int clusterIndex;
    CMMobility senderMobility;
    Coord CHCoord;
    int optimalClusterSize;
    
    string sendID;
    
}