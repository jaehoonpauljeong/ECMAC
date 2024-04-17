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

struct ClusterRef{
	Coord center;
	int distance;
	int channel;
};

packet Initiating
{
    ClusterRef refer[];
    int optimalClusterSize;
    int distance;
    Coord senderCoord;
    
    string sendID;
}