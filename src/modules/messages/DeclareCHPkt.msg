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
#include "InitiatingPkt_m.h"
}}

class Initiating;

class noncobject Coord;

packet DeclareCH extends Initiating
{
	int senderAddress = 0;
//	Coord CHCoord;
	int clusterIndex;
	int sequence;
}