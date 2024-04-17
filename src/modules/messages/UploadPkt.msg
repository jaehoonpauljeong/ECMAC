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
#include "CMACWSM_m.h"
}}

class CMACWSM;

class noncobject Coord;

//
// Defines all fields of an 802.11 MAC frame
//
packet Upload extends CMACWSM
{
    // SCMAC specifics
    string nodeID;
    double speed;
    Coord position;
}
