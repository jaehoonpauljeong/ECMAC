/*
 * MinLatencyTreeNetwork.cpp
 *
 *  Created on: Jun 29, 2015
 *      Author: jinho
 */

#include <MinLatencyTreeNetwork.h>
#include <cmath>
#include <algorithm>    // std::sort

namespace ScmacNetwork {

int MinLatencyTreeNetwork::getNeighbor (ScmacNode::MinLatencyTreeNode* node, int numNodes, int nodeid){
    double dist, xdist, ydist;
    for ( int i=0;i<numNodes;i++)
    {
        xdist = pow(node[i].getXLocation()-node[nodeid].getXLocation(),2);
        ydist = pow(node[i].getYLocation()-node[nodeid].getYLocation(),2);
        dist = std::sqrt(xdist+ydist);
        if (dist <= node[nodeid].getTransmissionPower())
        {
            node[nodeid].setNeighbor(i);
        }
    }
    return 0;
}

void MinLatencyTreeNetwork::resetNeighbor (ScmacNode::MinLatencyTreeNode* node, int size_f, int nodeid)
{
    for (int i=0;i<size_f;i++)
    {
        node[nodeid].neighTable[i] = 0;
    }
}

// Chris
int MinLatencyTreeNetwork::getNumNode (){
    return numberNodes;
}

void MinLatencyTreeNetwork::setNumNode (int numNodes){
    numberNodes = numNodes;
}
} /* namespace Scmac */
