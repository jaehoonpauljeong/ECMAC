/*
 * MinLatencyTreeNode.cpp
 *
 *  Created on: Jun 29, 2015
 *      Author: jinho
 */

#include <MinLatencyTreeNode.h>

namespace ScmacNode {

void MinLatencyTreeNode::setNodeId(int id){
    nodeid = id;
}

void MinLatencyTreeNode::setParentId(int id){
    parentid = id;
}

void MinLatencyTreeNode::resetParentId(void){
    parentid = -1;
}

void MinLatencyTreeNode::setLocation(double x, double y){
    xlocation = x;
    ylocation = y;
}

void MinLatencyTreeNode::setNeighbor(int neighid){
    neighTable[neighid] = 1;
}

void MinLatencyTreeNode::resetNeighbor(void){
    for (int i=0; i<1000; i++)
        neighTable[i] = 0;
}

void MinLatencyTreeNode::setClusterId(int id){
    clusterid = id;
}

void MinLatencyTreeNode::setPath(int layer, MinLatencyTreeNode* parent){
    pathToRoot[layer] = *parent;
}

void MinLatencyTreeNode::setTransmissionPower(double txDist){
    txPower = txDist;
}

void MinLatencyTreeNode::setChannel(int ch){
    channel = ch;
}

int MinLatencyTreeNode::getChannel(){
    return channel;
}

int MinLatencyTreeNode::getNodeId(){
    return nodeid;
}

int MinLatencyTreeNode::getParentId(){
    return parentid;
}

int MinLatencyTreeNode::getClusterId(){
    return clusterid;
}

double MinLatencyTreeNode::getXLocation(){
    return xlocation;
}

double MinLatencyTreeNode::getYLocation(){
    return ylocation;
}

double MinLatencyTreeNode::getTransmissionPower(){
    return txPower;
}

int MinLatencyTreeNode::sizeofNeighTable(){
    return sizeof(neighTable);
}

int MinLatencyTreeNode::checkNeighbor(int neighid){
    return neighTable[neighid];
}

} /* namespace Scmac */
