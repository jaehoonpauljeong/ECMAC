/*
 * MinLatencyTreeNode.h
 *
 *  Created on: Jun 29, 2015
 *      Author: jinho
 */

#ifndef MINLATENCYTREENODE_H_
#define MINLATENCYTREENODE_H_

//#include <Consts80211p.h>

//namespace ScmacNode {

class MinLatencyTreeNode {
public:
    void setNodeId(int id);
    void setParentId(int id);
    void setLocation(double x, double y);
    void resetParentId(void);
    void setTransmissionPower(double txDist);
    void setNeighbor(int neighid);
    void resetNeighbor(void);
    void setPath(int layer, MinLatencyTreeNode* parent);
    void setClusterId(int id);
    void setDist2Root(int dist);
    void settimeslot(int ts);
    void setDegree();
    void setChannel(int ch);
    int findNodeID(double x, double y);
    int getNodeId();
    int getParentId();
    int getClusterId();
    int getDist2Root();
    int getChannel();
    double getXLocation();
    double getYLocation();
    double getTransmissionPower();
    int getDegree();
    int gettimeslot();
    int sizeofNeighTable();
    int checkNeighbor(int neighid);

    MinLatencyTreeNode(){
        parentid = -1;
        timeslot = -1;
        clusterid = -1;
        channel = -1;
        neighTable = new int[1000];
    };

    virtual ~MinLatencyTreeNode(){
//        delete neighTable;
    };

    int* neighTable;
    MinLatencyTreeNode* pathToRoot;

private:
    int nodeid;
    int parentid;
    int timeslot;
    int clusterid;
    int channel;
    int dist2root;
    double xlocation;
    double ylocation;
    double txPower;
    int degree;

};

//} /* namespace Scmac */

#endif /* MINLATENCYTREENODE_H_ */
