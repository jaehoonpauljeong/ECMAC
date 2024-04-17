/*
 * MinLatencyTreeOptimalTree.h
 *
 *  Created on: Jun 29, 2015
 *      Author: jinho
 */
#include "MinLatencyTreeNode.h"
#include <vector>
#include <iostream>
#include <algorithm>    // std::sort
#include <queue>
#include <cmath>
#include <limits.h>
#include <stdio.h>
#include <string.h>

#ifndef MINLATENCYTREEOPTIMALTREE_H_
#define MINLATENCYTREEOPTIMALTREE_H_
using namespace std;

namespace ScmacTree {

class MinLatencyTreeOptimalTree {
private:
    int networkMaxFlow;
    int numClusters;

public:
    int optTreeDepth( int numNodes );
    int optNumChildren( int numNodes , int optd);
    int parentFinder(ScmacNode::MinLatencyTreeNode* nodeSet, int numNodes, int numChildNode);
    double getOptDeley( int numNodes , int optd);
    double getMaxDeley( int numNodes ,int numChild, int optd);
    int getNumClusters();
    void setClusterHead(int ch, int nodeid);
    void setNumClusters(int nch);
    int bfs(int numCluster, vector< vector<int> > rGraph, int s, int t, int parent[]);
    void dfs(int numCluster, vector< vector<int> > rGraph, int s, bool visited[]);
    vector<int> minCut(int numCluster, vector< vector<int> > graph, int s, int t);
    vector<int> maxFirstCut(int K, int numCluster, vector< vector<int> > graph, int numNodes);
    vector<int> maxKCut(int K, int numCluster, vector< vector<int> > graph, int numNodes);
    void setMaxFlow(int maxFlow);
    void clusterChannelAssign(ScmacNode::MinLatencyTreeNode* nodeSet,int numNodes, int numAvailChannels);

    MinLatencyTreeOptimalTree(){
        //clusterhead = new int[numNodes];// This is the constructor
    }

    MinLatencyTreeOptimalTree(int numNodes){
         clusterhead = new int[numNodes];// This is the constructor
     }

    virtual ~MinLatencyTreeOptimalTree(){
        //delete clusterhead;// This is the constructor
    }

    int* clusterhead;

};

} /* namespace ScmacTree */

#endif /* MINLATENCYTREEOPTIMALTREE_H_ */
