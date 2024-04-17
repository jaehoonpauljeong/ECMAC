/*
 * MinLatencyTreeNetwork.h
 *
 *  Created on: Jun 29, 2015
 *      Author: jinho
 */
#include "MinLatencyTreeNode.h"

#ifndef MINLATENCYTREENETWORK_H_
#define MINLATENCYTREENETWORK_H_

namespace ScmacNetwork {

class MinLatencyTreeNetwork {
public:
    //int locateNode (int nodeid);
    int getNeighbor (ScmacNode::MinLatencyTreeNode* node, int numNodes, int nodeid);
    //int getNodeID (void);
    int getNumNode (void);
    void resetNeighbor (ScmacNode::MinLatencyTreeNode* node, int size_f, int nodeid);
    void setNumNode (int numberNode);

    MinLatencyTreeNetwork();
    virtual ~MinLatencyTreeNetwork();

private:
    int numberNodes;
};

} /* namespace Scmac */

#endif /* MINLATENCYTREENETWORK_H_ */
