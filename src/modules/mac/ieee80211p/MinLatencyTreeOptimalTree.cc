/*
 * MinLatencyTreeOptimalTree.cpp
 *
 *  Created on: Jun 29, 2015
 *      Author: jinho
 */

#include <MinLatencyTreeOptimalTree.h>


namespace ScmacTree {

int MinLatencyTreeOptimalTree::optTreeDepth( int numNodes )
{
    int optd;
    double lowBound = log(numNodes)/2;
    double upperBound = (log(numNodes)+1)/2;

    int roundupLowBound = int(lowBound);
    int rounddownUpperBound = int(upperBound);

    optd = roundupLowBound;
    if (roundupLowBound - rounddownUpperBound >= 1)
    {
        cout << "LowBound is " << roundupLowBound << " upperBound is " << rounddownUpperBound  << endl;
    }
    return optd;
}

int MinLatencyTreeOptimalTree::optNumChildren( int numNodes , int optd)
{
    int optc;

    return (int)(pow(numNodes,(1/(double)optd))+1);
}

void MinLatencyTreeOptimalTree::setClusterHead(int ch, int nodeid)
{
    clusterhead[nodeid] = ch;
}

void MinLatencyTreeOptimalTree::setNumClusters(int nch)
{
    numClusters = nch;
}


int MinLatencyTreeOptimalTree::getNumClusters(){
    return numClusters;
}

/*---------------------------
 * OptTree Construction Algorithm TopDown Approach.
 * 1. Sort all nodes based on their x-coordinate
 * 2. divide the nodes into numChildNode clusters based on their x-coordinate
 * 3. remove all edges connecting any two nodes within a same cluster
 * 4. Select a node nearest to the center of area
 * 5. For all nodes in a center cluster, calculate their scores based on the connection to other clusters.
 * 6. Select a node with highest score as a root and its child as potential parent nodes
 * 7.
 * done outside class : 5. if depthTree > 2 then we repeat from step one with a set of ClusterHeads as leaves.
 */
int MinLatencyTreeOptimalTree::parentFinder(ScmacNode::MinLatencyTreeNode* nodeSet, int numNodes, int numChildNode)
{
    std::vector<double> sortVehX;
    std::vector<int> sortVehId;
    std::vector<int> clusterIndex;
    //Step 1: get x locations of all nodes
    for (int i=0; i<numNodes; i++)
    {
        sortVehX.push_back(nodeSet[i].getXLocation());
        sortVehId.push_back(i);
        clusterIndex.push_back(-1);
    }

    //bubble sort
    bool swapped = true;
    int tempBig, tempSmall;
    int tempBigNode, tempSmallNode;
    while (swapped == true)
    {
        swapped = false;
        for (int i=1; i<sortVehX.size() ; i++)
        {
            if (sortVehX[i-1] > sortVehX[i])
            {
                //swap weight
                tempBig = sortVehX[i-1];
                tempSmall = sortVehX[i];
                sortVehX[i-1] = tempSmall;
                sortVehX[i] = tempBig;
                //swap edgeFrom
                tempBigNode = sortVehId[i-1];
                tempSmallNode = sortVehId[i];
                sortVehId[i-1] = tempSmallNode;
                sortVehId[i] = tempBigNode;
                swapped = true;
            }
        }
    }

    std::cout << "Node ClusterIndex contains: " ;
        for (int i=0; i<numNodes; i++)
            std::cout << ' ' << sortVehX[i];
        std::cout << '\n';

    std::cout << "Node ClusterIndex contains: " ;
        for (int i=0; i<numNodes; i++)
            std::cout << ' ' << sortVehId[i];
        std::cout << '\n';

    //sort (myvector.begin(), myvector.end());
    //Step 2: form clusters of size equal to numChildNode
    std::vector<int> clusterCoverList;
    std::vector<int> clusterHeadList;
    int count = 1;
    int cluster = 0;
    int numCluster = 0;
    if (numNodes <= numChildNode){
        clusterCoverList.push_back(0);
        clusterHeadList.push_back(-1);
        for (int i=0; i<numNodes; i++)
        {
            clusterIndex[sortVehId[i]]=cluster;
            nodeSet[sortVehId[i]].setClusterId(cluster);
        }
    }else{
        for (int i=0; i<numNodes; i++)
        {
            clusterIndex[sortVehId[i]]=cluster;
            nodeSet[sortVehId[i]].setClusterId(cluster);

            if (count % numChildNode == 0)
            {
                clusterCoverList.push_back(0);
                clusterHeadList.push_back(-1);
                cluster++;
            }

            count++;
        }
    }
    numCluster = cluster+1; //+1 since clusterid start from 0
    //Step 3: Iteratively form tree from root
    std::vector<int> candidateList;
    std::vector<int> parentList;
    int root = (int)ceil(numNodes/2);
    parentList.push_back(root);

    for (int i=0; i<numNodes; i++)
    {
        if (nodeSet[root].checkNeighbor(i) == 1)
        {
            candidateList.push_back(i);
            clusterCoverList[clusterIndex[i]]=1;

        }
    }
    std::cout << root << " candidateList contains: " ;
    for (unsigned i=0; i<candidateList.size(); i++)
    {
        std::cout << ' ' << candidateList[i];
    }
    std::cout << '\n';
    //Step 3.2: get score of each candidate and select the one with max score
    int clusterMem;
    int totalScore = 0;
    int maxScore = 0;
    int coverCnt = 0;
    int remainCluster = clusterCoverList.size();
    unsigned int maxParent = root;
    std::vector<int>::iterator it;
    while (remainCluster > 0)
    {
        maxScore = 0;
        //cout << "maxParent :" << maxParent << " remainCluster :" << remainCluster << endl;
        for (int ii=0; ii<candidateList.size(); ii++)
        {
            totalScore = 0;
            for (int j=0; j<numNodes; j++)
            {
                if (nodeSet[(int)candidateList[ii]].checkNeighbor(j) == 1)
                {
                    clusterMem = clusterIndex[j];
                    if (clusterCoverList[clusterMem] == 0){
                        totalScore++;
                    }
                }
            }

            if (maxScore < totalScore)
            {
                maxScore = totalScore;
                maxParent = candidateList[ii];
            }
        }
        parentList.push_back(maxParent);
        //Step 3.3: Update ClusterCoverList wrt maxParent
        for (int j=0; j<numNodes; j++)
        {
            if (nodeSet[(int)maxParent].checkNeighbor(j) == 1)
            {
                clusterMem = clusterIndex[j];
                clusterCoverList[clusterMem] = 1;
                //Step 3.5: Update candidateList with neighbor of maxParent
                it = find (candidateList.begin(), candidateList.end(), j);
                if (it == candidateList.end())
                {
                    candidateList.push_back(j);
                }
            }
        }

        std::cout << "new candidateList contains: " ;
        for (unsigned i=0; i<candidateList.size(); i++)
        {
            std::cout << ' ' << candidateList[i];
        }
        std::cout << '\n';
        //Step 3.4: check if new cluster is covered or all cluster is covered
        coverCnt = 0;
        for (int k=0; k<clusterCoverList.size(); k++)
        {
            if (clusterCoverList[k] == 0)
            {
                coverCnt++;
            }
        }

        if (remainCluster == coverCnt)
        {
            break;
        }
        remainCluster = coverCnt;
    }
    cout << "maxParent :" << maxParent << " remainCluster :" << remainCluster << endl;
    std::cout << root << " parentList contains: " ;
    for (unsigned i=0; i<parentList.size(); i++)
    {
        std::cout << ' ' << parentList[i];
    }
    std::cout << '\n';

    //Step 4: Cluster head selection
    //Step 4.0: nodes in the parentList select their parent from parentList
    int tempRoot=root;
    nodeSet[tempRoot].setParentId(root);

    for (int tp=0; tp<parentList.size(); tp++)
    {
        for (int i=0; i<parentList.size(); i++)
        {
            if(nodeSet[tempRoot].checkNeighbor(parentList[i])==1)
            {
                if (nodeSet[parentList[i]].getParentId() == -1)
                    nodeSet[parentList[i]].setParentId(tempRoot);
            }
        }
        tempRoot = parentList[tp];
    }

    int parent;
    //Step 4.1: nodes in the parentList automatically become cluster head of associated cluster.
    for (int i=0; i<parentList.size(); i++)
    {
        parent = parentList[i];
        clusterHeadList[clusterIndex[parent]]=parent;
        clusterCoverList[clusterIndex[parent]]=2;//variable 2 indicates cluster head being found
        for (int j=0; j<numNodes; j++)
        {
            if (clusterIndex[j] == clusterIndex[parent])
            {
                if (nodeSet[j].getParentId() == -1)
                    nodeSet[j].setParentId(parent);
            }
        }
    }

    std::cout << "Node Parent before contains: " ;
    for (int i=0; i<numNodes; i++)
        std::cout << ' ' << nodeSet[i].getParentId();
    std::cout << '\n';
    //Step 4.2: find the cluster head for rest of clusters
    for (int k=0; k<numNodes; k++)
    {
        if (clusterCoverList[clusterIndex[k]] != 2)
        {
            for (int i=0; i<parentList.size(); i++)
            {
                if (nodeSet[k].checkNeighbor(parentList[i])==1)
                {
                    clusterHeadList[clusterIndex[k]]=k;
                    nodeSet[k].setParentId(parentList[i]);
                    clusterCoverList[clusterIndex[k]]=2;
                    break;
                }
            }
        }
    }

    std::cout << "clusterHeadList after contains : " ;
    for (unsigned i=0; i<clusterHeadList.size(); i++)
        std::cout << ' ' << clusterHeadList[i];
    std::cout << '\n';

    //Step 4.3: assign nodes to their discovered cluster heads
    for (int i=0; i<clusterHeadList.size(); i++)
    {
        for (int k=0; k<numNodes; k++)
        {
            if (clusterIndex[k] == clusterIndex[clusterHeadList[i]])
            {
                if (nodeSet[k].getParentId() == -1)
                    nodeSet[k].setParentId(clusterHeadList[i]);
            }
        }
    }
    std::cout << "Node Parent after contains: " ;
    for (int i=0; i<numNodes; i++)
        std::cout << ' ' << nodeSet[i].getParentId();
    std::cout << '\n';


    std::cout << "Node ClusterIndex contains: " ;
    for (int i=0; i<numNodes; i++)
        std::cout << ' ' << nodeSet[i].getClusterId();
    std::cout << '\n';

    setNumClusters(numCluster);

    return root;
}

double MinLatencyTreeOptimalTree::getOptDeley( int numNodes , int optd)
{
    return pow(optd,2)* pow(numNodes,(1/(double)optd)) - optd;
}

double MinLatencyTreeOptimalTree::getMaxDeley( int numNodes ,int numChild, int optd){
    double delay = numChild;
    double remainNode;
    for (int i=0; i<optd; i++){
        remainNode = ceil(remainNode/numChild);

        if (remainNode < numChild)
        {
            return (i+1)*(delay + remainNode);
        }

        delay = delay + remainNode;
    }
    return optd*delay;
}

/* Returns true if there is a path from source 's' to sink 't' in
  residual graph. Also fills parent[] to store the path */
int MinLatencyTreeOptimalTree::bfs(int numCluster, vector< vector<int> > rGraph, int s, int t, int parent[])
{
    // Create a visited array and mark all vertices as not visited
    bool visited[numCluster];
    memset(visited, 0, sizeof(visited));

    // Create a queue, enqueue source vertex and mark source vertex
    // as visited
    queue <int> q;
    q.push(s);
    visited[s] = true;
    parent[s] = -1;

    // Standard BFS Loop
    while (!q.empty())
    {
        int u = q.front();
        q.pop();

        for (int v=0; v<numCluster; v++)
        {
            if (visited[v]==false && rGraph[u][v] > 0)
            {
                q.push(v);
                parent[v] = u;
                visited[v] = true;
            }
        }
    }

    // If we reached sink in BFS starting from source, then return
    // true, else false
    return (visited[t] == true);
}

// A DFS based function to find all reachable vertices from s.  The function
// marks visited[i] as true if i is reachable from s.  The initial values in
// visited[] must be false. We can also use BFS to find reachable vertices
void MinLatencyTreeOptimalTree::dfs(int numCluster, vector< vector<int> > rGraph, int s, bool visited[])
{
    visited[s] = true;
    for (int i = 0; i < numCluster; i++)
        if (rGraph[s][i] && !visited[i])
            dfs(numCluster,rGraph, i, visited);
}

void MinLatencyTreeOptimalTree::setMaxFlow(int maxFlow)
{
    networkMaxFlow = maxFlow;
}
#define V 100
vector<int> MinLatencyTreeOptimalTree::minCut(int numCluster, vector< vector<int> > graph, int s, int t)
{
    int u, v;

    // Create a residual graph and fill the residual graph with
    // given capacities in the original graph as residual capacities
    // in residual graph
    vector< vector<int> > rGraph; // rGraph[i][j] indicates residual capacity of edge i-j
    //rGraph.resize(numCluster);  // resize top level vector
    //for (u = 0; u < numCluster; u++){
    //  rGraph.resize(numCluster);  // resize top level vector
    //    for (v = 0; v < numCluster; v++)
    //         rGraph[u][v] = graph[u][v];
    //}
    rGraph = graph;
    int parent[V];  // This array is filled by BFS and to store path

    // Augment the flow while tere is path from source to sink
    while (bfs(numCluster,rGraph, s, t, parent))
    {
        // Find minimum residual capacity of the edhes along the
        // path filled by BFS. Or we can say find the maximum flow
        // through the path found.
        int path_flow = INT_MAX;
        for (v=t; v!=s; v=parent[v])
        {
            u = parent[v];
            path_flow = min(path_flow, rGraph[u][v]);
        }

        // update residual capacities of the edges and reverse edges
        // along the path
        for (v=t; v != s; v=parent[v])
        {
            u = parent[v];
            rGraph[u][v] -= path_flow;
            rGraph[v][u] += path_flow;
        }
    }

    // Flow is maximum now, find vertices reachable from s
    bool visited[numCluster];
    memset(visited, false, sizeof(visited));
    dfs(numCluster,rGraph, s, visited);

    // Print all edges that are from a reachable vertex to
    // non-reachable vertex in the original graph
    int maxFlow = 0;;
    int weight;
    //sourceSideMinCutSet.clear();
    std::vector<int> sourceSideMinCutSet;

    for (int i = 0; i < numCluster; i++){
        for (int j = 0; j < numCluster; j++){
            if (visited[i]){
                sourceSideMinCutSet.push_back(i);
                if (!visited[j] && graph[i][j])
                {
                    cout << i << " - " << j << " weight: " << graph[i][j] << endl;
                    weight = graph[i][j];
                    maxFlow = maxFlow + weight;
                }
            }
        }
    }
    cout << "maxFlow: " << maxFlow << endl;
    std::vector<int>::iterator it;
    it = std::unique(sourceSideMinCutSet.begin(), sourceSideMinCutSet.end());
    sourceSideMinCutSet.resize( std::distance(sourceSideMinCutSet.begin(),it) );

    setMaxFlow(maxFlow);
    return sourceSideMinCutSet;
}


//Simple but effective solution which try to utilize all available channels while minimizing the inter-cluster interferences
vector<int> MinLatencyTreeOptimalTree::maxFirstCut(int K, int numCluster, vector< vector<int> > graph, int numNodes)
{
//Step 1: Sort all the edges in ascending order of its weight and creat K empty sets C1,..,Ck
//Step 2: Select an edge (x,y) from highest weight and place the node x and y into associate set based on following conditions,
//Step 2.1: if x and y have no setIndexs in the other sets, place node x and y into different sets Ci with smallest size.
//Step 2.2: if x has a setIndex in a set Ci and y has no setIndex in the other set, place y into smallest Cj, where j =! i.
//Step 2.3: if y has a setIndex in a set Ci and x has no setIndex in the other set, place x into smallest Cj, where j =! i.
//Step 2.4: if both x and y have the setIndex, skip.

vector< vector<int> > kGraph;
    vector<int> edgeWeight;
    vector<int> edgeFrom;
    vector<int> edgeTo;
    kGraph = graph;
    for (int i = 0; i < numCluster; i++)
    {
        for (int j = 0; j < numCluster; j++)
        {
            if (graph[i][j] != 0){
                edgeWeight.push_back(graph[i][j]);
                edgeFrom.push_back(i);
                edgeTo.push_back(j);
            }
        }
    }
    //Step 1
    bool swapped = true;
    int tempBig, tempSmall;
    int tempBigFrom, tempSmallFrom;
    int tempBigTo, tempSmallTo;
    while (swapped == true)
    {
        swapped = false;
        for (int i=1; i<edgeWeight.size() ; i++)
        {
            if (edgeWeight[i-1] < edgeWeight[i])
            {
                //swap weight
                tempBig = edgeWeight[i-1];
                tempSmall = edgeWeight[i];
                edgeWeight[i-1] = tempSmall;
                edgeWeight[i] = tempBig;
                //swap edgeFrom
                tempBigFrom = edgeFrom[i-1];
                tempSmallFrom = edgeFrom[i];
                edgeFrom[i-1] = tempSmallFrom;
                edgeFrom[i] = tempBigFrom;
                //swap edgeTo
                tempBigTo = edgeTo[i-1];
                tempSmallTo = edgeTo[i];
                edgeTo[i-1] = tempSmallTo;
                edgeTo[i] = tempBigTo;
                swapped = true;
            }
        }
    }
    cout << "after sort" << endl;
    for (int j=0; j < edgeWeight.size(); j++)
        cout << edgeWeight[j] << ' ';
    cout << endl;
    for (int j=0; j < edgeWeight.size(); j++)
        cout << edgeFrom[j] << ' ';
    cout << endl;
    for (int j=0; j < edgeWeight.size(); j++)
        cout << edgeTo[j] << ' ';
    cout << endl;

    //Step 2:
    vector<int> setIndex;
    vector<int> setChannelWeight;
    int minChannelWeight;
    int setValid=0;
    int setValid2;
    int remainCluster = numCluster;
    for (int i=0; i<numCluster; i++){
        setIndex.push_back(-1);
    }

    for (int i=0; i<K; i++){
        setChannelWeight.push_back(0);
    }

    for (int j=0; j < edgeWeight.size(); j++)
    {
        cout << j << "channel " << setValid << ":: edgeFrom[j] " <<edgeFrom[j] << " edgeTo[j] " << edgeTo[j] << endl;

        //select available channel (setValid) with lowest utilization
        minChannelWeight = edgeWeight.size();
        for (int i=0; i<setChannelWeight.size(); i++){
            if (minChannelWeight > setChannelWeight[i])
                setValid = i;//since setValid start from 1 instead of 0
        }

        //Step 2.2:
        if (setIndex[edgeFrom[j]] == -1 && setIndex[edgeTo[j]] == -1){

            //setValid++;
            //if (setValid > K)
            //  setValid = 1;

            setIndex[edgeFrom[j]] = setValid;
            setChannelWeight[setValid]++;
            minChannelWeight = edgeWeight.size();
            for (int i=0; i<setChannelWeight.size(); i++){
                if (setValid != i){
                    if (minChannelWeight > setChannelWeight[i])
                        setValid2 = i;
                }
            }

            setIndex[edgeTo[j]] = setValid2;
            setChannelWeight[setValid2]++;
            remainCluster=remainCluster-2;
            cout << "case 1 from&to " << edgeFrom[j] << "and " << edgeTo[j] << endl;
        }
        //Step 2.2:
        else if (setIndex[edgeFrom[j]] == -1 && setIndex[edgeTo[j]] > -1){
            //setIndex[edgeFrom[j]] = setIndex[edgeTo[j]];
            setIndex[edgeFrom[j]] = setValid;
            setChannelWeight[setValid]++;
            remainCluster--;
            cout << "case 2 from " << edgeFrom[j] << endl;
        }

        //Step 2.3:
        else if (setIndex[edgeTo[j]] == -1 && setIndex[edgeFrom[j]] > -1){
            //setIndex[edgeTo[j]] = setIndex[edgeFrom[j]];
            setIndex[edgeTo[j]] = setValid;
            setChannelWeight[setValid]++;
            remainCluster--;
            cout << "case 3 to " << edgeTo[j] << endl;
        }

        if (remainCluster < 1)
            break;
    }

    cout << "remainCluster " << remainCluster << " setValid " << setValid << endl;
    for (int i=0; i<numCluster; i++)
        cout << setIndex[i] << ' ';
    cout << endl;

    return setIndex;
}

//Complicated but best known method for solving max k cut is http://www.math.cmu.edu/~af1p/Texfiles/cuts.pdf
vector<int> MinLatencyTreeOptimalTree::maxKCut(int K, int numCluster, vector< vector<int> > graph, int numNodes)
{
    //Step 1: Sort all the edges in ascending order of its weight and creat K empty sets C1,..,Ck
    //Step 2: Select an edge (x,y) from lightest and
    //          place the node x and y into associate set based on following conditions
    //Step 2.1: if x and y have no setIndexs in the other sets, place both node x and y into a same empty set
    //Step 2.2: if x has a setIndex in a set Ci and y has no setIndex in the other set, place x and y into Ci
    //Step 2.3: if y has a setIndex in a set Ci and x has no setIndex in the other set, place x and y into Ci
    //Step 2.4: if x has a setIndex in a set Ci and y has setIndex in Cj, i!=j, skip

    vector< vector<int> > kGraph;
    vector<int> edgeWeight;
    vector<int> edgeFrom;
    vector<int> edgeTo;
    kGraph = graph;
    for (int i = 0; i < numCluster; i++)
    {
        for (int j = 0; j < numCluster; j++)
        {
            if (graph[i][j] != 0){
                edgeWeight.push_back(graph[i][j]);
                edgeFrom.push_back(i);
                edgeTo.push_back(j);
            }
        }
    }
    //Step 1
    bool swapped = true;
    int tempBig, tempSmall;
    int tempBigFrom, tempSmallFrom;
    int tempBigTo, tempSmallTo;
    while (swapped == true)
    {
        swapped = false;
        for (int i=1; i<edgeWeight.size() ; i++)
        {
            if (edgeWeight[i-1] > edgeWeight[i])
            {
                //swap weight
                tempBig = edgeWeight[i-1];
                tempSmall = edgeWeight[i];
                edgeWeight[i-1] = tempSmall;
                edgeWeight[i] = tempBig;
                //swap edgeFrom
                tempBigFrom = edgeFrom[i-1];
                tempSmallFrom = edgeFrom[i];
                edgeFrom[i-1] = tempSmallFrom;
                edgeFrom[i] = tempBigFrom;
                //swap edgeTo
                tempBigTo = edgeTo[i-1];
                tempSmallTo = edgeTo[i];
                edgeTo[i-1] = tempSmallTo;
                edgeTo[i] = tempBigTo;
                swapped = true;
            }
        }
    }
    cout << "after sort" << endl;
    for (int j=0; j < edgeWeight.size(); j++)
        cout << edgeWeight[j] << ' ';
    cout << endl;
    for (int j=0; j < edgeWeight.size(); j++)
        cout << edgeFrom[j] << ' ';
    cout << endl;
    for (int j=0; j < edgeWeight.size(); j++)
        cout << edgeTo[j] << ' ';
    cout << endl;

    //Step 2:
    vector<int> setIndex;
    vector<int> setChannelWeight;
    int minChannelWeight = numNodes*numNodes;
    int setValid=1;
    int remainCluster = numCluster;
    int remainChannel = K;
    int mergTo;
    for (int i=0; i<numCluster; i++){
        setIndex.push_back(0);
    }

    for (int i=0; i<K; i++){
        setChannelWeight.push_back(0);
    }

    setIndex[edgeFrom[0]] = setValid; //assign set 1 to nodes on edge with min weight
    setIndex[edgeTo[0]] = setValid; //assign set 1 to nodes on edge with min weight
    setChannelWeight[setValid-1] = edgeWeight[0];
    minChannelWeight = 1;
    remainChannel--;
    remainCluster=remainCluster-2;
    for (int j=1; j < edgeWeight.size(); j++)
    {
        cout << j << "channel " << setValid << ":: edgeFrom[j] " <<edgeFrom[j] << " edgeTo[j] " << edgeTo[j] << endl;
        //if (remainCluster > K){
        if (remainChannel > 0){
            setValid++;
            remainChannel--;
        }else{
            for (int i=0; i<K; i++){
                if (setChannelWeight[setValid-1] > setChannelWeight[i])
                    setValid = i+1;//since setValid start from 1 instead of 0
            }
        }
        //Step 2.2:
        if (setIndex[edgeFrom[j]] == 0 && setIndex[edgeTo[j]] == 0){

            //setValid++;
            //if (setValid > K)
            //  setValid = 1;

            setIndex[edgeFrom[j]] = setValid;
            setIndex[edgeTo[j]] = setValid;
            setChannelWeight[setValid-1] = setChannelWeight[setValid-1] + edgeWeight[j];
            remainCluster=remainCluster-2;
            cout << "case 1 from&to " << edgeFrom[j] << "and " << edgeTo[j] << endl;
        }
        //Step 2.2:
        else if (setIndex[edgeFrom[j]] == 0 && setIndex[edgeTo[j]] != 0){
            //setIndex[edgeFrom[j]] = setIndex[edgeTo[j]];
            setIndex[edgeFrom[j]] = setValid;
            setChannelWeight[setValid-1] = setChannelWeight[setValid-1] + edgeWeight[j];
            remainCluster--;
            cout << "case 2 from " << edgeFrom[j] << endl;
        }

        //Step 2.3:
        else if (setIndex[edgeTo[j]] == 0 && setIndex[edgeFrom[j]] != 0){
            //setIndex[edgeTo[j]] = setIndex[edgeFrom[j]];
            setIndex[edgeTo[j]] = setValid;
            setChannelWeight[setValid-1] = setChannelWeight[setValid-1] + edgeWeight[j];
            remainCluster--;
            cout << "case 3 to " << edgeTo[j] << endl;
        }

        if (remainCluster < 1)
            break;
        /*
            //Step 2.4: merge one set into another
            else if (setIndex[edgeTo[j]] != 0 && setIndex[edgeFrom[j]] != 0){
                mergTo = setIndex[edgeFrom[j]];
                setIndex[edgeFrom[j]] = setIndex[edgeTo[j]];
                for (int i=0; i<numCluster; i++){
                    if (setIndex[edgeFrom[i]] == mergTo){
                        setIndex[edgeFrom[i]] = setIndex[edgeTo[j]];
                        cout << "case 4 merge " << edgeFrom[i] << endl;
                    }
                }
                //remainCluster--;

            }
         */
        /*}else{
            //Step 2.2:
            if (setIndex[edgeFrom[j]] == 0 && setIndex[edgeTo[j]] == 0){
                setValid++;
                setIndex[edgeFrom[j]] = setValid;
                setIndex[edgeTo[j]] = setValid;
            }
            //Step 2.2:
            if (setIndex[edgeFrom[j]] == 0 && setIndex[edgeTo[j]] != 0){
                setValid++;
                setIndex[edgeFrom[j]] = setValid;
            }

            //Step 2.3:
            if (setIndex[edgeTo[j]] == 0 && setIndex[edgeFrom[j]] != 0){
                setValid++;
                setIndex[edgeTo[j]] = setValid;
            }
        }
         */
    }

    /*
    //Step 3: This mean those remaining clusters disconnected. Therefore same channel is fine.
    setValid++;
    for (int i=0; i<numCluster; i++){
        if (setIndex[i] == 0){
            cout << i << " disconnect??" << endl;
            setIndex[i] = setValid;
        }
    }
     */
    cout << "remainCluster " << remainCluster << " setValid " << setValid << endl;
    for (int i=0; i<numCluster; i++)
        cout << setIndex[i] << ' ';
    cout << endl;

    return setIndex;
}

void MinLatencyTreeOptimalTree::clusterChannelAssign(ScmacNode::MinLatencyTreeNode* nodeSet,int numNodes, int numAvailChannels){
    // real number of cluster
	int numClusters = getNumClusters();
	// cGraph: intercluster interference
	// tGraph: same
    int cGraph[numClusters][numClusters];
    int tGraph[numClusters][numClusters];
    cout << "++++++++++numCluster++++++ " << numClusters << endl;
    cout << "++++++++++numNodes++++++ " << numNodes << endl;
    for (int i=0; i < numClusters ; i++){
       for (int j=0; j < numClusters ; j++){
           cGraph[i][j]=0;
           tGraph[i][j]=0;
       }
    }
    cout << "++++++++++cGraph++++++ " << endl;
    // calculate interference
    for (int i=0; i < numNodes ; i++){
        for (int j=0; j < numNodes ; j++){
        	// if it is interference
            if (nodeSet[i].neighTable[j]==1){
            	// if it is the same cluster
                if (nodeSet[i].getClusterId() != nodeSet[j].getClusterId()){
                	// if it has parent
                    if (nodeSet[i].getParentId() > -1 && nodeSet[j].getParentId() > -1){
                        cGraph[nodeSet[i].getClusterId()][nodeSet[j].getClusterId()]++;
                        tGraph[nodeSet[i].getClusterId()][nodeSet[j].getClusterId()]++;
                    }else{
                        //This space is for handling a node without a reach of any parent node
                        //Left empty until we find a solution for them.
                        //cout << "i:" << i << " ,j:" << j << "::" << nodeSet[i].getParentId() << "," << nodeSet[j].getParentId() << endl;
                    }
                }
            }
        }
    }

    int sum_i[numClusters];
    // aver interference, high average has real interference
    double avr_interfere[numClusters];
    // number of interference.
    int count_interfere[numClusters];

    int remain_cluster = numClusters;
    int ClusterChannel[numClusters];
    int remain_channel = numAvailChannels;
    for (int k=0; k < numClusters ; k++){
        ClusterChannel[k]=0;
    }
    while (remain_cluster > 0){
        //0. calculate average inter-cluster interferences.
        for (int i=0; i < numClusters ; i++){
            //cout <<  i << "::";
            sum_i[i] = 0;
            avr_interfere[i] = 0;
            count_interfere[i] = 0;
            for (int j=0; j < numClusters ; j++){
                //cout  << cGraph[i][j] << ' ';
                sum_i[i] = sum_i[i] + cGraph[i][j];
                if (cGraph[i][j] > 0){
                    count_interfere[i]++;
                }
            }
            if (sum_i[i] > 0){
                avr_interfere[i] = (double) (sum_i[i]/count_interfere[i]);
            }
            //cout << avr_interfere[i] << endl;
        }
        //1. find max average interference
        int max_cl=0, max_avr = 0;
        for (int k=0; k < numClusters ; k++){
            if (avr_interfere[k] > max_avr){
                max_cl = k;
                max_avr = avr_interfere[k];
            }
        }
        if (max_avr==0){//this mean all conflicting clusters are resolved
            break;
        }
        //2. find max edge in max_cl from cGraph by
        int max_edge=0, max_interfere = 0;
        for (int l=0; l < numClusters ; l++){
            if (cGraph[max_cl][l] > max_interfere){
                max_edge = l;
                max_interfere = cGraph[max_cl][l];
            }
        }
        cout << remain_cluster << "-max_cl: " << max_cl << ",max_edge: " << max_edge << endl;
        cout << "max_cl_avr: " << max_avr << ",max_edge_weight: " << max_interfere << endl;
        //3. assigning different channels to the nodes on that edge.
        // and remove it from array
        if (remain_channel <= 0){//if all available channels are used then return back to first used channel
            remain_channel = numAvailChannels;
        }
        if (ClusterChannel[max_cl] == 0){
            ClusterChannel[max_cl] = remain_channel;
            remain_cluster--;
            remain_channel--;
            //update other interfering clusters who has different channel to this
            for (int i=0; i < numClusters; i++){
                if (ClusterChannel[i] != 0){
                    cGraph[i][max_cl] = 0;
                    cGraph[max_cl][i] = 0;
                }

            }
        }
        if (ClusterChannel[max_edge] == 0){
            ClusterChannel[max_edge] = remain_channel;
            remain_cluster--;
            remain_channel--;
            cGraph[max_cl][max_edge] = 0;
            cGraph[max_edge][max_cl] = 0;
            for (int i=0; i < numClusters; i++){
                cGraph[i][max_edge] = 0;
                cGraph[max_edge][i] = 0;
            }
        }else{
            cout << "ERROR" << endl;
        }

        //for (int i=0; i < numClusters; i++){
        //    cout << i << "::" <<  avr_interfere[i] << endl;
        //}
        //cout << "remain_cluster" << remain_cluster << endl;
    }
    cout << "channels:";
    for (int i=0; i < numClusters; i++){
        cout <<  ClusterChannel[i] << " ";
    }
    cout << endl;
    // assign channel to each node
    for (int i=0; i < numNodes ; i++){
        nodeSet[i].setChannel(ClusterChannel[nodeSet[i].getClusterId()]);
    }
    int avr_itf[numClusters];
    int itr_itf = 0;
    cout << "avr_itf_tgraph:";
    // check again the left interference, for future time slot assignment
    for (int j=0; j < numClusters; j++){
        avr_itf[j] = 0;
        itr_itf = 0;
        for (int k=0; k < numClusters; k++){
            if (ClusterChannel[j] == ClusterChannel[k]){
                avr_itf[j] = avr_itf[j] + tGraph[j][k];
                itr_itf++;
            }
        }
        cout << (int)avr_itf[j]/itr_itf << " ";
    }
    cout << endl;
}

} /* namespace ScmacTree */
