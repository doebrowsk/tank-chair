// Probabilistic Roadmap Algorithm
#include "prm.h"

// Graph data structure
Point::Point(double x1, double y1) {
    x=x1;
    y=y1;
}
Point::Point() {
    x=-1;
    y=-1;
}

std::string Point::toString() {
    return "("+std::to_string(x)+", "+std::to_string(y)+")";
}

bool operator==(const Point& p1, const Point& p2)
{
    return fabs(p1.x-p2.x)<MIN_DOUBLE_DIFF && fabs(p1.y-p2.y)<MIN_DOUBLE_DIFF;
}

int Graph::getIndex(Point p) {
    for (unsigned int i=0; i<nodes.size(); i++)
        if (nodes[i].point==p)
            return i;
    return -1;
}
	
void Graph::addNode(Point p) {
    GraphNode newNode;
    newNode.point = p;
    nodes.push_back(newNode);
}

bool Graph::existsEdge(int index1, int index2) {
    return searchVector(index2, nodes[index1].neighbors) && searchVector(index1, nodes[index2].neighbors);
}

bool Graph::existsEdge(Point p1, Point p2) {
    return existsEdge(getIndex(p1), getIndex(p2));
}

void Graph::addEdge(int index1, int index2) {
    if (!searchVector(index2, nodes[index1].neighbors))
        nodes[index1].neighbors.push_back(index2);
    if (!searchVector(index1, nodes[index2].neighbors))
        nodes[index2].neighbors.push_back(index1);
}

void Graph::addEdge(Point p1, Point p2) {
    int index1 = getIndex(p1);
    int index2 = getIndex(p2);
    if (index1 == -1 || index2 == -1) {
        std::cout << "Cannot find one of the points. Cannot add an edge.\n";
    }
    else {
        nodes[index1].neighbors.push_back(index2);
        nodes[index2].neighbors.push_back(index1);
    }
}

std::vector<Point> Graph::getShortestPath(Point source, Point dest) {
    return getShortestPath(getIndex(source), getIndex(dest));
}

std::vector<Point> Graph::getShortestPath(int sourceIndex, int destIndex) {
    // Initialization
    std::vector<double> distances = std::vector<double>(nodes.size(), std::numeric_limits<double>::infinity());
    std::vector<int> prevNode = std::vector<int>(nodes.size(), -1);
    distances[sourceIndex] = 0;
    std::set<std::pair<int,int> > q;
    q.insert(std::pair<int,int>(0, sourceIndex));
    
    while(!q.empty()) {
        //Remove and return best node
        int bestNode = q.begin()->second;
        q.erase(q.begin());
        if (bestNode==destIndex) {
            // Found the shortest path
            //std::cout << "Got to the goal node " << nodes[destIndex].point.toString() << "\n";
            int index = destIndex;
            std::vector<Point> result;
            for (int index = destIndex; index != -1; index=prevNode[index]){
                result.insert(result.begin(), nodes[index].point);
            }
            return result;
        }
        // process each neighbor
        for (int i=0; i<nodes[bestNode].neighbors.size(); i++) {
            int neighbor = nodes[bestNode].neighbors[i];
            double alt = distances[bestNode] + edgeValue(bestNode,neighbor);
            //std::cout << "Processing node " << nodes[neighbor].point.toString() << "\n";
            if (alt < distances[neighbor]) {
                //std::cout << "Modifyng distance to " << alt << "\n";
                q.erase(std::pair<int,int>(distances[neighbor], neighbor));
                distances[neighbor] = alt;
                q.insert(std::pair<int,int>(distances[neighbor], neighbor));
                prevNode[neighbor] = bestNode;
            }
        }
    }
    
    return std::vector<Point>();
}

// For each node get nearest neighbors to see if they should be connected
std::vector<Point> Graph::getNearestNeighbors(Point p, int limit) {
    std::vector<Point> graphPoints =std::vector<Point>(getNodesAsPoints());
    std::vector<std::pair<Point,double> > rankedPoints;
    for (int i=0; i<graphPoints.size(); i++) {
        std::pair<Point,double> a;
        a.first = graphPoints[i];
        a.second = dist(graphPoints[i], p);
        if (fabs(a.second-0)>MIN_DOUBLE_DIFF)  // don't add p itself
            rankedPoints.push_back(a);
    }
    std::sort(rankedPoints.begin(), rankedPoints.end(), comparePairs);
    std::vector<Point> result;
    for (int i=0; i<limit; i++) {
        //std::cout << "getClosestNeighbors for " << p.toString() << ": " << rankedPoints[i].first.toString() << "\n";
        result.push_back(rankedPoints[i].first);
    }
    return result;
}

bool Graph::comparePairs(std::pair<Point,double> p1, std::pair<Point,double> p2) {
    return p1.second<p2.second;
}

void Graph::printGraph() {
    // used for testing
    std::cout << "Graph:\n";
    if (nodes.size()==0)
        std::cout << "None\n";
    for (int i=0; i<nodes.size(); i++) {
        Point p=nodes[i].point;
        std::cout << "Node " << i << ": Point " << p.toString() << ", Neighbors: ";
        if (nodes[i].neighbors.size()==0)
            std::cout << "None";
        for (int j=0; j<nodes[i].neighbors.size(); j++) {
            std::cout << nodes[i].neighbors[j] << " ";
        }
        std::cout << "\n";
    }
}

void Graph::printForPython() {
    int numSamples = nodes.size();
    // print x values
    std::cout << "x = [";
    for (int i=0; i<numSamples; i++) {
        std::cout << nodes[i].point.x;
        if (i!=numSamples-1)
            std::cout << ", ";
    }
    std::cout << "]\n";
    // print y values
    std::cout << "\ty = [";
    for (int i=0; i<numSamples; i++) {
        std::cout << nodes[i].point.y;
        if (i!=numSamples-1)
            std::cout << ", ";
    }
    std::cout << "]\n";
    // get links
    std::vector<std::pair<int,int> > links;
    for (int i=0; i<numSamples; i++) {
        std::vector<int> neighbors = nodes[i].neighbors;
        for (int j=0; j<neighbors.size(); j++) {
            links.push_back(std::pair<int,int>(i,neighbors[j]));
        }
    }
    std::cout << "\tlinks = [";
    for (int i=0; i<links.size(); i++) {
        Point p1 = nodes[links[i].first].point;
        Point p2 = nodes[links[i].second].point;
        std::cout << "[" << p1.x << ", " << p1.y << ", " << p2.x << ", " << p2.y << "]";
        if (i!=links.size()-1)
            std::cout << ", ";
    }
    std::cout << "]\n";
}

double Graph::dist(Point a, Point b) {
    return sqrt(pow(a.x-b.x, 2)+pow(a.y-b.y, 2));
}

double Graph::edgeValue(int index1, int index2) {
    Point p1 = nodes[index1].point;
    Point p2 = nodes[index2].point;
    return sqrt(pow((p1.x-p2.x), 2) + pow((p1.y-p2.y), 2));
}

std::vector<Point> Graph::getNodesAsPoints() {
    std::vector<Point> result;
    for (int i=0; i<nodes.size(); i++) {
        result.push_back(nodes[i].point);
    }
    return result;
}

// Actual PRM part
ProbRoadmap::ProbRoadmap(std::vector<std::vector<int> >m) {
    maxX = m.size();
    maxY = m[0].size();
    map = m;
}
ProbRoadmap::ProbRoadmap() {
}
void ProbRoadmap::setMap(std::vector<std::vector<int> >m) {
    maxX = m.size();
    maxY = m[0].size();
    map = m;
}

// p.204 Algorithm 6 
Graph ProbRoadmap::constructRoadmap(int numNodes, int numNeighbors, int samplingType) {
	Graph roadmap;
	// Sample
	std::vector<Point> samples = samplePoints(samplingType, numNodes);
    for (int i=0; i<samples.size(); i++) {
        roadmap.addNode(samples[i]);
    }
	// Connect
	for (int i=0; i<roadmap.nodes.size(); i++) {
		Point p1 = roadmap.nodes[i].point;
        std::vector<Point> neighbors = roadmap.getNearestNeighbors(p1, numNeighbors);
		for (int j=0; j<neighbors.size(); j++) {
			Point p2 = neighbors[j];
			if (!roadmap.existsEdge(p1, p2) && existsPath(p1, p2))	{
				roadmap.addEdge(p1, p2);
			}
		}
	}
	return roadmap;
}


// p.206 Algorithm 7
std::vector<Point> ProbRoadmap::queryRoadmap(Point start, Point goal, int numNeighbors, Graph roadmap) {
    std::vector<Point> neighborsStart = roadmap.getNearestNeighbors(start, numNeighbors);
    std::vector<Point> neighborsGoal = roadmap.getNearestNeighbors(goal, numNeighbors);
	roadmap.addNode(start);
	roadmap.addNode(goal);
	// Connect start node to the graph
	for (int i=0; i<neighborsStart.size(); i++) {
		Point p = neighborsStart[i];
		if (!roadmap.existsEdge(start, p) && existsPath(start, p))	{
			roadmap.addEdge(start, p);
		}
	}
	// Connect goal node to the graph
	for (int i=0; i<neighborsGoal.size(); i++) {
		Point p = neighborsGoal[i];
		if (!roadmap.existsEdge(p, goal) && existsPath(p, goal))	{
			roadmap.addEdge(p, goal);
		}
	}
	// Get shortest path between start and goal
    std::vector<Point> shortestPath = roadmap.getShortestPath(start, goal);
	return shortestPath;
}

std::vector<Point> ProbRoadmap::samplePoints(int method, int numSamples) {
    std::vector<Point> result;
    switch (method) {
        case SAMPLING_RANDOM_UNIFORM: {
            // sample uniformly
            // throw away points inside obstacles
            for (int i=0; i<numSamples; i++) {
                Point p;
                do {
                    p = Point(fRand(0, maxX), fRand(0, maxY));
                }
                while (!isCollisionFree(p));
                result.push_back(p);
            }
            return result;
        }
        case SAMPLING_RANDOM_OBPRM: {
            // sample uniformly
            // push points inside obstacle outside
            for (int i=0; i<numSamples; i++) {
                Point p = Point(fRand(0, maxX), fRand(0, maxY));
                if (!isCollisionFree(p))
                    p = pushOutsideObstacle(p);
                /*
                Point p;
                do {
                    p = Point(fRand(0, maxX), fRand(0, maxY));
                }
                while (isCollisionFree(p));
                p = pushOutsideObstacle(p);
                 */
                result.push_back(p);
            }
            return result;
        }
        case SAMPLING_QUASIRANDOM_OBPRM: {
            // sample using Hammersley sequence
            // push points inside obstacle outside
            int baseY = 2;
            for (int i=1; i<numSamples+1; i++) {
                double x = i/float(numSamples);
                double y = getHaltonSequence(i, baseY);
                Point p = Point(x*(maxX-1),y*(maxY-1));
                if (!isCollisionFree(p))
                    p = pushOutsideObstacle(p);
                result.push_back(p);
            }
            return result;
        }
    }
    return result;
}

bool ProbRoadmap::existsPath(Point p1, Point p2) {
    // Get number of steps across the line
    double distance = Graph::dist(p1,p2);
    int numberOfSteps = (int)distance/STEP_SIZE_LP;
    // step across the line
    Point newP = p1;
    double stepX = (p2.x-p1.x)/float(numberOfSteps);
    double stepY = (p2.y-p1.y)/float(numberOfSteps);
    for (int i=0; i<numberOfSteps; i++) {
        newP.x = newP.x + stepX;
        newP.y = newP.y + stepY;
        if (!isCollisionFree(newP)) {
            return false;
        }
    }
    return true;
}


bool ProbRoadmap::isCollisionFree(Point p) {
    int x = int(p.x);
    int y = int(p.y);
    return map[x][y]==0;
}

Point ProbRoadmap::pushOutsideObstacle(Point p) {
    // Repeat until success
    int numIter = 0;
    while (numIter < MAX_ITER_OBPRM) {
        // Pick random direction
        double direction = fRand(0,2*M_PI);
        // Walk in this direction until find a free cell or hit the boundary
        Point newP = p;
        while (0<=newP.x && newP.x<maxX && 0<=newP.y && newP.y<=maxY) {
            if (isCollisionFree(newP))
                return newP;
            newP.x = newP.x + STEP_SIZE_OBPRM*sin(direction);
            newP.y = newP.y + STEP_SIZE_OBPRM*cos(direction);
        }
    }
    // Should never happen
    std::cout << "Could not push point " << p.toString() << " outside obstacle. Reached max number of iterations";
    return Point(-1,-1);
}

std::vector<Point> ProbRoadmap::optimizePath(std::vector<Point> path) {
    std::vector<Point> newPath;
    Point currPoint = path[0];
    for (int i=1; i<path.size(); i++) {
        if (i==path.size()-1 || !existsPath(currPoint, path[i+1])) {
            newPath.push_back(currPoint);
            currPoint = path[i];
            if (i==path.size()-1)
                newPath.push_back(currPoint);
        }
    }
    return newPath;
}

double ProbRoadmap::getHaltonSequence(int index, int base) {
    double result = 0;
    double f = 1;
    int i = index;
    while (i>0) {
        f = f/base;
        result = result + f*(i%base);
        i = int(i/base);
    }
    return result;
}

double ProbRoadmap::fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

template<typename T> bool searchVector(T element, std::vector<T> vector) {
    return std::find(vector.begin(), vector.end(), element) != vector.end();
}