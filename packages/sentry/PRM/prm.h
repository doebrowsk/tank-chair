// Probabilistic Roadmap Algorithm
#ifndef PRM_H_
#define PRM_H_

#include <vector>
#include <set>
#include <iostream>
#include <string>
#include <algorithm>
#include <limits>
#include <math.h>
#include <time.h>

// Sampling methods
#define SAMPLING_RANDOM_UNIFORM 0
#define SAMPLING_RANDOM_OBPRM 1
#define SAMPLING_QUASIRANDOM_OBPRM 2

// Constants
const double MIN_DOUBLE_DIFF=0.00001;
const double STEP_SIZE_OBPRM = 3; // for OBPRM
const double STEP_SIZE_LP = 0.1; // for local planner
const int MAX_ITER_OBPRM = 100; // max number of iterations for each particle

// Graph data structure
struct Point {
	double x;
	double y;
    
    Point(double x1, double y1);
    Point();
    
    std::string toString();
};
bool operator==(const Point& p1, const Point& p2);

struct GraphNode {
	Point point;
    std::vector<int> neighbors;
};

class Graph {
private:
    int getIndex(Point p);
    
public:
    std::vector<GraphNode> nodes;
    void addNode(Point p);
    bool existsEdge(int index1, int index2);
    bool existsEdge(Point p1, Point p2);
    void addEdge(int index1, int index2);
    void addEdge(Point p1, Point p2);
    std::vector<Point> getShortestPath(Point source, Point dest);
    std::vector<Point> getShortestPath(int sourceIndex, int destIndex);
    std::vector<Point> getNearestNeighbors(Point p, int limit);
    static bool comparePairs(std::pair<Point,double> p1, std::pair<Point,double> p2);
    void printGraph();
    void printForPython();
    static double dist(Point a, Point b);
    double edgeValue(int index1, int index2);
    std::vector<Point> getNodesAsPoints();
};

class ProbRoadmap {
public:
    ProbRoadmap(std::vector<std::vector<int> >m);
    ProbRoadmap();
    void setMap(std::vector<std::vector<int> >m);
    Graph constructRoadmap(int numNodes, int numNeighbors, int samplingType);
    std::vector<Point> queryRoadmap(Point start, Point goal, int numNeighbors, Graph roadmap);
    bool existsPath(Point p1, Point p2);
    std::vector<Point> samplePoints(int method, int numSamples);
    bool isCollisionFree(Point p);
    
    Point pushOutsideObstacle(Point p);
    std::vector<Point> optimizePath(std::vector<Point> path);
    double getHaltonSequence(int index, int base);
    double fRand(double fMin, double fMax);
private:
    int maxX;
    int maxY;
    std::vector<std::vector<int> >map;
};
template<typename T> bool searchVector(T element, std::vector<T> vector);

#endif