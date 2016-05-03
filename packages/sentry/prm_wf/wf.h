// Probabilistic Roadmap Algorithm
#ifndef WF_H_
#define WF_H_

/*
#include <vector>
#include <set>
#include <iostream>
#include <string>
#include <algorithm>
#include <limits>
#include <math.h>
#include <time.h>
 */
#include <queue>
#include "prm.cpp"

const double STEP_SIZE_WF = 0.1;

struct Cell {
    int x;
    int y;
    
    Cell(int x1, int y1);
    Cell();
};
bool operator==(const Cell& p1, const Cell& p2);

class Wavefront {
private:
    int maxX;
    int maxY;
    std::vector<std::vector<int> >map;
    std::vector<std::vector<int> >plan;
public:
    Wavefront();
    void setMap(std::vector<std::vector<int> >m);
    
    void constructPlan(Point goal);
    std::vector<Point> getPath(Point start);
    
    bool existsPath(Point p1, Point p2);
    bool isCollisionFree(Point p);
    std::vector<Point> optimizePath(std::vector<Point> path);
    Point cellsToPoint (Cell cell);
    Cell pointToCell (Point p);
    std::vector<Cell > getNeighbors (Cell cell);
};

#endif