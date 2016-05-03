// Wavefront Algorithm
#include "wf.h"

Cell::Cell(int x1, int y1) {
    x=x1;
    y=y1;
}
Cell::Cell() {
    x=-1;
    y=-1;
}

bool operator==(const Cell& p1, const Cell& p2)
{
    return p1.x==p2.x && p1.y==p2.y;
}

Wavefront::Wavefront() {
}
void Wavefront::setMap(std::vector<std::vector<int> >m) {
    maxX = m.size();
    maxY = m[0].size();
    map = m;
    plan = std::vector<std::vector<int> >(maxX, std::vector<int>(maxY, 0));
    for (int i=0;i<maxX;i++)
        for (int j=0;j<maxY;j++)
            plan[i][j]=map[i][j];
}

void Wavefront::constructPlan(Point goal) {
    // Init
    Cell goalCell = pointToCell(goal);
    //plan[goalCell.x][goalCell.y]=2;
    
    std::queue<std::pair<Cell,int> > cellsToUpdate;
    std::vector<Cell> processedCells;
    cellsToUpdate.push(std::pair<Cell,int>(goalCell,2));
    // Update map
    
    int numIter = 0;
    while (!cellsToUpdate.empty()) {
        // Take a cell
        std::pair<Cell, int> c = cellsToUpdate.front();
        cellsToUpdate.pop();
        //std::cout<< c.first.x << " " << c.first.y <<"\n";
        // Set distance
        plan[c.first.x][c.first.y] = c.second;
        // Look at neighbors
        std::vector<Cell> neighbors = getNeighbors(c.first);
        for (int i=0; i<neighbors.size(); i++) {
            if (plan[neighbors[i].x][neighbors[i].y]==0 && !searchVector(neighbors[i], processedCells)) {
                //std::cout << neighbors[i].x << " " << neighbors[i].y << "\n";
                cellsToUpdate.push(std::pair<Cell, int>(neighbors[i], c.second+1));
                processedCells.push_back(neighbors[i]);
            }
        }
        numIter++;
    }
    /*
    // print the result
    for (int j=maxY-1;j>=0;j--) {
        for (int i=0;i<maxX;i++)
            std::cout << plan[i][j] << " ";
        std::cout << "\n";
    }
     */
}
std::vector<Point> Wavefront::getPath(Point start) {
    std::vector<Point> result;
    Cell startP = pointToCell(start);
    
    if (plan[startP.x][startP.y]==0)
        return result;
    
    Cell currPoint = startP;
    result.push_back(start);
    bool foundNextCell;
    do {
        foundNextCell = false;
        // get neighbors
        std::vector<Cell> neighbors = getNeighbors(currPoint);
        // get a neighbor moving closer towards the goal
        for (int i=0; i<neighbors.size(); i++){
            Cell n = neighbors[i];
            if (plan[n.x][n.y]<plan[currPoint.x][currPoint.y] && plan[n.x][n.y]>1) {
                currPoint = n;
                foundNextCell = true;
                //std::cout << n.x << " " << n.y << "   " << plan[n.x][n.y] << "\n";
                result.push_back(cellsToPoint(n));
                continue;
            }
        }
    }
    while (foundNextCell);
    
    return result;
}

bool Wavefront::existsPath(Point p1, Point p2) {
    // Get number of steps across the line
    double distance = Graph::dist(p1,p2);
    int numberOfSteps = (int)distance/STEP_SIZE_WF;
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
bool Wavefront::isCollisionFree(Point p) {
    Cell c = pointToCell(p);
    return map[c.x][c.y]==0;
}
std::vector<Point> Wavefront::optimizePath(std::vector<Point> path) {
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

Point Wavefront::cellsToPoint (Cell cell) {
    double x = cell.x+0.5;
    double y = cell.y+0.5;
    return Point(x,y);
}
Cell Wavefront::pointToCell (Point p) {
    int x = int(p.x);
    int y = int(p.y);
    return Cell(x,y);
}
std::vector<Cell> Wavefront::getNeighbors (Cell cell) {
    std::vector<Cell> result;
    for (int x=cell.x-1; x<=cell.x+1; x++)
        for (int y=cell.y-1; y<=cell.y+1; y++) {
            if (Cell(x,y)==cell || x<0 || x>=maxX || y<0 || y>=maxY || !isCollisionFree(cellsToPoint(Cell(x,y))))
                continue;
            result.push_back(Cell(x,y));
        }
    return result;
}