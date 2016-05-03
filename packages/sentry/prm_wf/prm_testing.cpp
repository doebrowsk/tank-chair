// Probabilistic Roadmap Algorithm
#include "wf.cpp"
//// Testing Code

void testGraph() {
    Graph g = Graph();
    g.addNode(Point(0,0));
    g.addNode(Point(0,1));
    g.addNode(Point(1,0));
    g.addNode(Point(1,1));
    g.addNode(Point(1,2.1));
    g.addNode(Point(3,2));
    g.addNode(Point(2,3));
    g.addNode(Point(4,3));
    g.addNode(Point(3,4));
    g.addEdge(Point(0,0),Point(1,0));
    g.addEdge(Point(0,0),Point(0,1));
    g.addEdge(Point(0,1),Point(1,1));
    g.addEdge(Point(1,0),Point(1,1));
    g.addEdge(Point(1,0),Point(3,2));
    g.addEdge(Point(1,1),Point(3,2));
    g.addEdge(Point(1,1),Point(1,2.1));
    g.addEdge(Point(1,2.1),Point(2,3));
    g.addEdge(Point(3,2),Point(2,3));
    g.addEdge(Point(3,4),Point(2,3));
    g.addEdge(Point(4,3),Point(3,4));
    g.addEdge(Point(4,3),Point(3,2));
    g.printGraph();
    // test shortest path
    std::vector<Point> path = g.getShortestPath(Point(0,0), Point(3,4));
    std::cout << "Shortest path:\n";
    for (int i=0; i<path.size(); i++) {
        Point p = path[i];
        std::cout << p.toString() << ' ';
    }
    std::cout << "\n";
    // test nearest neighbors
    std::cout << "Nearest neighbors:\n";
    std::vector<Point> nearestNeighbors = g.getNearestNeighbors(Point(3,3), 5);
    for (int i=0; i<nearestNeighbors.size(); i++) {
        std::cout << nearestNeighbors[i].toString() << ' ';
    }
    std::cout << "\n";
}

std::vector<std::vector<int> > createMap(int obstacles[][4], int numObstacles) {
    int maxX = 100;
    int maxY = 100;
    std::vector<std::vector<int> > map = std::vector<std::vector<int> >(maxX, std::vector<int>(maxY, 0));
    // define obstacles
    // initialize with 0
    for (int i=0; i<maxX; i++)
        for (int j=0; j<maxY; j++)
            map[i][j]=0;
    // create obstacles
    for (int i=0; i<numObstacles; i++) {
        for (int x=obstacles[i][0]; x<obstacles[i][2]; x++)
            for (int y=obstacles[i][1]; y<obstacles[i][3]; y++) {
                map[x][y]=1;
            }
    }
    // print python code
    std::cout << "obstacles = [";
    for (int i=0; i<numObstacles; i++) {
        std::cout << "[" << obstacles[i][0] << ", " << obstacles[i][1] << ", " << obstacles[i][2] << ", " << obstacles[i][3] << "]";
        if (i!=numObstacles-1)
            std::cout << ", ";
    }
    std::cout << "]" << "\n";
    return map;
}

void testSampling(ProbRoadmap p) {
    // generate samples
    int numSamples = 1024;
    std::vector<Point> samples = p.samplePoints(SAMPLING_QUASIRANDOM_OBPRM, numSamples);
    // print x values
    std::cout << "x = [";
    for (int i=0; i<numSamples; i++) {
        std::cout << samples[i].x;
        if (i!=numSamples-1)
            std::cout << ", ";
    }
    std::cout << "]\n";
    // print y values
    std::cout << "y = [";
    for (int i=0; i<numSamples; i++) {
        std::cout << samples[i].y;
        if (i!=numSamples-1)
            std::cout << ", ";
    }
    std::cout << "]\n";
}

void printPathForPython(std::vector<Point> path) {
    std::cout << "\tpath = [";
    for (int i=0; i<path.size(); i++) {
        Point p = path[i];
        std::cout << "[" << p.x << ", " << p.y << "]";
        if (i!=path.size()-1)
            std::cout << ", ";
    }
    std::cout << "]\n";
}

void printPathInfo(std::vector<Point> path) {
    double length = 0;
    for (int i=1; i<path.size(); i++) {
        Point prevP = path[i-1];
        Point p = path[i];
        length += Graph::dist(prevP, p);
    }
    std::cout << "Total path length: " << length << "\n";
    std::cout << "Total number of points: " << path.size()-1 << "\n";
}

std::pair<double, int> getPathInfo(std::vector<Point> path) {
    double length = 0;
    for (int i=1; i<path.size(); i++) {
        Point prevP = path[i-1];
        Point p = path[i];
        length += Graph::dist(prevP, p);
    }
    return std::pair<double,int>(length, path.size()-1);
}

std::vector<std::vector<int> > createMap1() {
    int obstacles[7][4] = {
        {20,0,30,60},
        {10,60,30,70},
        {40,20,50,100},
        {60,40,100,50},
        {70,20,80,40},
        {90,50,100,70},
        {70,60,80,100}
    };
    return createMap(obstacles, 7);
}

std::vector<std::vector<int> > createMap2() {
    int obstacles[13][4] = {
        {0,20,30,30},
        {0,40,10,50},
        {0,60,30,70},
        {20,70,30,90},
        {10,80,20,90},
        {20,0,90,10},
        {80,10,90,20},
        {40,10,50,80},
        {20,40,40,50},
        {50,30,90,40},
        {60,20,70,60},
        {80,40,90,90},
        {60,70,70,100}
    };
    return createMap(obstacles, 13);
}

std::vector<std::vector<int> > createMap3() {
    int obstacles[10][4] = {
        {10,10,20,50},
        {10,52,20,100},
        {22,0,30,70},
        {22,72,30,100},
        {32,0,40,40},
        {32,42,40,100},
        {80,0,82,40},
        {80,42,82,100},
        {84,0,86,90},
        {84,92,86,100}
    };
    return createMap(obstacles, 10);
}

std::vector<std::vector<int> > createMap4() {
    int obstacles[18][4] = {
        {76,0,78,97},
        {76,99,78,100},
        {80,0,82,40},
        {80,42,82,100},
        {84,0,86,90},
        {84,92,86,100},
        {88,0,86,2},
        {88,2,86,100},
        {10,0,12,50},
        {10,52,12,100},
        {14,0,16,70},
        {14,72,16,100},
        {18,0,20,10},
        {18,12,20,100},
        {22,0,24,80},
        {22,82,24,100},
        {40,0,60,10},
        {40,11,60,100}
    };
    return createMap(obstacles, 18);
}

std::vector<std::vector<int> > createMap5() {
    int obstacles[3][4] = {
        {2,0,3,99},
        {3,98,97,99},
        {97,0,98,99}
    };
    return createMap(obstacles, 3);
}

void testEverything(ProbRoadmap p) {
    int maxIter = 100;
    int numSuccess = 0;
    int numLinks = 0;
    int numLinksSmooth = 0;
    double length = 0;
    double lengthSmooth = 0;
    std::pair<double, int> pathInfo;
    for (int i=0; i<maxIter; i++) {
        std::cout << "Run " << i << "\n";
        // construct roadmap
        int numSamples = 1000;
        int numNeighbors = 5;
        int samplingType = SAMPLING_QUASIRANDOM_OBPRM;
        Graph roadmap = p.constructRoadmap(numSamples, numNeighbors, samplingType);
        //roadmap.printGraph();
        //roadmap.printForPython();
        
        // query roadmap
        Point start = Point(10,10);
        Point goal = Point(55,25);
        std::vector<Point> path = p.queryRoadmap(start, goal, numNeighbors, roadmap);
        if (path.size()==0) {
            std::cout << "Run failed\n";
            continue;
        }
        else
            numSuccess++;
        //printPathForPython(path);
        //printPathInfo(path);
        pathInfo = getPathInfo(path);
        length += pathInfo.first;
        numLinks += pathInfo.second;
        
        path = p.optimizePath(path);
        pathInfo = getPathInfo(path);
        lengthSmooth += pathInfo.first;
        numLinksSmooth += pathInfo.second;
        //printPathForPython(path);
        //printPathInfo(path);
    }
    double failurePercentage = double(maxIter-numSuccess)/maxIter;
    if (failurePercentage < 1) {
        double avgLength = length/numSuccess;
        double avgNumLinks = numLinks/numSuccess;
        double avgLengthSmooth = lengthSmooth/numSuccess;
        double avgNumLinksSmooth = numLinksSmooth/numSuccess;
        std::cout << "Avg path length: " << avgLength << "\n";
        std::cout << "Avg number of rotations: " << avgNumLinks << "\n";
        std::cout << "Avg path length after smoothing: " << avgLengthSmooth << "\n";
        std::cout << "Avg number of rotations after smoothing: " << avgNumLinksSmooth << "\n";
    }
    std::cout << "Failure rate: " << failurePercentage << "\n";
}



int main() {
    srand (time(NULL));
    
    // PRM
    /*
    ProbRoadmap p;

    // set map
    std::vector<std::vector<int> > map = createMap2();
    p.setMap(map);
    
    // construct roadmap
    int numSamples = 1000;
    int numNeighbors = 5;
    int samplingType = SAMPLING_QUASIRANDOM_OBPRM;
    Graph roadmap = p.constructRoadmap(numSamples, numNeighbors, samplingType);
    
    // query roadmap
    Point start = Point(10,10);
    Point goal = Point(55,25);
    std::vector<Point> path = p.queryRoadmap(start, goal, numNeighbors, roadmap);
    if (path.size()==0) {
        std::cout << "Run failed\n";
    }
    else {
        path = p.optimizePath(path);
        printPathInfo(path);
    }
    */
    // Wavefront
    
    Wavefront w;
    // set map
    std::vector<std::vector<int> > map = createMap2();
    //std::vector<std::vector<int> > map = std::vector<std::vector<int> >(4, std::vector<int>(4, 0));
    //map[1][1]=1;
    w.setMap(map);
    w.constructPlan(Point(55, 25));
    std::cout<<"Plan constructed\n";
    std::vector<Point> path = w.getPath(Point(5,5));
    path = w.optimizePath(path);
    printPathForPython(path);
    printPathInfo(path);
    
    //testEverything(p);
    //testGraph();
    //testSampling(p);
    return 0;
}