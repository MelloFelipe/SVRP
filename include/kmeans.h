#include <fstream>
#include <sstream>
#include "graph.h"

class Point{

private:
    int pointId, clusterId;
    int dimensions;
    vector<double> values;

public:
    Point(int id, string line){
        dimensions = 0;
        pointId = id;
        stringstream is(line);
        double val;
        while(is >> val){
            values.push_back(val);
            dimensions++;
        }
        clusterId = 0; //Initially not assigned to any cluster
    }

    Point(int id, int x, int y) {
        dimensions = 2;
        pointId = id;
        clusterId = 0; //Initially not assigned to any cluster
        values.push_back(x);
        values.push_back(y);

    }

    int getDimensions();

    int getCluster();

    int getID();

    void setCluster(int val);

    double getVal(int pos);
};

class Cluster{

private:
    int clusterId;
    vector<double> centroid;
    vector<Point> points;

public:
    Cluster(int clusterId, Point centroid){
        this->clusterId = clusterId;
        for(int i=0; i<centroid.getDimensions(); i++){
            this->centroid.push_back(centroid.getVal(i));
        }
        this->addPoint(centroid);
    }

    void addPoint(Point p);

    bool removePoint(int pointId);

    int getId();

    Point getPoint(int pos);

    int getSize();

    double getCentroidByPos(int pos);

    void setCentroidByPos(int pos, double val);
};

class KMeans{
private:
    int K, iters, dimensions, total_points;
    vector<Cluster> clusters;

    int getNearestClusterId(Point point);

public:
    KMeans(int K, int iterations){
        this->K = K;
        this->iters = iterations;
    }

    void run(vector<Point>& all_points);
};

int kmeans_main(Graph g, int numberVehicles);
