#include "graph.h"

vector<vector<double> > probDemandsInRoute(Graph g, vector<int> route);
double probReachCapacity(int i, Graph g, vector<vector<double>> f, int capacity);
double probExceedsCapacity(int i, Graph g, vector<vector<double>> f, int capacity);
double expectedLength(Graph g, vector<vector<double>> f, int capacity);

