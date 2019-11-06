#include "SVRP.h"

class TabuSearchSVRP {

public:

    Graph g;
    double penalty, bestPenalizedSol, bestSol;
    int numVehicles, capacity, numSelected, numNearest, numRoutes;
    int itCount, numInfeasibleNearby;
    int currNoImprovement, maxNoImprovement;
    vector<int> posClient;
    vector<double> relativeDemand;
    vector<vector<int>> routes, bestRoute, closestNeighbours;

    // Etapas
    TabuSearchSVRP(Graph g, int numVehicles, int capacity);
    void neighbourhoodSearch();
    void incumbentUpdate();
    void coefficientUpdate();
    void intensification();

    // Funções
    double penalizedExpectedLength(int solVehicles);
    double pi(int rota, int cliente);
    double alpha(int rota);
    double A2(int a, int b, int c);
    double approxCostMove(int selectedClient, int selectedNeighbour, double relativeDemand);

};
