#include <algorithm>
#include <numeric>
#include <time.h>
#include "TabuSearchSVRP.h"

// Etapa 1: Inicialização
TabuSearchSVRP::TabuSearchSVRP(Graph inst, int numVehicles, int capacity) {

    this->g = inst;
    this->numVehicles = numVehicles;
    this->capacity = capacity;

    int h = min(this->g.numberVertices - 1, 10);

    // Computar os h vizinhos mais próximos de cada vértice
    for (int i = 0; i < this->g.numberVertices; i++) {

        vector<int> closest(this->g.numberVertices);
        vector<double> dist = this->g.adjMatrix[i];

        iota(closest.begin(), closest.end(), 0);

        sort(closest.begin(), closest.end(), [&dist](size_t i1, size_t i2){
            return dist[i1] < dist[i2];
        });

        closest.erase(closest.begin());
        closest.erase(closest.begin() + h, closest.end());

        this->closestNeighbours.push_back(closest);

    }

    this->posClient.resize(this->g.numberVertices);
    // Rotas de ida e volta ao depósito
    for (int i = 1; i < this->g.numberVertices; i++) {

        vector<int> route(1, i);
        this->routes.push_back(route);
        this->posClient[i] = i;

    }

    this->penalty = 1;

    bestPenalizedSol = penalizedExpectedLength(this->g.numberVertices - 1);

    /* Coeficiente que relaciona a demanda esperada do vértice com a total
    Obs: precisa testar se tá certo mas coloquei os calculos de demanda esperada
    na inicialização do grafo pra aproveitar os loops de lá. */
    for (int i = 1; i < this->g.numberVertices; i++) {
        relativeDemand.push_back(this->g.expectedDemand[i] / this->g.totalExpectedDemand);
    }


    // Ajuste de parâmetros

    this->numNearest = min(this->g.numberVertices - 1, 5);
    this->numSelected = min(this->g.numberVertices - 1, 5*this->numVehicles);
    this->numRoutes = this->g.numberVertices - 1;

    this->itCount = 0;
    this->currNoImprovement = 0;

    if (numVehicles < this->g.numberVertices - 1) {
        this->numInfeasibleNearby = 1;
        this->bestSol = numeric_limits<double>::max();
    } else {
        this->numInfeasibleNearby = 0;
        this->bestSol = this->bestPenalizedSol;
        this->bestRoute = this->routes;
    }

    this->maxNoImprovement = 50*this->g.numberVertices;

    // Procura na vizinhança de soluções
    this->itCount++;
    this->currNoImprovement++;

    /* Construir todas as soluções candidatas N(p, q, x)
    p = this->numNearest; q = this->numSelected; x = this->routes;
    contém todas as soluções que podem ser alcançadas removendo um dos
    q aleatorimantes selecionados clientes e inserindo imediatamente após (ou antes)
    um dos seus p vizinhos mais próximos.

    Escolher um cliente aleatoriamente.
    Escolher um vizinho desse cliente aleatoriamente.
    Computar o custo aproximado de se remover o cliente de sua rota e adiciona-lo
    após (ou antes) o vizinho escolhido.
    Repetir esse processo q vezes.
    Ordenar movimentos que são os melhores custos aproximados.
    Avaliar os 5 melhores movimentos precisamente e considere a melhor solução y
    Se o custo de y for melhor que o encontrado, atualizar a melhor solução e melhor rota. Vai para STEP 3.
    Se os 5 melhores movimentos forem todos tabu, considere os 5 próximos movimentos e repita o processo da linha 92.
    Seja z o melhor movimento não tabu computado, atribua a solução atual como z e vá para o STEP 3.

    Obs.:
    Ao remover o cliente da rota checar se isso diminui o numero de rotas, se sim, diminuir this->numRoutes.
    Adicionar o cliente adicionado à lista tabu?
    Nova this->routes é salva ou apenas movimento é salvo?
    */
    srand(time(0));
    vector<int> customers;

    for (int i = 1; i < this->g.numberVertices; i++)
        customers.push_back(i);

    random_shuffle(customers.begin(), customers.end(), [](int i){
      return rand()%i;
    });

    for(int i = 0; i < this->numSelected; i++) {

      int selectedClient = customers[i];

      int selectedNeighbour = rand()%this->numNearest;
      selectedNeighbour = this->closestNeighbours[selectedClient][selectedNeighbour];

      /* Computar ~F'(i,j,v) */
      double approxCostMoveC = approxCostMove(selectedClient, selectedNeighbour, relativeDemand[selectedClient]);
      cout << "****************************************" << endl;
      cout << "i=" << i << ":" << approxCostMoveC << endl;
    }
}


double TabuSearchSVRP::penalizedExpectedLength(int solVehicles) {
    return totalExpectedLength(g, capacity, routes) + penalty*abs(solVehicles - numVehicles);
}

double TabuSearchSVRP::pi(int rota, int cliente) {

  vector<vector<double>> f = probTotalDemand(g, routes[rota]);
  double before = routeExpectedLength(g, f, capacity, routes[rota]);

  vector<int> newRoute;
  for(int i = 0; i < routes[rota].size(); i++) {
    if(routes[rota][i] != cliente)
      newRoute.push_back(routes[rota][i]);
  }

  f = probTotalDemand(g, newRoute);
  double after = routeExpectedLength(g, f, capacity, newRoute);

  return before - after;
}

double TabuSearchSVRP::alpha(int rota) {
  double max = 0, aux = 0;
  for(int i = 0; i < routes[rota].size(); i++) {
    aux = pi(rota, i);
    if(aux > max)
      max = aux;
  }

  return max;
}

double TabuSearchSVRP::A2(int a, int b, int c) {
  return (g.adjMatrix[a][b]+g.adjMatrix[b][c]-g.adjMatrix[a][c])*g.vertices[b].probOfPresence;
}

double TabuSearchSVRP::approxCostMove(int selectedClient, int selectedNeighbour, double relativeDemand) {

  double approxCost = 0;

  int beforeClient = 0, beforeNeighbour = 0, afterClient = 0;

  if(routes[posClient[selectedClient]].size() == 2 && routes[posClient[selectedClient]][0])
    afterClient = routes[posClient[selectedClient]][1];

  for(int i = 1; i < routes[posClient[selectedClient]].size(); i++) {

    if(routes[posClient[selectedClient]][i] == selectedClient) {

      beforeClient = routes[posClient[selectedClient]][i-1];

      if(routes[posClient[selectedClient]].size() > i)
        afterClient = routes[posClient[selectedClient]][i+1];
      else
        afterClient = 0;

    }
  }

  for(int i = 1; i < routes[posClient[selectedNeighbour]].size(); i++) {

    if(routes[posClient[selectedNeighbour]][i] == selectedNeighbour)
      beforeNeighbour = routes[posClient[selectedNeighbour]][i-1];
  }

  if(posClient[selectedClient] == posClient[selectedNeighbour]) {

    approxCost = A2(beforeNeighbour, selectedClient, selectedNeighbour)
               - A2(beforeClient, selectedClient, afterClient);
  }

  else {

    approxCost = A2(beforeNeighbour, selectedClient, selectedNeighbour)
               - A2(beforeClient, selectedClient, afterClient)
               +alpha(posClient[selectedNeighbour])*relativeDemand
               -pi(posClient[selectedClient], selectedClient);
  }

  return approxCost;
}
