#include<iomanip>
#include "SVRP.h"

int main() {

    int numberVertices;
    Graph graph;

	/* Número de vértices do problema, incluindo o depósito */
    cin >> numberVertices;

	/* Cria um grafo completo respeitando a desigualdade triangular */
    graph.createInstance(numberVertices);
    graph.printInstance();
    
    /* Encontra um primeiro caminho utilizando TSP força bruta */
    vector<int> route = graph.TSP();

	/* Dado este caminho, computa todas as probabilidades de demanda total até cada vértice */
    vector<vector<double> > f = probDemandsInRoute(graph, route);

    cout << setprecision(3);

    for (int i = 0; i < graph.numberVertices; i++) {
        for (int j = 0; j < 20*graph.numberVertices; j++) {
            cout << f[i][j] << ' ';
        }
        cout << "\n\n";
    }

    return 0;

}
