#include<iomanip>
#include "SVRP.h"

int main() {

    int numberVertices;
    Graph graph;

	/* N�mero de v�rtices do problema, incluindo o dep�sito */
    cin >> numberVertices;

	/* Cria um grafo completo respeitando a desigualdade triangular */
    graph.createInstance(numberVertices);
    graph.printInstance();
    
    /* Encontra um primeiro caminho utilizando TSP for�a bruta */
    vector<int> route = graph.TSP();

	/* Dado este caminho, computa todas as probabilidades de demanda total at� cada v�rtice */
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
