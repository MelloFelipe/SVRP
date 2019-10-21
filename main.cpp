#include<iomanip>
#include "SVRP.h"

int main() {

    int numberVertices;
    Graph graph;
    int capacity;

	/* N�mero de v�rtices do problema, incluindo o dep�sito */
    cin >> numberVertices;

	// Temporario: capacidade igual a esperan�a da demanda vezes metade do n�mero de v�rtices
	capacity = 10*(numberVertices/2);
	
	/* Cria um grafo completo respeitando a desigualdade triangular */
    graph.createInstance(numberVertices);
    graph.printInstance();
    
    /* Encontra um primeiro caminho utilizando TSP for�a bruta */
    vector<int> route = graph.TSP();

	/* Dado este caminho, computa todas as probabilidades de demanda total at� cada v�rtice */
    vector<vector<double> > f = probDemandsInRoute(graph, route);

    //cout << setprecision(3);

    for (int i = 0; i < graph.numberVertices; i++) {
        for (int j = 0; j < 20*graph.numberVertices; j++) {
            cout << f[i][j] << ' ';
        }
        cout << "\n\n";
    }

	cout << "Expected length: ";
	cout << expectedLength(graph, f, capacity);
	
    return 0;

}
