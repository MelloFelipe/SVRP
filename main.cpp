#include<iomanip>
#include "SVRP.h"

int main() {

    int numberVertices;
    Graph graph;
    int capacity;
    char verbosity; 

	/* N�mero de v�rtices do problema, incluindo o dep�sito */
	cout << "Enter with number of vertices including depot: ";
    cin >> numberVertices;

	/* Imprimir todas as informa��es do grafo ou apenas a resposta */
	cout << "Visualize all problem information? (y/n): ";
	cin >> verbosity;
	
	// Temporario: capacidade igual a esperan�a da demanda vezes metade do n�mero de v�rtices
	capacity = 10*(numberVertices+1/2);
	
	/* Cria um grafo completo respeitando a desigualdade triangular */
    graph.createInstance(numberVertices);
    if(verbosity == 'y')
    	graph.printInstance();
    
    /* Encontra um primeiro caminho utilizando TSP for�a bruta */
    vector<int> route = graph.TSP();

	/* Dado este caminho, computa todas as probabilidades de demanda total at� cada v�rtice */
    vector<vector<double> > f = probDemandsInRoute(graph, route);

    //cout << setprecision(3);

	if(verbosity == 'y') {
		for (int i = 1; i < graph.numberVertices; i++) {
	        for (int j = 1; j <= 20*(graph.numberVertices-1); j++) {
	        	cout << "("<< i << "," << j << "): ";
	            cout << f[i][j] << " ";
	        }
	        cout << "\n\n";
	    }
	}

	cout << "Expected cost of TSP route: ";
	cout << expectedLength(graph, f, capacity);
	
    return 0;

}
