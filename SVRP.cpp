#include<cmath>
#include "SVRP.h"

/*
probDemandsInRoute: Calcula todas as probabilidades de demanda total até cada vértice
salvando estes valores em uma matriz. Nesta matriz, as linhas representam o último vértice
sendo considerado e as colunas a possível demanda até este vértice. Assim, se estamos na
3ª linha e 4ª coluna, computamos a probabilidade de que a demanda dos vértices 1, 2 e 3
sejam iguais a 4.

Entrada:
g: grafo do problema sendo considerado;
orderInRoute: vetor de inteiros que indica a ordem na qual os vértices são percorridos na rota.

Saída:
f: matriz n por 20*n, onde n é o número de vértices do grafo. A posição (i, j) da matriz
indica a probabilidade da demanda até o vértice i ser j.

Observação: demanda máxima de um vértice é 20.
*/
vector<vector<double>> probDemandsInRoute(Graph g, vector<int> orderInRoute) {

	// Inicializa a matriz com probabilidades 0
    vector<double> v(20*g.numberVertices + 1, 0);
    vector<vector<double>> f(g.numberVertices, v);
    int next, m = 1, routeSize = g.numberVertices - 1;

    while (m <= routeSize) {

		// Considera o próximo vértice
        next = orderInRoute[m-1];

		// Para todas as demandas possíveis
        for (int r = 1; r <= 20*g.numberVertices; r++) {

			// Se for o primeiro vértice, a probabilidade depende apenas da sua própria demanda
			// Se não, consideramos as probabilidades de todas as demandas possíveis dos vértices anteriores
            if (m == 1 && r <= 20) {

                f[1][r] = g.vertices[next].probDemand[r];

            } else if (r <= 20*m) {
                
                for (int k = 1; k <= min(20,r); k++) {

                    double probDemandK = g.vertices[next].probDemand[k];

                    if (probDemandK != 0) {
                        f[m][r] += probDemandK*f[m-1][r-k];
                    }

                }

            } 

        }

        m++;

    }

    return f;

}

/*
probReachCapacity: Calcula a probabilidade da demanda até o vértice 'i' do grafo 'g' ser
exatamente igual a capacidade 'capacity'. Utiliza as probabilidades calculadas na função
probDemandsInRoute, a diferença é que esta função considera todos os números possíveis
de falhas, o que é equivalente a aumentar a capacidade.

Entrada:
i: vértice cuja probabilidade de demanda até ele é calculado;
g: grafo do problema sendo considerado;
f: matriz n por 20*n, onde n é o número de vértices do grafo. A posição (i, j) da matriz
indica a probabilidade da demanda até o vértice i ser j.
capacity: capacidade máxima do veículo do problema.

Saída: double indicando a probabilidade da demanda até o vértice 'i' ser exatamente igual a
capacidade máxima do veículo.
*/
double probReachCapacity(int i, Graph g, vector<vector<double>> f, int capacity) {

    double probReachCap = 0;

	// Para todos os possíveis números de falhas "q"
    for (int q = 1; q <= floor(i*20/capacity); q++) {

		// Para todas as possíveis demandas do vértice "i"
        for (int k = 1; k <= 20; k++) {
            probReachCap += g.vertices[i].probDemand[k]*f[i-1][q*capacity-k];
        }

    }

    return probReachCap;

}

/*
probReachCapacity: Calcula a probabilidade da demanda até o vértice 'i' do grafo 'g' ser
maior do que a capacidade 'capacity'. Utiliza as probabilidades calculadas na função
probDemandsInRoute, a diferença é que esta função considera todos os números possíveis
de falhas, o que é equivalente a aumentar a capacidade.

Entrada:
i: vértice cuja probabilidade de demanda até ele é calculado;
g: grafo do problema sendo considerado;
f: matriz n por 20*n, onde n é o número de vértices do grafo. A posição (i, j) da matriz
indica a probabilidade da demanda até o vértice i ser j.
capacity: capacidade máxima do veículo do problema.

Saída: double indicando a probabilidade da demanda até o vértice 'i' ser maior do que a
capacidade máxima do veículo.
*/
double probExceedsCapacity(int i, Graph g, vector<vector<double>> f, int capacity) {

    double probExceedsCap = 0, probDemandExceeds = 0;

	// Para todos os possíveis números de falhas "q"
    for (int q = 1; q <= floor(i*20/capacity); q++) {

		// Para todas as possíveis demandas do vértice anterior a "i"
        for (int k = 1; k <= 19; k++) {

			// Para todas as possíveis demandas do vértice "i" maiores do que k
            for (int r = k + 1; r <= 20; r++) {
                probDemandExceeds += g.vertices[i].probDemand[r];
            }

			// Somar probabilidade da demanda anterior a i ser q*capacity-k
			// e a demanda em i ser maior do que k
            probExceedsCap += probDemandExceeds*f[i-1][q*capacity-k];

        }

        probDemandExceeds = 0;

    }

    return probExceedsCap;

}
