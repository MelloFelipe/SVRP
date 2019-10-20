#include<cmath>
#include "SVRP.h"

/*
probDemandsInRoute: Calcula todas as probabilidades de demanda total at� cada v�rtice
salvando estes valores em uma matriz. Nesta matriz, as linhas representam o �ltimo v�rtice
sendo considerado e as colunas a poss�vel demanda at� este v�rtice. Assim, se estamos na
3� linha e 4� coluna, computamos a probabilidade de que a demanda dos v�rtices 1, 2 e 3
sejam iguais a 4.

Entrada:
g: grafo do problema sendo considerado;
orderInRoute: vetor de inteiros que indica a ordem na qual os v�rtices s�o percorridos na rota.

Sa�da:
f: matriz n por 20*n, onde n � o n�mero de v�rtices do grafo. A posi��o (i, j) da matriz
indica a probabilidade da demanda at� o v�rtice i ser j.

Observa��o: demanda m�xima de um v�rtice � 20.
*/
vector<vector<double>> probDemandsInRoute(Graph g, vector<int> orderInRoute) {

	// Inicializa a matriz com probabilidades 0
    vector<double> v(20*g.numberVertices + 1, 0);
    vector<vector<double>> f(g.numberVertices, v);
    int next, m = 1, routeSize = g.numberVertices - 1;

    while (m <= routeSize) {

		// Considera o pr�ximo v�rtice
        next = orderInRoute[m-1];

		// Para todas as demandas poss�veis
        for (int r = 1; r <= 20*g.numberVertices; r++) {

			// Se for o primeiro v�rtice, a probabilidade depende apenas da sua pr�pria demanda
			// Se n�o, consideramos as probabilidades de todas as demandas poss�veis dos v�rtices anteriores
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
probReachCapacity: Calcula a probabilidade da demanda at� o v�rtice 'i' do grafo 'g' ser
exatamente igual a capacidade 'capacity'. Utiliza as probabilidades calculadas na fun��o
probDemandsInRoute, a diferen�a � que esta fun��o considera todos os n�meros poss�veis
de falhas, o que � equivalente a aumentar a capacidade.

Entrada:
i: v�rtice cuja probabilidade de demanda at� ele � calculado;
g: grafo do problema sendo considerado;
f: matriz n por 20*n, onde n � o n�mero de v�rtices do grafo. A posi��o (i, j) da matriz
indica a probabilidade da demanda at� o v�rtice i ser j.
capacity: capacidade m�xima do ve�culo do problema.

Sa�da: double indicando a probabilidade da demanda at� o v�rtice 'i' ser exatamente igual a
capacidade m�xima do ve�culo.
*/
double probReachCapacity(int i, Graph g, vector<vector<double>> f, int capacity) {

    double probReachCap = 0;

	// Para todos os poss�veis n�meros de falhas "q"
    for (int q = 1; q <= floor(i*20/capacity); q++) {

		// Para todas as poss�veis demandas do v�rtice "i"
        for (int k = 1; k <= 20; k++) {
            probReachCap += g.vertices[i].probDemand[k]*f[i-1][q*capacity-k];
        }

    }

    return probReachCap;

}

/*
probReachCapacity: Calcula a probabilidade da demanda at� o v�rtice 'i' do grafo 'g' ser
maior do que a capacidade 'capacity'. Utiliza as probabilidades calculadas na fun��o
probDemandsInRoute, a diferen�a � que esta fun��o considera todos os n�meros poss�veis
de falhas, o que � equivalente a aumentar a capacidade.

Entrada:
i: v�rtice cuja probabilidade de demanda at� ele � calculado;
g: grafo do problema sendo considerado;
f: matriz n por 20*n, onde n � o n�mero de v�rtices do grafo. A posi��o (i, j) da matriz
indica a probabilidade da demanda at� o v�rtice i ser j.
capacity: capacidade m�xima do ve�culo do problema.

Sa�da: double indicando a probabilidade da demanda at� o v�rtice 'i' ser maior do que a
capacidade m�xima do ve�culo.
*/
double probExceedsCapacity(int i, Graph g, vector<vector<double>> f, int capacity) {

    double probExceedsCap = 0, probDemandExceeds = 0;

	// Para todos os poss�veis n�meros de falhas "q"
    for (int q = 1; q <= floor(i*20/capacity); q++) {

		// Para todas as poss�veis demandas do v�rtice anterior a "i"
        for (int k = 1; k <= 19; k++) {

			// Para todas as poss�veis demandas do v�rtice "i" maiores do que k
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
