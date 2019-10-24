#include<cmath>
#include "SVRP.h"

/*
probDemandsInRoute: Calcula todas as probabilidades de demanda total até cada vértice
salvando estes valores em uma matriz. Nesta matriz, as linhas representam o último vértice
sendo considerado e as colunas a possível demanda até este vértice. Assim, se estamos na
3a linha e 4a coluna, computamos a probabilidade de que a demanda dos vértices 1, 2 e 3
sejam iguais a 4. A ordem dos vértices segue a ordem da rota.

Entrada:
g: grafo do problema sendo considerado;
orderInRoute: vetor de inteiros que indica a ordem na qual os vértices são percorridos na rota.

Saída:
f: matriz n por (20*(n-1))+1, onde n é o número de vértices do grafo.
f[m][r] = probabilidade da demanda total dos clientes 1, ..., m é igual a r.

Observação: demanda máxima de um vértice é 20.
*/
vector<vector<double>> probDemandsInRoute(Graph g, vector<int> orderInRoute) {

	// Inicializa a matriz com probabilidades 0
	int next, m = 1, routeSize = g.numberVertices - 1;
	vector<double> v(20*routeSize + 1, 0);
	vector<vector<double>> f(g.numberVertices, v);

	f[0][0] = 1; // termo neutro do produtório

	while (m <= routeSize) {

		// Considera o próximo vértice da rota
		next = orderInRoute[m-1];

			// Para todas as demandas possíveis
			for (int r = 1; r <= 20*routeSize; r++) {

				// Se for o primeiro vértice da rota, a probabilidade depende apenas da sua própria demanda
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
probDemandsInRoute, a diferença é que esta função considera todos os números de possíveis
falhas, o que é equivalente a aumentar a capacidade.

Entrada:
i: vértice cuja probabilidade de demanda até ele é calculado;
g: grafo do problema sendo considerado;
f: matriz n por (20*(n-1))+1, onde n é o número de vértices do grafo.
f[m][r] = probabilidade da demanda total dos clientes 1, ..., m é igual a r.
capacity: capacidade máxima do veículo do problema.
orderInRoute: ordem dos vértices na rota. orderInRoute[2] = 3o cliente da rota.

Saída: double indicando a probabilidade da demanda até o vértice 'i' ser exatamente igual a
capacidade máxima do veículo.
*/
double probReachCapacity(int i, Graph g, vector<vector<double>> f, int capacity, vector<int> orderInRoute) {

	double probReachCap = 0;
	int vtx = orderInRoute[i-1];

	// Para todos os possíveis números de falhas "q"
	for (int q = 1; q <= floor(i*20/capacity); q++) {

		// Para todas as possíveis "k" demandas do vértice "i"
		for (int k = 1; k <= 20; k++) {
			probReachCap += g.vertices[vtx].probDemand[k]*f[i-1][q*capacity-k];
		}

	}

	//cout << "probReachCapacity in client " << vtx << ": "<< probReachCap << endl;
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
f: matriz n por (20*(n-1))+1, onde n é o número de vértices do grafo.
f[m][r] = probabilidade da demanda total dos clientes 1, ..., m é igual a r.
capacity: capacidade máxima do veículo do problema.
orderInRoute: ordem dos vértices na rota. orderInRoute[2] = 3o cliente da rota.

Saída: double indicando a probabilidade da demanda até o vértice 'i' ser maior do que a
capacidade máxima do veículo.
*/
double probExceedsCapacity(int i, Graph g, vector<vector<double>> f, int capacity, vector<int> orderInRoute) {

	double probExceedsCap = 0, probDemandExceeds = 0;
	int vtx = orderInRoute[i-1];

	// Para todos os possíveis números de falhas "q"
	for (int q = 1; q < floor(i*20/capacity); q++) {

		// Para todas as possíveis "k" demandas do vértice anterior a "i"
		for (int k = 1; k <= 19; k++) {

			// Para todas as possíveis demandas "r" do vértice "i" maiores do que "k"
			for (int r = k + 1; r <= 20; r++) {
				probDemandExceeds += g.vertices[vtx].probDemand[r];
			}

			// Somar probabilidade de que a demanda em "i" é maior do que "k"
			// e que a demanda anterior a "i" seja igual a q*capacity-k
			probExceedsCap += probDemandExceeds*f[i-1][q*capacity-k];
			probDemandExceeds = 0;

		}

	}

	//cout << "probExceedsCapacity in client " << vtx << ": "<< probExceedsCap << endl;
	return probExceedsCap;

}

/*
expectedLength: Calcula o custo esperado de uma rota.

Entrada:
g: grafo do problema sendo considerado;
f: matriz n por (20*(n-1))+1, onde n é o número de vértices do grafo.
f[m][r] = probabilidade da demanda total dos clientes 1, ..., m é igual a r.
capacity: capacidade máxima do veículo do problema.
orderInRoute: ordem dos vértices na rota. orderInRoute[2] = 3o cliente da rota.

Saída: double indicando o custo esperado de se percorrer uma rota dada.
*/
double expectedLength(Graph g, vector<vector<double>> f, int capacity, vector<int> orderInRoute) {

	double expectedLength = 0, accPresence = 1, probReachCap;

	/* Como um vértice pode não estar presente, computa-se o custo da possibilidade de cada
	vértice ser o primeiro,	o que é igual a ele estar presente e os anteriores não, vezes a
	sua distância	ao depósito */
	for(int i = 1; i < g.numberVertices; i++) {

		for(int r = 1; r <= i-1; r++) {
			accPresence *= (1 - g.vertices[r].probOfPresence);
		}

		expectedLength += g.adjMatrix[0][i]*g.vertices[i].probOfPresence*accPresence;
		accPresence = 1;
	}

	/* Como um v�rtice pode n�o estar presente, computa-se o custo de cada v�rtice ser o �ltimo,
	o que � igual � ele estar presente e os posteriores n�o estarem presentes vezes a dist�ncia
	ao dep�sito */
	for(int i = 1; i < g.numberVertices; i++) {

		for(int r = i+1; r < g.numberVertices; r++) {
			accPresence *= (1 - g.vertices[r].probOfPresence);
		}

		expectedLength += g.adjMatrix[i][0]*g.vertices[i].probOfPresence*accPresence;
		accPresence = 1;
	}

	/* Como um v�rtice pode n�o estar presente, computa-se o custo de cada poss�vel pr�ximo v�rtice
	da rota, o que � igual � dois v�rtices estarem presentes e os v�rtices entre eles n�o, vezes
	a dist�ncia entre esses v�rtices */
	for(int i = 1; i < g.numberVertices; i++) {

		for(int j = i+1; j < g.numberVertices; j++) {

			for(int r = i+1; r <= j-1; r++) {
				accPresence *= (1 - g.vertices[r].probOfPresence);
			}

			expectedLength += g.adjMatrix[i][j]*g.vertices[i].probOfPresence*g.vertices[j].probOfPresence*accPresence;
			accPresence = 1;
		}
	}

	/* Computar o custo "recurso" esperado, ou seja, a capacidade limite ser atingida e portanto o
	ve�culo deve retornar ao dep�sito. */
	for(int i = 1; i < g.numberVertices; i++) {

		/* Somar a probabilidade de exceder vezes o custo de retornar ao dep�sito */
		expectedLength += (g.adjMatrix[i][0] + g.adjMatrix[0][i] - g.adjMatrix[i][i])*probExceedsCapacity(i,g,f,capacity,orderInRoute);
		probReachCap = probReachCapacity(i,g,f,capacity,orderInRoute);

		/* Somar a probabilidade de atingir exatamente a capacidade vezes o custo de retornar ao dep�sito */
		for(int j = i+1; j < g.numberVertices; j++) {

			for(int r = i+1; r <= j-1; r++) {
				accPresence *= (1 - g.vertices[r].probOfPresence);
			}

			expectedLength += probReachCap*g.vertices[j].probOfPresence*(g.adjMatrix[i][0] + g.adjMatrix[0][j] - g.adjMatrix[i][j])*accPresence;
			accPresence = 1;
		}
	}

	return expectedLength;
}
