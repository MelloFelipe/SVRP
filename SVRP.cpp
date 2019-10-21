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
	int next, m = 1, routeSize = g.numberVertices - 1;
    vector<double> v(20*routeSize + 1, 0);
    vector<vector<double>> f(g.numberVertices, v);
	f[0][0] = 1;
    while (m <= routeSize) {

		// Considera o pr�ximo v�rtice
        next = orderInRoute[m-1];

		// Para todas as demandas poss�veis
        for (int r = 1; r <= 20*routeSize; r++) {

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
double probReachCapacity(int i, Graph g, vector<vector<double>> f, int capacity, vector<int> orderInRoute) {

    double probReachCap = 0;
	int vtx = orderInRoute[i-1];
	
	// Para todos os poss�veis n�meros de falhas "q"
    for (int q = 1; q <= floor(i*20/capacity); q++) {
		
		// Para todas as poss�veis demandas do v�rtice "i"
        for (int k = 1; k <= 20; k++) {
            probReachCap += g.vertices[vtx].probDemand[k]*f[i-1][q*capacity-k];
        }

    }

	//cout << "probReachCapacity in client " << vtx << ": "<< probReachCap << endl;
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
double probExceedsCapacity(int i, Graph g, vector<vector<double>> f, int capacity, vector<int> orderInRoute) {

    double probExceedsCap = 0, probDemandExceeds = 0;
	int vtx = orderInRoute[i-1];
	
	// Para todos os poss�veis n�meros de falhas "q"
    for (int q = 1; q < floor(i*20/capacity); q++) {

		// Para todas as poss�veis demandas do v�rtice anterior a "i"
        for (int k = 1; k <= 19; k++) {

			// Para todas as poss�veis demandas do v�rtice "i" maiores do que k
            for (int r = k + 1; r <= 20; r++) {
                probDemandExceeds += g.vertices[vtx].probDemand[r];
            }

			// Somar probabilidade da demanda anterior a i ser q*capacity-k
			// e a demanda em i ser maior do que k
            probExceedsCap += probDemandExceeds*f[i-1][q*capacity-k];
			probDemandExceeds = 0;

        }    

    }

	//cout << "probExceedsCapacity in client " << vtx << ": "<< probExceedsCap << endl;
    return probExceedsCap;

}

/*
expectedLength: Calcula o custo esperado do segundo est�gio dado uma rota.

Entrada:
g: grafo do problema sendo considerado;
f: matriz n por 20*n, onde n � o n�mero de v�rtices do grafo. A posi��o (i, j) da matriz
indica a probabilidade da demanda at� o v�rtice i ser j.
capacity: capacidade m�xima do ve�culo do problema.

Sa�da: double indicando o custo esperado de se percorrer uma rota dada.
*/
double expectedLength(Graph g, vector<vector<double>> f, int capacity, vector<int> orderInRoute) {
	
	double expectedLength = 0, accPresence = 1, probReachCap;
	
	/* Como um v�rtice pode n�o estar presente, computa-se o custo de cada v�rtice ser o primeiro,
	o que � igual � ele estar presente e os anteriores n�o estarem presentes vezes a dist�ncia
	ao dep�sito */
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
