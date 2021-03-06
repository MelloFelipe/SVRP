#include "LShapedSVRP.h"

// Exemplo TSP:
// https://www.gurobi.com/documentation/9.0/examples/tsp_cpp_cpp.html#subsubsection:tsp_c++.cpp

double partialRouteExpectedCost(vector<int> S, double uExpectedDemand, double uDistance, vector<int> T, Graph g, int Q) {
    double expectedCost = 0.0;

    vector<int> h;
    int i;

    for (i = 0; i < S.size(); i++) {
        h.push_back(S[i]);
    }
    h.push_back(0);
    for (i = 0; i < T.size(); i++) {
        h.push_back(T[i]);
    }

    // Inicializa a matriz com probabilidades 0
    int next, orderInRoute = 1, routeSize = h.size();
    vector<double> v(20 * routeSize + 1, 0);
    vector<vector<double>> f(h.size() + 1, v);
    double probDemandK;

    f[0][0] = 1; // probabilidade da carga do veiculo até o depósito ser 0

    while (orderInRoute <= routeSize) {

        // Considera o próximo cliente da rota
        next = h[orderInRoute - 1];

        // Probabilidade de não haver nenhuma carga até o cliente, ou seja, todos ausentes
        f[orderInRoute][0] = (1 - g.vertices[next].probOfPresence) * f[orderInRoute - 1][0];

        // Para todas as demandas até o cliente possíveis
        for (int dem = 1; dem <= 20 * orderInRoute; dem++) {

            // Probabilidade do cliente estar ausente, mantendo a mesma demanda anterior
            f[orderInRoute][dem] += (1 - g.vertices[next].probOfPresence) * f[orderInRoute - 1][dem];

            // Para todas as demandas possíveis do cliente
            for (int k = 1; k <= min(20, dem); k++) {

                // Probabilidade do cliente estar presente com uma demanda k
                probDemandK = g.vertices[next].probOfPresence * g.vertices[next].probDemand[k];

                if (probDemandK > 0) {
                    f[orderInRoute][dem] += probDemandK * f[orderInRoute - 1][dem - k];
                }
            }
        }
        orderInRoute++;
    }

    return expectedCost;
}

vector<int> buildHeuristicR(double** sol, int n, Graph g, int Q) {
    int i, j, maxClient = 0;
    double max = 0.0;

    cout << "xsol:" << endl;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            cout << sol[i][j] << " ";
        }
        cout << endl;
    }

    for (i = 1; i < n; i++) {
        for (j = 2; j < n; j++) {
            if (sol[0][i] + sol[i][j] > max) {
                maxClient = i;
                max = sol[0][i] + sol[i][j];
            }
        }
    }

    // maxClient is vi
    vector<int> h;
    h.push_back(maxClient);
    double hExpectedDemand = g.expectedDemand[maxClient];

    while (maxClient != 0) {
        maxClient = 0;
        max = 0.0;
        double sum = 0.0;

        for (i = 0; i < n; i++) {
            if (hExpectedDemand + g.expectedDemand[i] < Q) {
                for (j = 0; j < h.size(); j++) {
                    if (i == h[j] && i != 0) {
                        sum = 0.0;
                        break;
                    }
                    sum += sol[h[j]][i];
                }
                if (sum > max) {
                    maxClient = i;
                    max = sum;
                }
            }
        }

        if (maxClient != 0) {
            h.push_back(maxClient);
            hExpectedDemand += g.expectedDemand[maxClient];
        }
    }

    return h;
}

vector<vector<int>> buildRoutesFromSol(double** sol, int n) {

    vector<vector<int>> routes;
    vector<bool> seen;
    seen.push_back(true);
    for (int i = 1; i < n; i++)
        seen.push_back(false);

    for (int first = 1; first < n; first++) {

        /* Construir rota a partir do próximo vértice adjascente ao depósito
         * que ainda não foi visitado */
        if (sol[0][first] > 0.5 && !seen[first]) {

            vector<int> route;

            // Adicionar primeiro cliente na rota
            route.push_back(first);
            seen[first] = true;

            // Construir rota a partir desse cliente
            int current = first;
            for (int next = 1; next < n; next++) {

                // Encontrar próximo na rota
                if (sol[current][next] > 0.5 && !seen[next]) {

                    // Adicioná-lo
                    route.push_back(next);
                    seen[next] = true;

                    // Atualizar cliente atual e resetar próximo
                    current = next;
                    next = 0;

                }
            }

            // Rota construída
            routes.push_back(route);
        }
    }

    return routes;

}

class optimalityCut : public GRBCallback
{
public:
    GRBVar** x;
    int n, m, Q;
    double L;
    Graph graph;
    vector<vector<int>> routes;
    vector<int> R, S, T, U;

    optimalityCut(GRBVar** xvars, int xn, int m, double lowerBound, Graph g, int capacity) {
        x = xvars;
        n = xn;
        m = m;
        L = lowerBound;
        graph = g;
        Q = capacity;
    }
protected:
    void callback() {
        try {
            
            if (where == GRB_CB_MIPNODE) {
                // MIP node callback

                if (getIntInfo(GRB_CB_MIPNODE_STATUS) == GRB_OPTIMAL) {

                    double** xsol = new double* [n];
                    for (int i = 0; i < n; i++) {
                        xsol[i] = getNodeRel(x[i], n);
                    }
                    R = buildHeuristicR(xsol, n, graph, Q);
                    S.clear();
                    S.push_back(R[0]);
                    T.clear();
                    double max = 0.0;
                    int argMax = 0;
                    for (int i = 1; i < R.size(); i++) {
                        if (xsol[R[i]][0] > max) {
                            max = xsol[R[i]][0];
                            argMax = i;
                        }
                    }
                    if (argMax != 0) {
                        T.push_back(R[argMax]);
                    }
                    U = R;
                    remove(U.begin(), U.end(), U[S[0]]);
                    if (!T.empty)
                        remove(U.begin(), U.end(), U[T[0]]);
                    double uExpectedDemand = 0.0;
                    double uDistance = numeric_limits<double>::max();
                    for (int i = 0; i < U.size(); i++) {
                        uExpectedDemand += graph.expectedDemand[U[i]];
                        if (graph.adjMatrix[U[i]][0] < uDistance)
                            uDistance = graph.adjMatrix[U[i]][0];
                    }
                    // Calcular custo recurso esperado da rota S, u, T
                    double Ph = partialRouteExpectedCost(S, uExpectedDemand, uDistance, T, graph, Q);
                }
            }

            if (where == GRB_CB_MIPSOL) {
                // Solução inteira viável encontrada
                //cout << "Solucao inteira viavel encontrada" << endl;
                double** xsol = new double* [n];
                for (int i = 0; i < n; i++) {
                    xsol[i] = getSolution(x[i], n);
                }
                /*cout << "xsol:" << endl;
                for (int i = 0; i < n; i++) {
                    for (int j = 0; j < n; j++) {
                        cout << xsol[i][j] << " ";
                    }
                    cout << endl;
                }*/
                // Armazenar custo esperado do segundo estágio
                routes = buildRoutesFromSol(xsol, n);
                /*cout << "Rotas:" << endl;
                for (int i = 0; i < routes.size(); i++) {
                    for (int j = 0; j < routes[i].size(); j++) {
                        cout << routes[i][j] << " ";
                    }
                    cout << endl;
                }*/
                double expectedCost = totalExpectedLength(graph, Q, routes);

                // Armazenar valor da solução encontrada
                double objValue = getDoubleInfo(GRB_CB_MIPSOL_OBJ);

                //cout << "objValue: " << objValue << "expectedCost: " << expectedCost << endl;
                // Verificar se viola objValue >= expectedCost
                if (L < expectedCost - objValue) {
                    GRBLinExpr expr = 0;
                    int rhs = 0;

                    /* Corte de optimalidade
                     * sum(x[i][j], xsol[i][j] = 1) <= sum(xsol[i][j]) - 1 */
                    for (int i = 0; i < n; i++) {
                        for (int j = 0; j < n; j++) {
                            if (xsol[i][j] > 0.5) {
                                expr += x[i][j];
                                rhs += 1;
                            }
                        }
                    }

                    addLazy(expr <= rhs - 1);

                }
                for (int i = 0; i < n; i++)
                    delete[] xsol[i];
                delete[] xsol;
            }
            
        }
        catch (GRBException e) {
            cout << "Error number: " << e.getErrorCode() << endl;
            cout << e.getMessage() << endl;
        }
        catch (...) {
            cout << "Error during callback" << endl;
        }
    }
};

void solveSVRP(Graph g, int m, int Q, double L) {
    GRBEnv *env = NULL;
    GRBVar **x = NULL;
    GRBVar *u = NULL;

    int n = g.numberVertices;

    x = new GRBVar*[n];
    for (int i = 0; i < n; i++)
        x[i] = new GRBVar[n];

    u = new GRBVar[n];

    try {

        // Inicializar ambiente e modelo
        env = new GRBEnv();
        GRBModel model = GRBModel(*env);

        /* Indicar que restrições do modelo serão 
         * adicionadas via callback */
        model.set(GRB_IntParam_LazyConstraints, 1);

        /* Criar variáveis binárias x[i][j] para cada aresta (i,j) e contínuas
         * u[i] para cada vértice i */
        for (int i = 0; i < n; i++) {

            if (i > 0)
                u[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "u_"+to_string(i));

            for (int j = 0; j < n; j++) {
                
                /* Cada variável recebe um custo associado que corresponde à distância 
                 * entre os pontos i e j. */
                x[i][j] = model.addVar(
                    0.0, 1.0, 
                    g.adjMatrix[i][j],
                    GRB_BINARY, 
                    "x_"+to_string(i)+"_"+to_string(j)
                );
            }
        }

        GRBLinExpr expr1 = 0, expr2 = 0;
        for (int j = 1; j < n; j++) {
            expr1 += x[0][j];
            expr2 += x[j][0];
        }
        
        /* Restrição sum(x[0][j]) = 2*m 
         * - as m rotas devem começar e terminar no depósito. */
        model.addConstr(expr1 == m, "m_routes_go");
        model.addConstr(expr2 == m, "m_routes_back");

        for (int i = 1; i < n; i++) {

            expr1 = 0;
            expr2 = 0;
            for (int j = 0; j < n; j++) {
                if (i != j) {
                    expr1 += x[i][j];
                    expr2 += x[j][i];
                }
            }

            /* Restrição sum(x[i][j]) = 2 para todo i > 0 fixo:
            * - cada cliente faz parte de uma rota com duas arestas incidindo sobre ele. */
            model.addConstr(expr1 == 1, "degr2go_" + to_string(i));
            model.addConstr(expr2 == 1, "degr2back_" + to_string(i));

            for (int j = 1; j < n; j++) {

                /* Restrição u[i] + q[j] = u[j] se x[i][j] = 1 para todo i != 0, j != 0:
                * - eliminação de subciclos (MTZ); q[j] é a demanda média do cliente j. */
                if (i != j)
                    model.addGenConstrIndicator(
                        x[i][j], true, // indicator x[i][j] == 1
                        u[i] + g.expectedDemand[j] == u[j],
                        "subtourelim_" + to_string(i) + "_" + to_string(j)
                    );
            }

            /* Restrição u[i] >= q[i] para todo cliente:
             * - a demanda de i deve ser atendida. */
            model.addConstr(u[i] >= g.expectedDemand[i], "met_demand_" + to_string(i));

            /* Restrição u[i] <= Q para todo cliente:
             * - a capacidade do veículo não pode ser ultrapassada. */
            model.addConstr(u[i] <= g.maxDemand, "capacity_" + to_string(i));

        }

        // Forbid edge from node back to itself
        for (int i = 0; i < n; i++)
            x[i][i].set(GRB_DoubleAttr_UB, 0);

        // Set callback function
        optimalityCut cb = optimalityCut(x, n, m, L, g, Q);

        model.setCallback(&cb);

        model.update();
        model.write("debug.lp");

        // Optimize model
        model.optimize();

        // Extract solution

        if (model.get(GRB_IntAttr_SolCount) > 0) {
            double** sol = new double* [n];
            for (int i = 0; i < n; i++)
                sol[i] = model.get(GRB_DoubleAttr_X, x[i], n);

            cout << "Matriz de solucao:" << endl;
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    if (sol[i][j] > 0.5)
                        cout << 1 << " ";
                    else
                        cout << 0 << " ";
                }
                cout << endl;
            }
            
            // Armazenar custo esperado do segundo estágio
            vector<vector<int>> routes = buildRoutesFromSol(sol, n);
            double expectedCost = totalExpectedLength(g, Q, routes);
            cout << "Rotas:" << endl;
            for (int i = 0; i < routes.size(); i++) {
                for (int j = 0; j < routes[i].size(); j++) {
                    cout << routes[i][j] << " ";
                }
                cout << endl;
            }
            cout << "totalExpectedLength: " << expectedCost << endl;
        }

    } catch (GRBException e) {
        cout << "Error number: " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch (...) {
        cout << "Error during optimization." << endl;
    }

    cout << "TERMINEI LSHAPED" << endl;
}