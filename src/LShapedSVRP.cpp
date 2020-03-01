#include "LShapedSVRP.h"

// Exemplo TSP:
// https://www.gurobi.com/documentation/9.0/examples/tsp_cpp_cpp.html#subsubsection:tsp_c++.cpp

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
                    next = 1;

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
    GRBVar** xvars;
    int n, m, Q;
    double L;
    Graph g;
    vector<vector<int>> routes;

    optimalityCut(GRBVar** xvars, int n, int m, double lowerBound, Graph g, int capacity) {
        xvars = xvars;
        n = n;
        m = m;
        L = lowerBound;
        g = g;
        Q = capacity;
    }
protected:
    void callback() {
        try {
            if (where == GRB_CB_MIPSOL) {
                // Found an integer feasible solution - does it visit every node?
                double** xsol = new double* [n];
                for (int i = 0; i < n; i++) {
                    xsol[i] = getSolution(xvars[i], n);

                }

                // Armazenar custo esperado do segundo estágio
                routes = buildRoutesFromSol(xsol, n);
                double expectedCost = totalExpectedLength(g, Q, routes);

                // Armazenar valor da solução encontrada
                double objValue = getDoubleInfo(GRB_CB_MIPSOL_OBJ);

                // Verificar se viola objValue >= expectedCost
                if (objValue < expectedCost) {

                    GRBLinExpr expr = 0;
                    int rhs = 0;

                    /* Corte de optimalidade
                     * sum(x[i][j], xsol[i][j] = 1) <= sum(xsol[i][j]) - 1 */
                    for (int i = 0; i < n; i++) {
                        for (int j = i + 1; j < n; j++) {
                            if (xsol[i][j] > 0.5) {
                                expr += xvars[i][j];
                                rhs += 1;
                            }
                        }
                    }

                    addLazy(expr <= rhs - 1);

                }
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
            model.addConstr(u[i] <= Q, "capacity_" + to_string(i));

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
                    cout << sol[i][j] << " ";
                }
                cout << endl;
            }
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    if (sol[i][j] > 0.5)
                        sol[j][i] = sol[i][j];
                }
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