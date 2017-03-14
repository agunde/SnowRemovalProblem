package InitialSolution;
/**
 * Created by Magnu on 04.12.2016.
 */

import java.util.ArrayList;
import java.util.Vector;


//This is the GiantTourGenerator. The name says it all.
public class GiantTourGenerator {
    private int n; // number of vertices
    private int degree[]; // degrees of vertices
    private int neg[], pos[]; // unbalanced vertices
    private int path[][], edges[][],
            cheapestEdge[][], f[][];
    private boolean defined[][];
    private Vector label[][]; // names of edges
    private float c[][];
    private boolean initialised;

    public GiantTourGenerator(int vertices) {
        n = vertices;
        degree = new int[n];
        defined = new boolean[n][n];
        label = new Vector[n][n];
        c = new float[n][n];
        f = new int[n][n];
        edges = new int[n][n];
        cheapestEdge = new int[n][n];
        path = new int[n][n];
        initialised = true;
    }

    public GiantTourGenerator addedge(String lab, int u, int v, float cost) {
        if (!defined[u][v])
            label[u][v] = new Vector();
        label[u][v].addElement(lab);
        if (!defined[u][v] || c[u][v] > cost) {
            c[u][v] = cost;
            cheapestEdge[u][v] = edges[u][v];
            defined[u][v] = true;
            path[u][v] = v;
        }
        edges[u][v]++;
        degree[u]++;
        degree[v]--;
        return this;
    }

    public ArrayList<Integer> cpp(int depot) {
        checkInitialised();
        leastCostPaths();
        checkValid();
        findFeasible();
        while (improvements()) ;
        return printCPT(depot); // a tour starting from 0
    }
    //This checks that we dont have any negative cycles, and that the graph is connected.
    //Should not be a problem for us.
    private void checkValid() {
        for (int i = 0; i < n; i++)
            if (c[i][i] < 0) {
                throw new Error("Negative cycles");
            } else for (int j = 0; j < n; j++) {
                if (!defined[i][j]) {
                    throw new Error("Not strongly connected");
                }
            }
    }
    //This is the method where we find the least cost paths between imbalanced nodes.
    private void leastCostPaths() {
        for (int k = 0; k < n; k++) {
            for (int i = 0; i < n; i++) {
                if (defined[i][k]) {
                    for (int j = 0; j < n; j++) {
                        if (defined[k][j] && (!defined[i][j] || c[i][j] > c[i][k] + c[k][j])) {
                            defined[i][j] = true;
                            path[i][j] = path[i][k];
                            c[i][j] = c[i][k] + c[k][j];

                            // return on negative cycle
                            if (i == j && c[i][j] < 0) {
                                return;
                            }
                        }
                    }
                }
            }
        }

    }
    //This is the cycle canceling algorithm.
    private void findFeasible() {
        int nn = 0, np = 0;
        for (int i = 0; i < n; i++) {
            if (degree[i] < 0) nn++;
            else if (degree[i] > 0) np++;
        }
        neg = new int[nn];
        pos = new int[np];
        nn = np = 0;

        for (int i = 0; i < n; i++) {
            if (degree[i] < 0) {
                neg[nn++] = i;
            } else if (degree[i] > 0) {
                pos[np++] = i;
            }
        }
        for (int u = 0; u < nn; u++) {
            int i = neg[u];
            for (int v = 0; v < np; v++) {
                int j = pos[v];
                f[i][j] = -degree[i] < degree[j] ? -degree[i] : degree[j];
                degree[i] += f[i][j];
                degree[j] -= f[i][j];
            }
        }
    }

    private boolean improvements() {
        GiantTourGenerator R = new GiantTourGenerator(n);
        for (int u = 0; u < neg.length; u++) {
            int i = neg[u];
            for (int v = 0; v < pos.length; v++) {
                int j = pos[v];
                if (edges[i][j] > 0)
                    R.addedge(null, i, j, c[i][j]);
                if (f[i][j] != 0)
                    R.addedge(null, j, i, -c[i][j]);
            }
        }
// find a negative cycle
        R.leastCostPaths();
        // cancel the cycle (if any)
        for (int i = 0; i < n; i++){
            if (R.c[i][i] < 0){
                int k = 0, u, v;
                boolean kunset = true;
                u = i;
                do{
                    v = R.path[u][i];
                    if(R.c[u][v] < 0 && (kunset || k > f[v][u])){
                        k = f[v][u];
                        kunset = false;
                    }

                } while ((u = v) != i);
                u = i;
                do {
                    v = R.path[u][i];
                    if (R.c[u][v] < 0){
                        f[v][u] -= k;
                    }
                    else{
                        f[u][v] += k;
                    }
                } while((u=v) != i);

                return true;

            }

        }
        return false;

    }
    //This is basically the algorithm presented in the papers. We just take the cheapest arc instead of a random arc so
    //We save ourselves some calculations.
    private ArrayList<Integer> printCPT(int start){
            ArrayList<Integer> ids = new ArrayList<Integer>();
        printPath: for( int v = start;;)
    { int u = v;
        // find an adjoined path, if any
        for( int i = 0; i < n; i++ )
        if( f[u][i] > 0 )
        { v = i;
            f[u][v]--;
            //System.out.println(
            //        " Take path from "+u+" to "+v+": ");
            for( int p; u != v; u = p )
            { p = path[u][v];
                //System.out.println(" Take edge " + label[u][p].elementAt(cheapestEdge[u][p]) +" from "+u+" to "+p);
                ids.add(Integer.parseInt((String) label[u][p].elementAt(cheapestEdge[u][p]))-1);
            }
            continue printPath;
        }
        // find an existing edge, using bridge last
        v = -1;
        for( int i = 0; i < n; i++ )
        if( edges[u][i] > 0 )
            if( v == -1 || i != path[u][start] )
                v = i;
        // finished when no more bridges
        if( v == -1 )
            return ids;
        // note how this uses each edge in turn
        //System.out.println("Take edge " +label[u][v].elementAt(edges[u][v]-1) +" from "+u+" to "+v);
        ids.add(Integer.parseInt((String) label[u][v].elementAt(edges[u][v]-1))-1);

    // remove edge that's been used
        edges[u][v]--;
    }
    }


    private void checkInitialised()
    { if( !initialised )
        throw new Error("GiantTourGenerator not initialised");
// the algorithm destroys some fields
        initialised = false;
    }

    //This is just a test, to check that everything works.
    public static void main(String[] args){
        GiantTourGenerator G = new GiantTourGenerator(4);
        G.addedge("a", 0, 1, 2).addedge("b", 0, 2, 3)
                .addedge("c", 1, 2, 4).addedge("d", 1, 3, 2)
                .addedge("e", 2, 3, 1).addedge("f", 3, 0, 1);
        G.cpp(0);
    }
}
