package InitialSolution;
// A Java program for Floyd Warshall All Pairs Shortest
// Path algorithm. This should not need any explanation.

public class floydWarshall {
    static int INF = 99999, V;
    int[][] board;
    int[][] path;

    public int[][] Algorithm(int graph[][]) {
        V = graph[0].length;
        int dist[][] = new int[V][V];
        int next[][] = new int[V][V];
        int i, j, k;



        /* Initialize the solution matrix same as input graph matrix.
           Or we can say the initial values of shortest distances
           are based on shortest paths considering no intermediate
           vertex. */
        for (i = 0; i < V; i++){
            for (j = 0; j < V; j++){
                dist[i][j] = graph[i][j];
                if(graph[i][j] > 0){
                    next[i][j] = j;
                }
                else{
                    next[i][j] = -1;
                }
            }
        }

        for(int x = 0; x < V; x++){
            for(int y = 0; y < V; y++){
                if (dist[x][y] == -1){
                    dist[x][y] = INF;
                }
            }
        }

        for (k = 0; k < V; k++) {
            // Pick all vertices as source one by one
            for (i = 0; i < V; i++) {
                // Pick all vertices as destination for the
                // above picked source
                for (j = 0; j < V; j++) {
                    // If vertex k is on the shortest path from
                    // i to j, then update the value of dist[i][j]
                    if (dist[i][k] + dist[k][j] < dist[i][j]){
                        dist[i][j] = dist[i][k] + dist[k][j];
                        next[i][j] = next[i][k];
                    }
                }
            }
        }

        for(int x = 0; x < V; x++){
            for(int y = 0; y < V; y++){
                if (dist[x][y] == INF){
                    dist[x][y] = -1;
                }
            }
        }

        this.path = next;
        this.board = dist;

        // Print the shortest distance matrix
        //printSolution(dist);

        return dist;

    }

    void printSolution(int dist[][])
    {
        for (int i=0; i<V; ++i)
        {
            for (int j=0; j<V; ++j)
            {
                if (dist[i][j]==INF)
                    System.out.print("INF ");
                else
                    System.out.print(dist[i][j]+"   ");
            }
            System.out.println();
        }
    }


}