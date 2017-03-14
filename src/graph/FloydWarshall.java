package graph;

//import java.text.NumberFormat;
import java.util.ArrayList;

public class FloydWarshall {

	static double[][] allPairShortestDistances;
	static int[][] successors;
	static public double timeTakenToComputeFloydWarshall;
	
	/**Returns the distance between and origin and destination in the graph.
	 * Is used to make accessing the all pairs shortest path matrix more neat.*/
	public static double distance(int originElementID, int destinationElementID){
		return allPairShortestDistances[originElementID][destinationElementID];
	}
	
	/**Returns an array with the elements on the shortest path from the 
	 * first given element to the second given element (excluding both a and b).*/
	public static ArrayList<Integer> shortestPathFromAtoB(int a, int b){
		ArrayList<Integer> path = new ArrayList<>();
		int successor = successors[a][b];
		if(successor == b){
			return path;
		}
		do {
			path.add(successor);
			if(successor == -1){
				System.out.println("The succeeding broke");
				break;
			}
			successor = successors[successor][b];
		} while(successor != b );
		return path;
	}
	
	/**The path going through all the supplied elements, and with the depot node and
	 * the path to the first element in the beginning of the returned list, and the
	 * path from the last element back to the depot node (and the depot node) at the
	 * end.*/
	public static ArrayList<Integer> completePathThroughElementsUsingDepotNode(int[] requiredElements){
		ArrayList<Integer> path = new ArrayList<>();
		path.add(Graph.getDeoptNodeIndex());
		path.addAll(shortestPathFromAtoB(Graph.getDeoptNodeIndex(), requiredElements[0]));
		for (int i = 0; i < requiredElements.length - 1; i++) {
			path.add(requiredElements[i]);
			path.addAll(shortestPathFromAtoB(requiredElements[i], requiredElements[i + 1]));
		}
		path.add(requiredElements[requiredElements.length - 1]);
		path.addAll(shortestPathFromAtoB(requiredElements[requiredElements.length - 1], Graph.getDeoptNodeIndex()));
		path.add(Graph.getDeoptNodeIndex());
		
		return path;
	}

	public static double[][] doFloydWarshall() {
		double startTime = System.currentTimeMillis();
		int numberOfElementsInGraph = Graph.nodes.length + Graph.edges.length
				+ Graph.arcs.length;
		allPairShortestDistances = new double[numberOfElementsInGraph][numberOfElementsInGraph];
		
//		Runtime runtime = Runtime.getRuntime();
//
//	    NumberFormat format = NumberFormat.getInstance();
//
//	    StringBuilder sb = new StringBuilder();
//	    long maxMemory = runtime.maxMemory();
//	    long allocatedMemory = runtime.totalMemory();
//	    long freeMemory = runtime.freeMemory();
//
//	    sb.append("free memory: " + format.format(freeMemory / 1024) + "<br/>");
//	    sb.append("allocated memory: " + format.format(allocatedMemory / 1024) + "<br/>");
//	    sb.append("max memory: " + format.format(maxMemory / 1024) + "<br/>");
//	    sb.append("total free memory: " + format.format((freeMemory + (maxMemory - allocatedMemory)) / 1024) + "<br/>");
//		
//		System.out.println(sb.toString());
	    
		successors = new int[numberOfElementsInGraph][numberOfElementsInGraph];

		// Initialize all distances as infinite
		for (int i = 0; i < numberOfElementsInGraph; i++) {
			for (int j = 0; j < numberOfElementsInGraph; j++) {
				allPairShortestDistances[i][j] = Double.POSITIVE_INFINITY;
				successors[i][j] = -1;
			}
			allPairShortestDistances[i][i] = 0;
			successors[i][i] = i;
		}
		
		int edgeOffset = Graph.nodes.length;
		int arcOffset = Graph.nodes.length + Graph.edges.length;
		
		/*
		 * For each edge, set the distance between the nodes at both end as the
		 * traversal cost of the edge, it it's shorter than the currently set
		 * shortest distance
		 */
		for (int i = 0; i < Graph.edges.length; i++) {
			Edge currentEdge = Graph.edges[i];
			int node1 = currentEdge.fromNodeId;
			int node2 = currentEdge.toNodeId;

			allPairShortestDistances[edgeOffset + i][node1] = Graph.nodes[node1].passThroughCost;
			allPairShortestDistances[node1][edgeOffset + i] = Graph.edges[i].passThroughCost;
			allPairShortestDistances[edgeOffset + i][node2] = Graph.nodes[node2].passThroughCost;
			allPairShortestDistances[node2][edgeOffset + i] = Graph.edges[i].passThroughCost;
			
			successors[edgeOffset + i][node1] = node1;
			successors[node1][edgeOffset + i] = edgeOffset + i;
			successors[edgeOffset + i][node2] = node2;
			successors[node2][edgeOffset + i] = edgeOffset + i;
		}

		// For each arc, set the distance {startnode,endnode} to the arc
		// traversal cost
		for (int i = 0; i < Graph.arcs.length; i++) {
			Arc currentArc = Graph.arcs[i];
			int fromNode = currentArc.fromNodeId;
			int toNode = currentArc.toNodeId;

			allPairShortestDistances[arcOffset + i][toNode] = Graph.nodes[toNode].passThroughCost;
			allPairShortestDistances[fromNode][arcOffset + i] = Graph.arcs[i].passThroughCost;
			
			successors[arcOffset + i][toNode] = toNode;
			successors[fromNode][arcOffset + i] = arcOffset + i;
		}
		
		// Do floyd-warshall
		for (int k = 0; k < numberOfElementsInGraph; k++) {
//			if(k%10 == 0) System.out.println("Now on the " + k + "th of " +numberOfElementsInGraph +"th iteration");
			for (int i = 0; i < numberOfElementsInGraph; i++) {
				for (int j = 0; j < numberOfElementsInGraph; j++) {
					if (allPairShortestDistances[i][j] > allPairShortestDistances[i][k]
							+ allPairShortestDistances[k][j]) {
						allPairShortestDistances[i][j] = allPairShortestDistances[i][k]
								+ allPairShortestDistances[k][j];
						successors[i][j] = successors[i][k];
					}

				}
			}
//			System.out.println("Row " + k + " of flw complete");

		}
		
		//Subtract destination cost from destinations
		for (int i = 0; i < numberOfElementsInGraph; i++) {
			for (int j = 0; j < numberOfElementsInGraph; j++) {
				if(i == j){
					continue;
				}
				allPairShortestDistances[j][i] -= Graph.getElementByID(i).getPassThroughCost();
			}
		}
		
		timeTakenToComputeFloydWarshall = 0.0 + System.currentTimeMillis() - startTime;

		return allPairShortestDistances;
	}
	
	
	public static String allPairsToString(){
		String outputString = "";
		for (int i = 0; i < allPairShortestDistances.length; i++) {
			for (int j = 0; j < allPairShortestDistances.length; j++) {
				outputString += allPairShortestDistances[i][j] + "\t";
			}
			outputString += "\n";
//			System.out.println("Finished line " + i + " of " + allPairShortestDistances.length);
		}
		return outputString;
	}
	
	public static String sucessorsToString(){
		String outputString = "";
		for (int i = 0; i < successors.length; i++) {
			for (int j = 0; j < successors.length; j++) {
				outputString += successors[i][j] + "\t";
			}
			outputString += "\n";
		}
		return outputString;
	}
	
}
