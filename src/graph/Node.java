package graph;

import java.util.Arrays;

public class Node implements ElementProperties, Cost {

	int ID;
	String name;
	double servicingCost;
	double passThroughCost;
	double demand;
	boolean isRequired;
	
	int[] inboundEdges;
	int[] outboundEdges;
	int[] inboundArcs;
	int[] outboundArcs;

	public Node(int globalElementID, String elementName, double demand, double servicingCost, boolean isRequired) {
		this.ID = globalElementID;
		this.name = elementName;
		this.demand = demand;
		this.servicingCost = servicingCost;
		this.passThroughCost = 0;
		this.isRequired = isRequired;
		
		this.inboundEdges = new int[0];
		this.outboundEdges = new int[0];
		this.inboundArcs = new int[0];
		this.outboundArcs = new int[0];
	}
	
	/**When creating an edge or arc, enables one to register it with the nodes it's connected to.*/
	public void addConnection(boolean edge, boolean inbound, int id){
		int[] increasedArray;
		if(edge){
			if(inbound){
				increasedArray = new int[inboundEdges.length + 1];
				System.arraycopy(inboundEdges, 0, increasedArray, 0, inboundEdges.length);
				increasedArray[increasedArray.length - 1] = id;
				inboundEdges = increasedArray;
			}else{
				increasedArray = new int[outboundEdges.length + 1];
				System.arraycopy(outboundEdges, 0, increasedArray, 0, outboundEdges.length);
				increasedArray[increasedArray.length - 1] = id;
				outboundEdges = increasedArray;
			}
		}else{
			if(inbound){
				increasedArray = new int[inboundArcs.length + 1];
				System.arraycopy(inboundArcs, 0, increasedArray, 0, inboundArcs.length);
				increasedArray[increasedArray.length - 1] = id;
				inboundArcs = increasedArray;				
			}else{
				increasedArray = new int[outboundArcs.length + 1];
				System.arraycopy(outboundArcs, 0, increasedArray, 0, outboundArcs.length);
				increasedArray[increasedArray.length - 1] = id;
				outboundArcs = increasedArray;
			}
		}
	}

	@Override
	public double getServicingCost() {
		return servicingCost;
	}

	@Override
	public double getPassThroughCost() {
		return passThroughCost;
	}

	@Override
	public String getName() {
		return name;
	}
	
	public void setName(String name){
		this.name = name;
	}

	@Override
	public boolean getIsRequired() {
		return isRequired;
	}

	@Override
	public double getDemand() {
		return demand;
	}
	
	public String toString(){
		String output = "\n";
		output += "Node ID: " + ID + "\t";
		output += "Node name: " + name + "\t";
		output += "Servicing Cost: " + servicingCost + "\t";
		output += "Pass Through Cost: " + passThroughCost + "\t";
		output += "Demand: " + demand + "\t";
		output += "Is required: " + isRequired + "\t";
		output += "Inbound Edges: " + Arrays.toString(inboundEdges) + "\t";
		output += "Outbound Edges: " + Arrays.toString(outboundEdges) + "\t";
		output += "Inbound Arcs: " + Arrays.toString(inboundArcs) + "\t";
		output += "Outbound Arcs: " + Arrays.toString(outboundArcs);
		return output;
	}

	@Override
	public int getID() {
		return ID;
	}
}
