package graph;

public class Edge implements ElementProperties, Cost {

	int ID;
	String name;
	double servicingCost;
	double passThroughCost;
	double demand;
	boolean isRequired;
	int fromNodeId;
	int toNodeId;

	public Edge(int globalElementID, String elementName, int fromNode, int toNode,
			double traversalCost, double servicingCost, double demand,
			boolean isRequired) {
		this.ID = globalElementID;
		this.name = elementName;
		this.fromNodeId = fromNode;
		this.toNodeId = toNode;
		this.passThroughCost = traversalCost;
		this.servicingCost = servicingCost;
		this.demand = demand;
		this.isRequired = isRequired;
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
		output += "Edge ID: " + ID + "\t";
		output += "Edge name: " + name + "\t";
		output += "From Node: " + fromNodeId + "\t";
		output += "To Node: " + toNodeId + "\t";
		output += "Servicing Cost: " + servicingCost + "\t";
		output += "Pass Through Cost: " + passThroughCost + "\t";
		output += "Demand: " + demand + "\t";
		output += "Is required: " + isRequired;
		return output;
	}

	@Override
	public int getID() {
		return ID;
	}
}
