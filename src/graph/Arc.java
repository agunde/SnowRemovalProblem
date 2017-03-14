package graph;

public class Arc implements ElementProperties, Cost {
	
	int ID;
	String name;
	double servicingCost;
	double passThroughCost;
	double demand;
	boolean isRequired;
	int fromNodeId;
	int toNodeId;

	public Arc(int globalElementID, String elementName, int fromNode, int toNode,
			double traversalCost, double servicingCost, double demand,
			boolean isRequired) {
		// TODO Auto-generated constructor stub
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
		output += "Arc ID: " + ID + "\t";
		output += "Arc name: " + name + "\t";
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
