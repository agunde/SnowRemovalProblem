package graph;

public interface ElementProperties {
	public int getID();
	public String getName();
	public boolean getIsRequired();
	
	public double getServicingCost();
	public double getPassThroughCost();
	public double getDemand();
}
