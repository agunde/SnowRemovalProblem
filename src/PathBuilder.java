/**
 * Created by andershg on 13.02.2017.
 */

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Hashtable;
import java.util.Vector;

public class PathBuilder {
    public ArrayList<VehicleLane> vehicleLanes;
    public ArrayList<VehicleSidewalk> vehicleSidewalks;
    public ArrayList<Nodes> nodes;
    public InstanceData inputdata;
    public ArrayList<Hashtable<Integer, Boolean>> feasibilityLane;
    public ArrayList<Hashtable<Integer, Boolean>> feasibilitySidewalk;


    public PathBuilder(ArrayList<VehicleLane> vehicleLanes, ArrayList<VehicleSidewalk> vehicleSidewalks, ArrayList<Nodes> nodes, InstanceData inputdata){
        this.vehicleLanes = vehicleLanes;
        this.vehicleSidewalks = vehicleSidewalks;
        this.nodes = nodes;
        this.inputdata = inputdata;
        feasibilityLane = new ArrayList<Hashtable<Integer, Boolean>>();
        for(int k = 0; k < vehicleLanes.size(); k++){
            feasibilityLane.add(new Hashtable<Integer, Boolean>());
        }
        feasibilitySidewalk = new ArrayList<Hashtable<Integer, Boolean>>();
        for(int k = 0; k < vehicleSidewalks.size(); k++){
            feasibilitySidewalk.add(new Hashtable<Integer, Boolean>());
        }
    }

    public Label buildPathLane(VehicleLane vehicleLane, Double[] dualValues) {
        Label L = new Label();
        return L;
    }


}
