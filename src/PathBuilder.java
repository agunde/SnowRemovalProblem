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
        L.node = 1;
        L.vehicle = vehicleLane;
        L.arraivingTime = 0;
        L.cost = -dualValues[1];
        L.lastTimePlowedNode = new int[nodes.size()][nodes.size()];
        Arrays.fill(L.lastTimePlowedNode, 0);
        L.numberOfTimesPlowed = new int[nodes.size()][nodes.size()];
        Arrays.fill(L.numberOfTimesPlowed, 0);

        ArrayList<Label> unprocessed = new ArrayList<>();
        ArrayList<Label> processed = new ArrayList<>();
        unprocessed.add(L);

        while(!unprocessed.isEmpty()){
            Label label = unprocessed.remove(0);
            for(int i = 0; i < nodes.size(); i++){
                if(inputdata.distanceLane[label.node][i] != -1){
                    Label newLabel = ExtendLabel(i, label);
                    if(newLabel != null) {
                        if(checkDominance(newLabel, unprocessed,processed)){
                    }
                }
            }
        }
        return L;
    }

    public Label buildPathSidewalk(VehicleLane vehicleLane, Double[] dualValues){
        Label L = new Label();
        return L;
    }


}
