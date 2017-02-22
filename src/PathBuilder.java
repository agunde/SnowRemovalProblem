/**
 * Created by andershg on 13.02.2017.
 */

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Hashtable;

public class PathBuilder {
    public ArrayList<VehicleLane> vehicleLanes;
    public ArrayList<VehicleSidewalk> vehicleSidewalks;
    public ArrayList<Node> nodes;
    public InstanceData inputdata;
    public ArrayList<Hashtable<Integer, Boolean>> feasibilityLane;
    public ArrayList<Hashtable<Integer, Boolean>> feasibilitySidewalk;


    public PathBuilder(ArrayList<VehicleLane> vehicleLanes, ArrayList<VehicleSidewalk> vehicleSidewalks, ArrayList<Node> nodes, InstanceData inputdata){
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
                    Label newLabel = ExtendLabelLane(i, label, false, dualValues);
                    if(newLabel != null) {
                        if(checkDominance(newLabel, unprocessed,processed)){
                            unprocessed.add(newLabel);
                        }
                    }
                    Label newLabel = ExtendLabelLane(i, label, true, dualValues);
                    if(newLabel != null) {
                        if(checkDominance(newLabel, unprocessed, processes)){
                            unprocessed.add(newLabel);
                        }
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

    //Spørsmål: HVORDAN SKAL MAN HÅNDTERE DUALVERDIENE
    public Label ExtendLabelLane(int node, Label L, boolean deadhead, Double[] dualValues) {
        Label L2 = new Label();
        L2.node = node;
        L2.predecessor = L;
        L2.vehicle = L.vehicle;
        //Man må legge inn kostnad her
        for (int i = 0; i < nodes.size(); i++){
            L2.numberOfTimesPlowed[i] = L.numberOfTimesPlowed[i].clone();
            L2.lastTimePlowedNode[i] = L.lastTimePlowedNode[i].clone();
        }
        if(!deadhead){
            L2.arraivingTime = L.arraivingTime + inputdata.distanceLane[L.node][node];
            L2.lastTimePlowedNode[L.node][node] = L2.arraivingTime;
            L2.numberOfTimesPlowed[L.node][node] = L.numberOfTimesPlowed[L.node][node];
            if(L2.numberOfTimesPlowed[L.node][node] > inputdata.numberOfPlowJobsLane[L.node][node]){
                return null;
            }
            if(L2.arraivingTime > inputdata.timeWindowLane[L.node][node]){
                return null;
            }
        }
        if(deadhead){
            L2.arraivingTime = L.arraivingTime + inputdata.deadheadingtimeLane[L.node][node];
        }
        if(L2.arraivingTime > inputdata.maxTime){
            return null;
        }
        return L2;
    }

    //BEGYNN MED VIDERE KODE HER


}
