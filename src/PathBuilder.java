/**
 * Created by andershg on 13.02.2017.
 */

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Hashtable;

public class PathBuilder {
    public ArrayList<VehicleLane> vehicleLanes;
    public ArrayList<VehicleSidewalk> vehicleSidewalks;
    public InstanceData inputdata;
    public ArrayList<Hashtable<Integer, Boolean>> feasibilityLane;
    public ArrayList<Hashtable<Integer, Boolean>> feasibilitySidewalk;
    public Double[] dualBetaSidewalk;
    public Double[] dualBetaLane;
    public Double[][] dualAlphaSidewalk;
    public Double[][] dualAlphaLane;
    public Double[][][] dualGammaSidewalk;
    public Double[][][] dualGammaLane;
    public Double[] dualSigmaSidewalk;
    public Double[] dualSigmaLane;


    public PathBuilder(ArrayList<VehicleLane> vehicleLanes, ArrayList<VehicleSidewalk> vehicleSidewalks, InstanceData inputdata){
        this.vehicleLanes = vehicleLanes;
        this.vehicleSidewalks = vehicleSidewalks;
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

    public Label buildPathLane(VehicleLane vehicleLane) {
        Label L = new Label();
        L.node = inputdata.startNode;
        L.vehicle = vehicleLane;
        L.arraivingTime = 0;
        L.cost = -dualBetaLane[L.vehicle.getNumber()];
        L.lastTimePlowedNode = new int[inputdata.antallNoder][inputdata.antallNoder];
        Arrays.fill(L.lastTimePlowedNode, 0);
        L.numberOfTimesPlowed = new int[inputdata.antallNoder][inputdata.antallNoder];
        Arrays.fill(L.numberOfTimesPlowed, 0);

        ArrayList<Label> unprocessed = new ArrayList<>();
        ArrayList<Label> processed = new ArrayList<>();
        unprocessed.add(L);

        while(!unprocessed.isEmpty()){
            Label label = unprocessed.remove(0);
            for(int i = 0; i < inputdata.antallNoder; i++){
                if(inputdata.plowingtimeLane[label.node][i] != -1){
                    Label newLabelLane = ExtendLabelLane(i, label, false);
                    if(newLabelLane != null) {
                        if(checkDominanceLane(newLabelLane, unprocessed,processed)){
                            unprocessed.add(newLabelLane);
                        }
                    }
                    Label newLabelSidewalk = ExtendLabelLane(i, label, true);
                    if(newLabelSidewalk != null) {
                        if(checkDominanceLane(newLabelSidewalk, unprocessed, processed)){
                            unprocessed.add(newLabelSidewalk);
                        }
                    }
                }

            }
        }
        return L;
    }

    public Label buildPathSidewalk(VehicleSidewalk vehicleSidewalk){
        Label L = new Label();
        L.node = inputdata.endNode;
        L.vehicle = vehicleSidewalk;
        L.arraivingTime = inputdata.maxTime;
        L.cost = -dualBetaSidewalk[L.vehicle.getNumber()];
        L.lastTimePlowedNode = new int[inputdata.antallNoder][inputdata.antallNoder];
        Arrays.fill(L.lastTimePlowedNode, 0);
        L.numberOfTimesPlowed = new int[inputdata.antallNoder][inputdata.antallNoder];
        Arrays.fill(L.numberOfTimesPlowed, 0);

        ArrayList<Label> unprocessed = new ArrayList<>();
        ArrayList<Label> processed = new ArrayList<>();
        unprocessed.add(L);

        while(!unprocessed.isEmpty()){
            Label label = unprocessed.remove(0);
            for(int i = 0; i < inputdata.antallNoder; i++){
                if(inputdata.plowingtimeSidewalk[i][label.node] != -1){
                    if(inputdata.numberOfPlowJobsSidewalk[i][L.node] > 1){
                        Label newLabel = ExtendLabelSidewalk(i, label, false);
                        if(newLabel != null){
                            if(checkDominanceSidewalk(newLabel, unprocessed,processed)){
                                unprocessed.add(newLabel);
                            }
                        }
                    }
                    Label newLabel = ExtendLabelSidewalk(i,label,true);
                    if(newLabel != null){
                        if(checkDominanceSidewalk(newLabel, unprocessed, processed)){
                            unprocessed.add(newLabel);
                        }
                    }
                }
            }
        }
        return L;
    }


    public Label ExtendLabelLane(int node, Label L, boolean deadhead) {
        Label L2 = new Label();
        L2.node = node;
        L2.predecessor = L;
        L2.vehicle = L.vehicle;
        for (int i = 0; i < inputdata.antallNoder; i++){
            L2.numberOfTimesPlowed[i] = L.numberOfTimesPlowed[i].clone();
            L2.lastTimePlowedNode[i] = L.lastTimePlowedNode[i].clone();
        }
        if(!deadhead){
            L2.arraivingTime = L.arraivingTime + inputdata.plowingtimeLane[L.node][node];
            L2.cost = L.cost - (L2.arraivingTime - L.arraivingTime)*dualSigmaLane[L2.vehicle.getNumber()]-dualAlphaLane[L.node][node]-(L2.arraivingTime-L.lastTimePlowedNode[L.node][node])*dualGammaLane[L2.vehicle.getNumber()][L.node][node];
            L2.lastTimePlowedNode[L.node][node] = L2.arraivingTime;
            L2.numberOfTimesPlowed[L.node][node] = L.numberOfTimesPlowed[L.node][node]+1;
            if(L2.numberOfTimesPlowed[L.node][node] > inputdata.numberOfPlowJobsLane[L.node][node]){
                return null;
            }
            if(L2.arraivingTime > inputdata.timeWindowLane[L.node][node]){
                return null;
            }
        }
        if(deadhead){
            L2.arraivingTime = L.arraivingTime + inputdata.deadheadingtimeLane[L.node][node];
            L2.cost = L.cost - (L2.arraivingTime - L.arraivingTime)*dualSigmaLane[L2.vehicle.getNumber()];
        }
        if(L2.arraivingTime > inputdata.maxTime){
            return null;
        }
        return L2;
    }

    public Label ExtendLabelSidewalk(int node, Label L, boolean deadhead){
        Label L2 = new Label();
        L2.node  = node;
        L2.predecessor = L;
        L2.vehicle = L.vehicle;
        for(int i = 0; i < inputdata.antallNoder; i++){
            L2.numberOfTimesPlowed[i] = L.numberOfTimesPlowed[i].clone();
            L2.lastTimePlowedNode[i] = L.lastTimePlowedNode[i].clone();
        }
        if(!deadhead){
            L2.arraivingTime = L.arraivingTime - inputdata.plowingtimeLane[node][L.node];
            L2.cost = L.cost -(L.arraivingTime - L2.arraivingTime)*dualSigmaSidewalk[L2.vehicle.getNumber()]-dualAlphaSidewalk[node][L.node];
            if(L.numberOfTimesPlowed[node][L.node] == 0){
                L2.lastTimePlowedNode[node][L.node] = L.arraivingTime;
                L2.cost = L2.cost - L.arraivingTime*dualGammaSidewalk[L2.vehicle.getNumber()][node][L.node];
            }
            L2.numberOfTimesPlowed[node][L.node] = L.numberOfTimesPlowed[node][L.node] + 1;
            if(L2.numberOfTimesPlowed[node][L.node] > inputdata.numberOfPlowJobsSidewalk[node][L.node]){
                return null;}
        }
        if(deadhead){
            L2.arraivingTime = L.arraivingTime - inputdata.deadheadingtimeSidewalk[node][L.node];
            L2.cost = L.cost -(L.arraivingTime - L2.arraivingTime)*dualSigmaSidewalk[L2.vehicle.getNumber()];
        }
        if(L2.arraivingTime < 0){
            return null;
        }
        return L2;
    }

    private boolean checkDominanceLane(Label newLabel, ArrayList<Label> unprocessed, ArrayList<Label> processed){
        ArrayList<Label> remove = new ArrayList<Label>();

        for(Label oldLabel : unprocessed) {
            if(dominateLabelLane(oldLabel, newLabel)) {
                unprocessed.removeAll(remove);
                return false;
            }
            else if(dominateLabelLane(newLabel,oldLabel)) {
                remove.add(oldLabel);
            }
        }
        unprocessed.removeAll(remove);

        remove = new ArrayList<Label>();
        for(Label oldLabel : processed) {
            if(dominateLabelLane(oldLabel, newLabel)) {
                processed.removeAll(remove);
                return false;
            }
            else if(dominateLabelLane(newLabel,oldLabel)) {
                remove.add(oldLabel);
            }
        }
        processed.removeAll(remove);
        return true;
    }

    private boolean checkDominanceSidewalk(Label newLabel, ArrayList<Label> unprocessed, ArrayList<Label> processed){
        ArrayList<Label> remove = new ArrayList<Label>();

        for(Label oldLabel : unprocessed) {
            if(dominateLabelSidewalk(oldLabel, newLabel)) {
                unprocessed.removeAll(remove);
                return false;
            }
            else if(dominateLabelSidewalk(newLabel,oldLabel)) {
                remove.add(oldLabel);
            }
        }
        unprocessed.removeAll(remove);

        remove = new ArrayList<Label>();
        for(Label oldLabel : processed) {
            if(dominateLabelSidewalk(oldLabel, newLabel)) {
                processed.removeAll(remove);
                return false;
            }
            else if(dominateLabelSidewalk(newLabel,oldLabel)) {
                remove.add(oldLabel);
            }
        }
        processed.removeAll(remove);
        return true;
    }

    private boolean dominateLabelLane(Label L1, Label L2){
        if(L1.node == L2.node && L1.arraivingTime <= L2.arraivingTime){
            double tempCostL2 = L2.cost;
            for(int i = 0; i < inputdata.antallNoder; i++){
                for(int j = 0; j < inputdata.antallNoder; j++){
                    int difference = L1.numberOfTimesPlowed[i][j] - L2.numberOfTimesPlowed[i][j];
                    if(difference >0){
                        tempCostL2 = tempCostL2 + difference*(-dualAlphaLane[i][j] - inputdata.plowingtimeLane[i][j]*dualSigmaLane[L2.vehicle.getNumber()]);
                    }
                }
            }
            if (L1.cost <= tempCostL2){
                return true;
            }
        }
       return false;
    }

    private boolean dominateLabelSidewalk(Label L1, Label L2){
        if(L1.node == L2.node && L1.arraivingTime >= L2.arraivingTime){
            double tempCostL2 = L2.cost;
            for(int i  = 0; i<inputdata.antallNoder;i++){
                for(int j = 0; j<inputdata.antallNoder; j++){
                    int difference = L1.numberOfTimesPlowed[i][j] - L2.numberOfTimesPlowed[i][j];
                    if(difference >0){
                        tempCostL2 = tempCostL2 + difference*(-dualAlphaSidewalk[i][j] - inputdata.plowingtimeSidewalk[i][j]*dualSigmaSidewalk[L2.vehicle.getNumber()]);
                    }
                }
            }
            if (L1.cost <= tempCostL2){
                return true;
            }
        }
        return false;
    }

    public void setDualBetaSidewalk(Double[] dualBetaSidewalk) {
        this.dualBetaSidewalk = dualBetaSidewalk;
    }

    public void setDualBetaLane(Double[] dualBetaLane) {
        this.dualBetaLane = dualBetaLane;
    }

    public void setDualAlphaSidewalk(Double[][] dualAlphaSidewalk) {
        this.dualAlphaSidewalk = dualAlphaSidewalk;
    }

    public void setDualAlphaLane(Double[][] dualAlphaLane) {
        this.dualAlphaLane = dualAlphaLane;
    }

    public void setDualGammaSidewalk(Double[][][] dualGammaSidewalk) {
        this.dualGammaSidewalk = dualGammaSidewalk;
    }

    public void setDualGammaLane(Double[][][] dualGammaLane) {
        this.dualGammaLane = dualGammaLane;
    }

    public void setDualSigmaSidewalk(Double[] dualSigmaSidewalk) {
        this.dualSigmaSidewalk = dualSigmaSidewalk;
    }

    public void setDualSigmaLane(Double[] dualSigmaLane) {
        this.dualSigmaLane = dualSigmaLane;
    }



    //BEGYNN MED VIDERE KODE HER


}
