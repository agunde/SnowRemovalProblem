package ColumnGeneration; /**
 * Created by andershg on 13.02.2017.
 */

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Hashtable;

public class PathBuilder {
    public ArrayList<VehicleLane> vehicleLane;
    public ArrayList<VehicleSidewalk> vehicleSidewalk;
    public InstanceData inputdata;
    public double[] dualBetaSidewalk;
    public double[] dualBetaLane;
    public double[][] dualAlphaSidewalk;
    public double[][] dualAlphaLane;
    public double[][][] dualGammaSidewalk;
    public double[][][] dualGammaLane;
    public double[] dualSigmaSidewalk;
    public double[] dualSigmaLane;


    public PathBuilder(ArrayList<VehicleLane> vehicleLane, ArrayList<VehicleSidewalk> vehicleSidewalk, InstanceData inputdata){
        this.vehicleLane = vehicleLane;
        this.vehicleSidewalk = vehicleSidewalk;
        this.inputdata = inputdata;

        this.dualBetaSidewalk = new double[this.vehicleSidewalk.size()];
        this.dualBetaLane = new double[this.vehicleLane.size()];
        this.dualAlphaSidewalk = new double[this.inputdata.antallNoder][this.inputdata.antallNoder];
        this.dualAlphaLane = new double[this.inputdata.antallNoder][this.inputdata.antallNoder];
        this.dualGammaSidewalk = new double[this.vehicleSidewalk.size()][this.inputdata.antallNoder][this.inputdata.antallNoder];
        this.dualGammaLane = new double[this.vehicleLane.size()][this.inputdata.antallNoder][this.inputdata.antallNoder];
        this.dualSigmaSidewalk = new double[this.vehicleSidewalk.size()];
        this.dualSigmaLane = new double[this.vehicleLane.size()];
    }

    public ArrayList<Label> buildPathLane(VehicleLane vehicleLane, int numberOfPaths) {
        Label L = new Label();
        L.node = inputdata.startNode;
        L.vehicle = vehicleLane;
        L.arraivingTime = 0;
        L.cost = -dualBetaLane[L.vehicle.getNumber()];
        L.lastTimePlowedNode = new int[inputdata.antallNoder][inputdata.antallNoder];
        L.numberOfTimesPlowed = new int[inputdata.antallNoder][inputdata.antallNoder];
        /*for( int[] row : L.lastTimePlowedNode){
            Arrays.fill(row,0);
        }

        for(int[] row: L.numberOfTimesPlowed){
            Arrays.fill(row,0);
        }*/
        ArrayList<Label> unprocessed = new ArrayList<>();
        Hashtable<Integer, ArrayList<Label>> unprocessedNode = new Hashtable<>();
        Hashtable<Integer, ArrayList<Label>> processedNode = new Hashtable<>();
        for (int i = 0; i < inputdata.antallNoder; i++){
            unprocessedNode.put(i, new ArrayList<>());
            processedNode.put(i, new ArrayList<>());
        }

        ArrayList<Label> processed = new ArrayList<>();
        unprocessed.add(L);
        unprocessedNode.get(L.node).add(L);
        ArrayList<Label> improvingLabels = new ArrayList<>();
        ArrayList<Label> badImprovingLabels = new ArrayList<>();

        while(!unprocessed.isEmpty()) {
            Label label = unprocessed.remove(0);
            for (Integer i : inputdata.deadheadingNeighborLane.get(label.node)) {
                if (inputdata.numberOfPlowJobsLane[label.node][i] > 0) {
                    Label newLabelPlow = ExtendLabelLane(i, label, false);
                    if (newLabelPlow != null) {
                        if (checkDominanceLane(newLabelPlow, unprocessed, unprocessedNode.get(newLabelPlow.node), improvingLabels, processedNode.get(newLabelPlow.node))) {
                            unprocessed.add(newLabelPlow);
                            ArrayList<Label> tempList = unprocessedNode.get(newLabelPlow.node);
                            tempList.add(newLabelPlow);
                        }
                        //Fjernet dominansetesten
                        /*unprocessed.add(newLabelPlow);
                        ArrayList<Label> tempList = unprocessedNode.get(newLabelPlow.node);
                        tempList.add(newLabelPlow);*/
                    }

                }
                Label newLabelDeadheading = ExtendLabelLane(i, label, true);
                if (newLabelDeadheading != null) {
                    if(newLabelDeadheading.node == inputdata.endNode && newLabelDeadheading.cost < 0){
                        if(newLabelDeadheading.cost < -5){
                            improvingLabels.add(newLabelDeadheading);
                            if(improvingLabels.size()>numberOfPaths){
                                improvingLabels.addAll(badImprovingLabels);
                                return improvingLabels;
                            }
                        }
                        else{
                            badImprovingLabels.add(newLabelDeadheading);
                        }

                    }
                    if (checkDominanceLane(newLabelDeadheading, unprocessed, unprocessedNode.get(newLabelDeadheading.node), improvingLabels, processedNode.get(newLabelDeadheading.node))) {
                        unprocessed.add(newLabelDeadheading);
                        ArrayList<Label> tempList = unprocessedNode.get(newLabelDeadheading.node);
                        tempList.add(newLabelDeadheading);
                    }
                    //Fjernet dominansetesten
                    /*unprocessed.add(newLabelDeadheading);
                    ArrayList<Label> tempList = unprocessedNode.get(newLabelDeadheading.node);
                    tempList.add(newLabelDeadheading);*/
                }
            }

            /*for(int i = 0; i < inputdata.antallNoder; i++){
                if(inputdata.deadheadingtimeLane[label.node][i] != -1){
                    if(inputdata.numberOfPlowJobsLane[label.node][i] > 0){
                        ColumnGeneration.Label newLabelPlow = ExtendLabelLane(i, label, false);
                        if(newLabelPlow != null) {
                            if(checkDominanceLane(newLabelPlow, unprocessed,unprocessedNode.get(newLabelPlow.node))){
                                unprocessed.add(newLabelPlow);
                                ArrayList<ColumnGeneration.Label> tempList = unprocessedNode.get(newLabelPlow.node);
                                tempList.add(newLabelPlow);
                            }
                        }

                    }

                    ColumnGeneration.Label newLabelDeadheading = ExtendLabelLane(i, label, true);
                    if(newLabelDeadheading != null) {
                        if(newLabelDeadheading.node == inputdata.endNode && newLabelDeadheading.cost < 0){
                            improvingLabels.add(newLabelDeadheading);
                            if(improvingLabels.size()>10){
                                return improvingLabels;
                            }

                        }
                        if(checkDominanceLane(newLabelDeadheading, unprocessed, unprocessedNode.get(newLabelDeadheading.node))){
                            unprocessed.add(newLabelDeadheading);
                            ArrayList<ColumnGeneration.Label> tempList = unprocessedNode.get(newLabelDeadheading.node);
                            tempList.add(newLabelDeadheading);
                        }
                    }
                }

            }*/

            processed.add(label);
            processedNode.get(label.node).add(label);
        }

        /*for(Label label : processedNode.get(inputdata.endNode)){
            if(label.cost < 0){
                improvingLabels.add(label);
            }
        }*/


        if(improvingLabels.size() >0){
            return improvingLabels;
        }
        if(badImprovingLabels.size() >0){
            return badImprovingLabels;
        }
        return null;
    }


    public ArrayList<Label> buildPathSidewalk(VehicleSidewalk vehicleSidewalk, int numberOfPaths){
        Label L = new Label();
        L.node = inputdata.endNode;
        L.vehicle = vehicleSidewalk;
        L.arraivingTime = inputdata.maxTime;
        L.cost = -dualBetaSidewalk[L.vehicle.getNumber()];
        L.lastTimePlowedNode = new int[inputdata.antallNoder][inputdata.antallNoder];
        for(int[] row : L.lastTimePlowedNode){
            Arrays.fill(row,inputdata.maxTime);
        }

        L.numberOfTimesPlowed = new int[inputdata.antallNoder][inputdata.antallNoder];
        /*for(int[] row : L.numberOfTimesPlowed){
            Arrays.fill(row,0);
        }*/

        ArrayList<Label> unprocessed = new ArrayList<>();
        Hashtable<Integer, ArrayList<Label>> unprocessedNode = new Hashtable<>();
        Hashtable<Integer, ArrayList<Label>> processedNode = new Hashtable<>();
        for (int i = 0; i < inputdata.antallNoder; i++){
            unprocessedNode.put(i, new ArrayList<>());
            processedNode.put(i, new ArrayList<>());
        }
        ArrayList<Label> processed = new ArrayList<>();
        unprocessed.add(L);
        unprocessedNode.get(L.node).add(L);
        ArrayList<Label> improvingLabels = new ArrayList<>();
        ArrayList<Label> badImprovingLabels = new ArrayList<>();

        while(!unprocessed.isEmpty()){
            Label label = unprocessed.remove(0);
            for(Integer i : inputdata.deadheadingNeighborSidewalkReversed.get(label.node)){
                if(inputdata.numberOfPlowJobsSidewalk[i][label.node] > 0){
                    Label newLabelPlow = ExtendLabelSidewalk(i, label, false);
                    if(newLabelPlow != null){
                        if(checkDominanceSidewalk(newLabelPlow, unprocessed,unprocessedNode.get(newLabelPlow.node), improvingLabels, processedNode.get(newLabelPlow.node))){
                            unprocessed.add(newLabelPlow);
                            ArrayList<Label> tempList = unprocessedNode.get(newLabelPlow.node);
                            tempList.add(newLabelPlow);
                        }
                        //Fjernet dominansesjekken
                        /*unprocessed.add(newLabelPlow);
                        ArrayList<Label> tempList = unprocessedNode.get(newLabelPlow.node);
                        tempList.add(newLabelPlow);*/
                    }
                }
                Label newLabel = ExtendLabelSidewalk(i,label,true);
                if(newLabel != null){
                    if(newLabel.node == inputdata.startNode && newLabel.cost < 0){
                        if(newLabel.cost < -5){
                            improvingLabels.add(newLabel);
                            if (improvingLabels.size() > numberOfPaths){
                                improvingLabels.addAll(badImprovingLabels);
                                return improvingLabels;
                            }
                        }
                        else{
                            badImprovingLabels.add(newLabel);
                        }

                    }
                    if(checkDominanceSidewalk(newLabel, unprocessed, unprocessedNode.get(newLabel.node), improvingLabels, processedNode.get(newLabel.node))){
                        unprocessed.add(newLabel);
                        ArrayList<Label> tempList = unprocessedNode.get(newLabel.node);
                        tempList.add(newLabel);
                    }
                    //Fjernet dominansetestingen
                    /*unprocessed.add(newLabel);
                    ArrayList<Label> tempList = unprocessedNode.get(newLabel.node);
                    tempList.add(newLabel);*/
                }

            }

            /*for(int i = 0; i < inputdata.antallNoder; i++){
                if(inputdata.deadheadingtimeSidewalk[i][label.node] != -1){
                    if(inputdata.numberOfPlowJobsSidewalk[i][label.node] > 0){
                        ColumnGeneration.Label newLabelPlow = ExtendLabelSidewalk(i, label, false);
                        if(newLabelPlow != null){
                            if(checkDominanceSidewalk(newLabelPlow, unprocessed,unprocessedNode.get(newLabelPlow.node))){
                                unprocessed.add(newLabelPlow);
                                ArrayList<ColumnGeneration.Label> tempList = unprocessedNode.get(newLabelPlow.node);
                                tempList.add(newLabelPlow);
                            }
                        }
                    }
                    ColumnGeneration.Label newLabel = ExtendLabelSidewalk(i,label,true);
                    if(newLabel != null){
                        if(newLabel.node == inputdata.startNode && newLabel.cost < 0){
                            improvingLabels.add(newLabel);
                            if (improvingLabels.size() > 10){
                                return improvingLabels;
                            }
                        }
                        if(checkDominanceSidewalk(newLabel, unprocessed, unprocessedNode.get(newLabel.node))){
                            unprocessed.add(newLabel);
                            ArrayList<ColumnGeneration.Label> tempList = unprocessedNode.get(newLabel.node);
                            tempList.add(newLabel);
                        }
                    }
                }
            }*/
            processed.add(label);
            processedNode.get(label.node).add(label);

        }

        for(Label label : processedNode.get(inputdata.startNode)){
            if(label.cost < 0){
                improvingLabels.add(label);
            }
        }
        if(improvingLabels.size() > 0){
            return improvingLabels;
        }
        if(badImprovingLabels.size() >0){
            return badImprovingLabels;
        }
        return null;
    }


    public Label ExtendLabelLane(int node, Label L, boolean deadhead) {
        Label L2 = new Label();
        L2.node = node;
        L2.predecessor = L;
        L2.vehicle = L.vehicle;
        L2.deadheading = deadhead;
        L2.numberOfTimesPlowed = new int[inputdata.antallNoder][inputdata.antallNoder];
        L2.lastTimePlowedNode = new int[inputdata.antallNoder][inputdata.antallNoder];
        for (int i = 0; i < inputdata.antallNoder; i++){
            L2.numberOfTimesPlowed[i] = L.numberOfTimesPlowed[i].clone();
            L2.lastTimePlowedNode[i] = L.lastTimePlowedNode[i].clone();
        }
        if(!deadhead){
            L2.arraivingTime = L.arraivingTime + inputdata.plowingtimeLane[L.node][node];
            L2.cost = L.cost - (L2.arraivingTime - L.arraivingTime)*dualSigmaLane[L2.vehicle.getNumber()]-dualAlphaLane[L.node][node]-(L.arraivingTime-L.lastTimePlowedNode[L.node][node])*dualGammaLane[L2.vehicle.getNumber()][L.node][node];
            L2.lastTimePlowedNode[L.node][node] = L.arraivingTime;
            L2.numberOfTimesPlowed[L.node][node] = L.numberOfTimesPlowed[L.node][node]+1;
            if(L2.numberOfTimesPlowed[L.node][node] > inputdata.numberOfPlowJobsLane[L.node][node]){
                return null;
            }
            /*if(L2.arraivingTime > inputdata.timeWindowLane[L.node][node]){
                return null;
            }*/
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
        L2.deadheading = deadhead;
        L2.numberOfTimesPlowed = new int[inputdata.antallNoder][inputdata.antallNoder];
        L2.lastTimePlowedNode = new int[inputdata.antallNoder][inputdata.antallNoder];
        for(int i = 0; i < inputdata.antallNoder; i++){
            L2.numberOfTimesPlowed[i] = L.numberOfTimesPlowed[i].clone();
            L2.lastTimePlowedNode[i] = L.lastTimePlowedNode[i].clone();
        }
        if(!deadhead){
            L2.arraivingTime = L.arraivingTime - inputdata.plowingtimeSidewalk[node][L.node];
            if(L2.numberOfTimesPlowed[node][L.node] > 0){
                L2.cost = L.cost -(L.arraivingTime - L2.arraivingTime)*dualSigmaSidewalk[L2.vehicle.getNumber()]-dualAlphaSidewalk[node][L.node]- L2.arraivingTime*dualGammaSidewalk[L2.vehicle.getNumber()][node][L.node] + L.lastTimePlowedNode[node][L.node]*dualGammaSidewalk[L2.vehicle.getNumber()][node][L.node];
            }
            else{
                L2.cost = L.cost -(L.arraivingTime - L2.arraivingTime)*dualSigmaSidewalk[L2.vehicle.getNumber()]-dualAlphaSidewalk[node][L.node]- L2.arraivingTime*dualGammaSidewalk[L2.vehicle.getNumber()][node][L.node];
            }
            L2.lastTimePlowedNode[node][L.node] = L2.arraivingTime;

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

    private boolean checkDominanceLane(Label newLabel, ArrayList<Label> unprocessed, ArrayList<Label> unprocessedNode, ArrayList<Label> processed, ArrayList<Label> processedNode){
        ArrayList<Label> remove = new ArrayList<>();

        for(Label oldLabel : unprocessedNode) {
            if(dominateLabelLane(oldLabel, newLabel)) {
                unprocessed.removeAll(remove);
                unprocessedNode.removeAll(remove);
                return false;
            }
            else if(dominateLabelLane(newLabel,oldLabel)) {
                remove.add(oldLabel);
            }
        }
        unprocessed.removeAll(remove);
        unprocessedNode.removeAll(remove);

        remove = new ArrayList<>();
        for(ColumnGeneration.Label oldLabel : processedNode) {
            if(dominateLabelLane(oldLabel, newLabel)) {
                processed.removeAll(remove);
                processedNode.removeAll(remove);
                return false;
            }
            else if(dominateLabelLane(newLabel,oldLabel)) {
                remove.add(oldLabel);
            }
        }
        processed.removeAll(remove);
        processedNode.removeAll(remove);
        return true;
    }

    private boolean checkDominanceSidewalk(Label newLabel, ArrayList<Label> unprocessed, ArrayList<Label> unprocessedNode, ArrayList<Label> processed, ArrayList<Label> processedNode){
        ArrayList<Label> remove = new ArrayList<Label>();

        for(Label oldLabel : unprocessedNode) {
            if(dominateLabelSidewalk(oldLabel, newLabel)) {
                unprocessed.removeAll(remove);
                unprocessedNode.removeAll(remove);
                return false;
            }
            else if(dominateLabelSidewalk(newLabel,oldLabel)) {
                remove.add(oldLabel);
            }
        }
        unprocessed.removeAll(remove);
        unprocessedNode.removeAll(remove);

        remove = new ArrayList<>();
        for(ColumnGeneration.Label oldLabel : processedNode) {
            if(dominateLabelSidewalk(oldLabel, newLabel)) {
                processed.removeAll(remove);
                processedNode.removeAll(remove);
                return false;
            }
            else if(dominateLabelSidewalk(newLabel,oldLabel)) {
                remove.add(oldLabel);
            }
        }
        processed.removeAll(remove);
        processedNode.removeAll(remove);
        return true;
    }

    private boolean dominateLabelLane(Label L1, Label L2){
        if(L1.node == L2.node && L1.arraivingTime <= L2.arraivingTime){
            double tempCostL2 = L2.cost;
            for(int i = 0; i < inputdata.antallNoder; i++){
                for(Integer j : inputdata.plowingNeighborLane.get(i)){
                    int difference = L1.numberOfTimesPlowed[i][j] - L2.numberOfTimesPlowed[i][j];
                    if(difference >0){
                        tempCostL2 = tempCostL2 + difference*(-dualAlphaLane[i][j]);
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
            for(int j  = 0; j<inputdata.antallNoder;j++){
                for(Integer i : inputdata.plowingNeighborSidewalkReversed.get(j)){
                    int difference = L1.numberOfTimesPlowed[i][j] - L2.numberOfTimesPlowed[i][j];
                    if(difference >0){
                        tempCostL2 = tempCostL2 + difference*(-dualAlphaSidewalk[i][j]);
                    }
                }
            }
            if (L1.cost <= tempCostL2){
                return true;
            }
        }
        return false;
    }

    public void setDualBetaSidewalk(double[] dualBetaSidewalk) {
        this.dualBetaSidewalk = dualBetaSidewalk;
    }

    public void setDualBetaLane(double[] dualBetaLane) {
        this.dualBetaLane = dualBetaLane;
    }

    public void setDualAlphaSidewalk(double[][] dualAlphaSidewalk) {
        this.dualAlphaSidewalk = dualAlphaSidewalk;
    }

    public void setDualAlphaLane(double[][] dualAlphaLane) {
        this.dualAlphaLane = dualAlphaLane;
    }

    public void setDualGammaSidewalk(double[][][] dualGammaSidewalk) {
        this.dualGammaSidewalk = dualGammaSidewalk;
    }

    public void setDualGammaLane(double[][][] dualGammaLane) {
        this.dualGammaLane = dualGammaLane;
    }

    public void setDualSigmaSidewalk(double[] dualSigmaSidewalk) {
        this.dualSigmaSidewalk = dualSigmaSidewalk;
    }

    public void setDualSigmaLane(double[] dualSigmaLane) {
        this.dualSigmaLane = dualSigmaLane;
    }



    //BEGYNN MED VIDERE KODE HER


}
