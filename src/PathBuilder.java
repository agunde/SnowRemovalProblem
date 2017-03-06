/**
 * Created by andershg on 13.02.2017.
 */

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Hashtable;

public class PathBuilder {
    public ArrayList<VehicleLane> vehicleLane;
    public ArrayList<VehicleSidewalk> vehicleSidewalk;
    public InstanceData inputdata;
    public ArrayList<Hashtable<Integer, Boolean>> feasibilityLane;
    public ArrayList<Hashtable<Integer, Boolean>> feasibilitySidewalk;
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
        feasibilityLane = new ArrayList<Hashtable<Integer, Boolean>>();
        for(int k = 0; k < vehicleLane.size(); k++){
            feasibilityLane.add(new Hashtable<Integer, Boolean>());
        }
        feasibilitySidewalk = new ArrayList<Hashtable<Integer, Boolean>>();
        for(int k = 0; k < vehicleSidewalk.size(); k++){
            feasibilitySidewalk.add(new Hashtable<Integer, Boolean>());
        }
        this.dualBetaSidewalk = new double[this.vehicleSidewalk.size()];
        this.dualBetaLane = new double[this.vehicleLane.size()];
        this.dualAlphaSidewalk = new double[this.inputdata.antallNoder][this.inputdata.antallNoder];
        this.dualAlphaLane = new double[this.inputdata.antallNoder][this.inputdata.antallNoder];
        this.dualGammaSidewalk = new double[this.vehicleSidewalk.size()][this.inputdata.antallNoder][this.inputdata.antallNoder];
        this.dualGammaLane = new double[this.vehicleLane.size()][this.inputdata.antallNoder][this.inputdata.antallNoder];
        this.dualSigmaSidewalk = new double[this.vehicleSidewalk.size()];
        this.dualSigmaLane = new double[this.vehicleLane.size()];
    }

    public Label buildPathLane(VehicleLane vehicleLane) {
        Label L = new Label();
        L.node = inputdata.startNode;
        L.vehicle = vehicleLane;
        L.arraivingTime = 0;
        L.cost = -dualBetaLane[L.vehicle.getNumber()];
        L.lastTimePlowedNode = new int[inputdata.antallNoder][inputdata.antallNoder];
        for( int[] row : L.lastTimePlowedNode){
            Arrays.fill(row,0);
        }
        L.numberOfTimesPlowed = new int[inputdata.antallNoder][inputdata.antallNoder];
        for(int[] row: L.numberOfTimesPlowed){
            Arrays.fill(row,0);
        }
        ArrayList<Label> unprocessed = new ArrayList<>();
        ArrayList<Label> processed = new ArrayList<>();
        unprocessed.add(L);

        while(!unprocessed.isEmpty()){
            Label label = unprocessed.remove(0);
            for(int i = 0; i < inputdata.antallNoder; i++){
                if(inputdata.deadheadingtimeLane[label.node][i] != -1){
                    if(inputdata.numberOfPlowJobsLane[label.node][i] > 0){
                        Label newLabelPlow = ExtendLabelLane(i, label, false);
                        if(newLabelPlow != null) {
                            if(checkDominanceLane(newLabelPlow, unprocessed,processed)){
                                unprocessed.add(newLabelPlow);
                            }
                        }

                    }

                    Label newLabelDeadheading = ExtendLabelLane(i, label, true);
                    if(newLabelDeadheading != null) {
                        if(newLabelDeadheading.node == inputdata.endNode && newLabelDeadheading.cost < 0){
                            return label;
                        }
                        if(checkDominanceLane(newLabelDeadheading, unprocessed, processed)){
                            unprocessed.add(newLabelDeadheading);
                        }
                    }
                }

            }
            processed.add(label);


        }
        return null;
    }

    //fortsett her
    public Label buildPathSidewalk(VehicleSidewalk vehicleSidewalk){
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
        for(int[] row : L.numberOfTimesPlowed){
            Arrays.fill(row,0);
        }

        ArrayList<Label> unprocessed = new ArrayList<>();
        ArrayList<Label> processed = new ArrayList<>();
        unprocessed.add(L);

        while(!unprocessed.isEmpty()){
            Label label = unprocessed.remove(0);
            for(int i = 0; i < inputdata.antallNoder; i++){
                if(inputdata.deadheadingtimeSidewalk[i][label.node] != -1){
                    if(inputdata.numberOfPlowJobsSidewalk[i][label.node] > 0){
                        Label newLabel = ExtendLabelSidewalk(i, label, false);
                        if(newLabel != null){
                            if(checkDominanceSidewalk(newLabel, unprocessed,processed)){
                                unprocessed.add(newLabel);
                            }
                        }
                    }
                    Label newLabel = ExtendLabelSidewalk(i,label,true);
                    if(newLabel != null){
                        if(newLabel.node == inputdata.startNode && newLabel.cost < 0){
                            return label;
                        }
                        if(checkDominanceSidewalk(newLabel, unprocessed, processed)){
                            unprocessed.add(newLabel);
                        }
                    }
                }
            }
            processed.add(label);

        }
        return null;
    }


    public Label ExtendLabelLane(int node, Label L, boolean deadhead) {
        Label L2 = new Label();
        L2.node = node;
        L2.predecessor = L;
        L2.vehicle = L.vehicle;
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
        L2.numberOfTimesPlowed = new int[inputdata.antallNoder][inputdata.antallNoder];
        L2.lastTimePlowedNode = new int[inputdata.antallNoder][inputdata.antallNoder];
        for(int i = 0; i < inputdata.antallNoder; i++){
            L2.numberOfTimesPlowed[i] = L.numberOfTimesPlowed[i].clone();
            L2.lastTimePlowedNode[i] = L.lastTimePlowedNode[i].clone();
        }
        if(!deadhead){
            L2.arraivingTime = L.arraivingTime - inputdata.plowingtimeSidewalk[node][L.node];
            L2.cost = L.cost -(L.arraivingTime - L2.arraivingTime)*dualSigmaSidewalk[L2.vehicle.getNumber()]-dualAlphaSidewalk[node][L.node];
            if(L.numberOfTimesPlowed[node][L.node] == 0){
                L2.lastTimePlowedNode[node][L.node] = L2.arraivingTime;
                L2.cost = L2.cost - L2.arraivingTime*dualGammaSidewalk[L2.vehicle.getNumber()][node][L.node];
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
    //GJØR ET FORSØK PÅ Å GENERERE ALLE PATHS

     public ArrayList<Label> generateAllPathsLane(VehicleLane vehicleLane){
        ArrayList<Label>  list = new ArrayList<>();
         Label L = new Label();
         L.node = inputdata.startNode;
         L.vehicle = vehicleLane;
         L.arraivingTime = 0;
         //L.cost = -dualBetaLane[L.vehicle.getNumber()];
         L.lastTimePlowedNode = new int[inputdata.antallNoder][inputdata.antallNoder];
         for( int[] row : L.lastTimePlowedNode){
             Arrays.fill(row,0);
         }
         L.numberOfTimesPlowed = new int[inputdata.antallNoder][inputdata.antallNoder];
         for(int[] row: L.numberOfTimesPlowed){
             Arrays.fill(row,0);
         }
         ArrayList<Label> unprocessed = new ArrayList<>();
         ArrayList<Label> processed = new ArrayList<>();
         unprocessed.add(L);

         while(!unprocessed.isEmpty()){
             Label label = unprocessed.remove(0);
             for(int i = 0; i < inputdata.antallNoder; i++){
                 if(inputdata.deadheadingtimeLane[label.node][i] != -1){
                     if(inputdata.numberOfPlowJobsLane[label.node][i] > 0){
                         Label newLabelPlow = ExtendLabelLaneAllPaths(i, label, false);
                         if(newLabelPlow != null) {
                             if(true){
                                 unprocessed.add(newLabelPlow);
                             }
                         }

                     }

                     Label newLabelDeadheading = ExtendLabelLaneAllPaths(i, label, true);
                     if(newLabelDeadheading != null) {
                         if(newLabelDeadheading.node == inputdata.endNode){
                             list.add(newLabelDeadheading);
                         }
                         unprocessed.add(newLabelDeadheading);
                     }
                 }

             }
             processed.add(label);


         }
         return list;

     }

    public Label ExtendLabelLaneAllPaths(int node, Label L, boolean deadhead) {
        Label L2 = new Label();
        L2.node = node;
        L2.predecessor = L;
        L2.vehicle = L.vehicle;
        L2.numberOfTimesPlowed = new int[inputdata.antallNoder][inputdata.antallNoder];
        L2.lastTimePlowedNode = new int[inputdata.antallNoder][inputdata.antallNoder];
        for (int i = 0; i < inputdata.antallNoder; i++){
            L2.numberOfTimesPlowed[i] = L.numberOfTimesPlowed[i].clone();
            L2.lastTimePlowedNode[i] = L.lastTimePlowedNode[i].clone();
        }
        if(!deadhead){
            L2.arraivingTime = L.arraivingTime + inputdata.plowingtimeLane[L.node][node];
            //L2.cost = L.cost - (L2.arraivingTime - L.arraivingTime)*dualSigmaLane[L2.vehicle.getNumber()]-dualAlphaLane[L.node][node]-(L.arraivingTime-L.lastTimePlowedNode[L.node][node])*dualGammaLane[L2.vehicle.getNumber()][L.node][node];
            L2.lastTimePlowedNode[L.node][node] = L.arraivingTime;
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
           //L2.cost = L.cost - (L2.arraivingTime - L.arraivingTime)*dualSigmaLane[L2.vehicle.getNumber()];
        }
        if(L2.arraivingTime > inputdata.maxTime){
            return null;
        }
        return L2;
    }

    public ArrayList<Label> generateAllPathsSidewalk(VehicleSidewalk vehicleSidewalk){
        ArrayList<Label> list = new ArrayList<>();
        Label L = new Label();
        L.node = inputdata.endNode;
        L.vehicle = vehicleSidewalk;
        L.arraivingTime = inputdata.maxTime;
        //L.cost = -dualBetaSidewalk[L.vehicle.getNumber()];
        L.lastTimePlowedNode = new int[inputdata.antallNoder][inputdata.antallNoder];
        for(int[] row : L.lastTimePlowedNode){
            Arrays.fill(row,inputdata.maxTime);
        }

        L.numberOfTimesPlowed = new int[inputdata.antallNoder][inputdata.antallNoder];
        for(int[] row : L.numberOfTimesPlowed){
            Arrays.fill(row,0);
        }

        ArrayList<Label> unprocessed = new ArrayList<>();
        ArrayList<Label> processed = new ArrayList<>();
        unprocessed.add(L);

        while(!unprocessed.isEmpty()){
            Label label = unprocessed.remove(0);
            for(int i = 0; i < inputdata.antallNoder; i++){
                if(inputdata.deadheadingtimeSidewalk[i][label.node] != -1){
                    if(inputdata.numberOfPlowJobsSidewalk[i][label.node] > 0){
                        Label newLabel = ExtendLabelSidewalkAllPaths(i, label, false);
                        if(newLabel != null){
                            if(true){
                                unprocessed.add(newLabel);
                            }
                        }
                    }
                    Label newLabel = ExtendLabelSidewalkAllPaths(i,label,true);
                    if(newLabel != null){
                        if(newLabel.node == inputdata.startNode){
                            list.add(newLabel);
                        }
                        if(true){
                            unprocessed.add(newLabel);
                        }
                    }
                }
            }
            processed.add(label);

        }
        return list;
    }

    public Label ExtendLabelSidewalkAllPaths(int node, Label L, boolean deadhead){
        Label L2 = new Label();
        L2.node  = node;
        L2.predecessor = L;
        L2.vehicle = L.vehicle;
        L2.numberOfTimesPlowed = new int[inputdata.antallNoder][inputdata.antallNoder];
        L2.lastTimePlowedNode = new int[inputdata.antallNoder][inputdata.antallNoder];
        for(int i = 0; i < inputdata.antallNoder; i++){
            L2.numberOfTimesPlowed[i] = L.numberOfTimesPlowed[i].clone();
            L2.lastTimePlowedNode[i] = L.lastTimePlowedNode[i].clone();
        }
        if(!deadhead){
            L2.arraivingTime = L.arraivingTime - inputdata.plowingtimeSidewalk[node][L.node];
            //L2.cost = L.cost -(L.arraivingTime - L2.arraivingTime)*dualSigmaSidewalk[L2.vehicle.getNumber()]-dualAlphaSidewalk[node][L.node];
            if(L.numberOfTimesPlowed[node][L.node] == 0){
                L2.lastTimePlowedNode[node][L.node] = L2.arraivingTime;
                //L2.cost = L2.cost - L2.arraivingTime*dualGammaSidewalk[L2.vehicle.getNumber()][node][L.node];
            }
            L2.numberOfTimesPlowed[node][L.node] = L.numberOfTimesPlowed[node][L.node] + 1;
            if(L2.numberOfTimesPlowed[node][L.node] > inputdata.numberOfPlowJobsSidewalk[node][L.node]){
                return null;}
        }
        if(deadhead){
            L2.arraivingTime = L.arraivingTime - inputdata.deadheadingtimeSidewalk[node][L.node];
            //L2.cost = L.cost -(L.arraivingTime - L2.arraivingTime)*dualSigmaSidewalk[L2.vehicle.getNumber()];
        }
        if(L2.arraivingTime < 0){
            return null;
        }
        return L2;
    }


}
