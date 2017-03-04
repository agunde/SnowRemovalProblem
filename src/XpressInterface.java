/**
 * Created by Anders H. Gundersen on 13.02.2017.
 */

import java.util.ArrayList;
import java.util.Hashtable;

import com.dashoptimization.XOctrl;
import com.dashoptimization.XPRB;
import com.dashoptimization.XPRBctr;
import com.dashoptimization.XPRBexpr;
import com.dashoptimization.XPRBprob;
import com.dashoptimization.XPRBvar;


public class XpressInterface {
    private XPRB xprb = new XPRB();
    private XPRBprob problem;

    private Hashtable<Integer, Label> pathList;

    //Variables
    private ArrayList<ArrayList<XPRBvar>> routeVariables;
    private XPRBvar[][] precedenceVariable;
    private XPRBvar makespanVariable;
    private XPRBvar[] waitVariable;



    //Constraints
    private XPRBctr[][] allLanesPlowedCons;
    private XPRBctr[][] allSidewalksPlowedCons;
    private XPRBctr[] chooseRouteLaneCons;
    private XPRBctr[] chooseRouteSidewalkCons;
    private XPRBctr[][][] precedenceLaneCons;
    private XPRBctr[][][] precedenceSidewalkCons;
    private XPRBctr[] makespanLaneCons;
    private XPRBctr[] makespanSidewalkCons;
    private XPRBctr[] maxWaitTimeCons;

    //Objective
    private XPRBexpr objective;

    private double totalTimeInSub;
    private double totalTimeInMaster;

    public ArrayList<VehicleLane>  vehicleLane;
    public ArrayList<VehicleSidewalk> vehicleSidewalk;
    public InstanceData inputdata;
    public PathBuilder builder;
    private Filewriter filewriter;

    public XpressInterface(ArrayList<VehicleLane> vehicleLane, ArrayList<VehicleSidewalk> vehicleSidewalk, InstanceData inputdata) {
        this.vehicleLane = vehicleLane;
        this.vehicleSidewalk = vehicleSidewalk;
        this.inputdata = inputdata;
        this.builder = new PathBuilder(vehicleLane, vehicleSidewalk, inputdata);
//		builder.BuildPaths(vessels.get(1),15);
//		System.exit(0);
        this.pathList = new Hashtable<Integer, Label>();
        this.filewriter = new Filewriter("output.txt");
        buildProblem();
        solveProblem();
    }

    private void buildProblem() {
        this.problem = xprb.newProb("problem 1");
        problem.setCutMode(1);
        //this.problem.setXPRSdblControl(XPRS.OPTIMALITYTOL, 0.00000000001);
//		this.problem.setXPRSintControl(XOctrl.XPRS_PRESOLVE, 0);
        this.problem.setXPRSintControl(XOctrl.XPRS_OUTPUTLOG, 1);
        this.problem.setXPRSintControl(XOctrl.XPRS_MIPLOG, -100);
        this.problem.setXPRSintControl(XOctrl.XPRS_CUTSTRATEGY,0);
//		this.problem.setXPRSintControl(XOctrl.XPRS_MAXTIME,-1000);
//		this.problem.setXPRSintControl(XOctrl.XPRS_HEURSTRATEGY, 3);
//		this.problem.getXPRSprob().setIntControl(XPRS.HEURSTRATEGY, 3);
//		this.problem.getXPRSprob().setIntControl(XPRS.HEURSEARCHTREESELECT, 1100);
//		this.problem.setXPRSintControl(XPRS.THREADS, 1);
//		this.problem.getXPRSprob().setIntControl(XPRS.THREADS, 1);
//		this.problem.setMsgLevel(4);
//		this.problem.setDictionarySize(XPRB.DICT_NAMES, 0);
//		this.problem.setColOrder(1);
        this.objective = new XPRBexpr();
        this.waitVariable = new XPRBvar[vehicleSidewalk.size()];
        this.precedenceVariable = new XPRBvar[inputdata.antallNoder][inputdata.antallNoder];
        this.routeVariables = new  ArrayList<ArrayList<XPRBvar>>();

        for(int k = 0; k < vehicleSidewalk.size(); k++){
            this.waitVariable[k] = problem.newVar("waitVar "+k, XPRB.PL,0, XPRB.INFINITY);
        }

        this.makespanVariable = problem.newVar("makespanVar", XPRB.PL,0,XPRB.INFINITY);
        this.objective.addTerm(this.makespanVariable,1);

        for (int i = 0; i < inputdata.antallNoder; i++){
            for(int j = 0; j < inputdata.antallNoder; j++){
                this.precedenceVariable[i][j] = problem.newVar("precedence "+i+" "+j, XPRB.BV);
            }
        }






        //Constraints
        this.allLanesPlowedCons = new XPRBctr[inputdata.antallNoder][inputdata.antallNoder];
        this.allSidewalksPlowedCons = new XPRBctr[inputdata.antallNoder][inputdata.antallNoder];
        this.chooseRouteLaneCons = new XPRBctr[vehicleLane.size()];
        this.chooseRouteSidewalkCons = new XPRBctr[vehicleSidewalk.size()];
        this.precedenceLaneCons = new XPRBctr[vehicleLane.size()][inputdata.antallNoder][inputdata.antallNoder];
        this.precedenceSidewalkCons = new XPRBctr[vehicleSidewalk.size()][inputdata.antallNoder][inputdata.antallNoder];
        this.makespanLaneCons = new XPRBctr[vehicleLane.size()];
        this.makespanSidewalkCons = new XPRBctr[vehicleSidewalk.size()];
        this.maxWaitTimeCons = new XPRBctr[vehicleSidewalk.size()];

        for(int i = 0; i < inputdata.antallNoder; i++){
            for(int j = 0; j < inputdata.antallNoder; j++) {
                if(inputdata.plowingtimeLane[i][j] > 0){
                    this.allLanesPlowedCons[i][j] = problem.newCtr("Plowing demand lanes con");
                    this.allLanesPlowedCons[i][j].setType(XPRB.G);
                    this.allLanesPlowedCons[i][j].add(inputdata.numberOfLanesOnArc[i][j]);
                }
                if(inputdata.plowingtimeSidewalk[i][j] > 0){
                    this.allSidewalksPlowedCons[i][j] = problem.newCtr("Plowing demand sidewalks con");
                    this.allSidewalksPlowedCons[i][j].setType(XPRB.G);
                    this.allSidewalksPlowedCons[i][j].add(inputdata.numberofSidewalksOnArc[i][j]);
                }
            }
        }

        for(int k = 0; k < vehicleLane.size(); k++){
            this.chooseRouteLaneCons[k] = problem.newCtr("Choose route lane con");
            this.chooseRouteLaneCons[k].setType(XPRB.E);
            this.chooseRouteLaneCons[k].add(1);

            this.makespanLaneCons[k] = problem.newCtr("Makespan lane con");
            this.makespanLaneCons[k].setType(XPRB.L);
            this.makespanLaneCons[k].add(0);
            this.makespanLaneCons[k].addTerm(this.makespanVariable,-1);
        }

        for(int k = 0; k < vehicleSidewalk.size(); k++){
            this.chooseRouteSidewalkCons[k] = problem.newCtr("Choose route sidewalk con");
            this.chooseRouteSidewalkCons[k].setType(XPRB.E);
            this.chooseRouteSidewalkCons[k].add(1);

            this.makespanSidewalkCons[k] = problem.newCtr("Makespan sidewalk con");
            this.makespanSidewalkCons[k].setType(XPRB.L);
            this.makespanSidewalkCons[k].add(-inputdata.maxTime);
            this.makespanSidewalkCons[k].addTerm(this.makespanVariable,-1);
            this.makespanSidewalkCons[k].addTerm(this.waitVariable[k],-1);

            this.maxWaitTimeCons[k] = problem.newCtr("Max wait time cons");
            this.maxWaitTimeCons[k].setType(XPRB.L);
            this.maxWaitTimeCons[k].add(inputdata.maxTime);
            this.maxWaitTimeCons[k].addTerm(this.waitVariable[k], 1);
        }

        for(int k = 0; k < vehicleLane.size(); k++){
            for(int i = 0; i <inputdata.antallNoder; i++){
                for(int j = 0; j < inputdata.antallNoder; j++){
                    if(inputdata.plowingtimeLane[i][j] > 0 && inputdata.plowingtimeSidewalk[i][j] > 0){
                        this.precedenceLaneCons[k][i][j] = problem.newCtr("Precedence lane con");
                        this.precedenceLaneCons[k][i][j].setType(XPRB.L);
                        this.precedenceLaneCons[k][i][j].add(0);
                        this.precedenceLaneCons[k][i][j].addTerm(this.precedenceVariable[i][j],-1);
                    }
                }
            }
        }

        for(int k = 0; k < vehicleSidewalk.size(); k++){
            for(int i = 0; i < inputdata.antallNoder; i++){
                for(int j = 0; j < inputdata.antallNoder; j++){
                    if(inputdata.plowingtimeLane[i][j] > 0 && inputdata.plowingtimeSidewalk[i][j] > 0){
                        this.precedenceSidewalkCons[k][i][j] = problem.newCtr("Precedence sidewalk con");
                        this.precedenceLaneCons[k][i][j].setType(XPRB.L);
                        this.precedenceSidewalkCons[k][i][j].add(0);
                        this.precedenceSidewalkCons[k][i][j].addTerm(this.precedenceVariable[i][j],1);
                        this.precedenceSidewalkCons[k][i][j].addTerm(this.waitVariable[k],1);
                    }
                }
            }
        }



    }
//Oppdatere dualveridiene her
    private void solveProblem(){
        //Lagt til en midlertidig metode
        generateInitialPaths();
        //Midlertidig metode slutt
        boolean optimalSolutionFound = false;
        while(!optimalSolutionFound){
            boolean tempBoolean = false;
            solveMasterProblem();
            for(VehicleLane v: vehicleLane) {
                Label labelLane = builder.buildPathLane(v);
                if(labelLane != null){
                    addLabelToMaster(labelLane, true);
                    tempBoolean = true;
                }
            }
            for(int v = 0; v < vehicleSidewalk.size(); v++) {
                Label labelSidewalk = builder.buildPathSidewalk(vehicleSidewalk.get(v));
                if(labelSidewalk != null){
                    addLabelToMaster(labelSidewalk, false);
                    tempBoolean = true;
                }
            }
            if (tempBoolean == false){
                optimalSolutionFound = true;
            }
        }
    }

    private void solveMasterProblem(){
        problem.setObj(this.objective);
        problem.setSense(XPRB.MINIM);
        problem.sync(XPRB.XPRS_PROB);
        problem.solve("g");

        double[] dualBetaSidewalk = new double[this.vehicleSidewalk.size()];
        double[] dualBetaLane = new double[this.vehicleLane.size()];
        double[][] dualAlphaSidewalk = new double[this.inputdata.antallNoder][this.inputdata.antallNoder];
        double[][] dualAlphaLane = new double[this.inputdata.antallNoder][this.inputdata.antallNoder];
        double[][][] dualGammaSidewalk = new double[this.vehicleSidewalk.size()][this.inputdata.antallNoder][this.inputdata.antallNoder];
        double[][][] dualGammaLane = new double[this.vehicleLane.size()][this.inputdata.antallNoder][this.inputdata.antallNoder];
        double[] dualSigmaSidewalk = new double[this.vehicleSidewalk.size()];
        double[] dualSigmaLane = new double[this.vehicleLane.size()];

        for(int i = 0; i < inputdata.antallNoder; i++){
            for(int j = 0; j < inputdata.antallNoder; j++){
                if(inputdata.plowingtimeLane[i][j] > 0){
                    dualAlphaLane[i][j] = allLanesPlowedCons[i][j].getDual();
                }
                if(inputdata.plowingtimeSidewalk[i][j] > 0){
                    dualAlphaSidewalk[i][j] = allSidewalksPlowedCons[i][j].getDual();
                }
            }
        }

        for(int k = 0; k < vehicleSidewalk.size(); k++){
            dualBetaSidewalk[k] = chooseRouteSidewalkCons[k].getDual();

            dualSigmaSidewalk[k] = maxWaitTimeCons[k].getDual();

            for(int i = 0; i < inputdata.antallNoder; i++){
                for(int j = 0; j < inputdata.antallNoder; j++){
                    if(inputdata.plowingtimeLane[i][j] > 0 && inputdata.plowingtimeSidewalk[i][j] > 0){
                        dualGammaSidewalk[k][i][j] = precedenceSidewalkCons[k][i][j].getDual();
                    }
                }
            }

        }

        for(int k = 0; k < vehicleLane.size(); k++){
            dualBetaLane[k] = chooseRouteLaneCons[k].getDual();

            dualSigmaLane[k] = makespanLaneCons[k].getDual();

            for(int i = 0; i < inputdata.antallNoder; i++){
                for(int j = 0; j < inputdata.antallNoder; j++){
                    if(inputdata.plowingtimeLane[i][j] > 0 && inputdata.plowingtimeSidewalk[i][j] > 0){
                        dualGammaLane[k][i][j] = precedenceLaneCons[k][i][j].getDual();
                    }
                }
            }
        }


        builder.setDualAlphaLane(dualAlphaLane);
        builder.setDualAlphaSidewalk(dualAlphaSidewalk);
        builder.setDualBetaLane(dualBetaLane);
        builder.setDualBetaSidewalk(dualBetaSidewalk);
        builder.setDualGammaLane(dualGammaLane);
        builder.setDualGammaSidewalk(dualGammaSidewalk);
        builder.setDualSigmaLane(dualSigmaLane);
        builder.setDualSigmaSidewalk(dualSigmaSidewalk);

    }


    private void addLabelToMaster(Label label, boolean LaneVehicle){
        XPRBvar lambdaVar = problem.newVar("lambda "+routeVariables.size(),XPRB.BV,0,XPRB.INFINITY);

        if(LaneVehicle == true){
            routeVariables.get(label.vehicle.getNumber()).add(lambdaVar);

            this.chooseRouteLaneCons[label.vehicle.getNumber()].addTerm(lambdaVar,1);
            this.makespanLaneCons[label.vehicle.getNumber()].addTerm(lambdaVar, label.arraivingTime);
        }
        else if(LaneVehicle == false){
            routeVariables.get(vehicleLane.size()+label.vehicle.getNumber()).add(lambdaVar);

            this.chooseRouteSidewalkCons[label.vehicle.getNumber()].addTerm(lambdaVar,1);
            this.maxWaitTimeCons[label.vehicle.getNumber()].addTerm(lambdaVar,inputdata.maxTime - label.arraivingTime);
        }
        pathList.put(routeVariables.size()-1, label);

        for(int i = 0; i < inputdata.antallNoder; i++){
            for(int j = 0; j < inputdata.antallNoder; j++){
                if(LaneVehicle == true){
                    if(inputdata.numberOfPlowJobsLane[i][j] > 0){
                        this.allLanesPlowedCons[i][j].addTerm(lambdaVar, label.numberOfTimesPlowed[i][j]);

                        if(inputdata.numberOfPlowJobsSidewalk[i][j] > 0){
                            this.precedenceLaneCons[label.vehicle.getNumber()][i][j].addTerm(lambdaVar,label.lastTimePlowedNode[i][j]);
                        }
                    }
                }
                else if(LaneVehicle == false){
                    if(inputdata.numberOfPlowJobsSidewalk[i][j] > 0){
                        this.allSidewalksPlowedCons[i][j].addTerm(lambdaVar, label.numberOfTimesPlowed[i][j]);

                        if(inputdata.numberOfPlowJobsLane[i][j] > 0){
                            this.precedenceSidewalkCons[label.vehicle.getNumber()][i][j].addTerm(lambdaVar,-label.lastTimePlowedNode[i][j]);
                        }
                    }
                }
            }
        }

    }

    //Midlertidig metode
    private void generateInitialPaths(){
        //lag tre nye labels her
        Label label1 = new Label();
        label1.node = 5;
        label1.vehicle = vehicleLane.get(0);
        label1.arraivingTime = 20;
        label1.cost = 0;
        int[][] lastTimePlowedNodeLabel1 = { {0,0,0,0,0,0},
                {0,0,3,0,0,0},
                {0,6,0,13,0,0},
                {0,0,17,0,0,0},
                {0,0,0,0,0,0},
                {0,0,0,0,0,0}};
        label1.lastTimePlowedNode = lastTimePlowedNodeLabel1;
        int[][] numberOfTimesPlowedLabel1 = { {0,0,0,0,0,0},
                {0,0,1,0,0,0},
                {0,1,0,1,0,0},
                {0,0,1,0,0,0},
                {0,0,0,0,0,0},
                {0,0,0,0,0,0}};
        label1.numberOfTimesPlowed = numberOfTimesPlowedLabel1;

        Label label2 = new Label();
        label2.node = 5;
        label2.vehicle = vehicleLane.get(1);
        label2.arraivingTime = 26;
        label2.cost = 0;
        int[][] lastTimePlowedNodeLabel2 = { {0,0,0,0,0,0},
                {0,0,0,0,6,0},
                {0,0,0,0,0,0},
                {0,0,0,0,20,0},
                {0,12,0,19,0,0},
                {0,0,0,0,0,0}};
        label2.lastTimePlowedNode = lastTimePlowedNodeLabel2;
        int[][] numberOfTimesPlowedLabel2 = { {0,0,0,0,0,0},
                {0,0,0,0,1,0},
                {0,0,0,0,0,0},
                {0,0,0,0,1,0},
                {0,1,0,1,0,0},
                {0,0,0,0,0,0}};
        label2.numberOfTimesPlowed = numberOfTimesPlowedLabel2;

        Label label3 = new Label();
        label3.node = 5;
        label3.vehicle = vehicleSidewalk.get(0);
        label3.arraivingTime = 24;
        label3.cost = 0;
        int[][] lastTimePlowedNodeLabel3 = { {30,30,30,30,30,30},
                {30,30,30,30,30,30},
                {30,30,30,15,30,30},
                {30,30,9,30,21,30},
                {30,30,30,30,30,30},
                {30,30,30,30,30,30}};
        label3.lastTimePlowedNode = lastTimePlowedNodeLabel3;
        int[][] numberOfTimesPlowedLabel3 = { {0,0,0,0,0,0},
                {0,0,0,0,0,0},
                {0,0,0,1,0,0},
                {0,0,1,0,1,0},
                {0,0,0,0,0,0},
                {0,0,0,0,0,0}};
        label3.numberOfTimesPlowed = numberOfTimesPlowedLabel3;

        /*Label label4 = new Label();
        label4.node = 5;
        label4.vehicle = vehicleSidewalk.get(1);
        label4.arraivingTime = 28;
        label4.cost = 0;
        int[][] lastTimePlowedNodeLabel4 = { {0,0,0,0,0,0},
                {0,0,0,0,0,0},
                {0,0,0,0,0,0},
                {0,0,0,0,22,0},
                {0,0,0,0,0,0},
                {0,0,0,0,0,0}};
        label4.lastTimePlowedNode = lastTimePlowedNodeLabel4;
        int[][] numberOfTimesPlowedLabel4 = { {0,0,0,0,0,0},
                {0,0,0,0,0,0},
                {0,0,0,0,0,0},
                {0,0,0,0,1,0},
                {0,0,0,0,0,0},
                {0,0,0,0,0,0}};
        label4.numberOfTimesPlowed = numberOfTimesPlowedLabel4;*/

        addLabelToMaster(label1,true);
        addLabelToMaster(label2,true);
        addLabelToMaster(label3,false);
        //addLabelToMaster(label4,false);
    }

}
