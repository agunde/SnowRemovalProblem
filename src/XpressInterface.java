/**
 * Created by Anders H. Gundersen on 13.02.2017.
 */

import java.lang.reflect.Array;
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
    private ArrayList<XPRBvar> routeVariables;
    private XPRBvar[][] precedenceVariable;
    private XPRBvar makespanVariable;
    private XPRBvar[] waitVariable;

    private XPRBvar[][] slackVariableAllLanePlowed;
    private XPRBvar[][] slackVariableAllSidewalkPlowed;
    private XPRBvar[] slackVariableChooseRouteLane;
    private XPRBvar[] slackVariableChooseRouteSidewalk;
    private XPRBvar[][][] slackVariablePrecedenceLane;
    private XPRBvar[][][] slackVariablePrecedenceSidewalk;
    private XPRBvar[] slackVariableMaxWaitTime;
    private XPRBvar[] slackVariableMakespanLane;
    private XPRBvar[] slackVariableMakespanSidewalk;



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
        //solveProblemAllPaths();
    }

    private void buildProblem() {
        this.problem = xprb.newProb("problem 1");
        /*problem.setCutMode(1);*/

        //this.problem.setXPRSdblControl(XPRS.OPTIMALITYTOL, 0.00000000001);
//		this.problem.setXPRSintControl(XOctrl.XPRS_PRESOLVE, 0);

        /*this.problem.setXPRSintControl(XOctrl.XPRS_OUTPUTLOG, 1);
        this.problem.setXPRSintControl(XOctrl.XPRS_MIPLOG, -100);
        this.problem.setXPRSintControl(XOctrl.XPRS_CUTSTRATEGY,0);*/
//
// this.problem.setXPRSintControl(XOctrl.XPRS_MAXTIME,-1000);
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
        this.routeVariables = new  ArrayList<>();

        this.slackVariableAllLanePlowed = new XPRBvar[inputdata.antallNoder][inputdata.antallNoder];
        this.slackVariableAllSidewalkPlowed = new XPRBvar[inputdata.antallNoder][inputdata.antallNoder];
        this.slackVariableChooseRouteLane = new XPRBvar[vehicleLane.size()];
        this.slackVariableChooseRouteSidewalk = new XPRBvar[vehicleSidewalk.size()];
        this.slackVariablePrecedenceLane = new XPRBvar[vehicleLane.size()][inputdata.antallNoder][inputdata.antallNoder];
        this.slackVariablePrecedenceSidewalk = new XPRBvar[vehicleSidewalk.size()][inputdata.antallNoder][inputdata.antallNoder];
        this.slackVariableMaxWaitTime = new XPRBvar[vehicleSidewalk.size()];
        this.slackVariableMakespanLane = new XPRBvar[vehicleLane.size()];
        this.slackVariableMakespanSidewalk = new XPRBvar[vehicleSidewalk.size()];


        for(int k = 0; k < vehicleSidewalk.size(); k++){
            this.waitVariable[k] = problem.newVar("waitVar "+k, XPRB.PL,0, XPRB.INFINITY);

            this.slackVariableMakespanSidewalk[k] = problem.newVar("slack variable makespan sidewalk", XPRB.PL, 0, XPRB.INFINITY);
            this.slackVariableMaxWaitTime[k] = problem.newVar("slack variable max wait time", XPRB.PL, 0, XPRB.INFINITY);
            this.slackVariableChooseRouteSidewalk[k] = problem.newVar("slack variable choose route sidewalk", XPRB.PL, 0, XPRB.INFINITY);

            this.objective.addTerm(this.slackVariableMakespanSidewalk[k],100);
            this.objective.addTerm(this.slackVariableMaxWaitTime[k],100);
            this.objective.addTerm(this.slackVariableChooseRouteSidewalk[k],100);
        }

        this.makespanVariable = problem.newVar("makespanVar", XPRB.PL,0,XPRB.INFINITY);
        this.objective.addTerm(this.makespanVariable,1);

        for (int i = 0; i < inputdata.antallNoder; i++){
            for(int j = 0; j < inputdata.antallNoder; j++){
                this.precedenceVariable[i][j] = problem.newVar("precedence "+i+" "+j, XPRB.PL,0,XPRB.INFINITY);
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
                if(inputdata.numberOfPlowJobsLane[i][j] > 0){
                    this.slackVariableAllLanePlowed[i][j] = problem.newVar("slack variable all lanes plowed", XPRB.PL, 0, XPRB.INFINITY);
                    this.objective.addTerm(this.slackVariableAllLanePlowed[i][j],100);

                    this.allLanesPlowedCons[i][j] = problem.newCtr("Plowing demand lanes con");
                    this.allLanesPlowedCons[i][j].setType(XPRB.G);
                    this.allLanesPlowedCons[i][j].add(inputdata.numberOfLanesOnArc[i][j]);
                    this.allLanesPlowedCons[i][j].addTerm(this.slackVariableAllLanePlowed[i][j],1);

                }
                if(inputdata.numberOfPlowJobsSidewalk[i][j] > 0){
                    this.slackVariableAllSidewalkPlowed[i][j] = problem.newVar("slack variable all sidewalks plowed", XPRB.PL, 0, XPRB.INFINITY);
                    this.objective.addTerm(this.slackVariableAllSidewalkPlowed[i][j],100);

                    this.allSidewalksPlowedCons[i][j] = problem.newCtr("Plowing demand sidewalks con");
                    this.allSidewalksPlowedCons[i][j].setType(XPRB.G);
                    this.allSidewalksPlowedCons[i][j].add(inputdata.numberofSidewalksOnArc[i][j]);
                    this.allSidewalksPlowedCons[i][j].addTerm(this.slackVariableAllSidewalkPlowed[i][j],1);
                }
            }
        }

        for(int k = 0; k < vehicleLane.size(); k++){
            this.slackVariableChooseRouteLane[k] = problem.newVar("slack variable choose route lane", XPRB.PL, 0, XPRB.INFINITY);
            this.objective.addTerm(this.slackVariableChooseRouteLane[k],100);
            this.slackVariableMakespanLane[k] = problem.newVar("slack variable makespan lane", XPRB.PL, 0, XPRB.INFINITY);
            this.objective.addTerm(this.slackVariableMakespanLane[k],100);

            this.chooseRouteLaneCons[k] = problem.newCtr("Choose route lane con");
            this.chooseRouteLaneCons[k].setType(XPRB.E);
            this.chooseRouteLaneCons[k].add(1);
            this.chooseRouteLaneCons[k].addTerm(this.slackVariableChooseRouteLane[k],1);

            this.makespanLaneCons[k] = problem.newCtr("Makespan lane con");
            this.makespanLaneCons[k].setType(XPRB.L);
            this.makespanLaneCons[k].add(0);
            this.makespanLaneCons[k].addTerm(this.makespanVariable,-1);
            this.makespanLaneCons[k].addTerm(this.slackVariableMakespanLane[k],-1);
        }

        for(int k = 0; k < vehicleSidewalk.size(); k++){
            this.chooseRouteSidewalkCons[k] = problem.newCtr("Choose route sidewalk con");
            //Endre til equal
            this.chooseRouteSidewalkCons[k].setType(XPRB.E);
            this.chooseRouteSidewalkCons[k].add(1);
            this.chooseRouteSidewalkCons[k].addTerm(this.slackVariableChooseRouteSidewalk[k],1);

            this.makespanSidewalkCons[k] = problem.newCtr("Makespan sidewalk con");
            this.makespanSidewalkCons[k].setType(XPRB.L);
            this.makespanSidewalkCons[k].add(-inputdata.maxTime);
            this.makespanSidewalkCons[k].addTerm(this.makespanVariable,-1);
            this.makespanSidewalkCons[k].addTerm(this.waitVariable[k],-1);
            this.makespanSidewalkCons[k].addTerm(this.slackVariableMakespanSidewalk[k],-1);

            this.maxWaitTimeCons[k] = problem.newCtr("Max wait time cons");
            this.maxWaitTimeCons[k].setType(XPRB.L);
            this.maxWaitTimeCons[k].add(inputdata.maxTime);
            this.maxWaitTimeCons[k].addTerm(this.waitVariable[k], 1);
            this.maxWaitTimeCons[k].addTerm(this.slackVariableMaxWaitTime[k],-1);
        }

        for(int k = 0; k < vehicleLane.size(); k++){
            for(int i = 0; i <inputdata.antallNoder; i++){
                for(int j = 0; j < inputdata.antallNoder; j++){
                    if(inputdata.plowingtimeLane[i][j] > 0 && inputdata.numberOfPlowJobsSidewalk[i][j] > 0){
                        this.slackVariablePrecedenceLane[k][i][j] = problem.newVar("slack variable precedence lane", XPRB.PL, 0, XPRB.INFINITY);
                        this.objective.addTerm(this.slackVariablePrecedenceLane[k][i][j],100);

                        this.precedenceLaneCons[k][i][j] = problem.newCtr("Precedence lane con");
                        this.precedenceLaneCons[k][i][j].setType(XPRB.L);
                        this.precedenceLaneCons[k][i][j].add(0);
                        this.precedenceLaneCons[k][i][j].addTerm(this.precedenceVariable[i][j],-1);
                        this.precedenceLaneCons[k][i][j].addTerm(this.slackVariablePrecedenceLane[k][i][j],-1);
                    }
                }
            }
        }

        for(int k = 0; k < vehicleSidewalk.size(); k++){
            for(int i = 0; i < inputdata.antallNoder; i++){
                for(int j = 0; j < inputdata.antallNoder; j++){
                    if(inputdata.plowingtimeLane[i][j] > 0 && inputdata.numberOfPlowJobsSidewalk[i][j] > 0){
                        this.slackVariablePrecedenceSidewalk[k][i][j] = problem.newVar("slack variable precedence sidewalk", XPRB.PL, 0, XPRB.INFINITY);
                        this.objective.addTerm(this.slackVariablePrecedenceSidewalk[k][i][j],100);

                        this.precedenceSidewalkCons[k][i][j] = problem.newCtr("Precedence sidewalk con");
                        this.precedenceSidewalkCons[k][i][j].setType(XPRB.L);
                        this.precedenceSidewalkCons[k][i][j].add(0);
                        this.precedenceSidewalkCons[k][i][j].addTerm(this.precedenceVariable[i][j],1);
                        this.precedenceSidewalkCons[k][i][j].addTerm(this.waitVariable[k],1);
                        this.precedenceSidewalkCons[k][i][j].addTerm(this.slackVariablePrecedenceSidewalk[k][i][j],-1);
                    }
                }
            }
        }





    }
//Oppdatere dualveridiene her
    private void solveProblem(){
        //Lagt til en midlertidig metode og counter
        int counter = 0;
        //generateInitialPaths();
        //Midlertidig metode slutt
        boolean optimalSolutionFound = false;
        while(!optimalSolutionFound){
            counter +=1;
            boolean tempBoolean = false;
            solveMasterProblem();
            for(VehicleLane v: vehicleLane) {
                ArrayList<Label> labelLane = builder.buildPathLane(v);
                if(labelLane != null){
                    for(Label label : labelLane){
                        addLabelToMaster(label, true);
                    }
                    tempBoolean = true;
                }
            }
            for(int v = 0; v < vehicleSidewalk.size(); v++) {
                ArrayList<Label> labelSidewalk = builder.buildPathSidewalk(vehicleSidewalk.get(v));
                if(labelSidewalk != null){
                    for(Label label : labelSidewalk){
                        addLabelToMaster(label, false);
                    }
                    tempBoolean = true;
                }
            }
            if (tempBoolean == false){
                optimalSolutionFound = true;
            }
        }
        System.out.println("Antall ganger solveMaster() blir kalt "+counter);
        printSolution();
    }

    private void solveMasterProblem(){
        problem.setObj(this.objective);
        problem.setSense(XPRB.MINIM);
        //problem.sync(XPRB.XPRS_PROB);

        //n er LP, g er MIP
        problem.solve("n");

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
                    //System.out.println("Alpha lane "+allLanesPlowedCons[i][j].getDual()+" is valid "+allLanesPlowedCons[i][j].isValid());
                }
                if(inputdata.numberofSidewalksOnArc[i][j] > 0){
                    dualAlphaSidewalk[i][j] = allSidewalksPlowedCons[i][j].getDual();
                    //System.out.println("Alpha sidewalk "+allSidewalksPlowedCons[i][j].getDual()+" is valid "+allSidewalksPlowedCons[i][j].isValid());
                }
            }
        }

        for(int k = 0; k < vehicleSidewalk.size(); k++){
            dualBetaSidewalk[k] = chooseRouteSidewalkCons[k].getDual();
            //System.out.println("Beta sidewalk "+chooseRouteSidewalkCons[k].getDual()+" is valid "+chooseRouteSidewalkCons[k].isValid());


            dualSigmaSidewalk[k] = maxWaitTimeCons[k].getDual();
            //System.out.println("Sigma sidewalk "+maxWaitTimeCons[k].getDual());


            for(int i = 0; i < inputdata.antallNoder; i++){
                for(int j = 0; j < inputdata.antallNoder; j++){
                    if(inputdata.plowingtimeLane[i][j] > 0 && inputdata.numberOfPlowJobsSidewalk[i][j] > 0){
                        dualGammaSidewalk[k][i][j] = precedenceSidewalkCons[k][i][j].getDual();
                        //System.out.println("Gamma sidewalk "+precedenceSidewalkCons[k][i][j].getDual());
                    }
                }
            }

        }

        for(int k = 0; k < vehicleLane.size(); k++){
            dualBetaLane[k] = chooseRouteLaneCons[k].getDual();
            //System.out.println("Beta lane "+chooseRouteLaneCons[k].getDual());

            dualSigmaLane[k] = makespanLaneCons[k].getDual();
            //System.out.println("Sigma lane "+makespanLaneCons[k].getDual());

            for(int i = 0; i < inputdata.antallNoder; i++){
                for(int j = 0; j < inputdata.antallNoder; j++){
                    if(inputdata.plowingtimeLane[i][j] > 0 && inputdata.numberOfPlowJobsSidewalk[i][j] > 0){
                        dualGammaLane[k][i][j] = precedenceLaneCons[k][i][j].getDual();
                        //System.out.println("Gamma lane "+makespanLaneCons[k].getDual());
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
        //gjør binær med BV, kontinuerlig med PL
        XPRBvar lambdaVar = problem.newVar("lambda "+routeVariables.size(),XPRB.PL,0,XPRB.INFINITY);

        if(LaneVehicle == true){
            routeVariables.add(lambdaVar);

            this.chooseRouteLaneCons[label.vehicle.getNumber()].addTerm(lambdaVar,1);
            this.makespanLaneCons[label.vehicle.getNumber()].addTerm(lambdaVar, label.arraivingTime);
        }
        else if(LaneVehicle == false){
            routeVariables.add(lambdaVar);

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

    private void printSolution() {
        for(int i = 0; i < routeVariables.size(); i++){
            if(routeVariables.get(i).getSol() >0.1){
                System.out.println(routeVariables.get(i).getSol()+" "+pathList.get(i).toString());
            }
        }

        for (int i = 0; i < inputdata.antallNoder; i++){
            for(int j = 0; j < inputdata.antallNoder; j++){
                if(inputdata.numberOfPlowJobsLane[i][j] > 0 && inputdata.numberOfPlowJobsSidewalk[i][j] > 0 ){
                    System.out.println("Precedence:");
                    String string = "";
                    for(int r = 0; r < routeVariables.size(); r++){
                        if(routeVariables.get(r).getSol() > 0.1){
                            Label temp = pathList.get(r);
                            if(temp.vehicle instanceof VehicleLane){
                                string += " Lane time "+temp.lastTimePlowedNode[i][j];
                            }
                            else{
                                string += " Sidewalk time "+temp.lastTimePlowedNode[i][j];
                            }
                        }
                    }
                    System.out.println(string);
                }
            }
        }
        System.out.println("Makespan: "+problem.getObjVal());

    }

    //Midlertidig metode
    private void generateInitialPaths(){
        //lag tre nye labels her
        Label label1 = new Label();
        label1.node = 5;
        label1.vehicle = vehicleLane.get(0);
        label1.arraivingTime = 14;
        label1.cost = 0;
        int[][] lastTimePlowedNodeLabel1 = { {0,0,0,0,0,0},
                {0,0,0,0,0,0},
                {0,11,0,3,0,0},
                {0,0,7,0,0,0},
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

        Label label1vol2 = new Label();
        label1vol2.node = 5;
        label1vol2.vehicle = vehicleLane.get(1);
        label1vol2.arraivingTime = 14;
        label1vol2.cost = 0;
        label1vol2.lastTimePlowedNode = lastTimePlowedNodeLabel1;
        label1vol2.numberOfTimesPlowed = numberOfTimesPlowedLabel1;

        Label label2 = new Label();
        label2.node = 5;
        label2.vehicle = vehicleLane.get(1);
        label2.arraivingTime = 14;
        label2.cost = 0;
        int[][] lastTimePlowedNodeLabel2 = { {0,0,0,0,0,0},
                {0,0,0,0,0,0},
                {0,0,0,0,0,0},
                {0,0,0,0,7,0},
                {0,8,0,6,0,0},
                {0,0,0,0,0,0}};
        label2.lastTimePlowedNode = lastTimePlowedNodeLabel2;
        int[][] numberOfTimesPlowedLabel2 = { {0,0,0,0,0,0},
                {0,0,0,0,1,0},
                {0,0,0,0,0,0},
                {0,0,0,0,1,0},
                {0,1,0,1,0,0},
                {0,0,0,0,0,0}};
        label2.numberOfTimesPlowed = numberOfTimesPlowedLabel2;

        Label label2vol2 = new Label();
        label2vol2.node = 5;
        label2vol2.vehicle = vehicleLane.get(0);
        label2vol2.arraivingTime = 14;
        label2vol2.cost = 0;
        label2vol2.lastTimePlowedNode = lastTimePlowedNodeLabel2;
        label2vol2.numberOfTimesPlowed = numberOfTimesPlowedLabel2;

        Label label3 = new Label();
        label3.node = 0;
        label3.vehicle = vehicleSidewalk.get(0);
        label3.arraivingTime = 2;
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

        Label label3vol2 = new Label();
        label3vol2.node = 0;
        label3vol2.vehicle = vehicleSidewalk.get(1);
        label3vol2.arraivingTime = 2;
        label3vol2.cost = 0;
        label3vol2.lastTimePlowedNode = lastTimePlowedNodeLabel3;
        label3vol2.numberOfTimesPlowed = numberOfTimesPlowedLabel3;


        addLabelToMaster(label1,true);
        addLabelToMaster(label2,true);
        addLabelToMaster(label3,false);
        addLabelToMaster(label1vol2,true);
        addLabelToMaster(label2vol2,true);
        addLabelToMaster(label3vol2,false);

    }

    private void solveProblemAllPaths(){
        //Lagt til en midlertidig metode
        //generateInitialPaths();
        //Midlertidig metode slutt
        for(VehicleLane v : vehicleLane){
            //ArrayList<Label> labels = builder.generateAllPathsLane(v);
            ArrayList<Label> labels = builder.generateFivePathsLane(v);
            for(Label l : labels){
                addLabelToMaster(l,true);
            }
        }
        for(VehicleSidewalk v: vehicleSidewalk){
            //ArrayList<Label> labels = builder.generateAllPathsSidewalk(v);
            ArrayList<Label> labels = builder.generateFivePathsSidewalk(v);
            for(Label l : labels){
                addLabelToMaster(l,false);
            }
        }
        solveMasterProblem();
        printSolution();

    }

}
