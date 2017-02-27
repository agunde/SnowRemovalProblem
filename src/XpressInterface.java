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
    public ArrayList<Node> nodes;
    public InstanceData inputdata;
    public PathBuilder builder;
    private Filewriter filewriter;

    public XpressInterface(ArrayList<VehicleLane> vehicleLane, ArrayList<VehicleSidewalk> vehicleSidewalk, ArrayList<Node> nodes, InstanceData inputdata) {
        this.vehicleLane = vehicleLane;
        this.vehicleSidewalk = vehicleSidewalk;
        this.nodes = nodes;
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
        this.makespanVariable = new XPRBvar;
        this.precedenceVariable = new XPRBvar[nodes.size()][nodes.size()];
        this.routeVariables = new  ArrayList<ArrayList<XPRBvar>>();

        for(int k = 0; k < vehicleSidewalk.size(); k++){
            this.waitVariable[k] = problem.newVar("waitVar "+k, XPRB.PL);
        }

        this.makespanVariable = problem.newVar("makespanVar", XPRB.PL);
        this.objective.addTerm(this.makespanVariable,1);

        for (int i = 0; i < nodes.size(); i++){
            for(int j = 0; j < nodes.size(); j++){
                this.precedenceVariable[i][j] = problem.newVar("precedence "+i+" "+j, XPRB.BV);
            }
        }






        //Constraints
        this.allLanesPlowedCons = new XPRBctr[nodes.size()][nodes.size()];
        this.allSidewalksPlowedCons = new XPRBctr[nodes.size()][nodes.size()];
        this.chooseRouteLaneCons = new XPRBctr[vehicleLane.size()];
        this.chooseRouteSidewalkCons = new XPRBctr[vehicleSidewalk.size()];
        this.precedenceLaneCons = new XPRBctr[vehicleLane.size()][nodes.size()][nodes.size()];
        this.precedenceSidewalkCons = new XPRBctr[vehicleSidewalk.size()][nodes.size()][nodes.size()];
        this.makespanLaneCons = new XPRBctr[vehicleLane.size()];
        this.makespanSidewalkCons = new XPRBctr[vehicleSidewalk.size()];
        this.maxWaitTimeCons = new XPRBctr[vehicleSidewalk.size()];

        for(int i = 0; i < nodes.size(); i++){
            for(int j = 0; j < nodes.size(); j++) {
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
            for(int i = 0; i <nodes.size(); i++){
                for(int j = 0; j < nodes.size(); j++){
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
            for(int i = 0; i < nodes.size(); i++){
                for(int j = 0; j < nodes.size(); j++){
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

    private void solveProblem(){
        boolean optimalSolutionFound = false;
        while(!optimalSolutionFound){
            boolean tempBoolean = false;
            Hashtable<Integer, Double[]> solution = solveMasterProblem();
            for(VehicleLane v: vehicleLane) {
                Label labelLane = builder.buildPathLane(v, solution.get(v));
                if(labelLane != null){
                    addLabelToMaster(labelLane, true);
                    tempBoolean = true;
                }
            }
            for(int v = 0; v < vehicleSidewalk.size(); v++) {
                Label labelSidewalk = builder.buildPathSidewalk(v, solution.get(vehicleLane.size() + v));
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

    private Hashtable<Integer, Double[]> solveMasterProblem(){
        Hashtable<Integer, Double[]> solution = new Hashtable<>();
        problem.setObj(this.objective);
        problem.setSense(XPRB.MINIM);
        problem.sync(XPRB.XPRS_PROB);
        problem.solve("g");

        //Hent ut dualverdiene

        return solution;
    }

    private void addLabelToMaster(Label label, boolean LaneVehicle){
        XPRBvar lambdaVar = problem.newVar("lambda "+routeVariables.size(),XPRB.BV,0,XPRB.INFINITY);

        if(LaneVehicle == true){
            routeVariables.get(label.vehicle.getNumber()).add(lambdaVar);
        }
        else if(LaneVehicle == false){
            routeVariables.get(vehicleLane.size()+label.vehicle.getNumber()).add(lambdaVar);
        }
        pathList.put(routeVariables.size()-1, label);

    }

}
