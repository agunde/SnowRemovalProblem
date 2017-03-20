package InitialSolution;

import java.util.ArrayList;
import java.util.Arrays;

import ColumnGeneration.Label;
import ColumnGeneration.PathBuilder;
import ColumnGeneration.XpressInterface;

/**
 * Created by Magnu on 06.12.2016.
 */

//The ColumnGeneration.Vehicle class. Has a task sequence and a route. A positive ID is a plowing truck. A negative ID is a smaller vehicle.
public class Vehicle{
    public int id;
    public ArrayList<Arc> route;
    public ArrayList<Integer> StartTimeForArcs;
    public ArrayList<Arc> tasks;
    public int totalLength;


    public Vehicle(int id, ArrayList<Arc> tasks, ArrayList<Arc> route){
        this.id = id;
        this.tasks = tasks;
        this.route = route;
        this.StartTimeForArcs = new ArrayList<>();
        //getStartTimesAndTotalCost();
    }
    //This is basically the simulating algorithm. Checks when the vehicle arrives at at arc, if it is the earliest plowing truck, the earliest time is set
    // to this. If it is a smaller vehicle, it cannot continue until it has waited.
    public void getStartTimesAndTotalCost(){
        ArrayList<Integer> startTimes = new ArrayList<>();
        int cost = 0;
        for(int x = 0; x<route.size(); x++){
            if(this.id < 0) {
                if (route.get(x).getEarliestStartingTimeForThisArc() > 0) {
                    if (cost < route.get(x).getEarliestStartingTimeForThisArc()) {
                        //route.get(x).setWaitingTime(route.get(x).getEarliestStartingTimeForThisArc() - cost);
                        for (int y = 0; y < route.get(x).haveToPreceed.size(); y++) {
                            route.get(x).haveToPreceed.get(y).setWaitingTime(cost - route.get(x).getEarliestStartingTimeForThisArc());
                        }
                        cost = route.get(x).getEarliestStartingTimeForThisArc();
                    }
                }
            }
            if (this.id > 0 && route.get(x).type == 1){
                route.get(x).setStartOfService(cost);
                route.get(x).serveArc();
            }
            if (this.id < 0 && route.get(x).type == 2){
                route.get(x).setStartOfService(cost);
                route.get(x).serveArc();
            }
            startTimes.add(cost);

            cost += route.get(x).length;

        }
        totalLength = cost;
        StartTimeForArcs = startTimes;
        //System.out.println("Halla");
    }

    //Simulates the plowing of the vehicles.
    public void reRoute(){
        getStartTimesAndTotalCost();
    }

    public void setRoute(ArrayList<Arc> route){
        this.route = route;
    }

    public Arc getArcWithHighestInverseWaitingTime(){
        int min = 0;
        Arc temp = tasks.get(tasks.size()-1);
        for(int x = 0; x < tasks.size(); x++){
            if(tasks.get(x).waitingTime < min){
                min = tasks.get(x).waitingTime;
                temp = tasks.get(x);
            }
        }
        return temp;
    }

    public Vehicle copyVehicle(){
        Vehicle copy = new Vehicle(this.id, (ArrayList<Arc>) this.tasks.clone(), (ArrayList<Arc>) this.route.clone());
        return copy;
    }

    //The output does not say whether the vehicle waits or not.
    @Override
    public String toString(){
        String s = "";
        if(route.size() == 0){
            s += "ColumnGeneration.Vehicle " + id + " stays in depot";
            return s;
        }


        for(int x = 0; x<route.size(); x++){
            s += "ColumnGeneration.Vehicle " + id + " Takes arc " + route.get(x).identifier + " from " + route.get(x).from.nr + " to " + route.get(x).to.nr
                    + ". This takes " + route.get(x).length + " time units";
            if(route.get(x).type == 2){
                s += ". This is a Sidewalk";
            }
            if(x> 0 && (StartTimeForArcs.get(x) -  (StartTimeForArcs.get(x-1) + route.get(x-1).length) > 0)){
                s += " And it has to wait for " + (StartTimeForArcs.get(x) -  (StartTimeForArcs.get(x-1) + route.get(x).length)) + " time units";
            }
            s +=". It starts plowing the arc at time " + StartTimeForArcs.get(x)+ "\n";
        }
        s += ". Total time of route is " + (StartTimeForArcs.get(StartTimeForArcs.size()-1) + route.get(route.size()-1).length);
        s += " Total time is " + totalLength;
        return s;
    }

    public void addColumnToMaster(XpressInterface xpi){
        Label label = new Label();
        if(this.id >0){
            //System.out.println("Vei");
            label.arraivingTime = totalLength;
            label.cost = 0;
            label.deadheading = true;
            label.node = xpi.inputdata.endNode;
            int[][] nrTraversed = new int[xpi.inputdata.antallNoder][xpi.inputdata.antallNoder];
            int[][] lastTimePlowed = new int[xpi.inputdata.antallNoder][xpi.inputdata.antallNoder];
            for (int i = 0; i < route.size(); i++) {
                if(nrTraversed[route.get(i).from.nr+1][route.get(i).to.nr+1] < xpi.inputdata.numberOfPlowJobsLane[route.get(i).from.nr+1][route.get(i).to.nr+1]){
                    if(nrTraversed[route.get(i).from.nr+1][route.get(i).to.nr+1] == 0){
                        lastTimePlowed[route.get(i).from.nr+1][route.get(i).to.nr+1] = StartTimeForArcs.get(i);
                    }
                    nrTraversed[route.get(i).from.nr+1][route.get(i).to.nr+1] += 1;
                }
            }
            label.numberOfTimesPlowed = nrTraversed;
            label.lastTimePlowedNode = lastTimePlowed;
            //label.vehicle = xpi.vehicleLane.get(this.id -1);
            //xpi.addLabelToMaster(label, true);

            for(int k = 0; k < xpi.vehicleLane.size(); k++){
                Label l2 = xpi.duplicateLabel(label);
                l2.vehicle = xpi.vehicleLane.get(k);
                xpi.addLabelToMaster(l2,true);

                //String string = l2.toString(xpi.inputdata.antallNoder);
                //System.out.println(string);
            }
        }
        else if(this.id < 0){
            //System.out.println("Fortau");
            int waitTime = 0;
            int cumWaitTime = 0;
            for (int x = 0; x < route.size(); x++){
                if(x> 0 && (StartTimeForArcs.get(x) -  (StartTimeForArcs.get(x-1) + route.get(x-1).length) > 0)){
                    waitTime += StartTimeForArcs.get(x) -  (StartTimeForArcs.get(x-1) + route.get(x).length);
                }
            }
            label.arraivingTime = xpi.inputdata.maxTime - totalLength + waitTime;
            label.cost = 0;
            label.deadheading = true;
            label.node = xpi.inputdata.startNode;
            int[][] nrTraversed = new int[xpi.inputdata.antallNoder][xpi.inputdata.antallNoder];
            int[][] lastTimePlowed = new int[xpi.inputdata.antallNoder][xpi.inputdata.antallNoder];
            label.lastTimePlowedNode = lastTimePlowed;
            for(int[] row : label.lastTimePlowedNode){
                Arrays.fill(row,xpi.inputdata.maxTime);
            }
            for (int i = 0; i < route.size(); i++) {
                if(i> 0 && (StartTimeForArcs.get(i) -  (StartTimeForArcs.get(i-1) + route.get(i-1).length) > 0)){
                    cumWaitTime += StartTimeForArcs.get(i) -  (StartTimeForArcs.get(i-1) + route.get(i).length);
                }
                if(route.get(i).type == 2){
                    if(nrTraversed[route.get(i).from.nr+1][route.get(i).to.nr+1] < xpi.inputdata.numberOfPlowJobsSidewalk[route.get(i).from.nr+1][route.get(i).to.nr+1]){
                        if(nrTraversed[route.get(i).from.nr+1][route.get(i).to.nr+1] == 0){
                            label.lastTimePlowedNode[route.get(i).from.nr+1][route.get(i).to.nr+1] = label.arraivingTime + StartTimeForArcs.get(i) - cumWaitTime;
                        }
                        nrTraversed[route.get(i).from.nr+1][route.get(i).to.nr+1] += 1;
                    }
                }

            }

            label.numberOfTimesPlowed = nrTraversed;
            //label.vehicle = xpi.vehicleSidewalk.get(Math.abs(this.id)-1);
            //xpi.addLabelToMaster(label, false);
            for(int k = 0; k < xpi.vehicleSidewalk.size(); k++){
                Label l2 = xpi.duplicateLabel(label);
                l2.vehicle = xpi.vehicleSidewalk.get(k);
                xpi.addLabelToMaster(l2,false);

                //String string = l2.toString(xpi.inputdata.antallNoder);
                //System.out.println(string);
            }


        }

    }


}
