package evolutionaryAlgorithm;

import InitialSolution.Arc;
import InitialSolution.ArcNodeIdentifier;
import InitialSolution.TypeComparator;
import InitialSolution.Vehicle;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Random;


/**
 * Created by Magnu on 15.02.2017.
 */
public class Fenotype {

    Random rng;
    private final ArrayList<Arc> lanes;
    private final ArrayList<Arc> sidewalks;
    public final int plowtrucks;
    public final int smallervehicles;

    public int[] originalLaneGeno;
    public int[] originalSidewalkGeno;

    public HashMap<Integer, Arc> arcMap;
    public HashMap<ArcNodeIdentifier, Arc> arcNodeMap;
    public int[][] FWLaneGraph;
    public int[][] FWSidewalkGraph;
    public int[][] FWLanePath;
    public int[][] FWSidewalkPath;
    public int[][] FWBestGraph;
    public int[][] FWBestPath;
    public int depot;


    public Fenotype(ArrayList<Arc> lanes, ArrayList<Arc> sidewalks, HashMap<Integer, Arc> arcMap, HashMap<ArcNodeIdentifier, Arc> arcNodeMap, int[][] FWlaneGraph, int[][] FWlanePath,
                    int[][] FWsidewalkGraph, int[][] FWsidewalkPath, int[][] FWBestGraph, int[][] FWBestPath, int depot, int plowtrucks, int smallervehicles) {
        this.lanes = lanes;
        this.sidewalks = sidewalks;
        this.arcNodeMap = arcNodeMap;
        this.arcMap = arcMap;
        this.FWLaneGraph = FWlaneGraph;
        this.FWSidewalkGraph = FWsidewalkGraph;
        this.FWBestGraph = FWBestGraph;
        this.FWLanePath = FWlanePath;
        this.FWSidewalkPath = FWsidewalkPath;
        this.FWBestPath = FWBestPath;
        this.depot = depot;
        this.plowtrucks = plowtrucks;
        this.smallervehicles = smallervehicles;

        this.originalLaneGeno = getAscendingLanes();
        this.originalSidewalkGeno = getAscendingSidewalks();
        rng = new Random();

    }

    public ArrayList<Arc> getLanes(){
        return this.lanes;
    }

    public ArrayList<Arc> getSidewalks(){
        return this.sidewalks;
    }

    public Genotype createRandomGenotype(){
        int[] tempLane = this.originalLaneGeno.clone();
        int[] tempSideWalk = this.originalSidewalkGeno.clone();
        int bound = rng.nextInt(10);
        int indexLane1 = rng.nextInt(tempLane.length);
        int indexLane2 = rng.nextInt(tempLane.length);
        int indexSW1 = rng.nextInt(tempSideWalk.length);
        int indexSW2 = rng.nextInt(tempSideWalk.length);


        swap(tempLane,indexLane1, indexLane2);
        swap(tempSideWalk,indexSW1, indexSW2);
        return new Genotype(tempLane,tempSideWalk, calculateFitness(tempLane,tempSideWalk));
    }



    public ArrayList<Vehicle> getFenotype(Genotype genotype) {
        ArrayList<Vehicle> vehicles = new ArrayList<>();
        Vehicle temp = new Vehicle(1, new ArrayList<Arc>(), new ArrayList<Arc>());
        vehicles.add(temp);
        int newVehicleId = 1;
        int[] lanesGenotype = genotype.getLaneGenome();
        for (int x = 0; x < lanesGenotype.length; x++) {
            if (lanesGenotype[x] == -1) {
                newVehicleId++;
                vehicles.add(new Vehicle(newVehicleId, new ArrayList<Arc>(), new ArrayList<Arc>()));
                continue;
            }
            vehicles.get(vehicles.size() - 1).tasks.add(arcMap.get(lanesGenotype[x]));
        }
        newVehicleId = -1;
        vehicles.add(new Vehicle(newVehicleId, new ArrayList<Arc>(), new ArrayList<Arc>()));
        int[] sidewalkGenotype = genotype.getSidewalkGenome();
        for (int x = 0; x < sidewalkGenotype.length; x++) {
            if (sidewalkGenotype[x] == -1) {
                newVehicleId--;
                vehicles.add(new Vehicle(newVehicleId, new ArrayList<Arc>(), new ArrayList<Arc>()));
                continue;
            }
            vehicles.get(vehicles.size() - 1).tasks.add(arcMap.get(sidewalkGenotype[x]));
        }

        for (int i = 0; i < vehicles.size(); i++) {
            vehicles.get(i).route = getTourFromTasks(vehicles.get(i).tasks, vehicles.get(i).id);

        }


        return vehicles;
    }

    public ArrayList<Vehicle> getFenotypefromGenome(int[] laneGenome, int[] sideWalkGenome) {
        ArrayList<Vehicle> vehicles = new ArrayList<>();
        Vehicle temp = new Vehicle(1, new ArrayList<Arc>(), new ArrayList<Arc>());
        vehicles.add(temp);
        int newVehicleId = 1;
        int[] lanesGenotype = laneGenome.clone();
        for (int x = 0; x < lanesGenotype.length; x++) {
            if (lanesGenotype[x] == -1) {
                newVehicleId++;
                vehicles.add(new Vehicle(newVehicleId, new ArrayList<Arc>(), new ArrayList<Arc>()));
                continue;
            }
            vehicles.get(vehicles.size() - 1).tasks.add(arcMap.get(lanesGenotype[x]));
            //System.out.println(arcNodeMap.get(lanesGenotype[x]));
        }

        temp = new Vehicle(-1, new ArrayList<Arc>(), new ArrayList<Arc>());
        vehicles.add(temp);
        newVehicleId = -1;
        int[] sidewalkGenotype = sideWalkGenome.clone();
        for (int x = 0; x < sidewalkGenotype.length; x++) {
            if (sidewalkGenotype[x] == -1) {
                newVehicleId--;
                vehicles.add(new Vehicle(newVehicleId, new ArrayList<Arc>(), new ArrayList<Arc>()));
                continue;
            }
            vehicles.get(vehicles.size() - 1).tasks.add(arcMap.get(sidewalkGenotype[x]));
            //System.out.println(arcNodeMap.get(sidewalkGenotype[x]));
        }

        for (int i = 0; i < vehicles.size(); i++) {
            //System.out.println(vehicles.get(i).tasks.size());
            vehicles.get(i).setRoute(getTourFromTasks(vehicles.get(i).tasks, vehicles.get(i).id));

        }


        return vehicles;
    }

    public Genotype createGenotype(ArrayList<Vehicle> initialVehicles, int fitness){
        int[] laneGenome = new int[plowtrucks-1 + this.getLanes().size()];
        int z = 0;
        Collections.sort(initialVehicles, new TypeComparator());
        for(int x = 0; x<plowtrucks;x++){
            if(x>0){
                laneGenome[z] = -1;
                z++;
            }
            for (int y = 0; y<initialVehicles.get(x).tasks.size(); y++){
                laneGenome[z] = initialVehicles.get(x).tasks.get(y).identifier;
                z++;
            }
        }
        int[] sidewalkGenome = new int[smallervehicles-1 + this.getSidewalks().size()];
        z = 0;
        for(int x = plowtrucks; x<(plowtrucks + smallervehicles);x++){
            if(x>plowtrucks){
                sidewalkGenome[z] = -1;
                z++;
            }
            for (int y = 0; y<initialVehicles.get(x).tasks.size(); y++){
                sidewalkGenome[z] = initialVehicles.get(x).tasks.get(y).identifier;
                z++;
            }
        }
        return new Genotype(laneGenome, sidewalkGenome, fitness);
    }

    public Genotype InitialGenotype(ArrayList<Vehicle> initialVehicles){
        int[] laneGenome = new int[plowtrucks-1 + this.getLanes().size()];
        int z = 0;
        Collections.sort(initialVehicles, new TypeComparator());
        for(int x = 0; x<plowtrucks;x++){
            if(x>0){
                laneGenome[z] = -1;
                z++;
            }
            for (int y = 0; y<initialVehicles.get(x).tasks.size(); y++){
                laneGenome[z] = initialVehicles.get(x).tasks.get(y).identifier;
                z++;
            }
        }
        int[] sidewalkGenome = new int[plowtrucks-1 + this.getSidewalks().size()];
        z = 0;
        for(int x = plowtrucks; x<(plowtrucks + smallervehicles);x++){
            if(x>plowtrucks){
                sidewalkGenome[z] = -1;
                z++;
            }
            for (int y = 0; y<initialVehicles.get(x).tasks.size(); y++){
                sidewalkGenome[z] = initialVehicles.get(x).tasks.get(y).identifier;
                z++;
            }
        }
        return new Genotype(laneGenome, sidewalkGenome, calculateFitness(laneGenome, sidewalkGenome));
    }

    public int calculateFitness(Genotype genotype) {
        ArrayList<Vehicle> vehicles = getFenotype(genotype);
        resetPlowingtimes();
        Collections.sort(vehicles, new TypeComparator());
        for (int x = 0; x < vehicles.size(); x++) {
            vehicles.get(x).reRoute();
        }
        return getMakeSpan(vehicles);
    }

    public int calculateFitness(ArrayList<Vehicle> vehicles) {
        resetPlowingtimes();
        Collections.sort(vehicles, new TypeComparator());
        for (int x = 0; x < vehicles.size(); x++) {
            vehicles.get(x).reRoute();
        }
        return getMakeSpan(vehicles);
    }

    public int[] calculateFitnessParameters(ArrayList<Vehicle> vehicles) {
        resetPlowingtimes();
        Collections.sort(vehicles, new TypeComparator());
        for (int x = 0; x < vehicles.size(); x++) {
            vehicles.get(x).reRoute();
        }
        return getMakeSpanParameters(vehicles);
    }

    public int calculateFitness(int[] laneGenome, int[] sidewalkGenome) {
        ArrayList<Vehicle> vehicles = getFenotypefromGenome(laneGenome,sidewalkGenome);
        resetPlowingtimes();
        Collections.sort(vehicles, new TypeComparator());
        for (int x = 0; x < vehicles.size(); x++) {
            vehicles.get(x).reRoute();
        }
        return getMakeSpan(vehicles);
    }


    public ArrayList<Arc> getTourFromTasks(ArrayList<Arc> tasks, int vehicleID) {
        ArrayList<Arc> route = new ArrayList<>();
        if(tasks.size() == 0){
            return route;
        }
        if (depot != tasks.get(0).from.nr) {
            route.addAll(getArcsFromPath(depot, tasks.get(0).from.nr, vehicleID));
        }
        for (int x = 0; x < tasks.size() - 1; x++) {
            route.add(tasks.get(x));
            //System.out.println(tasks.get(x+1));
            if (tasks.get(x).to.nr != tasks.get(x + 1).from.nr) {
                route.addAll(getArcsFromPath(tasks.get(x).to.nr, tasks.get(x + 1).from.nr, vehicleID));
            }
        }
        route.add(tasks.get(tasks.size() - 1));
        if (tasks.get(tasks.size() - 1).to.nr != depot) {
            route.addAll(getArcsFromPath(tasks.get(tasks.size() - 1).to.nr, depot, vehicleID));
        }
        return route;
    }
/**Deprecated*/
    public ArrayList<Arc> getArcsFromPath(int startNodeId, int endNodeId) {
        ArrayList<Integer> path = getPath(startNodeId, endNodeId);
        ArrayList<Arc> arcPath = new ArrayList<>();

        for (int x = 0; x < path.size() - 1; x++) {
            arcPath.add(arcNodeMap.get(new ArcNodeIdentifier(path.get(x), path.get(x + 1))));
            //System.out.println(arcPath.get(x));
        }
        return arcPath;
    }

    public ArrayList<Arc> getArcsFromPath(int startNodeId, int endNodeId, int vehicleID) {
        ArrayList<Integer> path = getPath(startNodeId, endNodeId, vehicleID);
        ArrayList<Arc> arcPath = new ArrayList<>();

        for (int x = 0; x < path.size() - 1; x++) {
            arcPath.add(arcNodeMap.get(new ArcNodeIdentifier(path.get(x), path.get(x + 1))));
            //System.out.println(arcPath.get(x));
        }
        return arcPath;
    }

    /**Deprecated*/
    public ArrayList<Integer> getPath(int startID, int endID) {
        ArrayList<Integer> path = new ArrayList<>();
        path.add(startID);
        int nextID = startID;
        while (nextID != endID) {
            int oldId = nextID;
            nextID = FWLanePath[oldId][endID];
            if (nextID == -1) {
                nextID = FWSidewalkPath[oldId][endID];
            }
            path.add(nextID);
        }
        return path;
    }

    public ArrayList<Integer> getPath(int startID, int endID, int vehicleId) {
        ArrayList<Integer> path = new ArrayList<>();
        path.add(startID);
        int nextID = startID;
        while (nextID != endID) {
            int oldId = nextID;
            if(vehicleId > 0){
                nextID = FWLanePath[oldId][endID];
            }
            else{
                nextID = FWBestPath[oldId][endID];
            }
            /*if (nextID == -1) {
                nextID = FWSidewalkPath[oldId][endID];
            }*/
            path.add(nextID);
        }
        return path;
    }

    public int getMakeSpan(ArrayList<Vehicle> vehicles) {
        int max = 0;
        for (int x = 0; x < vehicles.size(); x++) {
            if (vehicles.get(x).totalLength > max) {
                max = vehicles.get(x).totalLength;
            }
        }
        return max;
    }

    public void resetPlowingtimes() {
        for (int x = 0; x < lanes.size(); x++) {
            lanes.get(x).startOfService = -1;
            lanes.get(x).serviced = false;
            lanes.get(x).waitingTime = 0;
        }
        for (int x = 0; x < sidewalks.size(); x++) {
            sidewalks.get(x).startOfService = -1;
            sidewalks.get(x).serviced = false;
            sidewalks.get(x).waitingTime = 0;
        }
    }

    public int[] getAscendingLanes(){
        int[] ascendingLanes = new int[lanes.size()+ plowtrucks-1];
        int counter = 0;
        int extra = 0;
        int counter2 = 0;
        for (int i = 0; i < ascendingLanes.length; i++) {
            if (counter >= ((lanes.size()-1)/plowtrucks) && extra < (plowtrucks-1)){
                ascendingLanes[i] = -1;
                counter = 0;
                extra++;
                continue;
            }
            ascendingLanes[i] = lanes.get(counter2).identifier;
            counter++;
            counter2++;
        }
        return ascendingLanes;
    }

    public int[] getMakeSpanParameters(ArrayList<Vehicle> vehicles) {
        int[] makespanParam = new int[3];
        int max = 0;
        int type = 0;
        int index = 0;
        for (int x = 0; x < vehicles.size(); x++) {
            if (vehicles.get(x).totalLength > max) {
                max = vehicles.get(x).totalLength;
                type = vehicles.get(x).id;
                index = x;
            }
        }
        makespanParam[0] = max;
        makespanParam[1] = type;
        makespanParam[2] = index;
        return makespanParam;
    }

    public int[] getAscendingSidewalks(){
        int[] ascendingSidewalks = new int[sidewalks.size()+ smallervehicles-1];
        int counter = 0;
        int extra = 0;
        int counter2 = 0;
        for (int i = 0; i < ascendingSidewalks.length; i++) {
            if (counter >= ((sidewalks.size()-1)/smallervehicles) && extra < (smallervehicles-1)){
                ascendingSidewalks[i] = -1;
                counter = 0;
                extra++;
                continue;
            }
            ascendingSidewalks[i] = sidewalks.get(counter2).identifier;
            counter++;
            counter2++;
        }
        return ascendingSidewalks;
    }

    private void swap(int[] arr, int i, int j){
        int t = arr[i];
        arr[i] = arr[j];
        arr[j] = t;
        }


    }
