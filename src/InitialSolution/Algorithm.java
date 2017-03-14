package InitialSolution;



import java.util.*;

/**
 * Created by Magnu on 23.11.2016.
 */
public class Algorithm {

    public ArrayList<Node> nodes;
    public ArrayList<Arc> arcs;
    public ArrayList<Node> sideWalkNodes;
    public ArrayList<Arc> sideWalkArcs;
    public ArrayList<Arc> onlySideWalkArcs;
    public HashMap<Integer, Arc> arcMap;
    public HashMap<ArcNodeIdentifier, Arc> arcNodeMap;

    public int[][] graph;
    public int[][] swMatrix;
    public int[][] originalGraphClone;
    public int[][] bestMatrix;

    public int[][] fwGraph;
    public int[][] fwPath;
    public int[][] fwGraphSW;
    public int[][] fwPathSW;
    public int[][] fwBestGraph;
    public int[][] fwBestPath;

    public ArrayList<Integer> GiantTour;
    public ArrayList<Integer> GiantTourNoDuplicates;
    public ArrayList<Integer> GiantTourNoDuplicatesSW;
    public int depot;
    public int nrVehicles;
    public int nrSWVehicles;
    public int capacity;

    public ArrayList<Vehicle> vehicles;

    public GiantTourGenerator G;


    //The entire algorithm is placed in the constructor.
    public Algorithm(int[][] inputGraph, int[][] inputSWMatrix, int depot, int nrVehicles, int nrSWVehicles) {
        long startConstruction = System.currentTimeMillis();
        nodes = new ArrayList<>();
        arcs = new ArrayList<>();
        sideWalkArcs = new ArrayList<>();
        sideWalkNodes = new ArrayList<>();
        onlySideWalkArcs = new ArrayList<>();
        arcMap = new HashMap<>();
        arcNodeMap = new HashMap<>();

        this.depot = depot;
        this.nrVehicles = nrVehicles;
        this.nrSWVehicles = nrSWVehicles;

        graph = RemoveDepot(inputGraph);
        swMatrix = RemoveDepot(inputSWMatrix);
        originalGraphClone = graph.clone();
        floydWarshall fw1 = new floydWarshall();
        floydWarshall fw2 = new floydWarshall();
        floydWarshall fw3 = new floydWarshall();
        fwGraph = fw1.Algorithm(originalGraphClone);
        fwPath = fw1.path;
        fwGraphSW = fw2.Algorithm(swMatrix);
        fwPathSW = fw2.path;

        bestMatrix = new int[originalGraphClone.length][originalGraphClone.length];
        for(int x = 0; x < originalGraphClone.length; x++){
            for(int y = 0; y < originalGraphClone.length; y++){
                if (swMatrix[x][y] > originalGraphClone[x][y] && originalGraphClone[x][y] != -1){
                    bestMatrix[x][y] = originalGraphClone[x][y];
                }
                else if(swMatrix[x][y] > originalGraphClone[x][y] && originalGraphClone[x][y] == -1){
                    bestMatrix[x][y] = swMatrix[x][y];
                }
                else{
                    bestMatrix[x][y] = originalGraphClone[x][y];
                }
            }
        }
        fwBestGraph = fw2.Algorithm(bestMatrix);
        fwBestPath = fw2.path;


        for (int x = 0; x < graph.length; x++) {
            nodes.add(new Node(x));

        }

        int counter = 0;
        for (int x = 0; x < graph.length; x++) {
            for (int y = 0; y < graph[x].length; y++) {
                //If we have an entry in input matrix, we have an arc.
                if (graph[x][y] >= 0 && x != y) {
                    Arc tempArc = new Arc(nodes.get(x), nodes.get(y), graph[x][y], 1, counter);
                    arcMap.put(counter, tempArc);
                    arcNodeMap.put(new ArcNodeIdentifier(nodes.get(x).nr, nodes.get(y).nr), tempArc);
                    counter++;
                    arcs.add(tempArc);

                    nodes.get(x).outGoing.add(tempArc);
                    nodes.get(y).inComing.add(tempArc);

                    //If the Sidewalk matrix has a higher value, we have a sidewalk that needs to be serviced, with
                    //Precedence constraints.
                    if (swMatrix[x][y] > graph[x][y]) {
                        Arc tempSW = new Arc(nodes.get(x), nodes.get(y), swMatrix[x][y], 2, counter);
                        arcMap.put(counter, tempSW);
                        sideWalkArcs.add(tempSW);
                        tempSW.addPrecedingArc(tempArc);
                        tempArc.addSuccedingArc(tempSW);
                        nodes.get(x).outGoingSW.add(tempSW);
                        nodes.get(y).inComingSW.add(tempSW);
                        counter++;
                    }
                }
                //If we have an entry in the sidewalkMatrix where we don't have an entry in the laneMatrix, we have a sidewalk here
                //without precedence constraints
                if (graph[x][y] < 0 && swMatrix[x][y] >= 0 && x != y) {
                    Arc tempSW = new Arc(nodes.get(x), nodes.get(y), swMatrix[x][y], 2, counter);
                    arcMap.put(counter, tempSW);
                    arcNodeMap.put(new ArcNodeIdentifier(nodes.get(x).nr, nodes.get(y).nr), tempSW);
                    sideWalkArcs.add(tempSW);
                    onlySideWalkArcs.add(tempSW);
                    nodes.get(x).outGoingSW.add(tempSW);
                    nodes.get(y).inComingSW.add(tempSW);
                    counter++;
                }
            }
        }
        //We have to remove the nodes only available for the smaller vehicles to solve the CPP.
        sideWalkNodes = fixNodeList();
        G = new GiantTourGenerator(nodes.size());

        //Now, we are done initializing, Solve the CPP and create the Giant Tour for lanes.
        for (int x = 0; x < arcs.size(); x++) {
            G.addedge((arcs.get(x).identifier + 1) + "", arcs.get(x).from.GTnr, arcs.get(x).to.GTnr, arcs.get(x).length);
        }
        GiantTour = G.cpp(depot);

        //Now, we set the earliest startingTime for each arc in the Giant tour, so we can sort the sidewalks
        SetGiantTourEarliestService(GiantTour);
        int costGiantTour = getCostFromGiantTour();

        //Remove duplicates, as we don't want more than one vehicle to be assigned with more than one arc.
        GiantTourNoDuplicates = removeDuplicateArcs();
        //We need the cost of the tour to find the initial capacity
        int costGiantTourNoDuplicates = getCostFromGiantTourNoDuplicates();

        ArrayList<Vehicle> vehicles = new ArrayList<Vehicle>();

        if(nrVehicles > 1){
            capacity = costGiantTourNoDuplicates / nrVehicles - 1;
            ArrayList<int[]> Trips = split(GiantTourNoDuplicates, capacity, true);
            int toursGenBySplit = 0;
            while (toursGenBySplit != nrVehicles) {
                capacity += 1;
                Trips = split(GiantTourNoDuplicates, capacity, true);
                toursGenBySplit = Trips.size();
            }


            for (int x = 0; x < Trips.size(); x++) {
                ArrayList<Arc> tempTasks = convertFromArraytoArc(Trips.get(x));
                ArrayList<Arc> tempRoute = getTourFromTasks(tempTasks);
                //We add a new vehicle with a positive ID, since it is a plowing truck.
                Vehicle tempVehicle = new Vehicle(x + 1, tempTasks, tempRoute);
                vehicles.add(tempVehicle);
            }
        }
        else{
            vehicles.add(new Vehicle(1,convertFromIDtoArc(GiantTourNoDuplicates),convertFromIDtoArc(GiantTour)));
        }





        ///Now, we generate the giant tour for the smaller vehicles.
        ArrayList<Arc> BestSequence = getListOfSWArcs();
        ArrayList<Integer> BestSequenceIds = convertFromArcToID(BestSequence);
        GiantTourNoDuplicatesSW = BestSequenceIds;
        //What is the cost of this?
        if(nrSWVehicles > 1){
            int costOfTheSequence = getCostFromTourIds(BestSequenceIds);
            int capacitySW = costOfTheSequence / nrSWVehicles - 10;

            ArrayList<int[]> TripsSW = split(BestSequenceIds, capacitySW, true);
            int toursGenBySplitSW = 0;
            while (toursGenBySplitSW != nrSWVehicles) {
                capacitySW += 1;
                TripsSW = split(BestSequenceIds, capacitySW, true);
                toursGenBySplitSW = TripsSW.size();
            }
            for (int x = 0; x < TripsSW.size(); x++) {
                ArrayList<Arc> tempTasks = convertFromArraytoArc(TripsSW.get(x));
                ArrayList<Arc> tempRoute = getTourFromTasks(tempTasks);
                Vehicle tempVehicle = new Vehicle(-(x + 1), convertFromArraytoArc(TripsSW.get(x)), tempRoute);
                vehicles.add(tempVehicle);
            }
        }
        else{
            vehicles.add(new Vehicle(-1,BestSequence, getTourFromTasks(BestSequence)));
        }

        this.vehicles = vehicles;
        System.out.println("Konstruksjonen tar: " + ((System.currentTimeMillis()-startConstruction)) + " tusendelssekunder");

        System.out.println("Løsningsverdien fra Konstruksjonen er: " + getMakeSpan(vehicles));
        //Here, we do the localSearch from the paper. the arguments are the vehicles, alpha, beta and theta.

        //vehicles = localWaitSearch(vehicles, 100, 100, 10);
        //Simulate the best solution
        /*resetPlowingtimes();
        Collections.sort(vehicles, new TypeComparator());


        for (int x = 0; x < vehicles.size(); x++) {
            vehicles.get(x).reRoute();
        }
        for (int x = 0; x < vehicles.size(); x++) {
            System.out.println(vehicles.get(x));
        }

        //..And print the values.
        System.out.println("Løsningsverdien etter lokalsøket er: "+ getMakeSpan(vehicles));

        System.out.println("Er løsningen lovlig? " + feasibileSolution());
        */

    }
    //We dont need the depots in this implementation. They are always the first and the last node.
    public int[][] RemoveDepot(int[][] inputGraph) {
        int[][] graph = new int[inputGraph.length - 2][inputGraph.length - 2];
        for (int x = 1; x < inputGraph.length - 1; x++) {
            for (int y = 1; y < inputGraph.length - 1; y++) {
                graph[x - 1][y - 1] = inputGraph[x][y];
            }
        }
        return graph;
    }
    //This is a method that is implemented for the purpose of The CPP solving. We need the
    //network to be connected, so we have to mark the nodes that does not belong in the lane matrix. This is done
    //By setting in a negative number as a "Giant tour number"
    public ArrayList<Node> fixNodeList() {
        int counterOfOnlySW = 1;
        ArrayList<Node> onlySwNodes = new ArrayList<>();
        for (int x = 0; x < nodes.size(); x++) {
            if (nodes.get(x).inComing.size() == 0 && nodes.get(x).outGoing.size() == 0) {
                nodes.get(x).setGTNr(counterOfOnlySW * -1);
                onlySwNodes.add(nodes.remove(x));
                counterOfOnlySW++;
            }
        }

        for (int x = 0; x < nodes.size(); x++) {
            nodes.get(x).setGTNr(x);
        }
        return onlySwNodes;
    }
    //This method reset all the earliest time each node is visited, so we can rerun the simulation of the vehicles.
    public void resetPlowingtimes() {
        for (int x = 0; x < arcs.size(); x++) {
            arcs.get(x).startOfService = -1;
            arcs.get(x).serviced = false;
            arcs.get(x).waitingTime = 0;
        }
        for (int x = 0; x < sideWalkArcs.size(); x++) {
            sideWalkArcs.get(x).startOfService = -1;
            sideWalkArcs.get(x).serviced = false;
            sideWalkArcs.get(x).waitingTime = 0;

        }
        for (int x = 0; x < onlySideWalkArcs.size(); x++) {
            onlySideWalkArcs.get(x).startOfService = -1;
            onlySideWalkArcs.get(x).serviced = false;
        }
    }
    //Returns the makespan
    public int getMakeSpan(ArrayList<Vehicle> vehicles) {
        int max = 0;
        for (int x = 0; x < vehicles.size(); x++) {
            if (vehicles.get(x).totalLength > max) {
                max = vehicles.get(x).totalLength;
            }
        }
        return max;
    }


    //The LocalBreadthSearch from the paper
    public ArrayList<Vehicle> localBreadthSearch(ArrayList<Vehicle> vehicles, int alpha, int beta) {
        ArrayList<Vehicle> bestVehicles = copyVehicles(vehicles);
        int bestMakeSpan = getMakeSpan(bestVehicles);
        int counter = 0;
        while (counter < beta) {
            ArrayList<Vehicle> temp = localDepthSearch(copyVehicles(vehicles), alpha);
            if (getMakeSpan(temp) < bestMakeSpan) {
                bestMakeSpan = getMakeSpan(temp);
                bestVehicles = temp;
                counter = 0;
            }
            counter++;
        }
        return bestVehicles;
    }
    //The LocalDepthSearch from the paper
    public ArrayList<Vehicle> localDepthSearch(ArrayList<Vehicle> vehicles, int alpha) {
        ArrayList<Vehicle> originalVehicles = new ArrayList<>();
        for (int x = 0; x < vehicles.size(); x++) {
            originalVehicles.add(vehicles.get(x));
        }

        Random rng = new Random();
        int counter = 0;
        boolean temp;
        Vehicle switchTo;
        ArrayList<Vehicle> StreetVehicles = new ArrayList<>();
        ArrayList<Vehicle> SideWalkVehicles = new ArrayList<>();
        Collections.sort(vehicles, new MakeSpanComparator());
        for (int x = 0; x < vehicles.size(); x++) {
            if (vehicles.get(x).id > 0) {
                StreetVehicles.add(vehicles.get(x));
            } else {
                SideWalkVehicles.add(vehicles.get(x));
            }
        }

        while (counter < alpha) {
            Vehicle longestRouteVehicle = vehicles.get(0);
            Arc randomArc = longestRouteVehicle.tasks.get(rng.nextInt(longestRouteVehicle.tasks.size()));

            if (longestRouteVehicle.id > 1) {
                switchTo = StreetVehicles.get(rng.nextInt(StreetVehicles.size()));
                temp = Switch(originalVehicles, randomArc, longestRouteVehicle, switchTo);
            } else {
                switchTo = SideWalkVehicles.get(rng.nextInt(SideWalkVehicles.size()));
                //switchTo = SideWalkVehicles.get(rng.nextInt(SideWalkVehicles.size()-1) + 1);
                temp = Switch(originalVehicles, randomArc, longestRouteVehicle, switchTo);
            }
            //Now, we check if the solution is better, and move on.
            if (temp == true) {
                Collections.sort(vehicles, new MakeSpanComparator());
                Collections.sort(StreetVehicles, new MakeSpanComparator());
                Collections.sort(SideWalkVehicles, new MakeSpanComparator());
                counter = 0;
            } else {
                counter += 1;
            }
        }
        return vehicles;
    }
    //The LocalSearch from the papers.
    public ArrayList<Vehicle> localWaitSearch(ArrayList<Vehicle> vehicles, int alpha, int beta, int theta) {
        Random rng = new Random();
        ArrayList<Vehicle> bestVehicles = localBreadthSearch(copyVehicles(vehicles), alpha, beta);
        int bestMakeSpan = getMakeSpan(bestVehicles);
        int counter = 0;
        while (counter < theta) {
            Collections.sort(bestVehicles, new MakeSpanComparator());
            //if(bestVehicles.get(0).id > 0){
            //    return localBreadthSearch(copyVehicles(bestVehicles), iterations, iterations2);
            //}
            ArrayList<Vehicle> temp = copyVehicles(bestVehicles);
            ArrayList<Vehicle> streetTemp = new ArrayList<>();
            for (int x = 0; x < temp.size(); x++) {
                if (temp.get(x).id > 0) {
                    streetTemp.add(temp.get(x));
                }
            }
            Vehicle randomPlower = streetTemp.get(rng.nextInt(streetTemp.size()));
            Arc longestWaitingJob = randomPlower.getArcWithHighestInverseWaitingTime();

            Vehicle switchTo = streetTemp.get(rng.nextInt(streetTemp.size()));
            Collections.sort(temp, new TypeComparator());
            SwitchUnconditional(temp, longestWaitingJob, randomPlower, switchTo);
            resetPlowingtimes();
            for (int x = 0; x <temp.size(); x++){
                temp.get(x).reRoute();
            }

            //bestMakeSpan = getMakeSpan(temp);
            //System.out.println("Før:");
            //System.out.println(getMakeSpan(bestVehicles));
            temp = localBreadthSearch(temp,alpha,beta);
            Collections.sort(temp, new TypeComparator());
            resetPlowingtimes();
            for (int x = 0; x <temp.size(); x++){
                temp.get(x).reRoute();
            }
            if(getMakeSpan(temp)< getMakeSpan(bestVehicles)){
                bestVehicles = temp;
                counter = 0;
                System.out.println(counter);

            }
            else{
                counter++;
                System.out.println(counter);

            }



        }
        return bestVehicles;

    }


    //This is the BestSwitch method from the papers.
    public boolean Switch(ArrayList<Vehicle> vehicles, Arc switched, Vehicle switchFrom, Vehicle switchTo){
        ArrayList<Arc> switchFromTasks= switchFrom.tasks;
        ArrayList<Arc> switchToTasks = switchTo.tasks;

        int indexInOriginalTaskList = switchFromTasks.indexOf(switched);
        int index = -1;
        int makeSpan = getMakeSpan(vehicles);
        switchFromTasks.remove(switched);
        switchFrom.route = getTourFromTasks(switchFromTasks);

        for(int x = 0; x<switchToTasks.size()+1; x++){
            switchToTasks.add(x,switched);
            switchTo.route = getTourFromTasks(switchToTasks);
            resetPlowingtimes();
            for(int y = 0; y<vehicles.size(); y++){
                vehicles.get(y).reRoute();
            }
            if (makeSpan > getMakeSpan(vehicles)){
                makeSpan = getMakeSpan(vehicles);
                index = x;
            }
            switchToTasks.remove(switched);
            switchTo.route = getTourFromTasks(switchToTasks);
        }
        if(index >= 0 ){
            switchToTasks.add(index,switched);
            switchTo.route = getTourFromTasks(switchToTasks);
            resetPlowingtimes();
            for(int y = 0; y<vehicles.size(); y++){
                vehicles.get(y).reRoute();
            }
            return true;
        }
        else{
            switchFromTasks.add(indexInOriginalTaskList, switched);
            switchFrom.route = getTourFromTasks(switchFromTasks);
            for(int y = 0; y<vehicles.size(); y++){
                vehicles.get(y).reRoute();
            }
            return false;
        }

    }
    //This is the WaitSwitch from the papers.
    public int SwitchUnconditional(ArrayList<Vehicle> vehicles, Arc switched, Vehicle switchFrom, Vehicle switchTo){
        ArrayList<Arc> switchFromTasks= switchFrom.tasks;
        ArrayList<Arc> switchToTasks = switchTo.tasks;
        Random rng = new Random();

        int indexInOriginalTaskList = switchFromTasks.indexOf(switched);
        int index = 0;
        int makeSpan = getMakeSpan(vehicles);
        switchFromTasks.remove(switched);
        switchFrom.route = getTourFromTasks(switchFromTasks);
        index = rng.nextInt(switchToTasks.size()/2);
        switchToTasks.add(index,switched);
        switchTo.route = getTourFromTasks(switchToTasks);
        Collections.sort(vehicles, new TypeComparator());
        resetPlowingtimes();
        for(int y = 0; y<vehicles.size(); y++){
            vehicles.get(y).reRoute();
        }
        return indexInOriginalTaskList;


    }

    //Here, we copy a list of vehicles, and also the task sequences for each of them.
    //This is done because of since the objects are mutable.
    public ArrayList<Vehicle> copyVehicles(ArrayList<Vehicle> oldVehicles){
        ArrayList<Vehicle> newVehicles = new ArrayList<>();
        for(int x = 0; x< oldVehicles.size();x++){
            newVehicles.add(new Vehicle(oldVehicles.get(x).id,(ArrayList<Arc>) oldVehicles.get(x).tasks.clone(), (ArrayList<Arc>) oldVehicles.get(x).route.clone()) );
        }
        return newVehicles;
    }
    //Converter
    public ArrayList<Integer> convertFromArcToID(ArrayList<Arc> arcs){
        ArrayList<Integer> ids = new ArrayList<>();
        for(int x = 0; x<arcs.size(); x++){
            ids.add(arcs.get(x).identifier);
        }
        return ids;
    }
    //Converter
    public ArrayList<Arc> convertFromArraytoArc(int[] ids){
        ArrayList<Arc> arcs = new ArrayList<>();
        for(int x = 0; x<ids.length; x++){
            arcs.add(arcMap.get(ids[x]));
        }
        return arcs;
    }
    //Converter
    public ArrayList<Arc> convertFromIDtoArc(ArrayList<Integer> ids){
        ArrayList<Arc> arcs = new ArrayList<>();
        for(int x = 0; x<ids.size(); x++){
            arcs.add(arcMap.get(ids.get(x)));
        }
        return arcs;
    }

    //This is a method that finds the whole route from a task list. Adds additional arcs from depot,
    //between tasks, and path to depot
    public ArrayList<Arc> getTourFromTasks(ArrayList<Arc> tasks){
        ArrayList<Arc> route = new ArrayList<>();
        if(depot != tasks.get(0).from.nr){
            route.addAll(getArcsFromPath(depot, tasks.get(0).from.nr));
        }
        for(int x = 0; x<tasks.size()-1; x++){
            route.add(tasks.get(x));
            if(tasks.get(x).to.nr != tasks.get(x+1).from.nr){
                route.addAll(getArcsFromPath(tasks.get(x).to.nr,tasks.get(x+1).from.nr));
            }
        }
        route.add(tasks.get(tasks.size()-1));
        if(tasks.get(tasks.size()-1).to.nr != depot){
            route.addAll(getArcsFromPath(tasks.get(tasks.size()-1).to.nr, depot));
        }
        return route;
    }



    //A method to generate a path based on startnode id, and endnode id.
    public ArrayList<Arc> getArcsFromPath(int startNodeId, int endNodeId){
        ArrayList<Integer> path = getPath(startNodeId, endNodeId);
        ArrayList<Arc> arcPath = new ArrayList<>();

        for(int x = 0; x < path.size()-1; x++){
            arcPath.add(arcNodeMap.get(new ArcNodeIdentifier(path.get(x),path.get(x+1))));
            //System.out.println(arcPath.get(x));
        }
        return arcPath;
    }
    //Gets cost from a path based on startID, and endID.
    public int getCostFromPath(int StartNodeId, int endNodeId){
        ArrayList<Arc> arcs = getArcsFromPath(StartNodeId, endNodeId);
        int cost = 0;
        for(int x = 0; x < arcs.size(); x++){
            cost += arcs.get(x).length;
        }
        return cost;
    }


    //Iterates through the arcs, checks that they are serviced, and that they are serviced at legal times
    public boolean feasibileSolution(){
        boolean feasible = true;
        for(int x = 0; x< arcs.size(); x++){
            if (!arcs.get(x).serviced){
                feasible = false;
            }
        }
        for(int x = 0; x< sideWalkArcs.size(); x++){
            if(!sideWalkArcs.get(x).serviced || sideWalkArcs.get(x).startOfService< sideWalkArcs.get(x).getEarliestStartingTimeForThisArc()){
                feasible = false;
                System.out.println(sideWalkArcs.get(x).serviced);

            }
        }

        return feasible;
    }

    //A method which primaraly removes duplicates from the giant tour. This is done so that no more than one vehicle has the same
    //arc in its task sequence.
    public ArrayList<Integer> removeDuplicateArcs(){
        ArrayList<Integer> removed = new ArrayList<>();

        Iterator iterator = GiantTour.iterator();

        while (iterator.hasNext()) {
            int o = (int) iterator.next();
            if(!removed.contains(o)){
                removed.add(o);
            }
        }
        return removed;
    }
    //Returns the path as a set of nodeIDS.
    public ArrayList<Integer> getPath(int startID, int endID){
        ArrayList<Integer> path = new ArrayList<>();
        path.add(startID);
        int nextID = startID;
        while (nextID != endID){
            int oldId = nextID;
            nextID = fwPath[oldId][endID];
            if(nextID == -1){
                nextID = fwPathSW[oldId][endID];
            }
            path.add(nextID);
        }
        return path;
    }

    public int getCostFromGiantTour(){
        int cost = 0;
        for(int x = 0; x < GiantTour.size(); x++){
            cost += arcMap.get(GiantTour.get(x)).length;
        }
        return cost;
    }

    public int getCostFromTourIds(ArrayList<Integer> ids){
        int cost = 0;
        for(int x = 0; x < ids.size(); x++){
            cost += arcMap.get(ids.get(x)).length;
        }
        return cost;
    }


    public ArrayList<Arc> SetGiantTourEarliestService(ArrayList<Integer> arcIds){
        ArrayList<Arc> arcs = new ArrayList<Arc>();
        int time = 0;
        for(int x = 0; x < arcIds.size(); x++){
            arcMap.get(arcIds.get(x)).setStartOfServiceGT(time);
            arcs.add(arcMap.get(arcIds.get(x)));
            time += arcMap.get(arcIds.get(x)).length;
        }
        return arcs;
    }
    //This generates the Giant SideWalk Tour based on when its preceeding arc is serviced.
    public ArrayList<Arc> getListOfSWArcs(){
        ArrayList<Arc> sequence = new ArrayList<>();
        for(int x = 0; x < sideWalkArcs.size();x++){
            if(sideWalkArcs.get(x).haveToPreceed.size()>0){
                sequence.add(sideWalkArcs.get(x));
            }
        }
        Collections.sort(sequence);
        return getBestSequence(sequence);

    }
    //Inserts the sidewalks that has no precedence constraints into the the giant tour. Brute force to find the lowest tour cost.
    public ArrayList<Arc> getBestSequence(ArrayList<Arc> sequence){
        ArrayList<Arc> temp = (ArrayList<Arc>) sequence.clone();
        for(int x = 0; x<onlySideWalkArcs.size(); x++){
            int bestIndex = 0;
            int bestTime = 9999;
            for(int y = 0; y<temp.size()+1; y++){
                temp.add(y, onlySideWalkArcs.get(x));
                if(getSequenceCost(temp) < bestTime){
                    bestIndex = y;
                    bestTime = getSequenceCost(temp);
                    temp.remove(y);
                }
                else{
                    temp.remove(y);
                }
            }
            temp.add(bestIndex, onlySideWalkArcs.get(x));
        }
        return temp;
    }

    //A method to get the cost of an arc sequence
    public int getSequenceCost(ArrayList<Arc> sequence){
        int cost = 0;
        if(depot != sequence.get(0).from.nr){
            cost += fwGraph[depot][sequence.get(0).from.nr];
        }
        for(int x = 0; x<sequence.size()-1; x++){
            cost += sequence.get(x).length;
            if(sequence.get(x).to.nr != sequence.get(x+1).from.nr){
                cost += fwGraph[sequence.get(x).to.nr][sequence.get(x+1).from.nr];
            }
        }
        cost += sequence.get(sequence.size()-1).length;
        if(depot != sequence.get(sequence.size()-1).to.nr){
            cost += fwGraph[sequence.get(sequence.size()-1).to.nr][depot];
        }
        return cost;
    }

    public int getCostFromGiantTourNoDuplicates(){
        int cost = 0;
        for(int x = 0; x < GiantTourNoDuplicates.size(); x++){
            cost += arcMap.get(GiantTourNoDuplicates.get(x)).length;
        }
        return cost;
    }




    //The Split algorithm in the papers.
    public ArrayList<int[]> split(ArrayList<Integer> GiantTour, int capacity, boolean generateTrips ){
        int[] V = new int[GiantTour.size()+1];
        int[] P = new int[V.length];
        V[0] = 0;	//The cost of going to the depot is 0
        P[0] = depot;	//The predecessor of the depot is the depot
        for (int i = 1; i < V.length; i++) {
            V[i] = 99999;	//Initialize the cost of all other elements as infinity
        }

        int depot = this.depot;
        int j;
        int cost;

        for (int i = 1; i < V.length; i++) {
            j = i;
            cost = 0;

            do {
                if( i == j ){

                    cost = getCostFromPath(depot, arcMap.get(GiantTour.get(i-1)).from.nr) + arcMap.get(GiantTour.get(i-1)).length
                            + getCostFromPath(arcMap.get(GiantTour.get(i-1)).to.nr,depot);

                }else{

                    cost = cost - getCostFromPath( arcMap.get(GiantTour.get(j-2)).to.nr,depot)
                            + getCostFromPath(arcMap.get(GiantTour.get(j-2)).to.nr,arcMap.get(GiantTour.get(j-1)).from.nr)
                            + arcMap.get(GiantTour.get(j-1)).length
                            + getCostFromPath(arcMap.get(GiantTour.get(j-1)).to.nr,depot);
                }
                if ( (cost <= capacity) && (V[i-1] + cost < V[j]) ) {
                    V[j] = V[i-1] + cost;
                    P[j] = i - 1;
                }
                j++;
            } while ( (j < V.length) && (cost < capacity) );
        }

        if(generateTrips){
            return tripsSplittedFromTour(V, P, GiantTour);
        }
        else{
            return null;

        }



    }

    //Returns a list of all task sequences.
    public static ArrayList<int[]> tripsSplittedFromTour(int[] costs, int[] predecessors, ArrayList<Integer> Tour){
        ArrayList<int[]> ListOfTrips = new ArrayList<>();
        int j = Tour.size();
        int i;
        do {
            i = predecessors[j];
            int[] currentTrip = new int[j - i];
            for (int k = i + 1; k <=  j; k++) {
                currentTrip[k-(i + 1)] = Tour.get(k - 1);
            }
            ListOfTrips.add(0, currentTrip);
            j = i;
        } while (i != 0);
        return ListOfTrips;
    }





}
