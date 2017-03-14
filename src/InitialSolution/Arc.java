package InitialSolution;

import java.util.ArrayList;

/**
 * Created by Magnu on 23.11.2016.
 */

//The arc class
public class Arc implements Comparable<Arc>{
    public Node from;
    public Node to;
    public int length;
    public int type;
    public int identifier;
    public int startOfService;
    public int startOfServiceGT;
    public int waitingTime;
    public boolean serviced;
    public ArrayList<Arc> haveToPreceed;
    public ArrayList<Arc> haveToSucced;

    public Arc(Node from, Node to, int length, int type, int id){
        this.from = from;
        this.to = to;
        this.length = length;
        this.type = type;
        this.identifier = id;
        this.serviced = false;
        this.startOfService = -1;
        this.startOfServiceGT = -1;
        this.haveToPreceed = new ArrayList<>();
        this.haveToSucced = new ArrayList<>();
        this.waitingTime = 0;
    }
    public void setStartOfService(int time){
        if(startOfService < 0 || startOfService > time){
            startOfService = time;
        }
    }
    //This is used to sort the the sidewalks for the giant sidewalk tour
    public void setStartOfServiceGT(int time){
        if(startOfServiceGT < 0){
            startOfServiceGT = time;
        }
    }

    public void serveArc(){
        this.serviced = true;
    }

    public void addPrecedingArc(Arc precedingArc){
        haveToPreceed.add(precedingArc);
    }

    public void addSuccedingArc(Arc precedingArc){
        haveToSucced.add(precedingArc);
    }

    //Finds the earliest starting time to plow this arc for the smaller vehicles.
    public int getEarliestStartingTimeForThisArc(){
        int max = 0;
        for(int x = 0; x < haveToPreceed.size();x++){
            if(haveToPreceed.get(x).startOfService >= max){
                max = haveToPreceed.get(x).startOfService;
            }
        }
        return max;
    }
    //The same as above, for the giant tour.
    public int getEarliestStartingTimeForThisArcGT(){
        int max = 0;
        for(int x = 0; x < haveToPreceed.size();x++){
            if(haveToPreceed.get(x).startOfServiceGT >= max){
                max = haveToPreceed.get(x).startOfServiceGT;
            }
        }
        return max;
    }

    public void setWaitingTime(int time){
        this.waitingTime = time;
    }

    @Override
    //Used to sort the arclist, from which is serviced first to last. Used for generation of the Giant Sidewalk tour.
    public int compareTo(Arc o) {
        return this.getEarliestStartingTimeForThisArcGT()- o.getEarliestStartingTimeForThisArcGT();
    }
}
