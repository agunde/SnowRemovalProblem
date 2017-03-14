package InitialSolution;

import java.util.ArrayList;

/**
 * Created by Magnu on 23.11.2016.
 */

//This is the Node Class. This has a list of incoming and outgoing arcs. This is used to check that the network is connected.
    //Some of the methods was not used, as they were implemented in another class.
public class Node {
    public int nr;
    public int GTnr;
    public ArrayList<Arc> outGoing;
    public ArrayList<Arc> inComing;

    public ArrayList<Arc> inComingSW;
    public ArrayList<Arc> outGoingSW;


    public Node(int number){
        nr = number;
        GTnr = number;
        outGoing = new ArrayList<>();
        inComing = new ArrayList<>();
        inComingSW = new ArrayList<>();
        outGoingSW = new ArrayList<>();

    }

    public int getDegree(){
        return inComing.size() - outGoing.size();
    }

    public int getDegreeSW(){
        return inComingSW.size() - outGoingSW.size();
    }
    public void setGTNr(int newNr){
        this.GTnr = newNr;
    }


}
