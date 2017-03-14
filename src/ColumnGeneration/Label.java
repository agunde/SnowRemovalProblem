package ColumnGeneration; /**
 * Created by andershg on 13.02.2017.
 */

import java.util.ArrayList;
import java.util.Collections;

public class Label {

    public Vehicle vehicle;
    public int node;
    public Label predecessor;
    public int arraivingTime;
    public double cost;
    public int[][] lastTimePlowedNode;
    public int[][] numberOfTimesPlowed;
    public boolean deadheading;

    public ArrayList<Integer> getRouteLane() {
        ArrayList<Integer> list = new ArrayList<>();
        list.add(node);
        Label temp = predecessor;
        while(temp!=null){
            list.add(temp.node);
            temp = temp.predecessor;
        }
        Collections.reverse(list);
        return list;
    }

    public ArrayList<Boolean> getPlowListLane() {
        ArrayList<Boolean> list = new ArrayList<>();
        list.add(deadheading);
        Label temp = predecessor;
        while(temp!=null){
            list.add(temp.deadheading);
            temp = temp.predecessor;
        }
        Collections.reverse(list);
        return list;
    }

    public ArrayList<Integer> getRouteSidewalk() {
        ArrayList<Integer> list = new ArrayList<>();
        list.add(node);
        Label temp = predecessor;
        while(temp!=null){
            list.add(temp.node);
            temp = temp.predecessor;
        }
        return list;
    }

    public ArrayList<Boolean> getPlowListSidewalk() {
        ArrayList<Boolean> list = new ArrayList<>();
        list.add(deadheading);
        Label temp = predecessor;
        while(temp!=null){
            list.add(temp.deadheading);
            temp = temp.predecessor;
        }
        return list;
    }

    public String toString(){
        String string="";
        if(vehicle instanceof VehicleLane){
            string += "ColumnGeneration.Vehicle lane nr " + vehicle.getNumber() + " Time used "+arraivingTime;
            ArrayList<Integer> route = getRouteLane();
            ArrayList<Boolean> plow = getPlowListLane();
            for (int i = 0; i < route.size(); i++){
                if(plow.get(i) == true){
                    string += " deadhead to "+ (route.get(i)+1);
                }
                else{
                    string += " plow to " + (route.get(i)+1);
                }
            }
        }
        else if(vehicle instanceof VehicleSidewalk){
            string += "ColumnGeneration.Vehicle sidewalk nr " + vehicle.getNumber()+ " Time used "+arraivingTime;
            ArrayList<Integer> route = getRouteSidewalk();
            ArrayList<Boolean> plow = getPlowListSidewalk();
            for (int i = 0; i < route.size(); i++){
                if(plow.get(i) == true){
                    string += " deadhead to "+ (route.get(i)+1);
                }
                else{
                    string += " plow to " + (route.get(i)+1);
                }
            }
        }
        return string;
    }


}
