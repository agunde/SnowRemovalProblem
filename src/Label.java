/**
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

    public ArrayList<Integer> getRoute() {
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

}
