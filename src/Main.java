/**
 * Created by andershg on 14.02.2017.
 */

import java.util.ArrayList;
public class Main {

    public static void main(String[] args){
        ArrayList<VehicleLane> vehicleLane = new ArrayList<>();
        ArrayList<VehicleSidewalk> vehicleSidewalk = new ArrayList<>();
        ArrayList<Nodes> nodes = new ArrayList<>();
        String testdata = "TestCaseOriginal.txt";

        String instance = "TestSet.txt";
        InstanceData inputdata = new InstanceData(instance);
        InputReader.inputReader(vehicleLane, vehicleSidewalk, nodes, testdata, instance, inputdata);

        XpressInterface xpi = new XpressInterface(vehicleLane, vehicleSidewalk, nodes, inputdata);

    }
}
