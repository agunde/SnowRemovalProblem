/**
 * Created by andershg on 14.02.2017.
 */

import java.util.ArrayList;
public class Main {

    public static void main(String[] args){
        ArrayList<VehicleLane> vehicleLane = new ArrayList<>();
        ArrayList<VehicleSidewalk> vehicleSidewalk = new ArrayList<>();
        String testdata = "TestCaseOriginal.txt";

        String instance = "TestSet.txt";
        InstanceData inputdata = new InstanceData(instance);
        InputReader.inputReader(vehicleLane, vehicleSidewalk, testdata, instance, inputdata);

        XpressInterface xpi = new XpressInterface(vehicleLane, vehicleSidewalk, inputdata);

    }
}


// What to do:
//Fikse kode for å håndtere input
//Lag en heltallsløsning
//Lag en bedre printSolution()
