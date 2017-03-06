/**
 * Created by andershg on 15.02.2017.
 */

import java.util.ArrayList;


public class InputReader {

    public static void inputReader(ArrayList<VehicleLane> vehicleLane, ArrayList<VehicleSidewalk> vehicleSidewalk, String testdata, String instance, InstanceData inputdata){

        int numberOfVehiclesLane = 2;
        int numberOfVehiclesSidewalk = 2;
        int maxTime = 30;
        int antallNoder = 6;
        int startNode = 0;
        int endNode = 5;

        for(int k = 0; k < numberOfVehiclesLane; k++){
            VehicleLane vehicle = new VehicleLane(k);
            vehicleLane.add(vehicle);
        }

        for(int k = 0; k < numberOfVehiclesSidewalk; k++){
            VehicleSidewalk vehicle = new VehicleSidewalk(k);
            vehicleSidewalk.add(vehicle);
        }

        inputdata.maxTime = maxTime;
        inputdata.antallNoder = antallNoder;
        inputdata.startNode = startNode;
        inputdata.endNode = endNode;

        int plowingTimeLane[][] = { {-1,   -1,  -1, -1, -1, -1},
                {-1, -1,  3, -1, 6, -1},
                {-1, 3, -1, 4, -1, -1},
                {-1, -1, 4, -1, 1, -1},
                {-1, 6, -1, 1, -1, -1},
                {-1, -1, -1, -1, -1, -1}
        };

        int plowingTimeSidewalk[][] = { {-1,   -1,  -1, -1, -1, -1},
                {-1, -1,  -1, -1, -1, -1},
                {-1, -1, -1, 6, -1, -1},
                {-1, -1, 6, -1, 3, -1},
                {-1, -1, -1, -1, -1, -1},
                {-1, -1, -1, -1, -1, -1}
        };

        int deadheadingTimeLane[][] = { {-1,   0,  -1, -1, -1, -1},
                {-1, -1,  3, -1, 6, 0},
                {-1, 3, -1, 4, -1, -1},
                {-1, -1, 4, -1, 1, -1},
                {-1, 6, -1, 1, -1, -1},
                {-1, -1, -1, -1, -1, -1}
        };

        int deadheadingTimeSidewalk[][] = { {-1,   0,  -1, -1, -1, -1},
                {-1, -1,  3, -1, 6, 0},
                {-1, 3, -1, 4, -1, -1},
                {-1, -1, 4, -1, 1, -1},
                {-1, 6, -1, 1, -1, -1},
                {-1, -1, -1, -1, -1,-1}
        };

        int numberOfPlowJobsLane[][] = { {0,   0,  0, 0, 0, 0},
                {0, 0,  1, 0, 1, 0},
                {0, 1, 0, 1, 0, 0},
                {0, 0, 1, 0, 1,0},
                {0, 1, 0, 1, 0, 0},
                {0, 0, 0, 0, 0, 0}
        };

        int numberOfPlowJobsSidewalk[][] = { {0,   0,  0, 0, 0, 0},
                {0, 0,  0, 0, 0, 0},
                {0, 0, 0, 1, 0, 0},
                {0, 0, 1, 0, 1,0},
                {0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0}
        };

        int numberOfLanesOnArc[][] = { {0,   0,  0, 0, 0, 0},
                {0, 0,  1, 0, 1, 0},
                {0, 1, 0, 1, 0, 0},
                {0, 0, 1, 0, 1,0},
                {0, 1, 0, 1, 0, 0},
                {0, 0, 0, 0, 0, 0}
        };

        int numberOfSidewalksOnArc[][] = { {0,   0,  0, 0, 0, 0},
                {0, 0,  0, 0, 0, 0},
                {0, 0, 0, 1, 0, 0},
                {0, 0, 1, 0, 1,0},
                {0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0}
        };

        int timeWindowLane[][] = { {30,   30,  30, 30, 30, 30},
                {30, 30, 30, 30, 30, 30},
                {30, 30, 30, 30, 30, 30},
                {30, 30, 30, 30, 30, 30},
                {30, 30, 30, 30, 30, 30},
                {30, 30, 30, 30, 30, 30}
        };

        inputdata.plowingtimeLane = plowingTimeLane;
        inputdata.plowingtimeSidewalk = plowingTimeSidewalk;
        inputdata.deadheadingtimeLane = deadheadingTimeLane;
        inputdata.deadheadingtimeSidewalk = deadheadingTimeSidewalk;
        inputdata.numberOfPlowJobsLane = numberOfPlowJobsLane;
        inputdata.numberOfPlowJobsSidewalk = numberOfPlowJobsSidewalk;
        inputdata.numberOfLanesOnArc = numberOfLanesOnArc;
        inputdata.numberofSidewalksOnArc = numberOfSidewalksOnArc;
        inputdata.timeWindowLane = timeWindowLane;


    }
}
