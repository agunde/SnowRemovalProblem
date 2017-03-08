/**
 * Created by andershg on 15.02.2017.
 */

import com.sun.org.apache.xpath.internal.SourceTree;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;


public class InputReader {

    public static void inputReader(ArrayList<VehicleLane> vehicleLane, ArrayList<VehicleSidewalk> vehicleSidewalk, String testdata, String instance, InstanceData inputdata){

        int numberOfVehiclesLane;
        int numberOfVehiclesSidewalk;
        /*int numberOfVehiclesLane = 2;
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
        inputdata.timeWindowLane = timeWindowLane;*/

        try {
            File file = new File(testdata);
            FileReader reader = new FileReader(file);
            BufferedReader fr = new BufferedReader(reader);

            //Legg inn hvor mange av de øverste linjene man skal fjerne
            /*for(int i = 0; i < 7; i++){
                fr.readLine();
            }*/
            String line = fr.readLine();
            System.out.println(line);
            String[] list1 = line.split(":");
            inputdata.antallNoder = Integer.parseInt(list1[1].trim());
            System.out.println(inputdata.antallNoder);

            int[][] tempArray1 = new int[inputdata.antallNoder][inputdata.antallNoder];
            int[][] tempArray2 = new int[inputdata.antallNoder][inputdata.antallNoder];
            int[][] tempArray3 = new int[inputdata.antallNoder][inputdata.antallNoder];
            int[][] tempArray4 = new int[inputdata.antallNoder][inputdata.antallNoder];
            int[][] tempArray5 = new int[inputdata.antallNoder][inputdata.antallNoder];
            int[][] tempArray6 = new int[inputdata.antallNoder][inputdata.antallNoder];
            int[][] tempArray7 = new int[inputdata.antallNoder][inputdata.antallNoder];
            int[][] tempArray8 = new int[inputdata.antallNoder][inputdata.antallNoder];
            inputdata.numberOfPlowJobsLane = tempArray1;
            inputdata.numberOfPlowJobsSidewalk = tempArray2;
            inputdata.numberofSidewalksOnArc = tempArray3;
            inputdata.numberOfLanesOnArc = tempArray4;
            inputdata.plowingtimeLane = tempArray5;
            inputdata.plowingtimeSidewalk = tempArray6;
            inputdata.deadheadingtimeSidewalk = tempArray7;
            inputdata.deadheadingtimeLane = tempArray8;

            fr.readLine();
            line = fr.readLine();
            list1 = line.split(":");
            numberOfVehiclesLane = Integer.parseInt(list1[1].trim());
            System.out.println(numberOfVehiclesLane);

            for(int k = 0; k < numberOfVehiclesLane; k++){
                vehicleLane.add(new VehicleLane(k));
            }
            line = fr.readLine();
            list1 = line.split(":");
            numberOfVehiclesSidewalk = Integer.parseInt(list1[1].trim());
            System.out.println(numberOfVehiclesSidewalk);

            for(int k = 0; k < numberOfVehiclesSidewalk; k++){
                vehicleSidewalk.add(new VehicleSidewalk(k));
            }

            fr.readLine();
            line = fr.readLine();
            list1 = line.split(":");
            inputdata.startNode = Integer.parseInt(list1[1].trim())-1;
            line = fr.readLine();
            list1 = line.split(":");
            inputdata.endNode = Integer.parseInt(list1[1].trim())-1;
            System.out.println(inputdata.startNode);
            System.out.println(inputdata.endNode);

            fr.readLine();
            fr.readLine();
            line = fr.readLine();
            list1 = line.split(":");
            inputdata.maxTime = Integer.parseInt(list1[1].trim());

            fr.readLine();
            fr.readLine();
            for(int i = 0; i < inputdata.antallNoder; i++){
                line = fr.readLine();
                System.out.println(line);
                line = line.trim();
                System.out.println(line);
                list1 = line.split(",");
                for(int j = 0; j < inputdata.antallNoder;j++){
                    inputdata.numberOfLanesOnArc[i][j] = Integer.parseInt(list1[j].trim());
                    inputdata.numberOfPlowJobsLane[i][j] = Integer.parseInt(list1[j].trim());
                }
            }

            fr.readLine();
            fr.readLine();
            fr.readLine();
            for(int i = 0; i < inputdata.antallNoder; i++){
                line = fr.readLine();
                System.out.println(line);
                line = line.trim();
                System.out.println(line);
                list1 = line.split(",");
                for(int j = 0; j < inputdata.antallNoder;j++){
                    inputdata.plowingtimeLane[i][j] = Integer.parseInt(list1[j].trim());
                    inputdata.deadheadingtimeLane[i][j] = Integer.parseInt(list1[j].trim());
                }
            }

            fr.readLine();
            fr.readLine();
            fr.readLine();
            for(int i = 0; i < inputdata.antallNoder; i++){
                line = fr.readLine();
                System.out.println(line);
                line = line.trim();
                System.out.println(line);
                list1 = line.split(",");
                for(int j = 0; j < inputdata.antallNoder;j++){
                    inputdata.numberOfPlowJobsSidewalk[i][j] = Integer.parseInt(list1[j].trim());
                    inputdata.numberofSidewalksOnArc[i][j] = Integer.parseInt(list1[j].trim());
                }
            }

            fr.readLine();
            fr.readLine();
            fr.readLine();
            for(int i = 0; i < inputdata.antallNoder; i++){
                line = fr.readLine();
                System.out.println(line);
                line = line.trim();
                System.out.println(line);
                list1 = line.split(",");
                for(int j = 0; j < inputdata.antallNoder;j++){
                    inputdata.plowingtimeSidewalk[i][j] = Integer.parseInt(list1[j].trim());
                }
            }

            fr.readLine();
            fr.readLine();
            fr.readLine();
            for(int i = 0; i < inputdata.antallNoder; i++){
                line = fr.readLine();
                System.out.println(line);
                line = line.trim();
                System.out.println(line);
                list1 = line.split(",");
                for(int j = 0; j < inputdata.antallNoder;j++){
                    inputdata.deadheadingtimeSidewalk[i][j] = Integer.parseInt(list1[j].trim());
                }
            }

            System.out.println();
            System.out.println();
            for(int i = 0; i < inputdata.antallNoder; i++){
                for(int j = 0; j < inputdata.antallNoder; j++){
                    System.out.println(inputdata.numberOfLanesOnArc[i][j]);
                }
            }



        }
        catch(Exception e){
            e.printStackTrace();
        }

    }

}
