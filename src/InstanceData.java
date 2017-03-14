import java.util.ArrayList;
import java.util.Hashtable;

/**
 * Created by andershg on 13.02.2017.
 */
public class InstanceData {
    public int[][] plowingtimeLane;
    public int[][] deadheadingtimeLane;
    public int[][] numberOfPlowJobsLane;
    public int[][] timeWindowLane;
    public int[][] plowingtimeSidewalk;
    public int[][] deadheadingtimeSidewalk;
    public int[][] numberOfPlowJobsSidewalk;
    public int[][] numberOfLanesOnArc;
    public int[][] numberofSidewalksOnArc;
    public int maxTime;
    public int antallNoder;
    public int startNode;
    public int endNode;
    public String instanceName;
    public Hashtable<Integer,ArrayList<Integer>> deadheadingNeighborLane;
    public Hashtable<Integer,ArrayList<Integer>> deadheadingNeighborSidewalkReversed;
    public Hashtable<Integer,ArrayList<Integer>> plowingNeighborLane;
    public Hashtable<Integer,ArrayList<Integer>> plowingNeighborSidewalkReversed;

    public InstanceData(String instance){
        this.instanceName = instance;
        deadheadingNeighborSidewalkReversed = new Hashtable<>();
        deadheadingNeighborLane = new Hashtable<>();
        plowingNeighborSidewalkReversed = new Hashtable<>();
        plowingNeighborLane = new Hashtable<>();
    }


}
