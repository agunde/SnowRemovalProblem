/**
 * Created by andershg on 13.02.2017.
 */
public class InstanceData {
    public int[][] distanceLane;
    public int[][] deadheadingtimeLane;
    public int[][] numberOfPlowJobsLane;
    public int[][] timeWindowLane;
    public int[][] distanceSidewalk;
    public int[][] deadheadingtimeSidewalk;
    public int[][] numberOfPlowJobsSidewalk;
    public int[][] numberOfLanesOnArc;
    public int[][] numberofSidewalksOnArc;
    public int maxTime;
    public String instanceName;

    public InstanceData(String instance){
        this.instanceName = instance;
    }


}
