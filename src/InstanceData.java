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

    public InstanceData(String instance){
        this.instanceName = instance;
    }


}
