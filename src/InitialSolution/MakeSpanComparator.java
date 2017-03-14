package InitialSolution;

import java.util.Comparator;

/**
 * Created by Magnu on 10.12.2016.
 */

//This is just a comparator, so we can sort the vehicles from highest finishing time to lowest.
public class MakeSpanComparator implements Comparator<Vehicle> {
    @Override
    public int compare(Vehicle a, Vehicle b) {
        return b.totalLength- a.totalLength;
    }
}
