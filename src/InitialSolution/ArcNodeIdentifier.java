/**
 * Created by Magnu on 06.12.2016.
 */
package InitialSolution;

public final class ArcNodeIdentifier {

    private final Integer start, end;

    public ArcNodeIdentifier(Integer start, Integer end) {
        this.start = start;
        this.end = end;
    }

    public Integer getStart() {
        return start;
    }

    public Integer getEnd() {
        return end;
    }

    @Override
    public int hashCode() {
        return start.hashCode() ^ end.hashCode();
    }

    @Override
    public boolean equals(Object obj) {
        return (obj instanceof ArcNodeIdentifier) && ((ArcNodeIdentifier) obj).getStart() == this.start
                && ((ArcNodeIdentifier) obj).getEnd() == this.end;
    }
}