import java.util.ArrayList;

/**
 * Created by andershg on 13.02.2017.
 */
public class Nodes {
    public ArrayList<Label> unfinishedLabels;
    public int number;

    public Nodes(int number){
        unfinishedLabels = new ArrayList<Label>();
        this.number = number;
    }

    public void addUnfinishedLabel(Label label){
        unfinishedLabels.add(label);
    }

    public void emptyList(){
        unfinishedLabels.clear();
    }
}
