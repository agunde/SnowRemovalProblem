import java.util.ArrayList;

/**
 * Created by andershg on 13.02.2017.
 */
public class Node {
    public ArrayList<Label> finishedLabels;
    public int number;
    public ArrayList<Node> neighbourNodes;

    public Node(int number){
        finishedLabels = new ArrayList<Label>();
        this.number = number;
        this.neighbourNodes = new ArrayList<>();
    }

    public void addUnfinishedLabel(Label label){
        finishedLabels.add(label);
    }

    public void emptyList(){
        finishedLabels.clear();
    }

    public void addNeigbour(Node node){
        neighbourNodes.add(node);
    }

    public ArrayList<Node> getNeighbourNodes(){
        return neighbourNodes;
    }
}
