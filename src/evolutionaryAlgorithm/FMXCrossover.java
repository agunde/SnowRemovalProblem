package evolutionaryAlgorithm; /**
 * Created by Magnu on 22.09.2016.
 */
import java.util.Random;

public class FMXCrossover
{
    private int[] depots;
    private int[] parent1;
    private int[] parent2;
    private int[] parent1nd;
    private int[] parent2nd;
    private int[] offspring1nd;
    private int[] offspring2nd;
    private int[] segment1;
    private int[] segment2;
    private int   cutPoint1;
    private int   cutPoint2;

    /**
     * This is my implementation of the PMX used for Mating in Genetic Algorithm
     */
    public FMXCrossover(int[] parent1, int[] parent2){
        int depotCounter = 0;
        for(int x = 0; x< parent1.length; x++){
            if(parent1[x] == -1){
                depotCounter++;
            }
        }
        this.parent1 = new int[parent1.length];
        this.parent2 = new int[parent2.length];
        this.parent1nd = new int[parent1.length - depotCounter];
        this.parent2nd = new int[parent2.length - depotCounter];

        depots = new int[depotCounter];
        int taskCounter1 = 0;
        int taskCounter2 = 0;
        int depotCounter1 = 0;
        int depotCounter2 = 0;
        for(int x = 0; x< parent1.length; x++){
            if(parent1[x] == -1){
                depots[depotCounter1] = x;
                depotCounter1++;
            }
            else{
                parent1nd[taskCounter1] = parent1[x];
                taskCounter1++;
            }
            if(parent2[x] == -1){
                depotCounter2++;
            }
            else{
                parent2nd[taskCounter2] = parent2[x];
                taskCounter2++;
            }
        }

        for(int index = 0; index < parent1.length; index ++){
            this.parent1[index] = parent1[index];
            this.parent2[index] = parent2[index];
        }
        Random firstRNum  = new Random();
        Random secondRNum = new Random();


        /**
         * Here we define the cutpoints for the Swath
         */

        int randomNo_Boundary = (parent1nd.length) - 1;
        offspring1nd = new int[parent1nd.length];
        offspring2nd = new int[parent2nd.length];
        cutPoint1 = firstRNum.nextInt(randomNo_Boundary);
        cutPoint2 = secondRNum.nextInt(randomNo_Boundary);
        while(cutPoint1 == cutPoint2){
            cutPoint2 = secondRNum.nextInt(randomNo_Boundary);
        }
        if(cutPoint1 > cutPoint2){
            int temp = cutPoint1;    // Make sure CutPoint1 is greater than
            cutPoint1 = cutPoint2;    // cutPoint2 //
            cutPoint2 = temp;
        }
        create_Segments(cutPoint1, cutPoint2);
        crossOver(offspring1nd, parent1nd, parent2nd);
        crossOver(offspring2nd, parent2nd, parent1nd);

    }

    public FMXCrossover(int[] parent1Lane, int[] parent1Sidewalk, int[] parent2Lane, int[] parent2Sidewalk){
        int depotCounter = 0;
        for(int x = 0; x< parent1.length; x++){
            if(parent1[x] == -1){
                depotCounter++;
            }
        }
        this.parent1 = new int[parent1Lane.length];
        this.parent2 = new int[parent2.length];
        this.parent1nd = new int[parent1.length - depotCounter];
        this.parent2nd = new int[parent2.length - depotCounter];

        depots = new int[depotCounter];
        int taskCounter1 = 0;
        int taskCounter2 = 0;
        int depotCounter1 = 0;
        int depotCounter2 = 0;
        for(int x = 0; x< parent1.length; x++){
            if(parent1[x] == -1){
                depots[depotCounter1] = x;
                depotCounter1++;
            }
            else{
                parent1nd[taskCounter1] = parent1[x];
                taskCounter1++;
            }
            if(parent2[x] == -1){
                depotCounter2++;
            }
            else{
                parent2nd[taskCounter2] = parent2[x];
                taskCounter2++;
            }
        }

        for(int index = 0; index < parent1.length; index ++){
            this.parent1[index] = parent1[index];
            this.parent2[index] = parent2[index];
        }
        Random firstRNum  = new Random();
        Random secondRNum = new Random();


        /**
         * Here we define the cutpoints for the Swath
         */

        int randomNo_Boundary = (parent1nd.length) - 1;
        offspring1nd = new int[parent1nd.length];
        offspring2nd = new int[parent2nd.length];
        cutPoint1 = firstRNum.nextInt(randomNo_Boundary);
        cutPoint2 = secondRNum.nextInt(randomNo_Boundary);
        while(cutPoint1 == cutPoint2){
            cutPoint2 = secondRNum.nextInt(randomNo_Boundary);
        }
        if(cutPoint1 > cutPoint2){
            int temp = cutPoint1;    // Make sure CutPoint1 is greater than
            cutPoint1 = cutPoint2;    // cutPoint2 //
            cutPoint2 = temp;
        }
        create_Segments(cutPoint1, cutPoint2);
        crossOver(offspring1nd, parent1nd, parent2nd);
        crossOver(offspring2nd, parent2nd, parent1nd);

    }


    /**
     * For an Element given by its index check that it doesn't appear twice
     */
    private boolean check_forDuplicates(int [] offspring, int indexOfElement){
        for(int index = 0; index < offspring.length; index++){
            if((offspring[index] == offspring[indexOfElement]) &&
                    (indexOfElement != index) ){
                return true;
            }
        }
        return false;
    }

    /**
     *    If Element is Duplicated, replace it by using its mapping
     * */
    private void sort_Duplicates(int [] offspring, int indexOfElement){
        for(int index = 0; index < segment1.length; index++){
            if(segment1[index] == offspring[indexOfElement]){
                offspring[indexOfElement] = segment2[index];
            }
            else if(segment2[index] == offspring[indexOfElement]){
                offspring[indexOfElement] = segment1[index];
            }
        }
    }

    private void create_Segments(int cutPoint1, int cutPoint2){
        int capacity_ofSegments = (cutPoint2 - cutPoint1) + 1;
        segment1 = new int[capacity_ofSegments];
        segment2 = new int[capacity_ofSegments];
        int segment1and2Index = 0;
        for(int index = 0; index < parent1nd.length; index++){
            if((index >= cutPoint1) && (index <= cutPoint2)){
                int x = parent1nd[index];  int y = parent2nd[index];
                segment1[segment1and2Index] = x;
                segment2[segment1and2Index] = y;
                segment1and2Index++;
            }
        }
    }

    private void insert_Segments(int[] offspring, int[] segment){
        int segmentIndex = 0;
        for(int index = 0; index < offspring.length; index++){
            if((index >= cutPoint1) && (index <= cutPoint2)){
                offspring[index] = segment[segmentIndex];
                segmentIndex++;
            }
        }
    }

    /**
     *offspring2 gets segment 1, offspring1 gets segment2. For an explanation, check the report
     */
    public void crossOver(int [] offspring, int[] parentX, int[] parentY){
        if(offspring == offspring1nd){
            int[] segment = segment2;
            insert_Segments(offspring, segment);
        }

        else if(offspring == offspring2nd){
            int [] segment = segment1;
            insert_Segments(offspring, segment);
        }

        for(int index = 0; index < offspring.length; index++){
            if((index < cutPoint1) || (index > cutPoint2)){
                offspring[index] = parentX[index];
            }
        }

        for(int index = 0; index < offspring.length; index++){
            if((index < cutPoint1) || (index > cutPoint2)){
                while(check_forDuplicates(offspring, index)){
                    sort_Duplicates(offspring, index);
                }
            }
        }
    }

    public int[] getOffspring1()
    {
        int[] offspring1 = new int[offspring1nd.length + depots.length];
        for (int x = 0; x <depots.length;x++){
            offspring1[depots[x]] = -1;
        }
        int x = 0;
        for (int y = 0; y <offspring1nd.length;y++){
            if(offspring1[x] == -1){
                x++;
            }
            offspring1[x] = offspring1nd[y];
            x++;
        }

        return offspring1;
    }
    public int[] getOffspring2()
    {
        int[] offspring2 = new int[offspring2nd.length + depots.length];
        for (int x = 0; x <depots.length;x++){
            offspring2[depots[x]] = -1;
        }
        int x = 0;
        for (int y = 0; y <offspring2nd.length;y++){
            if(offspring2[x] == -1){
                x++;
            }
            offspring2[x] = offspring2nd[y];
            x++;
        }

        return offspring2;
    }

    public void printOffspring(int [] offspring1, int [] offspring2){
        System.out.println(" ");
        System.out.println("Parents");
        System.out.println("");
        for(int parent1Index = 0; parent1Index < parent1.length; parent1Index++){
            System.out.print(" " + parent1[parent1Index]);
        }
        System.out.println("");
        for(int parent2Index = 0; parent2Index < parent2.length; parent2Index++){
            System.out.print(" " + parent2[parent2Index]);
        }

        System.out.println("");
        System.out.println("Offsprings");
        for(int offspring1Index = 0;
            offspring1Index < offspring1.length; offspring1Index++){
            System.out.print(" " + offspring1[offspring1Index]);
        }

        System.out.println("");
        for(int offspring2Index = 0;
            offspring2Index < offspring2.length; offspring2Index++){
            System.out.print(" " + offspring2[offspring2Index]);
        }
        System.out.println("");
    }

    /**
     *
     * If you still don't understand it, You can see it here
     */
    public static void main(String[] args){
        int[] parent1 = {1,2,3,-1,4,5,-1,6,7,8};
        int[] parent2 = {8,7,-1,6,5,4,3,2,-1,1};

        FMXCrossover cross = new FMXCrossover(parent1,parent2);
        for(int x = 0; x<parent1.length;x++){
            System.out.print(cross.getOffspring1()[x] + " ");
        }
        System.out.println("FMXCrossover.main");
        for(int x = 0; x<parent1.length;x++){
            System.out.print(cross.getOffspring2()[x] + " ");
        }

    }
}
