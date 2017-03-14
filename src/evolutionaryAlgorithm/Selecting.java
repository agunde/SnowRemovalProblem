package evolutionaryAlgorithm;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

/**
 * Created by Magnu on 27.02.2017.
 */
public class Selecting {

    public static Random rng = new Random();
    public static final int tournamentSize = 5;
    public static final double selectionProb = 0.5;
    public static final double offSpringPerEpoch = 10;

    public static Genotype tournamentSelection(ArrayList<Genotype> population){
        ArrayList<Integer> chosenIndices = new ArrayList<>();
        ArrayList<Genotype> chosenChromosomes = new ArrayList<>();
        double prob;
        int temp;
        while(chosenChromosomes.size() < tournamentSize){
            temp = rng.nextInt(population.size());
            if(!chosenChromosomes.contains(population.get(temp))){
                chosenChromosomes.add(population.get(temp));
            }

        }

        Collections.sort(chosenChromosomes);
        prob = rng.nextDouble();
        for(int i = 0; i < tournamentSize; i++){
            if (prob > selectionProb){
                prob = prob + (prob*(1-prob));
            }
            else{
                return chosenChromosomes.get(i);
            }
        }
        return chosenChromosomes.get(chosenChromosomes.size()-1);



    }

    public static ArrayList<Genotype> holdTournament(ArrayList<Genotype> population){
        ArrayList<Genotype> chromosomes = new ArrayList<>();
        ArrayList<Integer> indices = new ArrayList<>();
        Genotype selected;
        while (indices.size() < offSpringPerEpoch){
            selected = tournamentSelection(population);
            if(!indices.contains(population.indexOf(selected))) {
                indices.add(population.indexOf(selected));
            }
        }
        for(int i = 0; i < indices.size(); i++){
            chromosomes.add(population.get(indices.get(i)));
        }
        return chromosomes;

    }

    public static ArrayList<Genotype> Mating(ArrayList<Genotype> population, Fenotype fenotype){
        ArrayList<Genotype> parents = Selecting.holdTournament(population);
        ArrayList<Genotype> offspring = new ArrayList<>();
        FMXCrossover tempLane;
        FMXCrossover tempSidewalk;
        Genotype tempOffspring1;
        Genotype tempOffspring2;
        for(int i = 0; i < (parents.size()-1); i = i+2){
            tempLane = new FMXCrossover(parents.get(i).getLaneGenome(),parents.get(i+1).getLaneGenome());
            tempSidewalk = new FMXCrossover(parents.get(i).getSidewalkGenome(),parents.get(i+1).getSidewalkGenome());
            tempOffspring1 = new Genotype(tempLane.getOffspring1(), tempSidewalk.getOffspring1(), fenotype.calculateFitness(tempLane.getOffspring1(), tempSidewalk.getOffspring2()));
            tempOffspring2 = new Genotype(tempLane.getOffspring2(), tempSidewalk.getOffspring2(), fenotype.calculateFitness(tempLane.getOffspring1(), tempSidewalk.getOffspring2()));
            /*if(rng.nextDouble() <= mutationRate){
                tempOffspring1 = Mutation(tempOffspring1);
            }*/
            offspring.add(tempOffspring1);
            offspring.add(tempOffspring2);
        }
        return offspring;
    }

}
/*
for (int i = 0; i < selectedParents.size(); i += 2) {
        FMXCrossover tempLaneX = new FMXCrossover(selectedParents.get(i).getLaneGenome(),selectedParents.get(i+1).getLaneGenome());
        FMXCrossover tempSidewalkX = new FMXCrossover(selectedParents.get(i).getSidewalkGenome(),selectedParents.get(i+1).getSidewalkGenome());
        Genotype offspring1 = new Genotype(tempLaneX.getOffspring1(), tempSidewalkX.getOffspring1(), fenotype.calculateFitness(tempLaneX.getOffspring1(), tempSidewalkX.getOffspring1()));
        Genotype offspring2 = new Genotype(tempLaneX.getOffspring2(), tempSidewalkX.getOffspring2(), fenotype.calculateFitness(tempLaneX.getOffspring2(), tempSidewalkX.getOffspring2()));
        children.add(offspring1);
        children.add(offspring2);
        }*/