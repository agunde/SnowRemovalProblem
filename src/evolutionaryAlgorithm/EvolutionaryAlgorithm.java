package evolutionaryAlgorithm;

import ColumnGeneration.XpressInterface;
import InitialSolution.Algorithm;
import InitialSolution.Arc;
import InitialSolution.TypeComparator;
import InitialSolution.Vehicle;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

public class EvolutionaryAlgorithm {

	Fenotype fenotype;
	Education education;
	Random rng = new Random();
	public Algorithm initial;
	public ArrayList<Vehicle> vehicles;
	public ArrayList<Arc> arcs;
	public ArrayList<Arc> sideWalkArcs;

	ArrayList<Genotype> adults;
	ArrayList<Genotype> selectedParents;
	ArrayList<Genotype> children;


	public EvolutionaryAlgorithm(int[][] inputGraph, int[][] inputSWGraph, int depot, int vehichles, int swVehicles){
		initial = new Algorithm(inputGraph, inputSWGraph, depot, vehichles, swVehicles);
		vehicles = initial.vehicles;
		arcs = initial.arcs;
		sideWalkArcs = initial.sideWalkArcs;
		this.fenotype = new Fenotype(arcs, sideWalkArcs, initial.arcMap, initial.arcNodeMap, initial.fwGraph, initial.fwPath, initial.fwGraphSW, initial.fwPathSW, initial.fwBestGraph, initial.fwBestPath, depot, vehichles, swVehicles);
		education = new Education(fenotype);
	}


	/*TODO:
	 * initialize population
	 * 
	 * while some condition(s):
	 * 	make pairs
	 * 	do crossovers (two point, as described)
	 * 	calculate fitnesses
	 * 	do mutations with fitness checking
	 * 	update population
	 * 		find best inidvidual
	 * 		kill some at random
	 * 		make new population and all that stuff
	 * 	output logging data*/
	@SuppressWarnings("unused")
	public void run() {
		/*if(EvolutionaryAlgorithmParams.ADULT_SELECTION == AdultSelection.FULL_REPLACEMENT && (EvolutionaryAlgorithmParams.NUMBER_OF_CROSSOVER_PAIRS + 0.0) != (EvolutionaryAlgorithmParams.POPULATION_SIZE + 0.0) / 2.0){
			System.out.println("Full generational replacement requires that there are exactly as many new children as there are spots in the population. " +
					"Aborting because number of crossoverpair is not equal to half the population size.");
			return;
		}*/

		long generationNumber = 0;
		long startTime = System.currentTimeMillis();
		long timeTaken;
		long lastGenerationFitnessWasUpdated = 0;
		Genotype bestIndividual;
		Genotype copyOfBestIndividual;
		Genotype currentGenerationsCandidate;

		adults = new ArrayList<>();
		selectedParents = new ArrayList<>();
		children = new ArrayList<>();

		int counter = 0;
		while (adults.size() < 200) {
			adults.add(fenotype.createRandomGenotype());
			counter++;
		}
		int newBestSolutionCounter = 0;
		bestIndividual = new Genotype(Collections.max(adults));
		double bestSolution = Collections.min(adults).fitness;
		double tempBestSolution = 0;
		while (true) {
			//System.out.println("Starting EA main loop");
			ArrayList<Genotype> offspring = Selecting.Mating(adults, fenotype);
			children = education.educateChildren(offspring);
			adults.addAll(children);
			updatePopulation();
			tempBestSolution = Collections.min(adults).fitness;
			newBestSolutionCounter++;

			if (tempBestSolution< bestSolution){
				bestIndividual = new Genotype(Collections.min(adults));
				bestSolution = tempBestSolution;
				System.out.println(bestSolution);
				newBestSolutionCounter = 0;
			}

			if(newBestSolutionCounter == 10000){
				ArrayList<Vehicle> bestResult = fenotype.getFenotype(bestIndividual);
				fenotype.resetPlowingtimes();
				Collections.sort(vehicles, new TypeComparator());
				for (Vehicle vehicle : bestResult) {
					vehicle.reRoute();
					for(int x = 0; x<vehicle.tasks.size();x++){
						System.out.println(vehicle.tasks.get(x).identifier + "  " + vehicle.tasks.get(x).type);
					}
					System.out.println("NESTE KJØRETØY");
				}
				for (Vehicle vehicle : bestResult) {
					System.out.println(vehicle);
				}
				break;

			}
		}
	}

	public void run(XpressInterface xpi) {
		/*if(EvolutionaryAlgorithmParams.ADULT_SELECTION == AdultSelection.FULL_REPLACEMENT && (EvolutionaryAlgorithmParams.NUMBER_OF_CROSSOVER_PAIRS + 0.0) != (EvolutionaryAlgorithmParams.POPULATION_SIZE + 0.0) / 2.0){
			System.out.println("Full generational replacement requires that there are exactly as many new children as there are spots in the population. " +
					"Aborting because number of crossoverpair is not equal to half the population size.");
			return;
		}*/

		long generationNumber = 0;
		long startTime = System.currentTimeMillis();
		long timeTaken;
		long lastGenerationFitnessWasUpdated = 0;
		Genotype bestIndividual;
		Genotype copyOfBestIndividual;
		Genotype currentGenerationsCandidate;

		adults = new ArrayList<>();
		selectedParents = new ArrayList<>();
		children = new ArrayList<>();

		int counter = 0;
		while (adults.size() < 200) {
			adults.add(fenotype.createRandomGenotype());
			counter++;
			//System.out.println("Creating Random offspring" + counter);
		}
		int newBestSolutionCounter = 0;
		bestIndividual = new Genotype(Collections.max(adults));
		double bestSolution = Collections.min(adults).fitness;
		double tempBestSolution = 0;
		while (true) {
			//System.out.println("Starting EA main loop");
			ArrayList<Genotype> offspring = Selecting.Mating(adults, fenotype);
			children = education.educateChildren(offspring);
			adults.addAll(children);
			updatePopulation();
			tempBestSolution = Collections.min(adults).fitness;
			newBestSolutionCounter++;

			if (tempBestSolution< bestSolution){
				bestIndividual = new Genotype(Collections.min(adults));
				bestSolution = tempBestSolution;
				System.out.println(bestSolution);
				newBestSolutionCounter = 0;
				/** Eksempel*/

				//ArrayList<Vehicle> randomSolution = fenotype.getFenotype(adults.get(rng.nextInt(adults.size())));

			}

			if(newBestSolutionCounter == 10000){
				ArrayList<Vehicle> bestResult = fenotype.getFenotype(bestIndividual);
				fenotype.resetPlowingtimes();
				Collections.sort(vehicles, new TypeComparator());
				for (Vehicle vehicle : bestResult) {
					vehicle.reRoute();
					/*for(int x = 0; x<vehicle.tasks.size();x++){
						System.out.println(vehicle.tasks.get(x).identifier + "  " + vehicle.tasks.get(x).type);
					}
					System.out.println("NESTE KJØRETØY");*/
				}
				//int makespan = fenotype.calculateFitness(bestResult);
				//xpi.inputdata.maxTime = fenotype.calculateFitness(bestResult);
				for (Vehicle vehicle : bestResult) {
					System.out.println(vehicle);
					vehicle.addColumnToMaster(xpi);
				}
				for (int i = 0; i < adults.size()-1; i++){
					ArrayList<Vehicle> randomSolution = fenotype.getFenotype(adults.get(i));
					fenotype.resetPlowingtimes();
					Collections.sort(vehicles, new TypeComparator());
					for (Vehicle vehicle : randomSolution) {
						vehicle.reRoute();
						//System.out.println(vehicle);
						vehicle.addColumnToMaster(xpi);
					}
				}

				break;

			}
		}
	}



	public void updatePopulation() {
		Collections.sort(adults);
		int remove = 1;
		while(adults.size() > 200){
			if(rng.nextDouble() >= 0.1){
				if(remove >= adults.size()){
					remove = 1;
				}
				adults.remove(adults.size()-remove);
			}
			remove++;
		}
	}


}






























