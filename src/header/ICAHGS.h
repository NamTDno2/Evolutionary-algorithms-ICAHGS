#ifndef ICAHGS_H
#define ICAHGS_H

#include "DataStructures.h"
#include "Solution.h"
#include "Decoder.h"
#include "LocalSearch.h"
#include <vector>
#include <random>

class ICAHGS {
public:
    ICAHGS(const Instance& inst, int popSize = 50, int numEmpires = 5);
    
    std::vector<Solution> run(int maxIterations = 100);
    
private:
    const Instance& instance;
    Decoder decoder;
    LocalSearch localSearch;
    
    int populationSize;
    int numImperialists;
    std::vector<Empire> empires;
    std::vector<Solution> paretoArchive;
    
    std::mt19937 rng;
    
    // Initialization
    void initializePopulation();
    void createEmpires(std::vector<Individual>& population);
    
    // ICA operations
    void assimilationAndRevolution();
    void imperialisticCompetition();
    
    // Genetic operators
    std::vector<int> orderCrossover(const std::vector<int>& parent1,
                                   const std::vector<int>& parent2);
    void mutate(std::vector<int>& permutation, double mutationRate = 0.05);
    
    // Pareto operations
    void updateParetoArchive(const Solution& solution);
    double calculateEmpirePower(const Empire& empire);
    
    // Utilities
    int selectRandomColony(Empire& empire);
    int selectWeakestEmpire();
    bool convergenceReached();
};

#endif // ICAHGS_H
