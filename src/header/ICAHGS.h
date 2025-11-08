#ifndef ICAHGS_H
#define ICAHGS_H

#include "DataStructures.h"
#include "Solution.h"
#include "Decoder.h"
#include "LocalSearch.h"
#include <vector>
#include <random>
#include <unordered_set>  // ← THÊM DÒNG NÀY (cho unordered_set)
#include <cstdint>

class ICAHGS {
public:
    ICAHGS(const Instance& inst, int popSize = 50, int numEmpires = 5);
    ~ICAHGS();  // ← THÊM DESTRUCTOR
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
    
    // **THÊM MỚI: Hash manager & duplicate tracker**
    SolutionHasher* hasher;  // ← THÊM
    std::unordered_set<uint64_t> seenHashes;  // ← THÊM

    // Initialization
    void initializePopulation();
    void createEmpires(std::vector<Individual>& population);

    // **THÊM MỚI: Duplicate detection**
    bool isDuplicate(Solution& solution);  // ← THÊM
    
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
