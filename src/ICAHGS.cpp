#include "ICAHGS.h"
#include <algorithm>
#include <ctime>
#include <iostream>
#include <limits> // Thêm thư viện này để sử dụng giá trị lớn nhất/nhỏ nhất

ICAHGS::ICAHGS(const Instance& inst, int popSize, int numEmp) 
    : instance(inst), decoder(inst), localSearch(inst),
      populationSize(popSize), numImperialists(numEmp) {
    
    rng.seed(static_cast<unsigned int>(time(nullptr)));
}

std::vector<Solution> ICAHGS::run(int maxIterations) {
    std::cout << "Initializing population..." << std::endl;
    initializePopulation();
    
    std::cout << "Starting ICAHGS optimization..." << std::endl;
    
    for (int iter = 0; iter < maxIterations; iter++) {
        std::cout << "Iteration " << (iter + 1) << "/" << maxIterations << std::endl;
        
        // Assimilation and Revolution
        assimilationAndRevolution();
        
        // Imperialistic Competition
        imperialisticCompetition();
        
        // Print progress
        if ((iter + 1) % 10 == 0) {
            std::cout << "  Archive size: " << paretoArchive.size() << std::endl;
            std::cout << "  Number of empires: " << empires.size() << std::endl;
        }
        
        // Check convergence
        if (empires.size() <= 1) { // Sửa thành <= 1 cho an toàn
            std::cout << "Converged: only one or zero empire remains" << std::endl;
            break;
        }
    }
    
    std::cout << "Optimization complete. Final archive size: " 
              << paretoArchive.size() << std::endl;
    
    return paretoArchive;
}

void ICAHGS::initializePopulation() {
    std::vector<Individual> population;
    
    // Generate random permutations
    for (int i = 0; i < populationSize; i++) {
        Individual ind(instance.getNumCustomers());
        
        // Shuffle permutation
        std::shuffle(ind.permutation.begin(), ind.permutation.end(), rng);
        
        // Decode
        ind.solution = decoder.decode(ind.permutation);
        
        // Update archive
        updateParetoArchive(ind.solution);
        
        population.push_back(ind);
    }
    
    // Create empires
    createEmpires(population);
}

void ICAHGS::createEmpires(std::vector<Individual>& population) {
    // Non-dominated sorting
    std::vector<Solution*> solutions;
    for (auto& ind : population) {
        solutions.push_back(&ind.solution);
    }
    ParetoRanking::nonDominatedSorting(solutions);
    
    // Sort population by Pareto rank and crowding distance
    std::sort(population.begin(), population.end(), 
              [](const Individual& a, const Individual& b) {
        if (a.solution.paretoRank != b.solution.paretoRank) {
            return a.solution.paretoRank < b.solution.paretoRank;
        }
        return a.solution.crowdingDistance > b.solution.crowdingDistance;
    });
    
    // Select imperialists (best individuals)
    empires.clear();
    for (int i = 0; i < numImperialists && i < populationSize; i++) {
        Empire empire;
        empire.imperialist = population[i];
        empire.power = 0; // Sẽ được tính lại ngay sau đây
        empires.push_back(empire);
    }
    
    // Distribute colonies
    int colonyIndex = numImperialists;
    while (colonyIndex < populationSize) {
        // Distribute to empire (round-robin)
        int empireIdx = (colonyIndex - numImperialists) % numImperialists;
        if (empireIdx < empires.size()) {
            empires[empireIdx].colonies.push_back(population[colonyIndex]);
        }
        colonyIndex++;
    }
    
    // Calculate initial powers
    for (auto& empire : empires) {
        empire.power = calculateEmpirePower(empire);
    }
}

void ICAHGS::assimilationAndRevolution() {
    for (auto& empire : empires) {
        for (size_t c = 0; c < empire.colonies.size(); c++) {
            // Crossover (Assimilation)
            std::vector<int> offspring = orderCrossover(
                empire.imperialist.permutation,
                empire.colonies[c].permutation);
            
            // Mutation (Revolution)
            mutate(offspring, 0.05);
            
            // Decode
            Solution offspringSol = decoder.decode(offspring);
            
            // Local search
            offspringSol = localSearch.improve(offspringSol, 50);
            
            // Update archive
            updateParetoArchive(offspringSol);
            
            // Replace colony if better
            if (offspringSol.dominates(empire.colonies[c].solution)) {
                empire.colonies[c].permutation = offspring;
                empire.colonies[c].solution = offspringSol;
                
                // Check if colony becomes better than imperialist (Revolution)
                if (offspringSol.dominates(empire.imperialist.solution)) {
                    std::swap(empire.imperialist, empire.colonies[c]);
                }
            }
        }
        
        // Update empire power
        empire.power = calculateEmpirePower(empire);
    }
}

void ICAHGS::imperialisticCompetition() {
    if (empires.size() <= 1) return;
    
    int weakestIdx = selectWeakestEmpire();
    
    if (empires[weakestIdx].colonies.empty()) {
        // Empire has no colonies, collapse it
        // Move imperialist to strongest empire as colony
        
        int strongestIdx = -1;
        double maxPower = -1.0;

        // ✅ Sửa logic tìm đế chế mạnh nhất
        for (size_t i = 0; i < empires.size(); ++i) {
            if (i == static_cast<size_t>(weakestIdx)) continue; // Bỏ qua đế chế yếu nhất
            if (strongestIdx == -1 || empires[i].power > maxPower) {
                maxPower = empires[i].power;
                strongestIdx = i;
            }
        }
        
        if (strongestIdx != -1) { // Đảm bảo tìm được đế chế mạnh nhất
             empires[strongestIdx].colonies.push_back(std::move(empires[weakestIdx].imperialist));
        }
       
        empires.erase(empires.begin() + weakestIdx);
        
        std::cout << "  Empire collapsed. Remaining empires: " 
                  << empires.size() << std::endl;
    } else {
        // Transfer weakest colony to the winner of the competition
        int colonyIdx = selectRandomColony(empires[weakestIdx]);
        
        // Logic to select the winner empire based on power (probability)
        double totalPower = 0;
        for (const auto& emp : empires) {
            totalPower += emp.power;
        }

        std::uniform_real_distribution<double> dist(0.0, totalPower);
        double pick = dist(rng);
        
        int winnerIdx = -1;
        double currentPower = 0;
        for (size_t i = 0; i < empires.size(); ++i) {
            currentPower += empires[i].power;
            if (pick <= currentPower) {
                winnerIdx = i;
                break;
            }
        }

        if (winnerIdx != -1 && winnerIdx != weakestIdx) {
            empires[winnerIdx].colonies.push_back(
                std::move(empires[weakestIdx].colonies[colonyIdx]));
            empires[weakestIdx].colonies.erase(
                empires[weakestIdx].colonies.begin() + colonyIdx);
        }
    }
}

std::vector<int> ICAHGS::orderCrossover(const std::vector<int>& parent1,
                                         const std::vector<int>& parent2) {
    int n = parent1.size();
    if (n < 2) {
        return parent1;
    }
    std::vector<int> offspring(n, -1);
    
    std::uniform_int_distribution<int> dist(0, n - 1);
    int start = dist(rng);
    int end = dist(rng);
    
    if (start > end) std::swap(start, end);
    
    // Use a boolean array for faster checking
    std::vector<bool> in_offspring(n + 1, false);
    
    // Copy segment from parent1
    for (int i = start; i <= end; i++) {
        offspring[i] = parent1[i];
        in_offspring[parent1[i]] = true;
    }
    
    // Fill remaining positions from parent2
    int offspring_pos = (end + 1) % n;
    int parent2_pos = (end + 1) % n;
    
    while (offspring_pos != start) {
        int gene = parent2[parent2_pos];
        if (!in_offspring[gene]) {
            offspring[offspring_pos] = gene;
            in_offspring[gene] = true; // FIX: Đánh dấu gene đã được thêm
            offspring_pos = (offspring_pos + 1) % n;
        }
        parent2_pos = (parent2_pos + 1) % n;
    }
    
    return offspring;
}

void ICAHGS::mutate(std::vector<int>& permutation, double mutationRate) {
    int n = permutation.size();
    if (n < 2) {
        return;
    }
    std::uniform_real_distribution<double> prob(0.0, 1.0);
    std::uniform_int_distribution<int> pos(0, n - 1);
    
    for (int i = 0; i < n; i++) {
        if (prob(rng) < mutationRate) {
            int j = pos(rng);
            std::swap(permutation[i], permutation[j]);
        }
    }
}

void ICAHGS::updateParetoArchive(const Solution& solution) {
    if (solution.systemCompletionTime >= INF) return;
    
    bool isDominated = false;
    
    // Remove solutions in the archive that are dominated by the new solution
    paretoArchive.erase(std::remove_if(paretoArchive.begin(), paretoArchive.end(),
        [&](const Solution& archiveSol) {
            if (solution.dominates(archiveSol)) {
                return true;
            }
            if (archiveSol.dominates(solution)) {
                isDominated = true;
            }
            return false;
        }),
        paretoArchive.end());

    if (!isDominated) {
        paretoArchive.push_back(solution);
    }
}

double ICAHGS::calculateEmpirePower(const Empire& empire) {
    double impPower = 1.0 / (empire.imperialist.solution.paretoRank + 1.0);
    
    if (!empire.colonies.empty()) {
        double avgColonyPower = 0;
        for (const auto& colony : empire.colonies) {
            avgColonyPower += 1.0 / (colony.solution.paretoRank + 1.0);
        }
        avgColonyPower /= empire.colonies.size();
        
        impPower = impPower + 0.1 * avgColonyPower;
    }
    
    return impPower;
}

int ICAHGS::selectRandomColony(Empire& empire) {
    if (empire.colonies.empty()) return -1;
    
    std::uniform_int_distribution<int> dist(0, empire.colonies.size() - 1);
    return dist(rng);
}

int ICAHGS::selectWeakestEmpire() {
    if (empires.empty()) return -1;

    int weakestIdx = 0;
    // ✅ Sửa: Khởi tạo minPower với power của phần tử đầu tiên
    double minPower = empires[0].power; 
    
    for (size_t i = 1; i < empires.size(); i++) {
        if (empires[i].power < minPower) {
            minPower = empires[i].power;
            weakestIdx = i;
        }
    }
    
    return weakestIdx;
}
