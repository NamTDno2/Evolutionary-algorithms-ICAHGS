#include "ICAHGS.h"
#include <algorithm>
#include <ctime>
#include <iostream>
#include <limits> // Thêm thư viện này để sử dụng giá trị lớn nhất/nhỏ nhất
#include <unordered_set>  // ← THÊM
#include <cstdint> 
#include <random>

ICAHGS::ICAHGS(const Instance& inst, int popSize, int numEmp) 
    : instance(inst), decoder(inst), localSearch(inst),
      populationSize(popSize), numImperialists(numEmp) {
    
    rng.seed(static_cast<unsigned int>(time(nullptr)));
    // **THÊM MỚI: Khởi tạo hasher**
    // hasher = new SolutionHasher(
    //     instance.getNumCustomers(),
    //     instance.numTrucks,
    //     instance.numDrones
    // );
    
}
ICAHGS::~ICAHGS() {
    // delete hasher;
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
// TODO: Fix duplicate detection for new Chromosome-based solution
bool ICAHGS::isDuplicate(Solution& solution) {
    // Tính hash cho solution
    solution.solutionHash = hasher->computeHash(solution);
    
    // Kiểm tra hash đã tồn tại chưa
    if (seenHashes.find(solution.solutionHash) != seenHashes.end()) {
        return true;  // TRÙNG LẶP!
    }
    
    // Chưa tồn tại, thêm vào
    seenHashes.insert(solution.solutionHash);
    return false;  // KHÔNG TRÙNG
}

void ICAHGS::initializePopulation() {
    std::vector<Individual> population;
    int attempts = 0;
    int maxAttemptsPerSolution = 100;  // Tối đa 100 attempts cho mỗi solution
    
    std::cout << "Initializing population with duplicate detection..." << std::endl;
    
    // Tạo đủ populationSize solutions
    while (population.size() < (size_t)populationSize) {
        attempts++;
        
        Chromosome chrom(instance.customers, 
                          instance.numTrucks, 
                          instance.numDrones, 
                          rng);

        Individual ind(chrom);

        // Individual ind(instance.getNumCustomers());
        
        // // Shuffle permutation
        // std::shuffle(ind.permutation.begin(), ind.permutation.end(), rng);
        
        // Decode
        // ind.solution = decoder.decodeIncremental(ind.permutation);
        ind.solution = decoder.decode(chrom, instance);
        
        // TODO: Kiểm tra duplicate
        // if (isDuplicate(ind.solution)) {
        //     // Nếu đã thử quá nhiều lần, giảm yêu cầu
        //     if (attempts > population.size() * maxAttemptsPerSolution) {
        //         std::cout << "  Too many duplicates, accepting this one anyway..." << std::endl;
        //         // Vẫn thêm vào để đủ số lượng
        //         population.push_back(ind);
        //         updateParetoArchive(ind.solution);
        //     } else {
        //         std::cout << "  Duplicate detected (attempt " << attempts << "), trying again..." << std::endl;
        //         continue;
        //     }
        // } else {
        //     // Not duplicate, keep it
        //     population.push_back(ind);
        //     updateParetoArchive(ind.solution);
        // }

        // Add to population
        population.push_back(ind);
        updateParetoArchive(ind.solution);
        
        if (population.size() % 10 == 0) {
            std::cout << "  Created " << population.size() << "/" << populationSize 
                      << " unique solutions..." << std::endl;
        }
    }
    
    std::cout << "Population initialized: " << population.size() 
              << " solutions (from " << attempts << " attempts)" << std::endl;
    
    if (attempts > population.size()) {
        std::cout << "Duplicate rate: " 
                  << (100.0 * (attempts - population.size()) / attempts) << "%" << std::endl;
    }
    
    // Kiểm tra trước khi create empires
    if (population.size() < (size_t)numImperialists) {
        std::cerr << "ERROR: Not enough solutions (" << population.size() 
                  << ") for " << numImperialists << " empires!" << std::endl;
        std::cerr << "Reducing number of empires..." << std::endl;
        numImperialists = std::max(1, (int)population.size() / 2);
    }
    
    // Create empires
    createEmpires(population);
}



void ICAHGS::createEmpires(std::vector<Individual>& population) {
    if (population.empty()) {
        std::cerr << "ERROR: Population is empty!" << std::endl;
        return;
    }
    
    // ========== BƯỚC 1: Non-dominated Sorting ==========
    std::vector<Solution*> solutions;
    for (auto& ind : population) {
        solutions.push_back(&ind.solution);
    }
    ParetoRanking::nonDominatedSorting(solutions);
    ParetoRanking::calculateCrowdingDistance(solutions);
    
    // ========== BƯỚC 2: Group by Front ==========
    // Use map<int, Front> so each rank maps to a Front that contains a vector of countries
    std::map<int, Front> fronts;
    for (auto& ind : population) {
        int rank = ind.solution.paretoRank;
        fronts[rank].countries.push_back(ind);
        fronts[rank].rankCompletionTime += ind.solution.systemCompletionTime;
        fronts[rank].rankWaitingTime += ind.solution.totalSampleWaitingTime;
    }
    
    std::cout << "Fronts structure:" << std::endl;
    for (auto& [rank, front] : fronts) {
        std::cout << "  Front " << rank << ": " << front.countries.size() << " solutions" << std::endl;
    }

    // ========== TÍNH POWER CHO INDIVIDUAL ===========

    
    double fitness = 0;
     for (auto& [rank, front] : fronts) {
        if (front.countries.size() == 0) continue;
        for (auto& ind : front.countries) {
            // Skip invalid (infeasible) solutions
            if (ind.solution.systemCompletionTime == INF ||
                ind.solution.totalSampleWaitingTime == INF) {
                ind.power = -1;
                continue;
            }
            fitness = ind.solution.systemCompletionTime/front.rankCompletionTime + ind.solution.totalSampleWaitingTime/front.rankWaitingTime;
            fitness = fitness*(rank-1)*2;
            ind.power = std::min(1.0 / fitness, 1e6);
                
        }
     }
    
    // ========== BƯỚC 3: Chọn Imperialists từ Fronts ==========
    std::vector<Individual> imperialists;
    double totalImperialistPower = 0;
    
    for (auto& [rank, front] : fronts) {
        // Shuffle the countries in this front to randomize selection
        // std::shuffle(front.countries.begin(), front.countries.end(), rng);

        // Sort by power (descending), feasible solutions first (power >= 0)
        sort(front.countries.begin(), front.countries.end(),
             [](const Individual& a, const Individual& b) {
                 // Feasible solutions come first (power >= 0)
                 if ((a.power >= 0) != (b.power >= 0)) {
                     return a.power >= 0;  // Feasible before infeasible
                 }
                 // Both feasible: sort by power descending
                 if (a.power >= 0 && b.power >= 0) {
                     return a.power > b.power;
                 }
                 // Both infeasible: leave as is
                 return false;
             });

        std::cout << "Selecting from Front " << rank << "..." << std::endl;

        for (auto& ind : front.countries) {
            // Skip infeasible individuals (power == -1)
            if (ind.power < 0) {
                continue;
            }

            imperialists.push_back(ind);
            totalImperialistPower += ind.power;
            std::cout << "  Selected imperialist #" << imperialists.size()
                      << " (rank=" << ind.solution.paretoRank
                      << ", CT=" << ind.solution.systemCompletionTime
                      << ", power=" << ind.power << ")" << std::endl;

            if (imperialists.size() >= (size_t)numImperialists) {
                break;
            }
        }
        
        if (imperialists.size() >= (size_t)numImperialists) {
            break;
        }
    }
    
    if (imperialists.size() < (size_t)numImperialists) {
        std::cerr << "WARNING: Only found " << imperialists.size() 
                  << " imperialists, need " << numImperialists << std::endl;
    }
    
    // ========== BƯỚC 4: Tạo Empires ==========
    empires.clear();
    for (auto& imp : imperialists) {
        Empire empire;
        empire.imperialist = imp;
        empire.power = 0;
        empires.push_back(empire);
        
    }
    
    std::cout << "Created " << empires.size() << " empires" << std::endl;
    
    // ========== BƯỚC 5: Phân Colonies ==========
    int colonyIndex = 0;
    for (size_t i = numImperialists; i < population.size(); i++) {
        int empireIdx = colonyIndex % empires.size();
        empires[empireIdx].colonies.push_back(population[i]);
        colonyIndex++;
    }
   
   // Distribute remaining individuals as colonies proportionally to empire imperialist power
    // int coloniesDistributed = 0;
    // int totalColonies = population.size() - numImperialists;
    
    // for (size_t e = 0; e < empires.size(); e++) {
    //     // Calculate portion of total power for this empire
    //     double portion = empires[e].imperialist.power / totalImperialistPower; 
        
    //     int numColoniesToAssign;
    //     if (e == empires.size() - 1) {
    //         // Last empire gets all remaining colonies 
    //         numColoniesToAssign = totalColonies - coloniesDistributed;
    //     } else {
    //         numColoniesToAssign = static_cast<int>(std::round(portion * totalColonies));
    //     }
        
    //     // Assign colonies from population to this empire
    //     for (int c = 0; c < numColoniesToAssign && (numImperialists + coloniesDistributed) < population.size(); c++) {
    //         empires[e].colonies.push_back(population[numImperialists + coloniesDistributed]);
    //         coloniesDistributed++;
    //     }
        
    //     std::cout << "  Empire " << e << " assigned " << empires[e].colonies.size() 
    //               << " colonies (portion=" << (portion * 100.0) << "%)" << std::endl;
    // }

    
    // Tính initial power
    for (auto& empire : empires) {
        empire.power = calculateEmpirePower(empire, 0.01);
    }
    
    std::cout << "Distributed " << (population.size() - numImperialists) 
              << " colonies among " << empires.size() << " empires" << std::endl;
}



void ICAHGS::assimilationAndRevolution() {

    std::vector<Individual> currentPopulation;
    for (auto& empire : empires) {
        for (size_t c = 0; c < empire.colonies.size(); c++) {
            // Crossover (Assimilation)
            Chromosome offspring = Chromosome::crossover(
                empire.imperialist.chrom,
                empire.colonies[c].chrom, rng);
            
            // Mutation (Revolution)
            // mutate(offspring, 0.05);
            offspring.mutate(0.05, rng);
            
            // Decode
            Solution offspringSol = decoder.decode(offspring, instance);
            
            // **TODO: KIỂM TRA DUPLICATE**
            // if (isDuplicate(offspringSol)) {
            //     // Nếu trùng, thử mutation mạnh hơn
            //     mutate(offspring, 0.15);  // Mutation rate cao hơn
            //     offspringSol = decoder.decode(offspring);
                
            //     // Check lại
            //     if (isDuplicate(offspringSol)) {
            //         continue;  // Skip nếu vẫn trùng
            //     }
            // }
            
            // Local search (CULPRIT)
            // offspringSol = localSearch.improve(offspringSol, 50);
            
            // Update archive
            updateParetoArchive(offspringSol);
            
            // Replace colony if better
            if (offspringSol.dominates(empire.colonies[c].solution) ||
                (offspringSol.systemCompletionTime < INF && 
                 empire.colonies[c].solution.systemCompletionTime >= INF)) {
                empire.colonies[c].chrom = offspring;
                empire.colonies[c].solution = offspringSol;
                
                // Revolution
                if (offspringSol.dominates(empire.imperialist.solution)) {
                    std::swap(empire.imperialist, empire.colonies[c]);
                }
            }

            currentPopulation.push_back(empire.colonies[c]);


        }
        currentPopulation.push_back(empire.imperialist);
        
        // Update empire power
        
        // empire.power = calculateEmpirePower(empire, 0.01);
    }

    std::vector<Solution*> solutions;
    for (auto& ind : currentPopulation) {
        solutions.push_back(&ind.solution);
    }
    ParetoRanking::nonDominatedSorting(solutions);
    ParetoRanking::calculateCrowdingDistance(solutions);
    
    // ========== BƯỚC 2: Group by Front ==========
    // Use map<int, Front> so each rank maps to a Front that contains a vector of countries
    std::map<int, Front> fronts;
    for (auto& ind : currentPopulation) {
        int rank = ind.solution.paretoRank;
        fronts[rank].countries.push_back(ind);
        fronts[rank].rankCompletionTime += ind.solution.systemCompletionTime;
        fronts[rank].rankWaitingTime += ind.solution.totalSampleWaitingTime;
    }
    
    

    // ========== TÍNH POWER CHO INDIVIDUAL ===========

    
    double fitness = 0;
     for (auto& [rank, front] : fronts) {
        if (front.countries.size() == 0) continue;
        for (auto& ind : front.countries) {
            // Skip invalid (infeasible) solutions
            if (ind.solution.systemCompletionTime == INF ||
                ind.solution.totalSampleWaitingTime == INF) {
                ind.power = -1;
                continue;
            }
            fitness = ind.solution.systemCompletionTime/front.rankCompletionTime + ind.solution.totalSampleWaitingTime/front.rankWaitingTime;
            fitness = fitness*(rank-1)*2;
            ind.power = std::min(1.0 / fitness, 1e6);
                
        }
     }
    
     for(auto& empire : empires){
        empire.power = calculateEmpirePower(empire, 0.01);
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
            if (pick <= currentPower && winnerIdx != weakestIdx) {
                winnerIdx = i;
                break;
            }
        }

        if (winnerIdx != -1) {
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

double ICAHGS::calculateEmpirePower(const Empire& empire, double xi) {
    // double impPower = 1.0 / (empire.imperialist.solution.paretoRank + 1.0);

    // if (!empire.colonies.empty()) {
    //     double avgColonyPower = 0;
    //     for (const auto& colony : empire.colonies) {
    //         avgColonyPower += 1.0 / (colony.solution.paretoRank + 1.0);
    //     }
    //     avgColonyPower /= empire.colonies.size();
        
    //     impPower = impPower + 0.1 * avgColonyPower;
    // }

    double impPower = empire.imperialist.power;

    if (!empire.colonies.empty()) {
        double avgColonyPower = 0;
        for (const auto& colony : empire.colonies) {
            avgColonyPower += colony.power;
        }
        avgColonyPower /= empire.colonies.size();
    
    impPower = impPower + xi * avgColonyPower;
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
