#include "ICAHGS.h"
#include <algorithm>
#include <ctime>
#include <chrono>  // For time-based stopping criterion
#include <iomanip> // For std::setprecision
#include <iostream>
#include <limits> // Thêm thư viện này để sử dụng giá trị lớn nhất/nhỏ nhất
#include <unordered_set>  // ← THÊM
#include <cstdint> 
#include <random>

ICAHGS::ICAHGS(const Instance& inst, int popSize, int numEmp) 
    : instance(inst), decoder(inst), localSearch(inst),
      populationSize(popSize), numImperialists(numEmp) {
    
    // Use fixed seed for reproducibility (can be overridden via setSeed())
    rng.seed(42);
    
    
}
ICAHGS::~ICAHGS() {
    
}

std::vector<Solution> ICAHGS::run(int maxIterations) {
    std::cout << "Initializing population..." << std::endl;
    initializePopulation();
    
    std::cout << "Starting ICAHGS optimization..." << std::endl;
    
    for (int iter = 0; iter < maxIterations; iter++) {
        std::cout << "Iteration " << (iter + 1) << "/" << maxIterations << std::endl;
        
        // Assimilation and Revolution
        assimilationAndRevolution();
        
        // cập nhật lại rank thuộc địa sau mỗi vòng lặp
        std::vector<Solution*> allSolutions;
        for (auto& empire : empires) {
            allSolutions.push_back(&empire.imperialist.solution);
            for (auto& colony : empire.colonies) {
                allSolutions.push_back(&colony.solution);
            }
        }

        // Imperialistic Competition
        imperialisticCompetition();
        
        if (empires.size() <= 1) { 
            std::cout << "Converged: only one or zero empire remains" << std::endl;
            break;
        }
    }
    
    std::cout << "Optimization complete. Final archive size: " 
              << paretoArchive.size() << std::endl;
    
    return paretoArchive;
}

// New method: Run with evaluation count limit instead of iterations
std::vector<Solution> ICAHGS::runWithEvaluationLimit(int maxEvaluations) {
    std::cout << "Initializing population..." << std::endl;
    SolutionEvaluator::resetCounter();  // Reset counter before starting
    
    initializePopulation();
    
    std::cout << "Starting ICAHGS optimization with evaluation limit: " 
              << maxEvaluations << std::endl;
    
    int iteration = 0;
    while (SolutionEvaluator::getEvaluationCount() < maxEvaluations) {
        iteration++;
        std::cout << "Iteration " << iteration 
                  << " | Evaluations: " << SolutionEvaluator::getEvaluationCount() 
                  << "/" << maxEvaluations << std::endl;
        
        // Assimilation and Revolution
        assimilationAndRevolution();
        
        // Update colony ranks after each iteration
        std::vector<Solution*> allSolutions;
        for (auto& empire : empires) {
            allSolutions.push_back(&empire.imperialist.solution);
            for (auto& colony : empire.colonies) {
                allSolutions.push_back(&colony.solution);
            }
        }

        // Imperialistic Competition
        imperialisticCompetition();
        
        if (empires.size() <= 1) { 
            std::cout << "Converged: only one or zero empire remains" << std::endl;
            break;
        }
    }
    
    std::cout << "Optimization complete. Total evaluations: " 
              << SolutionEvaluator::getEvaluationCount() << std::endl;
    std::cout << "Final archive size: " << paretoArchive.size() << std::endl;
    
    return paretoArchive;
}

// New method: Run with time limit (like benchmark paper)
std::vector<Solution> ICAHGS::runWithTimeLimit(double maxTimeSeconds) {
    std::cout << "Initializing population..." << std::endl;
    SolutionEvaluator::resetCounter();
    
    auto startTime = std::chrono::high_resolution_clock::now();
    initializePopulation();
    
    std::cout << "Starting ICAHGS optimization with time limit: " 
              << maxTimeSeconds << " seconds" << std::endl;
    
    int iteration = 0;
    while (true) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(currentTime - startTime).count();
        
        if (elapsed >= maxTimeSeconds) {
            std::cout << "Time limit reached: " << elapsed << "s" << std::endl;
            break;
        }
        
        iteration++;
        std::cout << "Iteration " << iteration 
                  << " | Time: " << std::fixed << std::setprecision(1) << elapsed << "s/" << maxTimeSeconds << "s"
                  << " | Evals: " << SolutionEvaluator::getEvaluationCount() << std::endl;
        
        // Assimilation and Revolution
        assimilationAndRevolution();
        
        // Update colony ranks
        std::vector<Solution*> allSolutions;
        for (auto& empire : empires) {
            allSolutions.push_back(&empire.imperialist.solution);
            for (auto& colony : empire.colonies) {
                allSolutions.push_back(&colony.solution);
            }
        }

        // Imperialistic Competition
        imperialisticCompetition();
        
        if (empires.size() <= 1) { 
            std::cout << "Converged: only one or zero empire remains" << std::endl;
            break;
        }
    }
    
    std::cout << "Optimization complete. Total iterations: " << iteration << std::endl;
    std::cout << "Total evaluations: " << SolutionEvaluator::getEvaluationCount() << std::endl;
    std::cout << "Final archive size: " << paretoArchive.size() << std::endl;
    
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
        
        // Decode using Split Algorithm (80%) or Greedy (20%) for diversity
        std::uniform_real_distribution<double> prob(0.0, 1.0);
        if (prob(rng) < 0.8) {
            // Use Split Algorithm (better quality)
            ind.solution = decoder.decode(chrom, instance);
        } else {
            // Use Greedy decoder (more diversity)
            ind.solution = decoder.decode(chrom, instance);
        }
        
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
    std::map<int, std::vector<Individual>> fronts;
    for (auto& ind : population) {
        int rank = ind.solution.paretoRank;
        fronts[rank].push_back(ind);
    }
    
    std::cout << "Fronts structure:" << std::endl;
    for (auto& [rank, front] : fronts) {
        std::cout << "  Front " << rank << ": " << front.size() << " solutions" << std::endl;
    }
    
    // ========== BƯỚC 3: Chọn Imperialists từ Fronts ==========
    std::vector<Individual> imperialists;
    
    for (auto& [rank, front] : fronts) {
        // Shuffle để random chọn
        // std::shuffle(front.begin(), front.end(), rng);

        // Sắp xếp các Individual trong front theo CrowdingDistance
        sort(front.begin(), front.end(), [](const Individual& a, const Individual& b) {
            return a.solution.crowdingDistance > b.solution.crowdingDistance;
        });
        std::cout << "Selecting from Front " << rank << "..." << std::endl;
        
        for (auto& ind : front) {
            imperialists.push_back(ind);
            std::cout << "  Selected imperialist #" << imperialists.size() 
                      << " (rank=" << ind.solution.paretoRank 
                      << ", CT=" << ind.solution.systemCompletionTime 
                      << ")" << std::endl;
            
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
    
    // Sort empires based on three-tier ranking
    sortEmpiresByTierRanking();
    
    std::cout << "Distributed " << (population.size() - numImperialists) 
              << " colonies among " << empires.size() << " empires" << std::endl;
}



// Trong ICAHGS.cpp

void ICAHGS::assimilationAndRevolution() {
    std::vector<Solution*> population;

    // Lấy iteration hiện tại (cần thêm biến này)
    static int iter = 0;
    iter++;

    for (auto& empire : empires) {
        for (size_t c = 0; c < empire.colonies.size(); c++) {
            // 1. Crossover
            Chromosome offspring = Chromosome::crossover(
                empire.imperialist.chrom,
                empire.colonies[c].chrom, rng);
            
            // 2. Mutation
            offspring.mutate(0.05, rng);
            
            // 3. Decode with Split Algorithm (prefer quality)
            std::uniform_real_distribution<double> prob(0.0, 1.0);
            Solution offspringSol;
            if (prob(rng) < 0.9) {
                // 
                offspringSol = decoder.decode(offspring, instance);
            } else {
                // 
                offspringSol = decoder.decode(offspring, instance);
            }
            
            // 4. QUALITY-BASED LOCAL SEARCH (Extended)
            // Strategy: Apply LS to FRONT 1 + FRONT 2 solutions
            // Balances computation speed (~60-70% saving) with solution diversity
            
            // OPTIMIZATION: Evaluate once for both quality check and LS input
            // This saves 1 evaluation per offspring (significant speedup!)
            SolutionEvaluator tempEvaluator(instance);
            tempEvaluator.evaluate(offspringSol);
            
            // Temporarily add to population for ranking
            std::vector<Solution*> tempPopulation;
            tempPopulation.push_back(&offspringSol);
            
            // Add all current empire solutions for comparison
            tempPopulation.push_back(&empire.imperialist.solution);
            for (auto& colony : empire.colonies) {
                tempPopulation.push_back(&colony.solution);
            }
            
            // Perform non-dominated sorting
            ParetoRanking::nonDominatedSorting(tempPopulation);
            
            // Check if offspring is in Front 1 or Front 2 (paretoRank <= 2)
            bool isInTopFronts = (offspringSol.paretoRank <= 2);
            
            // Use benchmark optimal setting: 20 iterations for all problem sizes
            // Paper evidence: 20-iteration config achieved highest average HV
            // and outperformed 10, 30, and 50-iteration configs
            int lsIterations = 20;
            
            // Apply LS if in Front 1 or Front 2 AND feasible
            // OPTIMIZATION: LS returns already-evaluated solution, no need to re-evaluate
            if (isInTopFronts && offspringSol.systemCompletionTime < INF) {
                offspringSol = localSearch.improve(offspringSol, lsIterations);
                // Note: localSearch.improve() returns evaluated solution
            }
            
            // 5. ĐỒNG BỘ NGƯỢC
            // Cập nhật lại Gen từ Lời giải đã được Local Search tối ưu
            updateChromosomeFromSolution(offspringSol, offspring);
            
            // Gán lại Gen đã update vào Solution để lưu trữ
            offspringSol.chrom = offspring;
            
            // 6. Update Archive
            updateParetoArchive(offspringSol);
            
            // 7. Thay thế Colony nếu tốt hơn (Logic giữ nguyên)
            if (offspringSol.dominates(empire.colonies[c].solution) ||
                (offspringSol.systemCompletionTime < INF && 
                 empire.colonies[c].solution.systemCompletionTime >= INF)) {
                
                empire.colonies[c].chrom = offspring; // Lưu Gen mới
                empire.colonies[c].solution = offspringSol; // Lưu Lời giải mới
                
                // Revolution (Thách đấu Imperialist)
                if (offspringSol.dominates(empire.imperialist.solution)) {
                    std::swap(empire.imperialist, empire.colonies[c]);
                }
            }
        }

        // Aggregate the population to do Non-Dominated Sorting
        population.push_back(&empire.imperialist.solution);
        for (auto& colony : empire.colonies) {
            population.push_back(&colony.solution);
        }
    }

    
    ParetoRanking::nonDominatedSorting(population);

    
    sortEmpiresByTierRanking();
}


void ICAHGS::imperialisticCompetition() {
    if (empires.size() <= 1) return;

    // 
    int weakestIdx = static_cast<int>(empires.size()) - 1;

    Empire& weakestEmpire = empires[weakestIdx];

    if (weakestEmpire.colonies.empty()) {
        // Empire yếu không còn colony → sụp đổ, chuyển imperialist sang empire mạnh nhất
        int strongestIdx = selectStrongestEmpire();
        if (strongestIdx != -1 && strongestIdx != weakestIdx) {
            empires[strongestIdx].colonies.push_back(std::move(weakestEmpire.imperialist));
        }
        empires.erase(empires.begin() + weakestIdx);

        std::cout << "  Empire collapsed. Remaining empires: "
                  << empires.size() << std::endl;
    } else {
        // Empire yếu vẫn còn colony → chọn colony yếu nhất theo Pareto + crowding
        int weakestColonyIdx = selectWeakestColonyByPareto(weakestEmpire);
        if (weakestColonyIdx < 0 ||
            weakestColonyIdx >= static_cast<int>(weakestEmpire.colonies.size())) {
            return; // safety
        }

        // Chọn empire thắng dựa trên power (ngoại trừ weakestIdx)
        double totalPower = 0.0;
        for (size_t i = 0; i < empires.size(); ++i) {
            if (i == static_cast<size_t>(weakestIdx)) continue;
            totalPower += empires[i].power;
        }
        if (totalPower <= 0.0) return;

        std::uniform_real_distribution<double> dist(0.0, totalPower);
        double pick = dist(rng);

        int winnerIdx = -1;
        double acc = 0.0;
        for (size_t i = 0; i < empires.size(); ++i) {
            if (i == static_cast<size_t>(weakestIdx)) continue;
            acc += empires[i].power;
            if (pick <= acc) {
                winnerIdx = static_cast<int>(i);
                break;
            }
        }
        if (winnerIdx == -1) {
            // Fallback: chọn empire đầu tiên khác weakestIdx
            winnerIdx = (weakestIdx == 0 ? 1 : 0);
        }

        // Chuyển colony từ weakest → winner
        empires[winnerIdx].colonies.push_back(
            std::move(weakestEmpire.colonies[weakestColonyIdx]));
        weakestEmpire.colonies.erase(
            weakestEmpire.colonies.begin() + weakestColonyIdx);
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
            in_offspring[gene] = true; 
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


void ICAHGS::updateChromosomeFromSolution(const Solution& sol, Chromosome& chrom) {
    int numCustomers = instance.getNumCustomers();
    std::vector<int> new_assignment(numCustomers);
    std::vector<int> new_permutation;
    new_permutation.reserve(numCustomers);

    // 1. Quét qua các tuyến xe tải (Trucks)
    // Truck ID trong assignment chạy từ 1 đến numTrucks
    for (size_t i = 0; i < sol.truckRoutes.size(); ++i) {
        int vehicleID = i + 1; 
        for (int custId : sol.truckRoutes[i].customers) {
            new_assignment[custId - 1] = vehicleID;
            new_permutation.push_back(custId);
        }
    }

    // 2. Quét qua các tuyến Drone
    // Drone ID trong assignment chạy từ (numTrucks + 1) trở đi
    for (size_t i = 0; i < sol.droneRoutes.size(); ++i) {
        int vehicleID = instance.numTrucks + i + 1;
        for (const auto& trip : sol.droneRoutes[i]) {
            for (int custId : trip.customers) {
                new_assignment[custId - 1] = vehicleID;
                new_permutation.push_back(custId);
            }
        }
    }

    // 3. Cập nhật lại Chromosome
    chrom.update(new_assignment, new_permutation);
}

void ICAHGS::sortEmpiresByTierRanking() {
    // Three-tier ranking system for empire sorting:
    // (1) Tier 1: Fewer infeasible solutions is better
    // (2) Tier 2: If tied on infeasibility, fewer dominated solutions is better
    // (3) Tier 3: If tied on domination, higher average crowding distance is better
    
    std::sort(empires.begin(), empires.end(), 
        [this](const Empire& a, const Empire& b) {
            int infeasibleA = countInfeasibleSolutions(a);
            int infeasibleB = countInfeasibleSolutions(b);
            
            // Tier 1: Compare infeasible solutions count
            if (infeasibleA != infeasibleB) {
                return infeasibleA < infeasibleB;  // Lower is better
            }
            
            // Tier 2: Compare dominated solutions count
            int dominatedA = countDominatedSolutions(a);
            int dominatedB = countDominatedSolutions(b);
            if (dominatedA != dominatedB) {
                return dominatedA < dominatedB;  // Lower is better
            }
            
            // Tier 3: Compare average crowding distance
            double crowdingA = calculateAverageCrowdingDistance(a);
            double crowdingB = calculateAverageCrowdingDistance(b);
            return crowdingA > crowdingB;  // Higher is better
        });
}

int ICAHGS::countInfeasibleSolutions(const Empire& empire) const {
    int count = 0;
    
    // Check imperialist
    if (empire.imperialist.solution.systemCompletionTime >= INF) {
        count++;
    }
    
    // Check all colonies
    for (const auto& colony : empire.colonies) {
        if (colony.solution.systemCompletionTime >= INF) {
            count++;
        }
    }
    
    return count;
}

int ICAHGS::countDominatedSolutions(const Empire& empire) const {
    int count = 0;
    
    // Check imperialist (rank 0 = non-dominated, rank > 0 = dominated)
    if (empire.imperialist.solution.paretoRank > 0) {
        count++;
    }
    
    // Check all colonies
    for (const auto& colony : empire.colonies) {
        if (colony.solution.paretoRank > 0) {
            count++;
        }
    }
    
    return count;
}

double ICAHGS::calculateAverageCrowdingDistance(const Empire& empire) const {
    if (empire.getTotalSize() == 0) {
        return 0.0;
    }
    
    double totalCrowdingDistance = 0.0;
    
    // Add imperialist's crowding distance
    totalCrowdingDistance += empire.imperialist.solution.crowdingDistance;
    
    // Add all colonies' crowding distances
    for (const auto& colony : empire.colonies) {
        totalCrowdingDistance += colony.solution.crowdingDistance;
    }
    
    return totalCrowdingDistance / empire.getTotalSize();
}

int ICAHGS::selectRandomColony(Empire& empire) {
    if (empire.colonies.empty()) return -1;
    
    std::uniform_int_distribution<int> dist(0, empire.colonies.size() - 1);
    return dist(rng);
}
// ========== Tìm worst front rank trong empire ==========
int ICAHGS::findWorstFrontRankInEmpire(const Empire& empire) const {
    int worstRank = 0;
    
    // Check imperialist
    worstRank = std::max(worstRank, empire.imperialist.solution.paretoRank);
    
    // Check all colonies
    for (const auto& colony : empire.colonies) {
        worstRank = std::max(worstRank, colony.solution.paretoRank);
    }
    
    return worstRank;
}

// ========== Chọn colony có CD nhỏ nhất trong front cụ thể ==========
int ICAHGS::selectColonyByLowestCrowdingInFront(
    const Empire& empire, 
    int targetFrontRank) {
    
    int selectedIdx = -1;
    double minCrowding = INF;
    
    // Duyệt qua tất cả colonies
    for (size_t i = 0; i < empire.colonies.size(); i++) {
        // Chỉ xét colonies trong target front
        if (empire.colonies[i].solution.paretoRank == targetFrontRank) {
            double cd = empire.colonies[i].solution.crowdingDistance;
            
            if (cd < minCrowding) {
                minCrowding = cd;
                selectedIdx = i;
            }
        }
    }
    
    // Nếu không tìm thấy colony trong target front,
    // có thể try random hoặc expand tìm front kế tiếp
    if (selectedIdx == -1 && !empire.colonies.empty()) {
    std::uniform_int_distribution<int> dist(
        0,
        static_cast<int>(empire.colonies.size()) - 1
    );
    selectedIdx = dist(rng);
}

    
    return selectedIdx;
}

// ========== Chọn colony yếu nhất trong empire ==========
int ICAHGS::selectWeakestColonyByPareto(const Empire& empire) {
    if (empire.colonies.empty()) {
        return -1;  // Không có colony
    }
    
    // Bước 1: Tìm worst front rank trong empire
    int worstFrontRank = findWorstFrontRankInEmpire(empire);
    
    // Bước 2: Tìm colony có CD nhỏ nhất trong worst front
    int weakestColonyIdx = selectColonyByLowestCrowdingInFront(empire, worstFrontRank);
    
    if (weakestColonyIdx == -1) {
        // Fallback: Chỉ return bất kỳ index nào
        weakestColonyIdx = 0;
    }
    
    return weakestColonyIdx;
}
int ICAHGS::selectStrongestEmpire() const {
    if (empires.empty()) return -1;
    int bestIdx = 0;
    double bestPower = empires[0].power;
    for (size_t i = 1; i < empires.size(); ++i) {
        if (empires[i].power > bestPower) {
            bestPower = empires[i].power;
            bestIdx = static_cast<int>(i);
        }
    }
    return bestIdx;
}


// int ICAHGS::selectWeakestEmpire() {
//     if (empires.empty()) return -1;
//     return empires.size() - 1;
// }
