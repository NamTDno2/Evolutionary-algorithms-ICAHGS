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

    for (auto& empire : empires) {
        for (size_t c = 0; c < empire.colonies.size(); c++) {
            // 1. Crossover
            Chromosome offspring = Chromosome::crossover(
                empire.imperialist.chrom,
                empire.colonies[c].chrom, rng);
            
            // 2. Mutation
            offspring.mutate(0.05, rng);
            
            // 3. Decode ban đầu
            Solution offspringSol = decoder.decode(offspring, instance);
            
            // 4. Local Search
            // Chạy 50 iterations để tinh chỉnh
            offspringSol = localSearch.improve(offspringSol, 50);
            
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

    //  -----------------------------------------------------------------------------------------
    //  DEBUG: log rank and target variables before sorting
    std::cout<<"=============================================";
    std::cout<<"\nBefore Sorting:";
    for (auto& empire : empires) {
        cout<<"\nImperialist "<<"\t";
        // empire.imperialist.solution.chrom.printGenotype();
        cout<<"Rank: "<<empire.imperialist.solution.paretoRank<<", SYST = "<<empire.imperialist.solution.systemCompletionTime<<", WAIT = "<<empire.imperialist.solution.totalSampleWaitingTime<<endl;
        for (size_t c = 0; c < empire.colonies.size(); c++) {
            cout<<"Colony "<<c<<"\t";
            // empire.colonies[c].solution.chrom.printGenotype();
            cout<<"Rank: "<<empire.colonies[c].solution.paretoRank<<", SYST = "<<empire.colonies[c].solution.systemCompletionTime<<", WAIT = "<<empire.colonies[c].solution.totalSampleWaitingTime<<endl;
        }
    }
    //  ------------------------------------------------------------------------------------------

    // Sort the newly modified population
    ParetoRanking::nonDominatedSorting(population);

    //  -----------------------------------------------------------------------------------------
    //  DEBUG: log rank and target variables after sorting
    std::cout<<"---------------------------------------------";
    std::cout<<"\nAfter Sorting:";
    for (auto& empire : empires) {
        cout<<"\n\nImperialist "<<"\t";
        // empire.imperialist.solution.chrom.printGenotype();
        cout<<"Rank: "<<empire.imperialist.solution.paretoRank<<", SYST = "<<empire.imperialist.solution.systemCompletionTime<<", WAIT = "<<empire.imperialist.solution.totalSampleWaitingTime<<endl;
        for (size_t c = 0; c < empire.colonies.size(); c++) {
            cout<<"Colony "<<c<<"\t";
            // empire.colonies[c].solution.chrom.printGenotype();
            cout<<"Rank: "<<empire.colonies[c].solution.paretoRank<<", SYST = "<<empire.colonies[c].solution.systemCompletionTime<<", WAIT = "<<empire.colonies[c].solution.totalSampleWaitingTime<<endl;
        }
    }
    cout<<endl;
    //  -----------------------------------------------------------------------------------------

    // Sort empires after each assimilation round
    sortEmpiresByTierRanking();
}


void ICAHGS::imperialisticCompetition() {
    if (empires.size() <= 1) return;
    
    
    
    if (empires[empires.size() - 1].colonies.empty()) {
        // Empire has no colonies, collapse it
        // Move imperialist to strongest empire (index 0) as colony
        empires[0].colonies.push_back(std::move(empires[empires.size() - 1].imperialist));
        empires.erase(empires.begin() + empires.size() - 1);
        
        std::cout << "  Empire collapsed. Remaining empires: " 
                  << empires.size() << std::endl;
    } else {
        // Transfer weakest colony to the winner of the competition
        int colonyIdx = selectRandomColony(empires[empires.size() - 1]);
        

        std::uniform_int_distribution<int> dist(0, empires.size() - 1 - 1);
        int winnerIdx = dist(rng);
        
        empires[winnerIdx].colonies.push_back(
            std::move(empires[empires.size() - 1].colonies[colonyIdx]));
        empires[empires.size() - 1].colonies.erase(
            empires[empires.size() - 1].colonies.begin() + colonyIdx);
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

// int ICAHGS::selectWeakestEmpire() {
//     if (empires.empty()) return -1;
//     return empires.size() - 1;
// }
