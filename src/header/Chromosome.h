#ifndef CHROMOSOME_H
#define CHROMOSOME_H

#include <vector>
#include <map>
#include <random>

// #include "DataStructures.h"

struct Customer; // Forward declaration

class Chromosome {
private:
    // Phần 1: Gán phương tiện cho khách hàng.
    // assignment[i] là ID của phương tiện phục vụ khách hàng có id = i+1.
    std::vector<int> assignment;

    // Phần 2: Hoán vị thứ tự chung của các khách hàng.
    std::vector<int> permutation;

    // Lưu trữ ngữ cảnh của bài toán để các phương thức có thể sử dụng
    const std::vector<Customer>* customers_;
    int num_technicians_;
    int num_drones_;

public:
    const std::vector<int>& getAssignment() const { return assignment; }
    const std::vector<int>& getPermutation() const { return permutation; }
    Chromosome() {};
    // Hàm khởi tạo cho trước mã gen để test
    Chromosome(const std::vector<int>& assign, const std::vector<int>& perm,
               const std::vector<Customer>* customers,
               int numTechnicians, int numDrones)
        : assignment(assign), permutation(perm),
          customers_(customers),
          num_technicians_(numTechnicians),
          num_drones_(numDrones) {}
    Chromosome(const std::vector<Customer>& customers, int numTechnicians, int numDrones, std::mt19937& rng);
    static Chromosome crossover(const Chromosome& p1, const Chromosome& p2, std::mt19937& rng);
    void mutate(double mutationRate, std::mt19937& rng);
    void printGenotype() const;
};

#endif // CHROMOSOME_H