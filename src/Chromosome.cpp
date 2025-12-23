#include <iostream>
#include <vector>
#include <numeric>   // Cho std::iota
#include <algorithm> // Cho std::shuffle, std::find
#include <random>    // Cho mt19937, uniform_int_distribution
#include <map>
#include <stdexcept> // Cho std::runtime_error
#include "DataStructures.h"
#include "Chromosome.h"

// --- CONSTRUCTOR: Khởi tạo một cá thể ngẫu nhiên hợp lệ ---
Chromosome::Chromosome(const std::vector<Customer> &customers, int numTechnicians, int numDrones, std::mt19937 &rng)
    : customers_(&customers), num_technicians_(numTechnicians), num_drones_(numDrones)
{

    if (customers.empty())
        return;

    int num_customers = customers.size();
    assignment.resize(num_customers);
    permutation.resize(num_customers);

    // Tạo danh sách ID phương tiện hợp lệ
    std::vector<int> technician_ids(numTechnicians);
    std::iota(technician_ids.begin(), technician_ids.end(), 1); // KTV IDs: 1, 2, ..., numTechnicians

    std::vector<int> all_vehicle_ids(numTechnicians + numDrones);
    std::iota(all_vehicle_ids.begin(), all_vehicle_ids.end(), 1); // KTV IDs: 1, 2, ..., numTechnicians; Drone IDs: numTechnician + 1, ..., numTechnicians + numDrones

    if (technician_ids.empty() && std::any_of(customers.begin(), customers.end(), [](const Customer &c)
                                              { return c.isStaffOnly; }))
    {
        throw std::runtime_error("Error: There are staff-only customers but no technicians available.");
    }

    // Phần 1: Tạo danh sách gán hợp lệ với ADAPTIVE STRATEGY
    // HIGH-TRUCK cases (30+ trucks): Ưu tiên drones (70% drone, 30% truck)
    // LOW-TRUCK cases: Balanced (50% drone, 50% truck)
    bool highTruckCount = (numTechnicians >= 30);
    double droneProbability = highTruckCount ? 0.70 : 0.50;  // Adaptive!
    
    for (int i = 0; i < num_customers; ++i)
    {
        if ((*customers_)[i].isStaffOnly)
        {
            // Staff-only MUST use truck
            std::uniform_int_distribution<size_t> distrib(0, technician_ids.size() - 1);
            assignment[i] = technician_ids[distrib(rng)];
        }
        else
        {
            // ADAPTIVE: Bias towards drones for high-truck cases
            std::uniform_real_distribution<double> prob(0.0, 1.0);
            if (prob(rng) < droneProbability && numDrones > 0) {
                // Assign to drone (random drone ID)
                std::uniform_int_distribution<int> droneDist(numTechnicians + 1, numTechnicians + numDrones);
                assignment[i] = droneDist(rng);
            } else {
                // Assign to truck (random truck ID)
                std::uniform_int_distribution<int> truckDist(1, numTechnicians);
                assignment[i] = truckDist(rng);
            }
        }
    }

    // Phần 2: Tạo hoán vị ngẫu nhiên
    std::iota(permutation.begin(), permutation.end(), 1); // Điền vào [1, 2, ..., N]
    std::shuffle(permutation.begin(), permutation.end(), rng);
}

// --- TOÁN TỬ DI TRUYỀN ---

// Hàm Lai ghép (static method)
Chromosome Chromosome::crossover(const Chromosome &p1, const Chromosome &p2, std::mt19937 &rng)
{
    Chromosome child = p1; // Sao chép ngữ cảnh và cấu trúc từ cha 1

    // Lai ghép Phần 1 (Assignment) bằng Uniform Crossover
    std::uniform_int_distribution<int> distrib(0, 1);
    for (size_t i = 0; i < child.assignment.size(); ++i)
    {
        int gene_p1 = p1.assignment[i];
        int gene_p2 = p2.assignment[i];

        int inherited_gene = (distrib(rng) == 0) ? gene_p1 : gene_p2;
        // Nếu khách hàng này chỉ dành cho KTV...
        // if (child.customers_[i])
        if ((*child.customers_)[i].isStaffOnly)
        {
            // ...thì phải đảm bảo gen được thừa hưởng là một KTV.
            // ID của KTV luôn <= num_technicians_
            if (inherited_gene <= child.num_technicians_)
            {
                child.assignment[i] = inherited_gene;
            }
            else
            {
                cout<<"Found and fixed invalid gene during crossover: Customer "<<(*child.customers_)[i].id
                    <<" is staff-only but inherited vehicle ID "<<inherited_gene<<endl;
                // Lỗi đã xảy ra! Gen được đề xuất (từ p1 hoặc p2) không hợp lệ.
                // Chúng ta phải chọn gen hợp lệ còn lại.
                // Nếu cả hai đều không hợp lệ, đây là một lỗi nghiêm trọng, nhưng trường hợp này
                // chúng ta giả định ít nhất một trong hai cha mẹ có gen hợp lệ tại vị trí này.
                int alternative_gene = (inherited_gene == gene_p1) ? gene_p2 : gene_p1;
                if (alternative_gene <= child.num_technicians_)
                {
                    child.assignment[i] = alternative_gene;
                }
                // Nếu cả hai gen đều không hợp lệ, ta có thể giữ lại gen ban đầu từ p1
                // hoặc báo lỗi, hoặc sửa chữa bằng cách gán một KTV ngẫu nhiên.
                // Ở đây, chúng ta chỉ cần giữ lại gen từ p1 (đã có sẵn trong child).
            }
        }
        else // Nếu là khách hàng thường, không cần kiểm tra, gán trực tiếp
        {
            child.assignment[i] = inherited_gene;
        }
    }

    // Lai ghép Phần 2 (Permutation) bằng Order Crossover (OX1)
    int size = p1.permutation.size();
    if (size > 0)
    {
        std::uniform_int_distribution<int> range_distrib(0, size - 1);
        int start = range_distrib(rng);
        int end = range_distrib(rng);
        if (start > end)
            std::swap(start, end);

        std::vector<bool> in_child(size + 1, false);
        for (int i = start; i <= end; ++i)
        {
            child.permutation[i] = p1.permutation[i];
            in_child[p1.permutation[i]] = true;
        }

        int child_idx = (end + 1) % size;
        for (int i = 0; i < size; ++i)
        {
            int parent_idx = (end + 1 + i) % size;
            int gene = p2.permutation[parent_idx];
            if (!in_child[gene])
            {
                child.permutation[child_idx] = gene;
                child_idx = (child_idx + 1) % size;
            }
        }
    }
    return child;
}

// Hàm Đột biến
void Chromosome::mutate(double mutation_rate, std::mt19937 &rng)
{
    std::uniform_real_distribution<double> distrib(0.0, 1.0);

    // Đột biến Phần 1 (Assignment)
    if (distrib(rng) < mutation_rate)
    {
        int customer_idx = std::uniform_int_distribution<int>(0, customers_->size() - 1)(rng);

        std::vector<int> valid_vehicles;
        if ((*customers_)[customer_idx].isStaffOnly)
        {
            valid_vehicles.resize(num_technicians_);
            std::iota(valid_vehicles.begin(), valid_vehicles.end(), 1);
        }
        else
        {
            valid_vehicles.resize(num_technicians_ + num_drones_);
            std::iota(valid_vehicles.begin(), valid_vehicles.end(), 1);
        }

        if (valid_vehicles.size() > 1)
        {
            int current_vehicle = assignment[customer_idx];
            int new_vehicle;
            do
            {
                new_vehicle = valid_vehicles[std::uniform_int_distribution<int>(0, valid_vehicles.size() - 1)(rng)];
            } while (new_vehicle == current_vehicle);
            assignment[customer_idx] = new_vehicle;
        }
    }

    // Đột biến Phần 2 (Permutation) bằng Swap
    if (distrib(rng) < mutation_rate)
    {
        int size = permutation.size();
        std::uniform_int_distribution<int> idx_distrib(0, size - 1);
        int idx1 = idx_distrib(rng);
        int idx2 = idx_distrib(rng);
        std::swap(permutation[idx1], permutation[idx2]);
    }

    // if (assignment[5] > num_technicians_)
    // {
    //     std::cout << "Mutation check: Customer " << (*customers_)[5].id
    //               << " is staff-only and assigned to vehicle ID " << assignment[5] << "\n";
    // }
    // if (assignment[6] > num_technicians_)
    // {
    //     std::cout << "Mutation check: Customer " << (*customers_)[6].id
    //               << " is staff-only and assigned to vehicle ID " << assignment[6] << "\n";
    // }
    // if (assignment[9] > num_technicians_)
    // {
    //     std::cout << "Mutation check: Customer " << (*customers_)[9].id
    //               << " is staff-only and assigned to vehicle ID " << assignment[9] << "\n";
    // }
}

// --- CÁC HÀM TIỆN ÍCH ---

// In ra kiểu gen (dữ liệu thô)
void Chromosome::printGenotype() const
{
    std::cout << "  Assignment:  ";
    for (int vehicle_id : assignment)
    {
        std::string vehicle_str = std::to_string(vehicle_id) + "(" + ((vehicle_id <= num_technicians_) ? "T" + std::to_string(vehicle_id) : "D" + std::to_string(vehicle_id - num_technicians_)) + ")";
        std::cout << vehicle_str << " ";
    }
    std::cout << "\n  Permutation: ";
    for (int customer_id : permutation)
    {
        std::cout << customer_id << " ";
    }
    std::cout << "\n";
}