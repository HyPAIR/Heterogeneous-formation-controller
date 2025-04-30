/**
 * file forward_kinematics.cpp
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief forward kinematics
 * data 2024-06-14
 * 
 * @copyright Copyroght(c) 2023
*/
#include "heterogeneous_formation_controller/forward_kinematics.h"
#include <iostream>
#include <set>
#include "traj_tracking/matplotlibcpp.h"

namespace plt = matplotlibcpp;
namespace forward_kinematics {
ForwardKinematics::ForwardKinematics(int num_robot, double height, std::vector<std::vector<double>> vertice_initial_set, std::vector<std::vector<double>> robot_pos_set)
 : num_robot_(num_robot), height_(height), vertice_initial_set_(vertice_initial_set), robot_pos_set_(robot_pos_set) {

}

bool ForwardKinematics::SolveFk(
    std::vector<std::vector<double>> vertice_initial_set,
    std::vector<std::vector<double>> robot_pos_set,
    std::vector<std::vector<double>>& pos_3d, 
    std::vector<std::vector<double>>& pos_2d, 
    std::vector<std::vector<int>>& taut_set,
    const double zr){
    if (FormationFeasible(vertice_initial_set, robot_pos_set)) {
        std::vector<Eigen::Vector2d> polygon;
        for (int i = 0; i < robot_pos_set.size(); i++ ) {
            polygon.push_back({robot_pos_set[i][0], robot_pos_set[i][1]});
        }
        for (int taut_num = 3; taut_num < robot_pos_set.size() + 1; taut_num++) {
            std::vector<std::vector<int>> taut_combinations = NChooseK(robot_pos_set.size(), taut_num);
            for (int i = 0; i < taut_combinations.size(); i++) {
                Eigen::MatrixXd A1;                      
                Eigen::VectorXd b1;                      
                Eigen::MatrixXd A2;                      
                Eigen::VectorXd b2; 
                Eigen::MatrixXd A11;
                Eigen::VectorXd b11;
                Eigen::Vector4d x;             
                Eigen::VectorXd lambda; 
                Eigen::Vector4d c;
                std::vector<Eigen::Vector4d> aij;  
                std::vector<double> bij;
                std::vector<int> ij_index_set;
                std::vector<int> robot_index_set;
                double zo;
                for (int j = 0; j < taut_combinations[i].size(); j++) {
                    ij_index_set.push_back(taut_combinations[i][j] - 1);
                }
                for (int j = 0; j < robot_pos_set.size(); j++) {
                    robot_index_set.push_back(j);
                }
                std::set<double> elementsSet(ij_index_set.begin(), ij_index_set.end());
                for (const double& element : robot_index_set) {
                    if (elementsSet.find(element) == elementsSet.end()) {
                        ij_index_set.push_back(element);
                    }
                }
                int i1 = ij_index_set[0];
                double xi1 = robot_pos_set[i1][0];
                double yi1 = robot_pos_set[i1][1];
                double xvi1 = vertice_initial_set_[i1][0];
                double yvi1 = vertice_initial_set_[i1][1];
                c = {-2 * robot_pos_set[i1][0], -2 * robot_pos_set[i1][1], 2 * vertice_initial_set_[i1][0], 2 * vertice_initial_set_[i1][1]};
                for (int j = 1; j < ij_index_set.size(); j++) {
                    int ij = ij_index_set[j];
                    aij.push_back({
                        robot_pos_set[ij][0] - robot_pos_set[i1][0],
                        robot_pos_set[ij][1] - robot_pos_set[i1][1],
                        vertice_initial_set_[i1][0] - vertice_initial_set_[ij][0],
                        vertice_initial_set_[i1][1] - vertice_initial_set_[ij][1]
                    });
                    bij.push_back(0.5 * (
                    xvi1 * xvi1 + yvi1 * yvi1 -
                    pow(vertice_initial_set_[ij][0], 2) - pow(vertice_initial_set_[ij][1], 2) -
                    xi1 * xi1 - yi1 * yi1 +
                    pow(robot_pos_set[ij][0], 2) + pow(robot_pos_set[ij][1], 2)));
                }
                ForwardKinematics::createMatrices(aij, bij, taut_num, A1, b1, A2, b2);
                if (CheckRankEquality(A1, b1)) {
                    findFullRowRankMatrix(A1, b1, A11, b11);
                    solveXAndLambda(c, A11, b11, x, lambda);
                    if (CheckFeasibility(x, A2, b2, xi1, yi1, xvi1, yvi1, zr, zo)) {
                        if (CheckForceClosure(x, polygon)) {
                            pos_3d.push_back({x[0], x[1], zo});
                            pos_2d.push_back({x[2], x[3]});
                            taut_set.push_back(ij_index_set);
                        }
                    }
                }
            }
        }
        return true;
    }
    return false;
}

bool ForwardKinematics::FormationFeasible(const std::vector<std::vector<double>>& initial_sets_2d, const std::vector<std::vector<double>>& robot_pos_set) {
    for (int i = 0; i < initial_sets_2d.size(); ++i) {
        for (int j = 0; j < initial_sets_2d.size(); ++j) {
            if (i != j) {
                double distance_r = hypot(robot_pos_set[i][0] - robot_pos_set[j][0], robot_pos_set[i][1] - robot_pos_set[j][1]);
                double distance_v = hypot(initial_sets_2d[i][0] - initial_sets_2d[j][0], initial_sets_2d[i][1] - initial_sets_2d[j][1]);
                if (distance_r >= distance_v) {
                    return false;
                }
            }
        }
    }
    return true;
}

void ForwardKinematics::generateCombinations(const int N, const int k, const int start, std::vector<int>& current, std::vector<std::vector<int>>& result) {
    if (current.size() == k) {
        result.push_back(current);
        return;
    }

    for (int i = start; i <= N; ++i) {
        current.push_back(i);
        generateCombinations(N, k, i + 1, current, result);
        current.pop_back(); // call back
    }
}

std::vector<std::vector<int>> ForwardKinematics::NChooseK(const int N, const int k) {
    std::vector<std::vector<int>> combinations;
    std::vector<int> current;
    generateCombinations(N, k, 1, current, combinations);
    return combinations;
}

void ForwardKinematics::createMatrices(
    const std::vector<Eigen::Vector4d>& aij,  
    const std::vector<double>& bij,          
    int k,                                 
    Eigen::MatrixXd& A1,                      
    Eigen::VectorXd& b1,                      
    Eigen::MatrixXd& A2,                      
    Eigen::VectorXd& b2                       
) {
    int N = aij.size() + 1; 

    // if (N < k + 1) {
    //     std::cout << "Error: The input size must be at least k + 1." << std::endl;
    //     return;
    // }

    // initialize A1 and b1
    A1.resize(k - 1, 4);
    b1.resize(k - 1);
    // from 2 to k，transfer from 1 to k-1
    for (int i = 1; i < k; ++i) { 
        A1.row(i - 1) = aij[i - 1].transpose(); 
        b1(i - 1) = bij[i - 1];
    }
    
    // initialize A2 and b2
    A2.resize(N - k, 4);
    b2.resize(N - k);
    // from k+1 to N，transfer from k to N-1
    for (int i = k; i < N; ++i) { 
        A2.row(i - k) = aij[i - 1].transpose();
        b2(i - k) = bij[i - 1];
    }
}

bool ForwardKinematics::CheckRankEquality(const Eigen::MatrixXd& A1, const Eigen::VectorXd& b1) {
    // A¯1 = [A1 b1]
    Eigen::MatrixXd A1_bar(A1.rows(), A1.cols() + 1);
    A1_bar << A1, b1;
    // calculate rank
    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp_A1(A1);
    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp_A1_bar(A1_bar);
    int rank_A1 = lu_decomp_A1.rank();
    int rank_A1_bar = lu_decomp_A1_bar.rank();
    return rank_A1 == rank_A1_bar;
}   

void ForwardKinematics::findFullRowRankMatrix(const Eigen::MatrixXd& A1, const Eigen::VectorXd& b1, Eigen::MatrixXd& A11, Eigen::VectorXd& b11) {
    int n = A1.rows();
    int m = A1.cols();
    Eigen::MatrixXd augmented = Eigen::MatrixXd::Zero(n, m + 1);
    augmented << A1, b1;
    
    // Gaussian elimination
    int rank = 0;
    for (int col = 0; col < m; ++col) {
        // Find the pivot row
        int pivot = rank;
        while (pivot < n && std::abs(augmented(pivot, col)) < 1e-10) {
            ++pivot;
        }

        if (pivot < n) {
            // Swap the current row with the pivot row
            augmented.row(rank).swap(augmented.row(pivot));

            // Normalize the pivot row
            double pivotValue = augmented(rank, col);
            augmented.row(rank) /= pivotValue;

            // Eliminate below and above
            for (int row = 0; row < n; ++row) {
                if (row != rank) {
                    double factor = augmented(row, col);
                    augmented.row(row) -= factor * augmented.row(rank);
                }
            }

            ++rank;
        }
    }

    // Extract A11 and b11
    A11 = augmented.topRows(rank).leftCols(m);
    b11 = augmented.topRows(rank).col(m);
    // int rank;
    // Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(A1);
    // rank = lu_decomp.rank();

    // // 提取前 rank 行组成的行满秩子矩阵 A11 和 b11
    // A11 = A1.topRows(rank);
    // b11 = b1.head(rank);
}

// solve x & λ
void ForwardKinematics::solveXAndLambda(
    const Eigen::Vector4d& c,        
    const Eigen::MatrixXd& A11,      
    const Eigen::VectorXd& b11,     
    Eigen::Vector4d& x,             
    Eigen::VectorXd& lambda         
) {
    Eigen::Matrix4d H;
    H.diagonal() << 2, 2, -2, -2;

    // compute H-1
    Eigen::Matrix4d H_inv = H.inverse();

    // compute B
    Eigen::MatrixXd A11H_inv = A11 * H_inv;
    Eigen::MatrixXd A11H_invA11T_inv = (A11H_inv * A11.transpose()).inverse();
    Eigen::MatrixXd B = H_inv - H_inv * A11.transpose() * A11H_invA11T_inv * A11H_inv;

    // compute C
    Eigen::MatrixXd C = A11H_invA11T_inv * A11H_inv;

    // compute D
    Eigen::MatrixXd D = -A11H_invA11T_inv;

    // solve x
    x = -B * c + C.transpose() * b11;

    // solve λ
    lambda = C * c - D * b11;
}

bool ForwardKinematics::CheckFeasibility(
    const Eigen::Vector4d& x,       
    const Eigen::MatrixXd& A2,       
    const Eigen::VectorXd& b2,       
    double xi1, double yi1,         
    double xvi1, double yvi1,        
    double zr,                      
    double& zo                       
) {
    // compute A2x
    if (A2.rows() != 0) {
        Eigen::VectorXd A2x = A2 * x;

        // check A2x > b2
        for (int i = 0; i < A2x.size(); ++i) {
            if (A2x(i) <= b2(i)) {
                std::cout << "Condition A2*x > b2 not met at index " << i << std::endl;
                return false;
            }
        }
    }

    double xo = x(0);
    double yo = x(1);
    double xvo = x(2);
    double yvo = x(3);

    // calculate f(x)
    double fx = (xi1 - xo) * (xi1 - xo) + (yi1 - yo) * (yi1 - yo)
                - (xvi1 - xvo) * (xvi1 - xvo) - (yvi1 - yvo) * (yvi1 - yvo);

    // check f(x) < 0
    if (fx >= 0) {
        std::cout << "Condition f(x) < 0 not met, f(x) = " << fx << std::endl;
        return false;
    }

    // compute zo
    zo = zr - std::sqrt(-fx);

    return true;
}

bool ForwardKinematics::isPointInConvexPolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon) {
    int n = polygon.size();
    bool inside = true;
    for (int i = 0; i < n; ++i) {
        const Eigen::Vector2d& p1 = polygon[i];
        const Eigen::Vector2d& p2 = polygon[(i + 1) % n];
        Eigen::Vector2d edge = p2 - p1;
        Eigen::Vector2d toPoint = point - p1;
        double crossProduct = edge.x() * toPoint.y() - edge.y() * toPoint.x();
        if (crossProduct < 0) {
            inside = false;
            break;
        }
    }
    return inside;
}

bool ForwardKinematics::CheckForceClosure(
    const Eigen::Vector4d& x,                         
    const std::vector<Eigen::Vector2d>& polygon       
) {
    Eigen::Vector2d ro = x.head<2>();
    return isPointInConvexPolygon(ro, polygon);
}

}