// Created by weijian on 14/06/24.
#pragma once
#include <Eigen/Dense>
#include <vector>

#ifndef SRC_FORWARD_KINEMATICS_H
#define SRC_FORWARD_KINEMATICS_H

namespace forward_kinematics {

class ForwardKinematics {
public:
    explicit ForwardKinematics(int num_robot, double height, std::vector<std::vector<double>> vertice_initial_set, std::vector<std::vector<double>> robot_pos_set);
    bool SolveFk(
        std::vector<std::vector<double>> vertice_initial_set,
        std::vector<std::vector<double>> robot_pos_set,
        std::vector<std::vector<double>>& pos_3d, 
        std::vector<std::vector<double>>& pos_2d, std::vector<std::vector<int>>& taut_set,
        const double zr
        );

private:
    bool FormationFeasible(const std::vector<std::vector<double>>& initial_sets_2d, const std::vector<std::vector<double>>& robot_pos_set);
    void generateCombinations(const int N, const int k, const int start, std::vector<int>& current, std::vector<std::vector<int>>& result);    
    std::vector<std::vector<int>> NChooseK(const int num_robot, const int num_taut);\
    void createMatrices(const std::vector<Eigen::Vector4d>& aij, const std::vector<double>& bij,           
        int k,                                   
        Eigen::MatrixXd& A1,                      
        Eigen::VectorXd& b1,                      
        Eigen::MatrixXd& A2,       
        Eigen::VectorXd& b2                    
    );
    bool CheckRankEquality(const Eigen::MatrixXd& A1, const Eigen::VectorXd& b1);
    void findFullRowRankMatrix(const Eigen::MatrixXd& A1, const Eigen::VectorXd& b1, Eigen::MatrixXd& A11, Eigen::VectorXd& b11);
    void solveXAndLambda(
        const Eigen::Vector4d& c,       
        const Eigen::MatrixXd& A11,      
        const Eigen::VectorXd& b11,      
        Eigen::Vector4d& x,              
        Eigen::VectorXd& lambda          
    );
    bool CheckFeasibility(    
    const Eigen::Vector4d& x,        
    const Eigen::MatrixXd& A2,       
    const Eigen::VectorXd& b2,     
    double xi1, double yi1,          
    double xvi1, double yvi1,        
    double zr,                       
    double& zo                      
    );
    bool CheckForceClosure(const Eigen::Vector4d& x, const std::vector<Eigen::Vector2d>& polygon);
    bool isPointInConvexPolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon);    
    int num_robot_; 
    double height_;
    std::vector<std::vector<double>> vertice_initial_set_; 
    std::vector<std::vector<double>> robot_pos_set_;
};

}

#endif // SRC_FORWARD_KINEMATICS_H