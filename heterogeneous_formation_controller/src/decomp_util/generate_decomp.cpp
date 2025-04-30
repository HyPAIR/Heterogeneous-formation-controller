// Created by weijian on 17/11/23.
#include <iostream>
#include <ros/ros.h>
#include "task_assignment/Hungarian.h"
#include <formation_generation/param.hpp>
#include <formation_generation/mission.hpp>
#include "heterogeneous_formation_controller/heterogeneous_formation_controller.h"
#include "heterogeneous_formation_controller/math/generate_obs.h"
#include "heterogeneous_formation_controller/visualization/plot.h"
#include "heterogeneous_formation_controller/yaml_all.h"
#include <decomp_util/decomp_util/seed_decomp.h>
#include <decomp_util/decomp_geometry/geometric_utils.h>
#include <fstream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include "heterogeneous_formation_controller/yaml_all.h"
#include <formation_generation/timer.hpp>

using namespace heterogeneous_formation_controller;

// 自定义函数，将 Eigen 矩阵转换为 YAML 格式
YAML::Node eigenToYAML(const Eigen::MatrixXd& matrix) {
    YAML::Node node;
    for (int i = 0; i < matrix.rows(); i++) {
        YAML::Node rowNode;
        for (int j = 0; j < matrix.cols(); j++) {
            rowNode.push_back(matrix(i, j));
        }
        node.push_back(rowNode);
    }
    return node;
}

int main(int argc, char* argv[])
{
    auto config_ = std::make_shared<PlannerConfig>();
    ros::init (argc, argv, "swarm_traj_planner_rbp");
    ros::NodeHandle nh( "~" );
    ros::Rate rate(10);
    int j = 0;
    SwarmPlanning::Mission mission;
	vector<vector<double>> diff_start, diff_goal;
	vector<vector<double>> car_start, car_goal;
    visualization::Init(nh, "map", "/hmfpc_test_vis");
	math::GenerateObstacle generateobs;
  	std::vector<math::Pose> obstacle;
	std::vector<math::Polygon2d> polys_orig, polys;
	std::vector<std::vector<math::Vec2d>> poly_vertices_set;
    vec_Vec2f obs;
    visualization::Init(nh, "map", "/hmfpc_test_vis");

	obstacle.push_back({-30, 30, 0.1});
	obstacle.push_back({-15, 30, 0.8});
	obstacle.push_back({0, 30, 0.3});
	obstacle.push_back({15, 30, 0.56});
	obstacle.push_back({30, 30, 0.26});
	obstacle.push_back({-30, -30, 0.75});
	obstacle.push_back({-15, -30, 0.83});
	obstacle.push_back({0, -30, 0.34});
	obstacle.push_back({15, -30, 0.2});
	obstacle.push_back({30, -30, 0.98});
	obstacle.push_back({-30, 15, 0.25});
	obstacle.push_back({-15, 15, 0.34});
	obstacle.push_back({0, 15, 0.63});
	obstacle.push_back({15, 15, 0.45});
	obstacle.push_back({30, 15, 0.72});
	obstacle.push_back({-30, -15, 0.23});
	obstacle.push_back({-15, -15, 0.62});
	obstacle.push_back({0, -15, 0.27});
	obstacle.push_back({15, -15, 0.86});
	obstacle.push_back({30, -15, 0.56});
	obstacle.push_back({22.5, 0, 0.25});
	obstacle.push_back({-22.5, 0, 0.89});

	for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
		math::Vec2d centre(obstacle[obs_ind].x(), obstacle[obs_ind].y());
		std::vector<math::Vec2d> poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), false);
        for (int i = 0; i < poly_vertices.size(); i++) {
            // Obstacles
            obs.push_back(Vec2f(poly_vertices[i].x(), poly_vertices[i].y()));
        }
		poly_vertices_set.push_back(poly_vertices);
		// polys_orig.push_back(math::Polygon2d(poly_vertices));
		polys.push_back(math::Polygon2d(poly_vertices));
	} 
    obs.push_back(Vec2f(60, 60));
    obs.push_back(Vec2f(-60, 60));
    obs.push_back(Vec2f(-60, -60));
    obs.push_back(Vec2f(60, -60));
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> pointsHyperplane;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, 2>> pointsVertices;
    int index = 0;
    Timer timer_step;
    timer_step.reset();
    for (double i = -50; i < 50; i += 0.2) {
        for (double j =  -50; j < 50; j += 0.2) {
            Vec2f pos(i, j);  
            SeedDecomp2D decomp(pos);
            decomp.set_obs(obs);
            // Initialize SeedDecomp2D
            decomp.set_local_bbox(Vec2f(2, 2));
            decomp.dilate(.1);
            const auto poly  = decomp.get_polyhedron();
            auto linear_cons = LinearConstraint<2>(pos, poly.vs_);
            Eigen::MatrixXd matrix_hp(linear_cons.A().rows(), 3);
            for (int k = 0; k < linear_cons.A().rows(); k++) {
                matrix_hp(k, 0) = linear_cons.A()(k, 0);
                matrix_hp(k, 1) = linear_cons.A()(k, 1);
                matrix_hp(k, 2) = linear_cons.b()(k, 0);
            }
            pointsHyperplane.push_back(matrix_hp);
            const auto vertices = cal_vertices(poly);
            Eigen::MatrixXd matrix_ver(vertices.size(), 2);
            for (int k = 0; k < vertices.size(); k++) {
                matrix_ver(k, 0) = vertices[k](0);
                matrix_ver(k, 1) = vertices[k](1);
            }
            pointsVertices.push_back(matrix_ver);
        }
    }
    timer_step.stop();
    std::cout << "Decomp generation runtime: " << timer_step.elapsedSeconds() << std::endl;
    YAML::Node yamlHyperplane, yamlVertices;
    for (int i = 0; i < pointsHyperplane.size(); i++) {
        // 将每个矩阵数据添加到 YAML 文档 
        yamlHyperplane["point_" + std::to_string(i)] = eigenToYAML(pointsHyperplane[i]);
    }
    // 将YAML文档写入文件
    std::ofstream outFileHyperplane("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/src/decomp_util/hyperplane.yaml");
    outFileHyperplane << yamlHyperplane;
    outFileHyperplane.close();

    std::cout << "Hyperplane data has been written to points.yaml." << std::endl;

    for (int i = 0; i < pointsVertices.size(); i++) {
        // 将每个矩阵数据添加到 YAML 文档 
        yamlVertices["point_" + std::to_string(i)] = eigenToYAML(pointsVertices[i]);
    }
    // 将YAML文档写入文件
    std::ofstream outFileVertice("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/src/decomp_util/vertices.yaml");
    outFileVertice << yamlVertices;
    outFileVertice.close();

    std::cout << "Vertics data has been written to points.yaml." << std::endl;


    while (ros::ok()) {
		for (int i = 0; i < polys.size(); i++) {
			visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Red, i, "Obstacle"+  std::to_string(i));
			visualization::Trigger();   
		}
        // Draw obstacles
        // int i = 0;
        // auto color_obs = visualization::Color::Red;
        // for(const auto& it: obs) {
        //     std::vector<double> xs, ys;
        //     xs.push_back(it(0));
        //     xs.push_back(it(0));
        //     ys.push_back(it(1));
        //     ys.push_back(it(1));
  		// 	visualization::PlotPoints(xs, ys, 0.1, color_obs, i++, "obstacle");  
  		//     visualization::Trigger();      
        // }

        // Draw ellispoid
        // const auto E = decomp.get_ellipsoid();
        // int num = 40; // number of points on trajectory to draw
        // std::vector<double> xs, ys;
        // for (const auto& it: E.sample(num)) {
        //     xs.push_back(it[0]);
        //     ys.push_back(it[1]);
        // }
        // auto color = visualization::Color::Magenta;
        // visualization::Plot(xs, ys, 0.2, color, 0, "Ellispoid");
  		// visualization::Trigger();
        
        // Draw polygon
        // std::vector<math::Vec2d> poly_vertices;
        // std::string ss("POLYGON((");
        // for (size_t i = 0; i < vertices.size(); i++) {
        //     math::Vec2d vertice(vertices[i](0), vertices[i](1));
        //     poly_vertices.push_back(vertice);
        // }
        // auto color_poly = visualization::Color::Green;
        // color_poly.set_alpha(0.1);
        // visualization::PlotPolygon(math::Polygon2d(poly_vertices), 0.1, color_poly, 0, "POLYGON");
        // visualization::Trigger();   

        // Write title at the lower right corner on canvas
        ros::spinOnce();
        rate.sleep();	
	}

	return 0;
}