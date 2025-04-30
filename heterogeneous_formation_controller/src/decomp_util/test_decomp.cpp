// Created by weijian on 17/11/23.
#include <ros/ros.h>
#include "heterogeneous_formation_controller/visualization/plot.h"
#include "heterogeneous_formation_controller/heterogeneous_formation_controller.h"

#include <decomp_util/decomp_util/seed_decomp.h>
#include <decomp_util/decomp_util/ellipsoid_decomp.h>
#include <decomp_util/decomp_geometry/geometric_utils.h>

#include <fstream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

using namespace heterogeneous_formation_controller;

int main(int argc, char **argv) {
    ros::init (argc, argv, "swarm_traj_planner_rbp");
    ros::NodeHandle nh( "~" );
    ros::Rate rate(10);
    int j = 0;
    // SwarmPlanning::Mission mission;
    // vector<vector<double>> diff_start, diff_goal;
    // vector<vector<double>> car_start, car_goal;
    visualization::Init(nh, "map", "/hmfpc_test_vis");
    // Obstacles
    vec_Vec2f obs;
    obs.push_back(Vec2f(3,3));
    obs.push_back(Vec2f(-3,3));
    obs.push_back(Vec2f(2,-3));
    obs.push_back(Vec2f(-4,-3));

    EllipsoidDecomp2D decomp_util;
    decomp_util.set_local_bbox(Eigen::Vector2d(2, 2));


    // int maxWidth = bbox_width / mapPtr_->resolution;
    int maxWidth;

    vec_E<Polyhedron2D> decompPolys;

    // generate corridor with idx and next_idx
    decomp_util.set_obs(obs);
    vec_Vec2f line = {{-1.933, -0.927}, {3.042, 2.994}};
    decomp_util.dilate(line);
    Polyhedron2D poly = decomp_util.get_polyhedrons()[0];
    decompPolys.push_back(poly);


    while (ros::ok()) {
        // Draw obstacles
        int i = 0;
        auto color_obs = visualization::Color::Red;
        for(const auto& it: obs) {
            std::vector<double> xs, ys;
            xs.push_back(it(0));
            xs.push_back(it(0));
            ys.push_back(it(1));
            ys.push_back(it(1));
  			visualization::PlotPoints(xs, ys, 0.1, color_obs, i++, "obstacle");  
  		    visualization::Trigger();      
        }

        // Draw ellispoid
        // const auto E = decomp_util.get_ellipsoid();
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
        const auto poly = decomp_util.get_polyhedrons()[0];
        const auto vertices = cal_vertices(poly);
        std::vector<math::Vec2d> poly_vertices;
        std::string ss("POLYGON((");
        for (size_t i = 0; i < vertices.size(); i++) {
            math::Vec2d vertice(vertices[i](0), vertices[i](1));
            poly_vertices.push_back(vertice);
        }
        auto color_poly = visualization::Color::Green;
        color_poly.set_alpha(0.1);
        visualization::PlotPolygon(math::Polygon2d(poly_vertices), 0.1, color_poly, 0, "POLYGON");
        visualization::Trigger();   

        // Write title at the lower right corner on canvas
        ros::spinOnce();
        rate.sleep();	
    }
    return 0;
}
