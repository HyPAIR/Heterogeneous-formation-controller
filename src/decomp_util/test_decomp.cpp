// Created by weijian on 17/11/23.
#include <ros/ros.h>
#include "formation_planner/visualization/plot.h"
#include "formation_planner/formation_planner.h"

#include <decomp_util/decomp_util/seed_decomp.h>
#include <decomp_util/decomp_geometry/geometric_utils.h>

#include <fstream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

using namespace formation_planner;

int main(int argc, char **argv) {
    ros::init (argc, argv, "swarm_traj_planner_rbp");
    ros::NodeHandle nh( "~" );
    ros::Rate rate(10);
    int j = 0;
    // SwarmPlanning::Mission mission;
    // vector<vector<double>> diff_start, diff_goal;
    // vector<vector<double>> car_start, car_goal;
    visualization::Init(nh, "map", "/fp_test_vis");
    // Obstacles
    vec_Vec2f obs;
    obs.push_back(Vec2f(3,3));
    obs.push_back(Vec2f(-3,3));
    obs.push_back(Vec2f(2,-3));
    obs.push_back(Vec2f(-4,-3));

    // Seed
    const Vec2f pos(3, 3);

    // Initialize SeedDecomp2D
    SeedDecomp2D decomp(pos);
    decomp.set_obs(obs);
    decomp.set_local_bbox(Vec2f(2, 2));
    decomp.dilate(.1);


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
        const auto E = decomp.get_ellipsoid();
        int num = 40; // number of points on trajectory to draw
        std::vector<double> xs, ys;
        for (const auto& it: E.sample(num)) {
            xs.push_back(it[0]);
            ys.push_back(it[1]);
        }
        auto color = visualization::Color::Magenta;
        visualization::Plot(xs, ys, 0.2, color, 0, "Ellispoid");
  		visualization::Trigger();
        
        // Draw polygon
        const auto poly = decomp.get_polyhedron();
        auto linear_cons = LinearConstraint<2>(pos, poly.vs_);
        auto A_m = linear_cons.A();
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
