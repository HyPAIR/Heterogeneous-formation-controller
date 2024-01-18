/**
 * file generate_obs.h
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief generate obstacles
 * data 2023-11-22
 * 
 * @copyright Copyroght(c) 2023
*/
#pragma once
#include <iostream>
#include <vector>
#include "vec2d.h"
#include <cmath>

namespace formation_planner {
namespace math {
class GenerateObstacle{
// compute the rotated points
 public: 
    std::vector<Vec2d> rotatePoint(const Vec2d& center, double angle, bool inflat) {
        std::vector<Vec2d> rotatedPoints;
        double width, height;
        if (inflat) {
            width = 4.5;
            height = 3.5; 
        }
        else {
            // width = 10.0;
            // height = 27.0;
            // width = 20.0;
            // height = 22.0;
            width = 3;
            height = 2;
            // width = 40;
            // height = 40;
        }
        std::vector<Vec2d> relativeVertex;
        relativeVertex.push_back({-width / 2, -height / 2 });
        relativeVertex.push_back({ width / 2, -height / 2 });
        relativeVertex.push_back({width / 2, height / 2 });
        relativeVertex.push_back({ -width / 2, height / 2 });
        double s = sin(angle);
        double c = cos(angle);

        // 将点相对于中心点平移到原点
        for (int i = 0; i < relativeVertex.size(); i++) {
            double translatedX = relativeVertex[i].x();
            double translatedY = relativeVertex[i].y();

            // 进行旋转
            Vec2d rotatedpoint(translatedX * c - translatedY * s + center.x(), translatedX * s + translatedY * c + center.y());
            rotatedPoints.push_back(rotatedpoint);
        }
        return rotatedPoints;
    }
};
}
}