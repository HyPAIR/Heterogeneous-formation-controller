// Created by weijian on 17/11/23.
#include <vector>
#include "heterogeneous_formation_controller/math/vec2d.h"
#include "heterogeneous_formation_controller/vehicle_model.h"
using namespace heterogeneous_formation_controller;
using namespace math;
VehicleModel vehicle;

class GenerateCar{
// 计算旋转后的点坐标
 public: 
    std::vector<Vec2d> rotatePoint(const Vec2d& center, double angle) {
        std::vector<Vec2d> rotatedPoints;
        double width = vehicle.half_length * 2;
        double height = vehicle.width;
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