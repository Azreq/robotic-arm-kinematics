#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include "RobotKinematics.hpp"
#include "raylib.h"

class RobotVisualizer {
private:
    int screenWidth;
    int screenHeight;
    Vector2 offset;

public:
    RobotVisualizer(int width = 800, int height = 600);
    ~RobotVisualizer();

    void render(const RobotPose& pose, double target_x, double target_y);
    bool shouldClose();
};

#endif