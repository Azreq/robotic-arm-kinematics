#include "Visualizer.hpp"
#include <cmath>

RobotVisualizer::RobotVisualizer(int width, int height) : screenWidth(width), screenHeight(height) {
    InitWindow(screenWidth, screenHeight, "2D Robotic Arm Kinematics Engine");
    SetTargetFPS(60);
    offset = { (float)screenWidth / 2.0f, (float)screenHeight / 1.5f };
}

RobotVisualizer::~RobotVisualizer() {
    CloseWindow();
}

bool RobotVisualizer::shouldClose() {
    return WindowShouldClose();
}

void RobotVisualizer::render(const RobotPose& pose, double target_x, double target_y) {
    BeginDrawing();
    ClearBackground(RAYWHITE);

    DrawLine(0, offset.y, screenWidth, offset.y, LIGHTGRAY);
    DrawLine(offset.x, 0, offset.x, screenHeight, LIGHTGRAY);

    float scale = 15.0f;

    Vector2 targetPos = { offset.x + (float)target_x * scale, offset.y - (float)target_y * scale };
    DrawCircleV(targetPos, 6.0f, RED);

    for (size_t i = 0; i < pose.joint_positions.size() - 1; ++i) {
        Vector2 start = { 
            offset.x + (float)pose.joint_positions[i].x * scale, 
            offset.y - (float)pose.joint_positions[i].y * scale 
        };
        Vector2 end = { 
            offset.x + (float)pose.joint_positions[i+1].x * scale, 
            offset.y - (float)pose.joint_positions[i+1].y * scale 
        };

        DrawLineEx(start, end, 8.0f, DARKBLUE);
        DrawCircleV(start, 8.0f, DARKGRAY);
    }

    Vector2 eePos = { 
        offset.x + (float)pose.joint_positions.back().x * scale, 
        offset.y - (float)pose.joint_positions.back().y * scale 
    };
    DrawCircleV(eePos, 6.0f, GREEN);

    DrawText("2D Robotic Arm Kinematics Simulator", 10, 10, 20, DARKGRAY);
    DrawText("Green: End-Effector | Red: Target", 10, 35, 15, GRAY);

    EndDrawing();
}