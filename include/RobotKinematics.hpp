#ifndef ROBOT_KINEMATICS_HPP
#define ROBOT_KINEMATICS_HPP

#include <vector>
#include "MatrixMath.hpp"

using namespace std;

struct Vector2D {
    double x;
    double y;
};

struct RobotPose {
    vector<Vector2D> joint_positions;
    double end_effector_angle_deg;
};

class GrublerValidator {
public:
    static int calculatePlanarDOF(int num_links, int num_j1_joints, int num_j2_joints = 0);
};

class PlanarRoboticArm3DOF {
private:
    double L1, L2, L3;

public:
    PlanarRoboticArm3DOF(double l1, double l2, double l3);

    RobotPose computeFK(double theta1_deg, double theta2_deg, double theta3_deg);
    Matrix computeJacobian(double theta1_deg, double theta2_deg, double theta3_deg);
    vector<double> computeIK(double target_x, double target_y, double init_t1 = 10.0, double init_t2 = 10.0, double init_t3 = 10.0);
};

class PlanarRoboticArm2DOF {
private:
    double L1, L2;

public:
    PlanarRoboticArm2DOF(double l1, double l2);

    RobotPose computeFK(double t1_deg, double t2_deg);
    Matrix computeJacobian(double t1_deg, double t2_deg);
};

class PRRoboticArm {
private:
    double L2;

public:
    PRRoboticArm(double l2);

    RobotPose computeFK(double d1, double theta2_deg);
    Matrix computeJacobian(double theta2_deg);
};

#endif