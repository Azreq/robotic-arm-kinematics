#include "RobotKinematics.hpp"
#include <cmath>
#include <iostream>

const double PI = 3.14159265358979323846;

double deg2rad(double deg) {
    return deg * (PI / 180.0);
}

double rad2deg(double rad) {
    return rad * (180.0 / PI);
}

int GrublerValidator::calculatePlanarDOF(int num_links, int num_j1_joints, int num_j2_joints) {
    return 3 * (num_links - 1) - (2 * num_j1_joints) - num_j2_joints;
}

PlanarRoboticArm3DOF::PlanarRoboticArm3DOF(double l1, double l2, double l3) : L1(l1), L2(l2), L3(l3) {}

RobotPose PlanarRoboticArm3DOF::computeFK(double theta1_deg, double theta2_deg, double theta3_deg) {
    double t1 = deg2rad(theta1_deg);
    double t2 = deg2rad(theta2_deg);
    double t3 = deg2rad(theta3_deg);

    RobotPose pose;
    pose.joint_positions.resize(4);

    pose.joint_positions[0] = {0.0, 0.0};
    pose.joint_positions[1].x = L1 * cos(t1);
    pose.joint_positions[1].y = L1 * sin(t1);
    pose.joint_positions[2].x = pose.joint_positions[1].x + L2 * cos(t1 + t2);
    pose.joint_positions[2].y = pose.joint_positions[1].y + L2 * sin(t1 + t2);
    pose.joint_positions[3].x = pose.joint_positions[2].x + L3 * cos(t1 + t2 + t3);
    pose.joint_positions[3].y = pose.joint_positions[2].y + L3 * sin(t1 + t2 + t3);

    pose.end_effector_angle_deg = theta1_deg + theta2_deg + theta3_deg;

    return pose;
}

Matrix PlanarRoboticArm3DOF::computeJacobian(double theta1_deg, double theta2_deg, double theta3_deg) {
    double t1 = deg2rad(theta1_deg);
    double t12 = deg2rad(theta1_deg + theta2_deg);
    double t123 = deg2rad(theta1_deg + theta2_deg + theta3_deg);

    Matrix J(2, 3);

    J.data[0][0] = -L1 * sin(t1) - L2 * sin(t12) - L3 * sin(t123);
    J.data[0][1] = -L2 * sin(t12) - L3 * sin(t123);
    J.data[0][2] = -L3 * sin(t123);

    J.data[1][0] = L1 * cos(t1) + L2 * cos(t12) + L3 * cos(t123);
    J.data[1][1] = L2 * cos(t12) + L3 * cos(t123);
    J.data[1][2] = L3 * cos(t123);

    return J;
}

vector<double> PlanarRoboticArm3DOF::computeIK(double target_x, double target_y, double init_t1, double init_t2, double init_t3) {
    double t1 = init_t1, t2 = init_t2, t3 = init_t3;
    int max_steps = 100;
    double tolerance = 1e-3;

    for (int step = 0; step < max_steps; ++step) {
        RobotPose current_pose = computeFK(t1, t2, t3);
        double current_x = current_pose.joint_positions[3].x;
        double current_y = current_pose.joint_positions[3].y;

        double error_x = target_x - current_x;
        double error_y = target_y - current_y;

        if (sqrt(error_x * error_x + error_y * error_y) < tolerance) break;

        Matrix J = computeJacobian(t1, t2, t3);
        Matrix J_T = J.transpose();
        Matrix JJT = J.multiply(J_T);
        Matrix JJT_inv = JJT.inverse2x2();
        Matrix J_pseudo = J_T.multiply(JJT_inv);

        Matrix delta_x(2, 1);
        delta_x.data[0][0] = error_x;
        delta_x.data[1][0] = error_y;

        Matrix delta_theta = J_pseudo.multiply(delta_x);

        t1 += rad2deg(delta_theta.data[0][0]);
        t2 += rad2deg(delta_theta.data[1][0]);
        t3 += rad2deg(delta_theta.data[2][0]);
    }

    return {t1, t2, t3};
}

PlanarRoboticArm2DOF::PlanarRoboticArm2DOF(double l1, double l2) : L1(l1), L2(l2) {}

RobotPose PlanarRoboticArm2DOF::computeFK(double t1_deg, double t2_deg) {
    double t1 = deg2rad(t1_deg);
    double t2 = deg2rad(t2_deg);

    RobotPose pose;
    pose.joint_positions.resize(3);
    pose.joint_positions[0] = {0.0, 0.0};
    pose.joint_positions[1] = {L1 * cos(t1), L1 * sin(t1)};
    pose.joint_positions[2] = {pose.joint_positions[1].x + L2 * cos(t1 + t2),
                               pose.joint_positions[1].y + L2 * sin(t1 + t2)};
    pose.end_effector_angle_deg = t1_deg + t2_deg;

    return pose;
}

Matrix PlanarRoboticArm2DOF::computeJacobian(double t1_deg, double t2_deg) {
    double t1 = deg2rad(t1_deg);
    double t12 = deg2rad(t1_deg + t2_deg);

    Matrix J(2, 2);
    J.data[0][0] = -L1 * sin(t1) - L2 * sin(t12);
    J.data[0][1] = -L2 * sin(t12);
    J.data[1][0] =  L1 * cos(t1) + L2 * cos(t12);
    J.data[1][1] =  L2 * cos(t12);

    return J;
}

PRRoboticArm::PRRoboticArm(double l2) : L2(l2) {}

RobotPose PRRoboticArm::computeFK(double d1, double theta2_deg) {
    double t2 = deg2rad(theta2_deg);

    RobotPose pose;
    pose.joint_positions.resize(3);
    pose.joint_positions[0] = {0.0, 0.0};
    pose.joint_positions[1] = {d1, 0.0};
    pose.joint_positions[2] = {d1 + L2 * cos(t2), L2 * sin(t2)};
    pose.end_effector_angle_deg = theta2_deg;

    return pose;
}

Matrix PRRoboticArm::computeJacobian(double theta2_deg) {
    double t2 = deg2rad(theta2_deg);

    Matrix J(2, 2);
    J.data[0][0] = 1.0;
    J.data[0][1] = -L2 * sin(t2);
    J.data[1][0] = 0.0;
    J.data[1][1] = L2 * cos(t2);

    return J;
}