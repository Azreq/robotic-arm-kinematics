#include <iostream>
#include "RobotKinematics.hpp"
#include "Visualizer.hpp"

using namespace std;

int main() {
    int dof = GrublerValidator::calculatePlanarDOF(4, 3);
    if (dof != 3) {
        cout << "invalid dof" << endl;
        return 1;
    }

    double l1, l2, l3;
    cout << "input L1 L2 L3: ";
    cin >> l1 >> l2 >> l3;

    PlanarRoboticArm3DOF robot(l1, l2, l3);

    int mode;
    cout << "mode (1: FK, 2: IK): ";
    cin >> mode;

    if (mode == 1) {
        double t1, t2, t3;
        cout << "input theta1 theta2 theta3: ";
        cin >> t1 >> t2 >> t3;

        RobotPose pose = robot.computeFK(t1, t2, t3);
        
        RobotVisualizer visualizer;
        while (!visualizer.shouldClose()) {
            visualizer.render(pose, pose.joint_positions[3].x, pose.joint_positions[3].y);
        }
    } else if (mode == 2) {
        double tx, ty;
        cout << "input target X Y: ";
        cin >> tx >> ty;

        vector<double> angles = robot.computeIK(tx, ty);
        RobotPose pose = robot.computeFK(angles[0], angles[1], angles[2]);

        RobotVisualizer visualizer;
        while (!visualizer.shouldClose()) {
            visualizer.render(pose, tx, ty);
        }
    }

    return 0;
}