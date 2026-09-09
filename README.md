# 2D Robotic Arm Kinematics & Simulation Engine

A C++17 computational engineering framework designed to model, solve, and visualize planar robotic mechanisms. This repository demonstrates core principles of linear algebra, mechanical degree-of-freedom validation, trigonometric forward kinematics, Jacobian pseudo-inverse numerical inverse kinematics, and real-time GUI rendering.

---

## Key Features

* **Multi-Configuration Support:** Supports planar $2R$, $3R$ (Redundant), and $PR$ (Prismatic-Revolute Hybrid) mechanisms.
* **Custom Linear Algebra Library (`MatrixMath`):** Built-in matrix operations (multiplication, transpose, determinants, and $2 \times 2$ matrix inversion) with zero external math dependencies.
* **Grübler's Criterion Validator:** Automated Degree of Freedom (DOF) validation before running kinematics computations.
* **Jacobian Inverse Kinematics Engine:** Solves end-effector position targets using Moore-Penrose Pseudo-Inverse ($J^{\dagger} = J^T (J J^T)^{-1}$).
* **Real-time GUI Visualizer:** Interactive 2D joint-link rendering using Raylib.

---

## Supported Robot Configurations

### 1. 3-DOF Planar Arm ($3R$ Serial Manipulator)
* **Variables:** $\theta_1, \theta_2, \theta_3$
* **Jacobian Matrix:** $2 \times 3$ matrix for position control with redundancy.

### 2. 2-DOF Planar Arm ($2R$ Serial Manipulator)
* **Variables:** $\theta_1, \theta_2$
* **Jacobian Matrix:** Exact $2 \times 2$ square Jacobian solver.

### 3. PR Hybrid Arm (1 Prismatic Base + 1 Revolute Joint)
* **Variables:** $d_1$ (linear slider displacement), $\theta_2$ (joint angle)
* **Jacobian Matrix:**
  $$J = \begin{bmatrix} 1 & -L_2 \sin(\theta_2) \\ 0 & L_2 \cos(\theta_2) \end{bmatrix}$$

---

## Mathematical Foundation

### Degree of Freedom (Grübler's Criterion)
$$m = 3(N - 1) - 2j_1 - j_2$$

### Moore-Penrose Pseudo-Inverse (Numerical IK)
$$\Delta \theta = J^{\dagger} \cdot \Delta x \quad \text{where} \quad J^{\dagger} = J^T (J J^T)^{-1}$$

---

## Build and Run Instructions

### Dependencies
* C++17 Compiler (`g++` or `clang`)
* CMake ($\ge 3.16$)
* Raylib (`sudo apt install libraylib-dev`)

### Compilation Steps

```bash
# Clone repository
git clone [https://github.com/Azreq/robotic-arm-kinematics.git](https://github.com/Azreq/robotic-arm-kinematics.git)
cd robotic-arm-kinematics

# Create build directory
mkdir build && cd build

# Generate build files and compile
cmake ..
make

# Run executable
./robotic_arm
