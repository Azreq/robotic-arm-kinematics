#ifndef MATRIX_MATH_HPP
#define MATRIX_MATH_HPP

#include <vector>

using namespace std;

class Matrix {
public:
    int rows, cols;
    vector<vector<double>> data;

    Matrix(int r, int c);
    Matrix(int r, int c, double initial);

    static Matrix identity(int size);
    Matrix transpose() const;
    Matrix multiply(const Matrix& other) const;
    Matrix inverse2x2() const;
    
    void print() const;
};

#endif