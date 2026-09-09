#include "MatrixMath.hpp"
#include <iostream>
#include <cmath>

Matrix::Matrix(int r, int c) : rows(r), cols(c), data(r, vector<double>(c, 0.0)) {}

Matrix::Matrix(int r, int c, double initial) : rows(r), cols(c), data(r, vector<double>(c, initial)) {}

Matrix Matrix::identity(int size) {
    Matrix result(size, size);
    for (int i = 0; i < size; ++i) {
        result.data[i][i] = 1.0;
    }
    return result;
}

Matrix Matrix::transpose() const {
    Matrix result(cols, rows);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            result.data[j][i] = data[i][j];
        }
    }
    return result;
}

Matrix Matrix::multiply(const Matrix& other) const {
    if (cols != other.rows) {
        return Matrix(0, 0);
    }

    Matrix result(rows, other.cols, 0.0);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < other.cols; ++j) {
            for (int k = 0; k < cols; ++k) {
                result.data[i][j] += data[i][k] * other.data[k][j];
            }
        }
    }
    return result;
}

Matrix Matrix::inverse2x2() const {
    if (rows != 2 || cols != 2) return Matrix(0, 0);

    double det = (data[0][0] * data[1][1]) - (data[0][1] * data[1][0]);
    if (abs(det) < 1e-9) return Matrix(0, 0);

    double inv_det = 1.0 / det;
    Matrix result(2, 2);
    result.data[0][0] = data[1][1] * inv_det;
    result.data[0][1] = -data[0][1] * inv_det;
    result.data[1][0] = -data[1][0] * inv_det;
    result.data[1][1] = data[0][0] * inv_det;

    return result;
}

void Matrix::print() const {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            cout << data[i][j] << " ";
        }
        cout << endl;
    }
}