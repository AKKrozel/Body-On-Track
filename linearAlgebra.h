#pragma once

#include <cmath>

class Vector2d {
public:
    double x, y;

    Vector2d();
    Vector2d(double x, double y);

    Vector2d operator+(const Vector2d& other) const;
    Vector2d operator-(const Vector2d& other) const;
    Vector2d operator*(double scalar) const;
    Vector2d operator/(double scalar) const;

    double magnitude() const;
    void normalize();
};

class Matrix2d {
public:
    double m[2][2];

    Matrix2d(double a11 = 0, double a12 = 0, double a21 = 0, double a22 = 0);
    Matrix2d(const double data[2][2]);

    Matrix2d operator*(const Matrix2d& other) const;
    Vector2d operator*(const Vector2d& vec) const;
};

Matrix2d createRotationMatrix(double angle);
double dotP(const Vector2d& a, const Vector2d& b);
Vector2d normalize(Vector2d vector);
