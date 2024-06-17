#include "linearAlgebra.h"

Vector2d::Vector2d() : x(0), y(0) {}

Vector2d::Vector2d(double x, double y) : x(x), y(y) {}

Vector2d Vector2d::operator+(const Vector2d& other) const {
    return Vector2d(x + other.x, y + other.y);
}

Vector2d Vector2d::operator-(const Vector2d& other) const {
    return Vector2d(x - other.x, y - other.y);
}

Vector2d Vector2d::operator*(double scalar) const {
    return Vector2d(x * scalar, y * scalar);
}

Vector2d Vector2d::operator/(double scalar) const {
    return Vector2d(x / scalar, y / scalar);
}

double Vector2d::magnitude() const {
    return std::sqrt(x * x + y * y);
}

void Vector2d::normalize() {
    double mag = magnitude();
    if (mag != 0.0) {
        x /= mag;
        y /= mag;
    }
}

Matrix2d::Matrix2d(double a11, double a12, double a21, double a22) {
    m[0][0] = a11; m[0][1] = a12;
    m[1][0] = a21; m[1][1] = a22;
}

Matrix2d::Matrix2d(const double data[2][2]) {
    m[0][0] = data[0][0]; m[0][1] = data[0][1];
    m[1][0] = data[1][0]; m[1][1] = data[1][1];
}

Matrix2d Matrix2d::operator*(const Matrix2d& other) const {
    return Matrix2d(
        m[0][0] * other.m[0][0] + m[0][1] * other.m[1][0], m[0][0] * other.m[0][1] + m[0][1] * other.m[1][1],
        m[1][0] * other.m[0][0] + m[1][1] * other.m[1][0], m[1][0] * other.m[0][1] + m[1][1] * other.m[1][1]
    );
}

Vector2d Matrix2d::operator*(const Vector2d& vec) const {
    return Vector2d(
        m[0][0] * vec.x + m[0][1] * vec.y,
        m[1][0] * vec.x + m[1][1] * vec.y
    );
}

Matrix2d createRotationMatrix(double angle) {
    double cosTheta = cos(angle);
    double sinTheta = sin(angle);
    return Matrix2d(
        cosTheta, -sinTheta,
        sinTheta, cosTheta
    );
}

double dotP(const Vector2d& a, const Vector2d& b) {
    return a.x * b.x + a.y * b.y;
}

Vector2d normalize(Vector2d vector) {
    double mag = vector.magnitude();
    if (mag == 0.0) {
        return Vector2d();
    }
    return Vector2d(vector.x / mag, vector.y / mag);
}
