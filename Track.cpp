//#pragma once

#include "Track.h"

Track::Track(double Ti, double Tf, int numSplineLines) : numSplineLines(numSplineLines) {
    numSplinePoints = numSplineLines + 1;
    double dT = (Tf - Ti) / static_cast<double>(numSplineLines);
    double T = Ti;
    largestSegmentLen = 0.0;
    for (int i = 0; i <= numSplineLines; i++) {
        T_values.push_back(T);
        std::vector<double> pointOfInterest = curveFunction(T);
        T += dT;

        X_values.push_back(pointOfInterest[0]);
        Y_values.push_back(pointOfInterest[1]);
        if (i == 0) {
            S_values.push_back(0);
        }
        else {
            double dist = std::sqrt((X_values[i] - X_values[i - 1]) * (X_values[i] - X_values[i - 1]) +
                (Y_values[i] - Y_values[i - 1]) * (Y_values[i] - Y_values[i - 1]));
            double distFromStart = S_values.back() + dist;
            if (dist > largestSegmentLen) {
                largestSegmentLen = dist;
            }
            S_values.push_back(distFromStart);
        }
    }

    for (int i = 0; i <= numSplineLines; i++) {
        if (i == 0 || i == numSplineLines || i == 1 || i == numSplineLines - 1) {
            radiusOfCurvature_values.push_back(0);
            slope_Values.push_back(0);
        }
        else {
            double kNextLocal = (Y_values[i + 1] - Y_values[i]) / (X_values[i + 1] - X_values[i]);
            if (X_values[i + 1] < X_values[i])
                kNextLocal *= -1;
            double kPrevLocal = (Y_values[i] - Y_values[i - 1]) / (X_values[i] - X_values[i - 1]);
            if (X_values[i] < X_values[i - 1])
                kPrevLocal *= -1;
            double k = (kNextLocal + kPrevLocal) * 0.5;
            slope_Values.push_back(k);
        }
    }

    for (int i = 0; i <= numSplineLines; i++) {
        if (i == 0 || i == numSplineLines || i == 1 || i == numSplineLines - 1) {
            radiusOfCurvature_values.push_back(0);
            slope_Values.push_back(0);
        }
        else {
            double kNext = slope_Values[i + 1];
            double kPrev = slope_Values[i - 1];
            double dS = ((S_values[i + 1] - S_values[i]) + (S_values[i] - S_values[i - 1])) * 0.5;
            double K = (std::atan(kNext) - std::atan(kPrev)) / (2 * dS);
            double RC = 1 / K;
            radiusOfCurvature_values.push_back(RC);
        }
    }

    xMin = *std::min_element(X_values.begin(), X_values.end());
    xMax = *std::max_element(X_values.begin(), X_values.end());
    yMin = *std::min_element(Y_values.begin(), Y_values.end());
    yMax = *std::max_element(Y_values.begin(), Y_values.end());
}

std::vector<double> Track::curveFunction(double T) {
    double X = T;
    double Y = T * T * (-7.0 + 2.0 * T * T) / 6.0;
    return { X, Y };
}

int Track::getClosestEntryByS(double s) const {
    int closestEntry = 0;
    double minDifference = std::numeric_limits<double>::max();
    for (int i = 1; i < numSplinePoints; i++) {
        double Difference = std::abs(s - S_values[i]);
        if (Difference < minDifference) {
            minDifference = Difference;
            closestEntry = i;
        }
    }
    return closestEntry;
}

int Track::getClosestEntryByX(double x) const {
    int closestEntry = 0;
    double minDifference = std::numeric_limits<double>::max();
    for (int i = 1; i < numSplinePoints; i++) {
        double Difference = std::abs(x - X_values[i]);
        if (Difference < minDifference) {
            minDifference = Difference;
            closestEntry = i;
        }
    }
    return closestEntry;
}

int Track::getClosestEntryByDist(double x, double y) const {
    int closestEntry = 0;
    double minDifference = std::numeric_limits<double>::max();
    for (int i = 1; i < numSplinePoints; i++) {
        double x_diff = (x - X_values[i]);
        double y_diff = (y - Y_values[i]);
        double Difference = std::sqrt(x_diff * x_diff + y_diff * y_diff);
        if (Difference < minDifference) {
            minDifference = Difference;
            closestEntry = i;
        }
    }
    return closestEntry;
}

void Track::draw(sf::RenderWindow& window) const {
    sf::Color Orange = sf::Color(200, 100, 0);
    float thickness = 3.0f * drawingScaleFactor;
    for (int i = 1; i < numSplinePoints; i++) {
        sf::Vector2f start(X_values[i - 1], Y_values[i - 1]);
        sf::Vector2f end(X_values[i], Y_values[i]);
        drawThickLine(window, start, end, thickness, Orange);
    }
}

std::vector<double> Track::getXValues() const { return X_values; }
std::vector<double> Track::getYValues() const { return Y_values; }
std::vector<double> Track::getTValues() const { return T_values; }
std::vector<double> Track::getSValues() const { return S_values; }
std::vector<double> Track::getRCValues() const { return radiusOfCurvature_values; }
std::vector<double> Track::getKValues() const { return slope_Values; }
double Track::getXMin() const { return xMin; }
double Track::getXMax() const { return xMax; }
double Track::getYMin() const { return yMin; }
double Track::getYMax() const { return yMax; }
double Track::getLargestS() const { return largestSegmentLen; }

