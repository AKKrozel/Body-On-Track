#pragma once

#include <vector>//
#include <cmath>//
#include <algorithm>
#include <limits>
#include <SFML/Graphics.hpp>//

#include "drawingTools.h"
#include "linearAlgebra.h"

extern double drawingScaleFactor;

class Track {
private:
    std::vector<double> T_values; // collection of curve parameterization values
    std::vector<double> S_values; // collection of arclength distance values
    std::vector<double> X_values; // collection of x-position values
    std::vector<double> Y_values; // collection of y-position values
    std::vector<double> radiusOfCurvature_values; // collection of radius of curvature values 
    std::vector<double> slope_Values; // collection of slope values
    double xMin, xMax, yMin, yMax, numSplineLines, numSplinePoints, largestSegmentLen;

public:
    Track(double Ti, double Tf, int numSplineLines);

    std::vector<double> curveFunction(double T);

    int getClosestEntryByS(double s) const;
    int getClosestEntryByX(double x) const;
    int getClosestEntryByDist(double x, double y) const;

    void draw(sf::RenderWindow& window) const;

    std::vector<double> getXValues() const;
    std::vector<double> getYValues() const;
    std::vector<double> getTValues() const;
    std::vector<double> getSValues() const;
    std::vector<double> getRCValues() const;
    std::vector<double> getKValues() const;
    double getXMin() const;
    double getXMax() const;
    double getYMin() const;
    double getYMax() const;
    double getLargestS() const;
};

