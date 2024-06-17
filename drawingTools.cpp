//#pragma once
#define _USE_MATH_DEFINES

#include "drawingTools.h"

float distanceBetweenPoints(sf::Vector2f start, sf::Vector2f end) {
    return std::sqrt((end.x - start.x) * (end.x - start.x) + (end.y - start.y) * (end.y - start.y));
}

float angleBetweenPoints(sf::Vector2f start, sf::Vector2f end) {
    return std::atan2(end.y - start.y, end.x - start.x);
}

void drawThickLine(sf::RenderWindow& window, sf::Vector2f start, sf::Vector2f end, float thickness, sf::Color color) {
    sf::RectangleShape line;
    line.setFillColor(color);
    line.setSize(sf::Vector2f(distanceBetweenPoints(start, end), thickness));
    line.setOrigin(0, thickness * 0.5);
    line.setPosition(start);
    line.setRotation(static_cast<double>(angleBetweenPoints(start, end)) * (180.0 / M_PI));

    window.draw(line);
}

// Scales a drawing window based on the minimum and maximum values to be desplayed on the window
// Returns the scaling factor used to convert from window pixels to window distance
double setupWindowScaling(double xMin, double  xMax, double yMin, double yMax, sf::RenderWindow& window) {
    double borderScaleFactor = 1.1;

    sf::Vector2u windowDimensions = window.getSize();
    double maxWindowDimension = std::max(windowDimensions.x, windowDimensions.y);

    // Set the drawing to fill the larger window dimension with 2/12 of the dimension leftover as border
    double width = (xMax - xMin);
    double height = (yMax - yMin);
    width *= borderScaleFactor;
    height *= borderScaleFactor;
    // Keep the drawing scaling 1:1
    if (width > height && windowDimensions.x < windowDimensions.y) {
        height = width;
        double yxRatio = windowDimensions.y / windowDimensions.x;
        height *= yxRatio;
    }
    else if (width < height && windowDimensions.x > windowDimensions.y) {
        width = height;
        double xyRatio = windowDimensions.x / windowDimensions.y;
        width *= xyRatio;
    }
    else if (width > height && windowDimensions.x > windowDimensions.y) {
        height = width;
        double xyRatio = windowDimensions.x / windowDimensions.y;
        width *= xyRatio;
    }
    else if (width < height && windowDimensions.x < windowDimensions.y) {
        width = height;
        double yxRatio = windowDimensions.y / windowDimensions.x;
        height *= yxRatio;
    }
    else {
        width, height = std::max(width, height);
        double xyRatio = windowDimensions.x / windowDimensions.y;
        width *= xyRatio;
        double yxRatio = windowDimensions.y / windowDimensions.x;
        height *= yxRatio;
    }


    double lowerX = 0.5 * ((xMin + xMax) - width);
    double upperY = 0.5 * ((yMin + yMax) + height);
    double negativeUpperY = -upperY;

    sf::View view(sf::FloatRect(lowerX, negativeUpperY, width, height));

    view.setSize(view.getSize().x, -view.getSize().y);
    view.setCenter(view.getCenter().x, -view.getCenter().y);
    window.setView(view);

    // return the scaling factor used to convert from pixels to lengths
    double drawingScaleFactor = width / maxWindowDimension;
    return drawingScaleFactor;
}