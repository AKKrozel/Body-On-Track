//#pragma once

#include "Spring.h"

Spring::Spring(double xA, double yA, double restLen, double stiffness, double anchorSidePixels) :
    xA(xA), yA(yA), restLen(restLen), stiffness(stiffness) {
    anchorSideLen = anchorSidePixels * drawingScaleFactor;
}

void Spring::draw(sf::RenderWindow& window, Body& body) {
    if (stiffness != 0) {
        sf::RectangleShape anchor(sf::Vector2f(anchorSideLen, anchorSideLen));
        sf::Color Wheat = sf::Color(120, 111, 90);
        anchor.setFillColor(Wheat);
        anchor.setPosition(xA - anchorSideLen * 0.5f, yA - anchorSideLen * 0.5f);
        window.draw(anchor);

        double x = body.getX();
        double y = body.getY();
        double xDist = x - xA;
        double yDist = y - yA;
        float distance = std::sqrt(xDist * xDist + yDist * yDist);
        sf::VertexArray line(sf::Lines, 2);
        line[0].position = sf::Vector2f(xA, yA);
        line[1].position = sf::Vector2f(x, y);

        if (distance > restLen) {
            line[0].color = line[1].color = sf::Color::Blue;
        }
        else {
            line[0].color = line[1].color = sf::Color::Red;
        }
        window.draw(line);
    }
}

double Spring::getAnchorX() const { return xA; }
double Spring::getAnchorY() const { return yA; }
double Spring::getRestLength() const { return restLen; }
double Spring::getStiffness() const { return stiffness; }
double Spring::getAnchorSize() const { return anchorSideLen; }
