#pragma once

#include <SFML/Graphics.hpp>
#include <cmath>

#include "Body.h"

extern double drawingScaleFactor;

class Body; // forward declaration of Body class ; needed due to Body and Spring mutually referencing eachother

class Spring {
private:
    double xA;  // Anchor X position
    double yA;  // Anchor Y position
    double restLen;   // Spring rest length
    double stiffness;  // Spring stiffness
    double anchorSideLen;  // Size of the anchor rectangle

public:
    Spring(double xA, double yA, double restLen, double stiffness, double anchorSidePixels);

    void draw(sf::RenderWindow& window, Body& body);

    double getAnchorX() const;
    double getAnchorY() const;
    double getRestLength() const;
    double getStiffness() const;
    double getAnchorSize() const;
};

