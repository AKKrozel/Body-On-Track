#pragma once

#include <SFML/Graphics.hpp>
#include <cmath>//
#include <vector>

float distanceBetweenPoints(sf::Vector2f, sf::Vector2f);
float angleBetweenPoints(sf::Vector2f, sf::Vector2f);
void drawThickLine(sf::RenderWindow&, sf::Vector2f, sf::Vector2f, float, sf::Color);
double setupWindowScaling(double, double, double, double, sf::RenderWindow&);
