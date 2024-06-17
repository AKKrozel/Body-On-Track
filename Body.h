#pragma once
#define _USE_MATH_DEFINES

#include <SFML/Graphics.hpp>//
#include <vector>//
#include "Track.h"
#include "Spring.h"
#include "linearAlgebra.h"//
#include "drawingTools.h"//

extern double drawingScaleFactor;
extern double dt;
extern const double lowestCollisionPossible;
extern double collisionTol;
extern const int maxNumRollbacks;
extern const double g;

class Spring;

class Body {
private:
    double s; // arclength position
    double v; // speed tangent to track

    double x; // x-position
    double y; // y-position
    double vx; // x-speed
    double vy; // y-speed

    double yProp; // proposed next y-position
    double vyProp; // proposed next y-speed
    double xProp; // proposed next x-position
    double vxProp; // proposed next x-velocity

    double m; // mass
    double elasticity; // elasticity (0 to 1)
    double stickyness; // stickyness (0 to 1)
    double friction; // friction coefficient
    double drag; // drag coefficient
    double radius; // radius for drawing
    bool onTrack; // decides simulation behavior ; if onTrack==true then the body sticks to track rather than checking for track collisions

    Track track; // track interacting with
    std::vector<Spring> connectedSprings; // springs interacting with

public:
    Body(double s, double v, double x, double y, double vx, double vy, double m, double elasticity, double stickyness, double friction, double drag, double radiusPixels, bool onTrack, const Track& track, const std::vector<Spring>& springs);

    void draw(sf::RenderWindow& window);
    void update();

    double getS() const;
    double getV() const;
    double getX() const;
    double getY() const;
    double getVX() const;
    double getVY() const;
    double getM() const;
    double getElasticity() const;
    double getStickyness() const;
    double getFriction() const;
    double getDrag() const;
    double getRadius() const;
    bool getOnTrack() const;

private:
    void goToNext_S_V();
    void flyOffTrack(double k);
    bool checkIfShouldFlyOff(double k, double signedRC);
    void maybeFlyOff();

    double findSpringForceMag(double c, double xSP, double ySP, double R);
    void updateOnTrack();

    void setProposedNext_x_y_vx_vy(double dtInput);
    bool checkIfProposedBelowTrack(int closestXEntry);
    double findMinDistToTrackNode(double inputX, double inputY);
    void setPanicProposition(int closestXEntry);
    void setPreCollisionProposition(int& closestXEntry);
    void setCollisionProposition(int& closestXEntry);

    void acceptProposition();
    void updateOffTrack();

    double hFunction(double sInput, double vInput);
    double hPxFunction(double xInput, double yInput, double vxInput, double vYInput);
    double hPyFunction(double xInput, double yInput, double vxInput, double vYInput);
    double gFunction(double vInput);
};
