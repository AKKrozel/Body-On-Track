//#pragma once
#define _USE_MATH_DEFINES

#include "Body.h"

Body::Body(double s, double v, double x, double y, double vx, double vy, double m, double elasticity, double stickyness, double friction, double drag, double radiusPixels, bool onTrack, const Track& track, const std::vector<Spring>& springs) :
    s(s), v(v), x(x), y(y), vx(vx), vy(vy), m(m), elasticity(elasticity), stickyness(stickyness), friction(friction), drag(drag), onTrack(onTrack), track(track), connectedSprings(springs) {

    radius = radiusPixels * drawingScaleFactor;

    yProp = y;
    xProp = x;
    vyProp = vy;
    vxProp = vx;
}

void Body::draw(sf::RenderWindow& window) {
    sf::Color color = sf::Color::Green;
    if (onTrack) {
        color = sf::Color::Green;
    }
    sf::CircleShape circle(radius);
    circle.setFillColor(color);
    circle.setPosition(x - radius, y - radius);
    window.draw(circle);
}

void Body::update() {
    if (onTrack) {
        updateOnTrack();
    }
    else {
        updateOffTrack();
    }
}

void Body::goToNext_S_V() {
    double sNext = s;
    double vNext = v;
    double k1 = gFunction(vNext);
    double m1 = hFunction(sNext, vNext);
    sNext = s + (dt * 0.5) * k1;
    vNext = v + (dt * 0.5) * m1;
    double k2 = gFunction(vNext);
    double m2 = hFunction(sNext, vNext);
    sNext = s + (dt * 0.5) * k2;
    vNext = v + (dt * 0.5) * m2;
    double k3 = gFunction(vNext);
    double m3 = hFunction(sNext, vNext);
    sNext = s + (dt) * k3;
    vNext = v + (dt) * m3;
    double k4 = gFunction(vNext);
    double m4 = hFunction(sNext, vNext);

    s = s + dt * ((k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0);
    v = v + dt * ((m1 + 2.0 * m2 + 2.0 * m3 + m4) / 6.0);
}

void Body::flyOffTrack(double k) {
    onTrack = false;
    vx = v * std::cos(std::atan(k));
    vy = v * std::sin(std::atan(k));
}

bool Body::checkIfShouldFlyOff(double k, double signedRC) {
    Vector2d FgVector(0, -m * g);
    Vector2d sumForces = FgVector;
    for (Spring spring : connectedSprings) {
        double xSP = spring.getAnchorX() - x;
        double ySP = spring.getAnchorY() - y;
        double c = spring.getStiffness();
        double R = spring.getRestLength();
        Vector2d DVector(xSP, ySP);
        Vector2d DNormalizedVector = normalize(DVector);
        Vector2d FspVector = DNormalizedVector * findSpringForceMag(c, xSP, ySP, R);

        sumForces = sumForces + FspVector;
    }
    Vector2d aVector = sumForces / m;
    Vector2d tanVector(1, k);
    Vector2d tanNormalizeVector = normalize(tanVector);

    double RC = std::abs(signedRC);
    double aCRequiredMag = v * v / RC;

    bool shouldFlyOff = false;
    bool trackCurvedUp = signedRC >= 0;
    if (trackCurvedUp) {
        double thetaROT = M_PI * 0.5;
        Matrix2d rotationMatrix = createRotationMatrix(thetaROT);
        Vector2d aCNormalizeVector = rotationMatrix * tanNormalizeVector;
        shouldFlyOff = dotP(aVector, aCNormalizeVector) > aCRequiredMag;
    }
    else {
        double thetaROT = -M_PI * 0.5;
        Matrix2d rotationMatrix = createRotationMatrix(thetaROT);
        Vector2d aCNormalizeVector = rotationMatrix * tanNormalizeVector;
        shouldFlyOff = dotP(aVector, aCNormalizeVector) < aCRequiredMag;
    }
    return shouldFlyOff;
}

void Body::maybeFlyOff() {
    int closestEntry = track.getClosestEntryByS(s);
    x = track.getXValues()[closestEntry];
    y = track.getYValues()[closestEntry];
    double k = track.getKValues()[closestEntry];
    double signedRC = track.getRCValues()[closestEntry];
    bool shouldFlyOff = checkIfShouldFlyOff(k, signedRC);
    if (shouldFlyOff) {
        flyOffTrack(k);
    }
}

double Body::findSpringForceMag(double c, double xSP, double ySP, double R) {
    return c * (std::sqrt(xSP * xSP + ySP * ySP) - R);
}

void Body::updateOnTrack() {
    goToNext_S_V();
    maybeFlyOff();
}

void Body::setProposedNext_x_y_vx_vy(double dtInput) {
    double xNext = x;
    double yNext = y;
    double vxNext = vx;
    double vyNext = vy;
    double k1 = gFunction(vyNext);
    double m1 = hPyFunction(xNext, yNext, vxNext, vyNext);
    double p1 = gFunction(vxNext);
    double n1 = hPxFunction(xNext, yNext, vxNext, vyNext);
    xNext = x + (dtInput * 0.5) * k1;
    yNext = y + (dtInput * 0.5) * p1;
    vxNext = vx + (dtInput * 0.5) * n1;
    vyNext = vy + (dtInput * 0.5) * m1;
    double k2 = gFunction(vyNext);
    double m2 = hPyFunction(xNext, yNext, vxNext, vyNext);
    double p2 = gFunction(vxNext);
    double n2 = hPxFunction(xNext, yNext, vxNext, vyNext);
    xNext = x + (dtInput * 0.5) * k2;
    yNext = y + (dtInput * 0.5) * p2;
    vxNext = vx + (dtInput * 0.5) * m2;
    vyNext = vy + (dtInput * 0.5) * m2;
    double k3 = gFunction(vyNext);
    double m3 = hPyFunction(xNext, yNext, vxNext, vyNext);
    double p3 = gFunction(vxNext);
    double n3 = hPxFunction(xNext, yNext, vxNext, vyNext);
    xNext = x + (dtInput) * k3;
    yNext = y + (dtInput) * p3;
    vxNext = vx + (dtInput) * m3;
    vyNext = vy + (dtInput) * m3;
    double k4 = gFunction(vyNext);
    double m4 = hPyFunction(xNext, yNext, vxNext, vyNext);
    double p4 = gFunction(vxNext);
    double n4 = hPxFunction(xNext, yNext, vxNext, vyNext);

    yProp = y + dtInput * ((k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0);
    vyProp = vy + dtInput * ((m1 + 2.0 * m2 + 2.0 * m3 + m4) / 6.0);
    xProp = x + dtInput * ((p1 + 2.0 * p2 + 2.0 * p3 + p4) / 6.0);
    vxProp = vx + dtInput * ((n1 + 2.0 * n2 + 2.0 * n3 + n4) / 6.0);
}

bool Body::checkIfProposedBelowTrack(int closestXEntry) {
    double closeX = track.getXValues()[closestXEntry];
    double closeY = track.getYValues()[closestXEntry];
    double minXDifference = xProp - closeX;

    bool below = false;
    double xMin = track.getXMin();
    double yMin = track.getYMin();
    double xMax = track.getXMax();
    double yMax = track.getYMax();
    if (closeX < xProp) {
        double nextCloseX = track.getXValues()[closestXEntry + 1];
        double nextCloseY = track.getYValues()[closestXEntry + 1];
        if (((nextCloseY - closeY) / (nextCloseX - closeX)) * (xProp - closeX) + closeY > yProp) {
            below = true;
        }
    }
    else {
        double nextCloseX = track.getXValues()[closestXEntry - 1];
        double nextCloseY = track.getYValues()[closestXEntry - 1];
        if (((closeY - nextCloseY) / (closeX - nextCloseX)) * (xProp - nextCloseX) + nextCloseY > yProp) {
            below = true;
        }
    }
    if (xProp < xMin || yProp < yMin - lowestCollisionPossible || xProp > xMax || yProp > yMax) {
        below = false;
    }
    return below;
}

double Body::findMinDistToTrackNode(double inputX, double inputY) {
    double closestEntry = track.getClosestEntryByDist(xProp, yProp);
    double closeX = track.getXValues()[closestEntry];
    double closeY = track.getYValues()[closestEntry];
    double xDiff = xProp - closeX;
    double yDiff = yProp - closeY;
    double minDifference = std::sqrt(xDiff * xDiff + yDiff * yDiff);
    return minDifference;
}

void Body::setPanicProposition(int closestXEntry) {
    s = track.getSValues()[closestXEntry];
    v = 0.0;
    xProp = track.getXValues()[closestXEntry];
    yProp = track.getYValues()[closestXEntry];
    vxProp = 0.0;
    vyProp = 0.0;
    onTrack = true;
}

void Body::setPreCollisionProposition(int& closestXEntry) {
    double minDifference = 1.0;
    double alteredDt = 0.0;
    double lowDt = 0.0;
    double highDt = dt;
    int numRollbacks = 0;
    bool below = true;
    while (below == true || collisionTol < minDifference) {
        alteredDt = (lowDt + highDt) * 0.5;

        setProposedNext_x_y_vx_vy(alteredDt);
        closestXEntry = track.getClosestEntryByX(xProp);
        below = checkIfProposedBelowTrack(closestXEntry);

        if (below) {
            highDt = alteredDt;
        }
        else {
            lowDt = alteredDt;
            minDifference = findMinDistToTrackNode(xProp, yProp);
        }

        numRollbacks++;

        if (numRollbacks > maxNumRollbacks) {
            setPanicProposition(closestXEntry);
            minDifference = 0.0;
            below = false;
        }
    }
    dt = alteredDt;
}

void Body::setCollisionProposition(int& closestXEntry) {

    setPreCollisionProposition(closestXEntry);

    Vector2d vVector(vxProp, vyProp);
    Vector2d tanVector(1, track.getKValues()[closestXEntry]);
    double vDotTan = dotP(vVector, tanVector);
    double tanDotTan = dotP(tanVector, tanVector);
    Vector2d vTanCompVector = tanVector * (vDotTan / tanDotTan);
    Vector2d vNormalCompVector = vVector - vTanCompVector;
    Vector2d scaledVNormalCompVector = vNormalCompVector * elasticity;
    Vector2d reflectVVector = vTanCompVector - scaledVNormalCompVector;

    if (scaledVNormalCompVector.magnitude() / reflectVVector.magnitude() < stickyness) {
        onTrack = true;
        v = reflectVVector.magnitude();
        if (dotP(tanVector, vVector) < 0) {
            v = v * -1;
        }
        s = track.getSValues()[closestXEntry];
    }
    else {
        vxProp = reflectVVector.x;
        vyProp = reflectVVector.y;
    }
}

void Body::acceptProposition() {
    x = xProp;
    y = yProp;
    vx = vxProp;
    vy = vyProp;
}

void Body::updateOffTrack() {
    setProposedNext_x_y_vx_vy(dt);
    int closestXEntry = track.getClosestEntryByX(xProp);
    bool below = checkIfProposedBelowTrack(closestXEntry);
    if (below == true) {
        setCollisionProposition(closestXEntry);
    }
    acceptProposition();
}

double Body::hFunction(double sInput, double vInput) {
    int closestEntry = track.getClosestEntryByS(sInput);
    double k = track.getKValues()[closestEntry];
    double a = -g * k / (std::sqrt(1 + k * k)); // gravity
    a += -friction * vInput / m; // friction
    a += -0.5 * drag * vInput * std::abs(vInput) / m;// air resistance
    for (Spring spring : connectedSprings) {
        double c = spring.getStiffness();
        double xA = spring.getAnchorX();
        double yA = spring.getAnchorY();
        double xSP = xA - track.getXValues()[closestEntry];
        double ySP = yA - track.getYValues()[closestEntry];
        double R = spring.getRestLength();
        a += (findSpringForceMag(c, xSP, ySP, R) * (xSP + k * ySP)) / (m * std::sqrt(1 + k * k) * std::sqrt(xSP * xSP + ySP * ySP)); // spring
    }
    return a;
}

double Body::hPxFunction(double xInput, double yInput, double vxInput, double vyInput) {
    double ax = 0.0;
    ax += -0.5 * drag * vxInput * sqrt(vxInput * vxInput + vyInput * vyInput) / m; // air resistance
    for (Spring spring : connectedSprings) {
        double R = spring.getRestLength();
        double c = spring.getStiffness();
        double xA = spring.getAnchorX();
        double yA = spring.getAnchorY();
        double xDist = xInput - xA;
        double yDist = yInput - yA;
        double L = std::sqrt(xDist * xDist + yDist * yDist);
        double S = L - R;
        ax += -c * S * (xDist / L) / m; //spring
    }
    return ax;
}

double Body::hPyFunction(double xInput, double yInput, double vxInput, double vyInput) {
    double ay = -g; //gravity
    ay += -0.5 * drag * vyInput * sqrt(vxInput * vxInput + vyInput * vyInput) / m; // air resistance
    for (Spring spring : connectedSprings) {
        double R = spring.getRestLength();
        double c = spring.getStiffness();
        double xA = spring.getAnchorX();
        double yA = spring.getAnchorY();
        double xDist = xInput - xA;
        double yDist = yInput - yA;
        double L = std::sqrt(xDist * xDist + yDist * yDist);
        double S = L - R;
        ay += -c * S * (yDist / L) / m; // spring
    }
    return ay;
}

double Body::gFunction(double vInput) {
    return vInput;
}

double Body::getS() const { return s; }
double Body::getV() const { return v; }
double Body::getX() const { return x; }
double Body::getY() const { return y; }
double Body::getVX() const { return vx; }
double Body::getVY() const { return vy; }
double Body::getM() const { return m; }
double Body::getElasticity() const { return elasticity; }
double Body::getStickyness() const { return stickyness; }
double Body::getFriction() const { return friction; }
double Body::getDrag() const { return drag; }
double Body::getRadius() const { return radius; }
bool Body::getOnTrack() const { return onTrack; }