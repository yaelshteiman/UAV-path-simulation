//
// Created by Yael Shtaiman on 16/12/2024.
//

#ifndef UAV_H
#define UAV_H

#include <fstream>
#include <iomanip>
#include <iostream>

class UAV {
private:
    int id;
    float x, y, z; //position
    float velocity;
    float azimuth; // in degrees
    float turningRadius;
    bool holdingPattern;
    float targetX, targetY;
    bool hasActiveTarget;
    float holdingCenterX;
    float holdingCenterY;

public:
    UAV(int id, float x0, float y0, float z0, float v0, float az, float r);

    void updatePosition(float dt); // calculate new position based on azimuth and velocity
    void moveToTarget(float targetX, float targetY, float dt);
    void holdAtPoint(float centerX, float centerY, float dt); // holding pattern around a target point
    void moveStraight(float dt);
    bool isAtTarget(float targetX, float targetY) const;

    void logData(const std::string& fileName, float time) const;

    // getters and setters
    int getId() const;
    bool isHoldingPattern() const;
    void setHoldingPattern(bool state);
    float getTargetX() const;
    float getTargetY() const;
    float getHoldingCenterX() const;
    float getHoldingCenterY() const;
    void setTarget(float targetX, float targetY);
    bool hasTarget() const;
};



#endif //UAV_H

