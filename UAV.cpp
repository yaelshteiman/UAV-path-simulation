//
// Created by Yael Shtaiman on 16/12/2024.
//

#include "UAV.h"
#define M_PI 3.14159265358979323846f
#define M_PI_2 1.5707963267948966

UAV::UAV(int id, float x0, float y0, float z0, float v0, float az, float r) :
    id(id), x(x0), y(y0), z(z0), velocity(v0), azimuth(az), turningRadius(r), holdingPattern(false),
    hasActiveTarget(false), holdingCenterX(0.0f), holdingCenterY(0.0f)
{}

void UAV::updatePosition(float dt) {
    float radians = azimuth * M_PI / 180.0f;
    x += velocity * dt * cos(radians);
    y += velocity * dt * sin(radians);
}

bool UAV::isAtTarget(float targetX, float targetY) const {
    const float epsilon = 1.0f;
    float distance = std::sqrt(std::pow(targetX - x, 2) + std::pow(targetY - y, 2));
    return distance <= epsilon;
}

void UAV::moveToTarget(float targetX, float targetY, float dt) {
    if (isAtTarget(targetX, targetY)) {
        if (!isHoldingPattern()) {
            setHoldingPattern(true);

            // Offset the center to the right by the radius (clockwise logic)
            float radians = azimuth * M_PI / 180.0f; // Current azimuth in radians
            holdingCenterX = targetX + turningRadius * cos(radians - M_PI_2); // Offset 90 degrees right
            holdingCenterY = targetY + turningRadius * sin(radians - M_PI_2);
        }
        holdAtPoint(holdingCenterX, holdingCenterY, dt);
        return;
    }

    // calculate the target azimuth
    float dx = targetX - x;
    float dy = targetY - y;
    float targetAzimuth = atan2(dy, dx) * 180.0f / M_PI;

    // adjust azimuth considering the turning radius
    float deltaAzimuth = targetAzimuth - azimuth;

    if (deltaAzimuth > 180.0f) deltaAzimuth -= 360.0f;
    if (deltaAzimuth < -180.0f) deltaAzimuth += 360.0f;

    // Limit the azimuth change based on maximum turn rate
    float maxTurnRate = (velocity / turningRadius) * dt * 180.0f / M_PI; // degrees per time step
    if (deltaAzimuth > maxTurnRate) deltaAzimuth = maxTurnRate;
    if (deltaAzimuth < -maxTurnRate) deltaAzimuth = -maxTurnRate;

    azimuth += deltaAzimuth * (velocity / turningRadius);
    if (azimuth >= 360.0f) azimuth -= 360.0f;
    if (azimuth < 0.0f) azimuth += 360.0f;

    updatePosition(dt);
}

void UAV::holdAtPoint(float centerX, float centerY, float dt) {
    float angleToCenter = atan2(y - centerY, x - centerX);

    // move clockwise: decrease the angle over time
    angleToCenter -= (velocity / turningRadius) * dt;

    // update position based on the new angle
    x = centerX + turningRadius * cos(angleToCenter);
    y = centerY + turningRadius * sin(angleToCenter);

    // update the azimuth to point tangent to the circle
    azimuth = angleToCenter * 180.0f / M_PI - 90.0f; // tangent to the orbit
    if (azimuth < 0.0f) azimuth += 360.0f;
}

void UAV::logData(const std::string& fileName, float time) const {
    std::ofstream outFile(fileName, std::ios_base::app);
    if (outFile.is_open()) {
        outFile << std::fixed << std::setprecision(2); // Set two decimal places
        outFile << time << " " << x << " " << y << " " << azimuth << "\n";
        outFile.close();
    }
}

int UAV::getId() const {
    return id;
}

bool UAV::isHoldingPattern() const {
    return holdingPattern;
}

void UAV::setHoldingPattern(bool state) {
    holdingPattern = state;
}

void UAV::moveStraight(float dt) {
    updatePosition(dt);
}

void UAV::setTarget(float targetX, float targetY) {
    this->targetX = targetX;
    this->targetY = targetY;
    hasActiveTarget = true;
}

bool UAV::hasTarget() const {
    return hasActiveTarget;
}

float UAV::getTargetX() const {
    return targetX;
}

float UAV::getTargetY() const {
    return targetY;
}

float UAV::getHoldingCenterX() const {
    return holdingCenterX;
}

float UAV::getHoldingCenterY() const {
    return holdingCenterY;
}
