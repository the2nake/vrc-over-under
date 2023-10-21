/* home.vn2007@gmail.com - 2023 */

#include "PIDController.hpp"

#include "common.hpp"

#include "main.h"

#include <chrono>
#include <cmath>

PIDController::PIDController(PIDConfig config)
{
    this->setPIDConstants(config);
}

void PIDController::setPIDConstants(PIDConfig config)
{
    this->fF = config.fF;
    this->kP = config.kP;
    this->kI = config.kI;
    this->kD = config.kD;
    this->cutIntegral = config.cutIntegral;
    this->integralCutThreshold = config.integralCutThreshold;
}

void PIDController::setPIDConstants(double fF, double kP, double kI, double kD, bool cutIntegral, double integralCutThreshold)
{
    this->fF = fF;
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->cutIntegral = cutIntegral;
    this->integralCutThreshold = integralCutThreshold;
}

void PIDController::resetPIDSystem()
{
    this->active = false;
    this->previousUpdateTimestamp = std::chrono::high_resolution_clock::now();
    this->prevError = 0.0;
    this->integral = 0.0;
    this->derivative = 0.0;
}

void PIDController::startPID(double target)
{
    this->active = true;
    this->target = target;
    this->previousUpdateTimestamp = std::chrono::high_resolution_clock::now();
}

void PIDController::setTarget(double target)
{
    this->target = target;
}

void PIDController::updatePID(double sensorValue)
{
    if (!(this->active.load()))
    {
        this->output = 0.0;
        return;
    }

    auto now = std::chrono::high_resolution_clock::now();
    auto dT = time_between<std::chrono::milliseconds>(previousUpdateTimestamp, now) / 1000.0;

    double error = this->target.load() - sensorValue;

    this->integral += error * dT;
    if (cutIntegral && (std::abs(error) <= this->integralCutThreshold || (std::signbit(error) != std::signbit(prevError))))
    {
        this->integral = 0;
    }

    if (dT != 0)
    {
        this->derivative = (error - this->prevError) / dT;
    }

    prevError = error;
    this->previousUpdateTimestamp = std::chrono::high_resolution_clock::now();

    // pros::screen::print(TEXT_MEDIUM, 3, "Sensor: %f Target: %f dT: %f", sensorValue, this->target.load(), dT);
    // pros::screen::print(TEXT_MEDIUM, 4, "E: %f I: %f D: %f", error, integral, derivative);
    // pros::screen::print(TEXT_MEDIUM, 5, "kP: %f kI: %f kD: %f", kP, kI, kD);
    // pros::screen::print(TEXT_MEDIUM, 6, "P: %f I: %f D: %f", error * kP, integral * kI, derivative * kD);
    this->output = (this->fF + error * kP + integral * kI + derivative * kD);
}