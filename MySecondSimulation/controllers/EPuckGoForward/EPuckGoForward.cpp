// File:          EPuckGoForward.cpp
// Date:          XX/XX/XXXX
// Description:   Example in Lecture 7.2
// Author:        Leo Wu
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#include <memory>
#include <array>
#include <string>
#include <iostream>
#include <iomanip>

const int TIME_STEP {32};
const double MAX_SPEED {6.28};
const int NUM_DIST_SENSORS {8};

int main(int argc, char **argv) {
  // create robot on the heap
  std::unique_ptr<webots::Robot> robot {std::make_unique<webots::Robot>()};

  // or create robot on the stack
  // webots::Robot robot{};

  // DistanceSensor and Motor are owned and managed by Robot, 
  // so we can use Raw Pointers for these components as we are just observing them
  // and not responsible for freeing them 

  // initialise distance sensors
  std::array<webots::DistanceSensor*, NUM_DIST_SENSORS> ps {};
  std::array<std::string, NUM_DIST_SENSORS> psNames {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };
  for (int i {0}; i < NUM_DIST_SENSORS; ++i) {
    ps[i] = robot->getDistanceSensor(psNames[i]);
    ps[i]->enable(TIME_STEP);
  }
  
  // initialise motors
  webots::Motor* leftMotor {robot->getMotor("left wheel motor")};
  webots::Motor* rightMotor {robot->getMotor("right wheel motor")};
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);

  // feedback loop: step simulation until an exit event is received
  while (robot->step(TIME_STEP) != -1) {

    // read sensors outputs
    std::array<double, NUM_DIST_SENSORS> psValues {};
    for (int i {0}; i < NUM_DIST_SENSORS; ++i) {
      psValues[i] = ps[i]->getValue();
    }
    
    // for debugging, you can print the sensor readings to the console
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Time: " << robot->getTime() << ' ';
    std::cout << "DistanceSensors: ";
    for (const auto elem : psValues) {
      std::cout << elem << ' ';
    }

    // detect obstacles
    constexpr double obstacleThreshold {90.0};
    bool rightObstacle {
      psValues[0] > obstacleThreshold ||
      psValues[1] > obstacleThreshold ||
      psValues[2] > obstacleThreshold
    };
    bool leftObstacle {
      psValues[5] > obstacleThreshold ||
      psValues[6] > obstacleThreshold ||
      psValues[7] > obstacleThreshold
    };
    
    // for debugging, you can print the detection results to the console
    std::cout << "leftObstacle: " << leftObstacle << ' '
              << "rightObstacle: " << rightObstacle << ' ';
    
    // initialize motor speeds at 50% of MAX_SPEED.
    constexpr double speedScale {0.5};
    double leftSpeed {speedScale * MAX_SPEED};
    double rightSpeed {speedScale * MAX_SPEED};
    // modify speeds according to obstacles
    if (leftObstacle) {
      // turn right
      leftSpeed  = speedScale * MAX_SPEED;
      rightSpeed = -speedScale * MAX_SPEED;
    } else if (rightObstacle) {
      // turn left
      leftSpeed  = -speedScale * MAX_SPEED;
      rightSpeed = speedScale * MAX_SPEED;
    }
    
    // for debugging, you can print the speeds to the console
    std::cout << "leftSpeed: " << leftSpeed << ' '
              << "rightSpeed: " << rightSpeed << std::endl;
    
    // write actuators inputs
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
  }

  return 0;
}