// File:          MySecondControllerSln2.cpp
// Date:
// Description:
// Author:
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Keyboard.hpp>

#include <array>
#include <string>
#include <iostream>
#include <iomanip>

const int TIME_STEP {64};
const double MAX_SPEED {6.28};
const int N_DIST_SENSORS {8};

int main(int argc, char **argv) {

  webots::Robot robot {};

  // DistanceSensor and Motor are owned and managed by Robot, 
  // so we can use Raw Pointers for these components as we are just observing them
  // and not responsible for freeing them 

  // initialise distance sensors
  std::array<webots::DistanceSensor*, N_DIST_SENSORS> ps {};
  const std::array<std::string, N_DIST_SENSORS> psNames {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };
  for (int i {0}; i < N_DIST_SENSORS; ++i) {
    ps[i] = robot.getDistanceSensor(psNames[i]);
    ps[i]->enable(TIME_STEP);
  }
  
  // initialise motors
  webots::Motor* leftMotor {robot.getMotor("left wheel motor")};
  webots::Motor* rightMotor {robot.getMotor("right wheel motor")};
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  
  // initialise keyboard
  webots::Keyboard* keyboard {robot.getKeyboard()};
  keyboard->enable(TIME_STEP);

  // feedback loop: step simulation until an exit event is received
  while(robot.step(TIME_STEP) != -1) {

    // read sensors outputs
    std::array<double, N_DIST_SENSORS> psValues {};
    for(int i {0}; i < N_DIST_SENSORS; ++i) {
      psValues[i] = ps[i]->getValue();
    }
    
    // for debugging, you can print the sensor readings to the console
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Time: " << robot.getTime() << ' ';
    std::cout << "DistanceSensors: ";
    for(const auto elem : psValues) {
      std::cout << elem << ' ';
    }

    // detect obstacles
    const double obstacleThreshold {90.0};
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
    const double speedScale {0.5};
    double leftSpeed {speedScale * MAX_SPEED};
    double rightSpeed {speedScale * MAX_SPEED};
    
    int k {keyboard->getKey()};
    // std::cout << k << std::endl;
    if(k == 'T') { // Turn right
      leftSpeed  = speedScale * MAX_SPEED;
      rightSpeed = -speedScale * MAX_SPEED;
    } else if(k == 'B') { // Backward
      leftSpeed  = -speedScale * MAX_SPEED;
      rightSpeed = -speedScale * MAX_SPEED;    
    } else { // Roam
      // modify speeds according to obstacles
      if(leftObstacle) {
        // turn right
        leftSpeed  = speedScale * MAX_SPEED;
        rightSpeed = -speedScale * MAX_SPEED;
      } else if(rightObstacle) {
        // turn left
        leftSpeed  = -speedScale * MAX_SPEED;
        rightSpeed = speedScale * MAX_SPEED;
      }
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