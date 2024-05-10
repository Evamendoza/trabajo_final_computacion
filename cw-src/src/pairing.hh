#ifndef PAIRING_ROBOT_HH
#define PAIRING_ROBOT_HH
#include "robot.hh"


class PairingRobot;
// Definition of a pointer to an aggregation robot (unnecessary)
typedef std::shared_ptr<PairingRobot> PairingRobotPtr;

class PairingRobot : public mrs::Robot {
public:

  // Constructor for the Flocking Robot
  PairingRobot(unsigned int id, const mrs::Position2d & p, 
	       const mrs::RobotSettings & settings = mrs::defaultRobotSettings,
	       const mrs::Velocity2d & vel = mrs::Velocity2d::Random());

  // Method to compute the action
  const mrs::Velocity2d & action(std::vector<mrs::RobotPtr> & swarm);

  // Name of the type of flocking robot
  std::string name() const {return std::string("Pair");}

  // TODO: Add new methods here as you deem necessary
  
  mrs::RobotPtr clone() const;
private:
  // TODO: Add data to the class as you deem necessary
  
};


#endif
