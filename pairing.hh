#ifndef PAIRING_ROBOT_HH
#define PAIRING_ROBOT_HH

#include "robot.hh"

class PairingRobot;
typedef std::shared_ptr<PairingRobot> PairingRobotPtr;

class PairingRobot : public mrs::Robot {
public:
  PairingRobot(unsigned int id, const mrs::Position2d& p,
               const mrs::RobotSettings& settings = mrs::defaultRobotSettings,
               const mrs::Velocity2d& vel = mrs::Velocity2d::Random());

  const mrs::Velocity2d& action(std::vector<mrs::RobotPtr>& swarm);

  std::string name() const { return std::string("Pair"); }

  bool isPairedWith(const mrs::RobotPtr& robot) const;

  mrs::RobotPtr clone() const;

private:
  // TODO: Add data to the class as you deem necessary
};

#endif