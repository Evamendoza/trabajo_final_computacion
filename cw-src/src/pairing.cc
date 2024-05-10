#include <base.hh>
#include <environment.hh>

#include "pairing.hh"

// Environment class to obtain the size of the environment
extern mrs::Environment env;

// TODO: Implement here your pairing-unpairing robot

// Constructor
PairingRobot::PairingRobot(unsigned int id, const mrs::Position2d & p, 
			   const mrs::RobotSettings & settings,
			   const mrs::Velocity2d & vel) :
  mrs::Robot(id, p, settings)
{
}

// Action (main) method
const mrs::Velocity2d &
PairingRobot::action(std::vector<mrs::RobotPtr> & neigh)
{
  float vx=0, vy=0, ry=0, rx=0, ux=0, uy=0;
  // Loop through neighboring robots
    for (int contador=0;const auto &neighbor : neigh;contador++) {

       ux += swarm[contador]->velocity()[0] / swarm.size();
        uy += swarm[contador]->velocity()[1] / swarm.size();

        vx += ((mrs::distance(this->position(), swarm[contador]->position()) * (std::cos(mrs::angle(this->position(), swarm[contador]->position()))))) * 1 * (mrs::distance(this->position(), swarm[contador]->position()) - 2);
        vy += ((mrs::distance(this->position(), swarm[contador]->position()) * (std::sin(mrs::angle(this->position(), swarm[contador]->position()))))) * 1 * (mrs::distance(this->position(), swarm[contador]->position()) - 2);
    }

    if (m_vel[0] > 0.8) {
        m_vel[0] = 0.8;
    }
    if (m_vel[1] > 0.8) {
        m_vel[1] = 0.8;
    }
    if (m_vel[0] < -0.8) {
        m_vel[0] = -0.8;
    }
    if (m_vel[1] < -0.8) {
        m_vel[1] = -0.8;
    }

    m_vel[0] += (vx * 1.0 + 1.0 * rx + ux * 0.2);
    m_vel[1] += (vy * 1.0 + 1.0 * ry + uy * 0.2);

}


    // Return the calculated velocity
   

  //return this->m_vel;


mrs::RobotPtr
PairingRobot::clone() const
{
  PairingRobotPtr newRobot = std::make_shared<PairingRobot>(m_id, m_pos, m_settings, m_vel);
  
  return std::dynamic_pointer_cast<mrs::Robot>(newRobot);
}
