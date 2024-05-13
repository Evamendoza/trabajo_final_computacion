#include <base.hh>
#include <environment.hh>
#include <cmath>

#include "pairing.hh"

// Environment class to obtain the size of the environment
extern mrs::Environment env;

// Constructor
PairingRobot::PairingRobot(unsigned int id, const mrs::Position2d &p,
                           const mrs::RobotSettings &settings,
                           const mrs::Velocity2d &vel) : mrs::Robot(id, p, settings)

{
}

// Action (main) method
const mrs::Velocity2d &PairingRobot::action(std::vector<mrs::RobotPtr> &neigh)
{   m_vel = mrs::Velocity2d::Random();
     const float closeThreshold = 4.0; // Define una distancia umbral para considerar "muy cerca"

    for (auto &n : neigh)
    {
        // Calcula la distancia entre el robot actual y el robot vecino
        float dist = mrs::distance(m_pos, n->position());

        // Supongamos que el color opuesto se determina por la diferencia en la primera componente de color
        if (settings().color[0] != n->settings().color[0]) 
        {
            if (dist < closeThreshold) 
            {
                // Si están muy cerca y tienen un color "opuesto", "pegarse"
                m_vel = (n->position() - m_pos); // Moverse directamente hacia el otro robot
                m_vel.normalize(); // Normaliza para controlar la velocidad a una unidad estándar
                m_vel *= settings().vMax; // Ajustar a la velocidad máxima permitida
                break; // Deja de revisar otros una vez que se decide "pegarse"
            }
        }
    }

    // Si no se encontró ningún robot cercano de color opuesto, moverse de manera aleatoria
    if (m_vel == mrs::Velocity2d::Zero())
    {
        m_vel = mrs::Velocity2d::Random();
    }

    return m_vel;
}
mrs::RobotPtr
PairingRobot::clone() const
{
    PairingRobotPtr newRobot = std::make_shared<PairingRobot>(m_id, m_pos, m_settings, m_vel);
    return std::dynamic_pointer_cast<mrs::Robot>(newRobot);
}
