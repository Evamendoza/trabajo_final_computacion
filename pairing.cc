#include <base.hh>
#include <environment.hh>
#include <cmath>

#include "pairing.hh"

// Environment class to obtain the size of the environment
extern mrs::Environment env;

// Constructor
PairingRobot::PairingRobot(unsigned int id, const mrs::Position2d &p,
                           const mrs::RobotSettings &settings,
                           const mrs::Velocity2d &vel) 
    : mrs::Robot(id, p, settings), m_vel(vel) // Inicializa m_vel
{
}

// Action (main) method
const mrs::Velocity2d &PairingRobot::action(std::vector<mrs::RobotPtr> &neigh) {
    m_vel = 0.8 * mrs::Velocity2d::Random(); // Velocidad inicial aleatoria por defecto
    const float closeThreshold = 1; // Distancia para considerar "muy cerca"
    const float minDistance = 0.5; // Distancia mínima que deben mantener
    bool foundPartner = false;
    int cont = 0;

    for (auto &n : neigh) {
        // Calcula la distancia entre el robot actual y el robot vecino
        float dist = mrs::distance(m_pos, n->position());

        // Verifica si el color es opuesto basado en la primera componente
        if (settings().color[0] != n->settings().color[0]) {
            do {
                if (dist < closeThreshold) {
                    foundPartner = true;
                    mrs::Position2d direction = n->position() - m_pos;
                    direction.normalize(); // Normalizar la dirección

                    if (dist > minDistance) {
                        // Si la distancia es mayor que la mínima, moverse hacia el otro robot
                        m_vel = direction * settings().vMin; // Ajustar a la velocidad mínima permitida
                    } else {
                        // Si están demasiado cerca, ajustar la velocidad para mantener la distancia mínima
                        m_vel = direction * (settings().vMin * (dist / minDistance));
                    }
                    cont++;
                    std::cout << "Contador: " << cont << std::endl;
                }
                // Recalcular la distancia en cada iteración del do-while
                dist = mrs::distance(m_pos, n->position());
            } while (dist < closeThreshold && settings().color[0] != n->settings().color[0]);
        }
    }

    // Si no se encontró ningún robot para emparejar, mantiene la velocidad aleatoria inicial
    if (!foundPartner) {
        m_vel = mrs::Velocity2d::Random();
    }

    return m_vel;
}

mrs::RobotPtr PairingRobot::clone() const {
    PairingRobotPtr newRobot = std::make_shared<PairingRobot>(m_id, m_pos, m_settings, m_vel);
    return std::dynamic_pointer_cast<mrs::Robot>(newRobot);
}

