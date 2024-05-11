/*#include <base.hh>
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
    float vx = 0, vy = 0;
    static int behavior = 0; // Variable para controlar el comportamiento actual del robot
    static int behavior_timer = 0; // Temporizador para cambiar de comportamiento

    switch (behavior) {
        case 0: // Deambular
            // Implementar el comportamiento de deambular aquí
            // Puedes usar una velocidad aleatoria o un movimiento en línea recta
            
            // Ejemplo de movimiento aleatorio:
            m_vel = mrs::Velocity2d::Random();
            m_vel *= 0.8; // Escalar la velocidad a un rango máximo
            
            break;

        case 1: // Movimiento coordinado
            // Implementar el comportamiento de movimiento coordinado aquí
            // Puedes usar una velocidad constante y mantener una distancia con los robots emparejados
            
            // Ejemplo de movimiento coordinado:
            if (!neigh.empty()) {
                // Obtener la posición promedio de los robots vecinos emparejados
                mrs::Position2d avgPosition;
                for (auto& n : neigh) {
                    if (n->id() != m_id) {
                        avgPosition += n->position();
                    }
                }
                avgPosition /= neigh.size() - 1;
                
                // Calcular la dirección hacia la posición promedio
                mrs::Position2d direction = avgPosition - m_pos;
                direction.normalize();
                
                // Calcular la velocidad en función de la dirección
                m_vel = direction * 0.8; // Velocidad constante
                
                // Mantener una distancia constante de separación
                float desiredDistance = 2.0; // Distancia deseada
                float currentDistance = mrs::distance(m_pos, avgPosition);
                if (currentDistance < desiredDistance) {
                    // Alejarse si está demasiado cerca
                    m_vel -= direction * 0.2;
                } else if (currentDistance > desiredDistance) {
                    // Acercarse si está demasiado lejos
                    m_vel += direction * 0.2;
                }
            }
            
            break;

        case 2: // Repulsión
            // Implementar el comportamiento de repulsión aquí 
            // Puedes usar una velocidad aleatoria o alejarse de los robots vecinos
            
            // Ejemplo de comportamiento de repulsión:
            for (auto& n : neigh) {
                if (n->id() != m_id) {
                    // Calcular la dirección de alejamiento de los robots vecinos
                    mrs::Position2d direction = m_pos - n->position();
                    direction.normalize();
                    
                    // Agregar la dirección a la velocidad
                    m_vel += direction * 0.8;
                }
            }
            
            break;
    }

    // Actualizar el temporizador de cambio de comportamiento
    behavior_timer++;
    if (behavior_timer > 10000) { // Cambiar de comportamiento cada 10 segundos
        behavior = (behavior + 1) % 3; // Cambiar al siguiente comportamiento
        behavior_timer = 0; // Reiniciar el temporizador
    }
    
    return m_vel;

}

mrs::RobotPtr
PairingRobot::clone() const
{
    PairingRobotPtr newRobot = std::make_shared<PairingRobot>(m_id, m_pos, m_settings, m_vel);
    return std::dynamic_pointer_cast<mrs::Robot>(newRobot);
}*/
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
    float vx = 0, vy = 0;
    static int behavior = 0; // Variable para controlar el comportamiento actual del robot
    static int behavior_timer = 0; // Temporizador para cambiar de comportamiento

    switch (behavior) {
        case 0: // Atracción
            // Implementar el comportamiento de atracción aquí
            // Puedes hacer que los robots se atraigan entre sí
            
            // Ejemplo de comportamiento de atracción:
            for (auto& n : neigh) {
                if (n->id() != m_id && !isPairedWith(n)) {
                    // Calcular la dirección de atracción hacia los robots vecinos no emparejados
                    mrs::Position2d direction = n->position() - m_pos;
                    direction.normalize();
                    
                    // Agregar la dirección a la velocidad
                    m_vel += direction * 0.8;
                }
            }
            
            break;
        
        case 1: // Movimiento coordinado
            // Implementar el comportamiento de movimiento coordinado aquí
            // Puedes usar una velocidad constante y mantener una distancia con los robots emparejados
            
            // Ejemplo de movimiento coordinado:
            if (!neigh.empty()) {
                // Obtener la posición promedio de los robots vecinos emparejados
                mrs::Position2d avgPosition;
                int pairedCount = 0;
                for (auto& n : neigh) {
                    if (n->id() != m_id && isPairedWith(n)) {
                        avgPosition += n->position();
                        pairedCount++;
                    }
                }
                
                if (pairedCount > 0) {
                    avgPosition /= pairedCount;
                    
                    // Calcular la dirección hacia la posición promedio
                    mrs::Position2d direction = avgPosition - m_pos;
                    direction.normalize();
                    
                    // Calcular la velocidad en función de la dirección
                    m_vel = direction * 0.8; // Velocidad constante
                    
                    // Mantener una distancia constante de separación
                    float desiredDistance = 2.0; // Distancia deseada
                    float currentDistance = mrs::distance(m_pos, avgPosition);
                    if (currentDistance < desiredDistance) {
                        // Alejarse si está demasiado cerca
                        m_vel -= direction * 0.2;
                    } else if (currentDistance > desiredDistance) {
                        // Acercarse si está demasiado lejos
                        m_vel += direction * 0.2;
                    }
                }
            }
            
            break;

        case 2: // Repulsión
            // Implementar el comportamiento de repulsión aquí 
            // Puedes usar una velocidad aleatoria o alejarse de los robots vecinos
            
            // Ejemplo de comportamiento de repulsión:
            for (auto& n : neigh) {
                if (n->id() != m_id && isPairedWith(n)) {
                    // Calcular la dirección de alejamiento de los robots vecinos emparejados
                    mrs::Position2d direction = m_pos - n->position();
                    direction.normalize();
                    
                    // Agregar la dirección a la velocidad
                    m_vel += direction * 0.8;
                }
            }
            
            break;
    }

    // Actualizar el temporizador de cambio de comportamiento
    behavior_timer++;
    if (behavior_timer > 10000) { // Cambiar de comportamiento cada 10 segundos
        behavior = (behavior + 1) % 3; // Cambiar al siguiente comportamiento
        behavior_timer = 0; // Reiniciar el temporizador
    }
    
    return m_vel;
}

bool
PairingRobot::isPairedWith(const mrs::RobotPtr& robot) const
{
    // Verificar si el identificador del robot está presente en el conjunto de parejas emparejadas
    return pairedRobots_.count(robot->id()) > 0;
}
