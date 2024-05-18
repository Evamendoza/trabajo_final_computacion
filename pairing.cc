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
const mrs::Velocity2d &PairingRobot::action(std::vector<mrs::RobotPtr> &neigh) {
    m_vel =  mrs::Velocity2d::Random(); // Velocidad inicial aleatoria por defecto
    
    //m_vel=mrs::Velocity2d();
    const float closeThreshold = 1; // Distancia para considerar "muy cerca"
    const float minDistance = 0.5; // Distancia mínima que deben mantener
    bool foundPartner = false;
    int contador = 0;
    float vx;
    float vy;
    float ux,uy,rx,ry;
    float modVel;
    bool emparejado=false;
    
    for (auto &n : neigh) {
        m_vel = mrs::Velocity2d::Random();
       
        // Calcula la distancia entre el robot actual y el robot vecino
        float dist = mrs::distance(m_pos, n->position())-minDistance;
        float angulo =mrs::angle(m_pos, n->position());


        //Fuerza de cohesion 
        if ((this->sameColor(n)==false) && settings().visible==true) {
            vx = dist * cos(angulo);
            vy = dist * sin(angulo);

            modVel = sqrt(vx * vx + vy * vy);

            vx /= modVel;
            vy /= modVel;
            m_vel = mrs::Velocity2d(vx, vy);
            m_vel={vx+m_vel[0],vy+m_vel[1]};
            std::cout << "Se siente atraido " << n->id() << std::endl;
            std::cout<<"n vel: " << n->velocity()[0] << "," << n->velocity()[1] << std::endl;
            
        }
        //repeleer 
        else if ((this->sameColor(n)==true)||(settings().visible==false)){
         vx=dist*cos(angulo);
         vy=dist*sin(angulo);            
         m_vel={-vx +m_vel[0],-vy+m_vel[1]};

         m_vel=0.4*m_vel;
         std::cout << "Repulsion con robot ID: " << n->id() << std::endl;
         

         
         }
        

         
        //Obtener velocidad del robot
        if ((settings().visible==true)&&(this->sameColor(n)==false)){
            //ux=n->velocity()[0]/2;
            //uy=n->velocity()[1]/2;

            n->dejarVisible(false);
            std::cout<<"n vel: " << n->velocity()[0] << "," << n->velocity()[1] << std::endl;
            std::cout<<"Velocidad: " << m_vel << std::endl;
            
            m_vel /= neigh.size();
            m_vel += n->velocity();
            
            std::cout << "Emparejado con robot ID: " << n->id() << std::endl;
             
            


            }

        }
        //

        // Verifica si el color es opuesto basado en la primera componente
        //settings().color[0] != n->settings().color[0]
      
    

    // Si no se encontró ningún robot para emparejar, mantiene la velocidad aleatoria inicial
    /*if (!foundPartner) {
        m_vel = mrs::Velocity2d::Random();
    }*/

    return m_vel;
}

mrs::RobotPtr PairingRobot::clone() const {
    PairingRobotPtr newRobot = std::make_shared<PairingRobot>(m_id, m_pos, m_settings, m_vel);
    return std::dynamic_pointer_cast<mrs::Robot>(newRobot);
}