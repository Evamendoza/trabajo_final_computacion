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
    const float minDistance = 0.4; // Distancia mínima que deben mantener
    bool foundPartner = false;
    int contador = 0;
    float vx=0, ax=0, rx=0, cx=0;
    float vy=0, ay=0, ry=0, cy=0;
    float ux,uy;
    float modVel;
    bool emparejado=false;
    
    for (auto &n : neigh) {
       
        // Calcula la distancia entre el robot actual y el robot vecino
        float dist = mrs::distance(m_pos, n->position());
        float angulo =mrs::angle(m_pos, n->position());

        if ((dist<=0.5)&&(this->sameColor(n)==false)){
            this->dejarVisible(false);
            std::cout << "visible" << std::endl;
        }
        //Fuerza de atraccion
        if ((this->sameColor(n)==false) && (this->settings().visible==true)&&(n->settings().visible==true)) {
            ax += dist * cos(angulo) * (dist-minDistance);
            ay += dist * sin(angulo) * (dist-minDistance);

            //modVel = sqrt(ax * ax + ay * ay);

            //ax = ax/modVel;
            //ay = ay/modVel;
            //m_vel = mrs::Velocity2d(vx, vy);
            //m_vel={vx+m_vel[0],vy+m_vel[1]};
            //std::cout << "Se siente atraido " << n->id() << std::endl;
            //std::cout<<"n vel: " << n->velocity()[0] << "," << n->velocity()[1] << std::endl;
            
        }
        //repeleer 
        else if ((this->sameColor(n)==true)||((n->settings().visible==true)&&(dist>0.3))||((this->settings().visible==true)&&(dist>0.5))){
            rx += -dist * cos(angulo) * dist;
            ry += -dist * sin(angulo) * dist;

            //modVel = sqrt(rx * rx + ry * ry);

            //rx = rx/modVel;
            //ry = ry/modVel;
            //std::cout << "Se repele " << n->id() << std::endl;

         
         }
        
         
        //Obtener velocidad del robot
        //alineacion 
        
        if ((this->sameColor(n)==false)&&(dist<=0.5)){
            //ux=n->velocity()[0]/2;
            //uy=n->velocity()[1]/2;

            //cx=this->velocity()[0]/2+n->velocity()[0]/2;
            //cy=this->velocity()[1]/2+n->velocity()[1]/2;
            //cx += dist * cos(angulo) * (dist-minDistance);
            //cy += dist * sin(angulo) * (dist-minDistance);
            //std::cout<<"velocidad= "<<this->velocity()<<std::endl;

            cx=10;
            cy=10;
            std::cout<<"velocidad2= "<<m_vel<<std::endl;


            }
            

        }

        if (m_vel[0]>0.3){
            m_vel[0]=0.3;
        }
        if (m_vel[0]<-0.3){
            m_vel[0]=-0.3;
        }
        if (m_vel[1]>0.3){
            m_vel[1]=0.3;
        }
        if (m_vel[1]<-0.3){
            m_vel[1]=-0.3;
        }
        m_vel[0]+=rx*0.5+ax*0.1+cx*0.3;
        m_vel[1]+=ry*0.5+ay*0.1+cy*0.3;

        if ((m_vel[0]==0)&&(m_vel[1]==0)){
            m_vel[0]=1;
            m_vel[1]=1;
        }

        //modVel = sqrt(rx * rx + ry * ry);

            //rx = rx/modVel;
            //ry = ry/modVel;

        if((neigh.size()==0)){
            m_vel = mrs::Velocity2d::Random();
        }

        //std::cout<<"velocidad= "<<m_vel<<std::endl;
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