/**
 * @file <argos3/plugins/robots/drone/hardware/radios_default_actuator.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "radios_default_actuator.h"

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/networking/tcp_socket.h>
#include <argos3/plugins/robots/drone/hardware/robot.h>

namespace argos {

   /****************************************/
   /****************************************/

   void CRadiosDefaultActuator::Init(TConfigurationNode& t_tree) {
      try {
         /* Parent class init */
         CCI_RadiosActuator::Init(t_tree);
         if(!CRobot::GetInstance().GetSocket().IsConnected()) {
            LOGERR << "[WARNING] Robot is not connected to a router" << std::endl;
         }
         m_vecInterfaces.emplace_back("wifi");
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the radio default actuator", ex);
      }
   }

   
   /****************************************/
   /****************************************/

   void CRadiosDefaultActuator::Update() {
      /* send messages if the socket is connected */
      if(CRobot::GetInstance().GetSocket().IsConnected()) {
         for(const CByteArray& c_message : m_vecInterfaces[0].Messages) {
            CRobot::GetInstance().GetSocket().SendByteArray(c_message);
         }
         /* Flush data from the control interface */
         m_vecInterfaces[0].Messages.clear();
      }
   }

   /****************************************/
   /****************************************/

   void CRadiosDefaultActuator::Reset() {
      m_vecInterfaces[0].Messages.clear();
   }

   /****************************************/
   /****************************************/

   REGISTER_ACTUATOR(CRadiosDefaultActuator,
                     "radios", "default",
                     "Michael Allwright [allsey87@gmail.com]",
                     "1.0",
                     "Hardware implementation of the radio actuator.",
                     "This actuator sends messages to other robots using the local network.",
                     "Usable"
   );

   /****************************************/
   /****************************************/

}
   
