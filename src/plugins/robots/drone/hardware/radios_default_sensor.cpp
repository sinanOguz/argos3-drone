/**
 * @file <argos3/plugins/robots/drone/hardware/radios_default_sensor.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "radios_default_sensor.h"

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/networking/tcp_socket.h>
#include <argos3/plugins/robots/drone/hardware/robot.h>

namespace argos {

   /****************************************/
   /****************************************/

   void CRadiosDefaultSensor::Init(TConfigurationNode& t_tree) {
      try {
         /* Parent class init */
         CCI_RadiosSensor::Init(t_tree);
         if(!CRobot::GetInstance().GetSocket().IsConnected()) {
            LOGERR << "[WARNING] Robot is not connected to a router" << std::endl;
         }
         m_vecInterfaces.emplace_back("wifi");
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the wifi default sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CRadiosDefaultSensor::Update() {
      /* clear the messages from the interface */
      m_vecInterfaces[0].Messages.clear();
      /* receive messages if the socket is connected */
      if(CRobot::GetInstance().GetSocket().IsConnected()) {
         /* buffer for receiving messages */
         CByteArray cMessage;
         /* read in the new messages to the control interface */
         while(CRobot::GetInstance().GetSocket().GetEvents().count(CTCPSocket::EEvent::InputReady) == 1) {
            if(CRobot::GetInstance().GetSocket().ReceiveByteArray(cMessage) == false) {
               break;
            }
            else {
               m_vecInterfaces[0].Messages.emplace_back(cMessage);
            }
         }
      }
   }

   /****************************************/
   /****************************************/

   void CRadiosDefaultSensor::Reset() {
      /* Clear the existing data */
      m_vecInterfaces[0].Messages.clear();
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CRadiosDefaultSensor,
                   "radios", "default",
                   "Michael Allwright [allsey87@gmail.com]",
                   "1.0",
                   "Hardware implementation of the radio sensor.",
                   "This sensor receives messages from other robots using the local network.",
                   "Usable"
   );

   /****************************************/
   /****************************************/

}
