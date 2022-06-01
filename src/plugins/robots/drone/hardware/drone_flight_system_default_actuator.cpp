/**
 * @file <argos3/plugins/robots/drone/hardware/drone_flight_system_default_actuator.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 * @author Sinan Oguz - <soguz.ankara@gmail.com>
 */

#include "drone_flight_system_default_actuator.h"
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/drone/hardware/robot.h>
#include <argos3/plugins/robots/drone/hardware/pixhawk.h>

#include <termios.h>

#include <cerrno>
#include <cstring>

#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111

namespace argos {

   /****************************************/
   /****************************************/
   
   void CDroneFlightSystemDefaultActuator::Init(TConfigurationNode& t_tree) {
      try {
         CCI_DroneFlightSystemActuator::Init(t_tree);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in the drone flight system actuator.", ex);
      }
   }
  
   /****************************************/
   /****************************************/

   void CDroneFlightSystemDefaultActuator::Update() {
      if(CRobot::GetInstance().GetPixhawk().Ready()) {
        CVector3& cInitialOrientation =
            CRobot::GetInstance().GetPixhawk().GetInitialOrientation().value();
   /*       CVector3& cInitialPosition =
            CRobot::GetInstance().GetPixhawk().GetInitialPosition().value(); */
         uint8_t unTargetSystem =
            CRobot::GetInstance().GetPixhawk().GetTargetSystem().value();
         // CVector3& fTargetPosition = m_cTargetPosition;
         CVector3& fTargetVelocity = m_cTargetVelocity;
         /* ENU to NED */
          // fTargetPosition.Set(fTargetPosition.GetX(), -fTargetPosition.GetY(), -fTargetPosition.GetZ());
         fTargetVelocity.Set(fTargetVelocity.GetX(), -fTargetVelocity.GetY(), -fTargetVelocity.GetZ());
         fTargetVelocity.RotateZ(CRadians(cInitialOrientation.GetZ()));
          // fTargetPosition.RotateZ(CRadians(cInitialOrientation.GetZ()));
         /* initialize a setpoint struct */
         mavlink_set_position_target_local_ned_t tSetpoint;
         tSetpoint.target_system    = CRobot::GetInstance().GetPixhawk().GetTargetSystem().value();
         tSetpoint.target_component = CRobot::GetInstance().GetPixhawk().GetTargetComponent().value();
         tSetpoint.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
				   		          MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
         tSetpoint.coordinate_frame = MAV_FRAME_LOCAL_NED;
         tSetpoint.vx = fTargetVelocity.GetX();
         tSetpoint.vy = fTargetVelocity.GetY();
         tSetpoint.vz = fTargetVelocity.GetZ();
          /* tSetpoint.x = fTargetPosition.GetX() + cInitialPosition.GetX();
         tSetpoint.y = fTargetPosition.GetY() + cInitialPosition.GetY();
         tSetpoint.z = fTargetPosition.GetZ() + cInitialPosition.GetZ(); */
         /* ENU to NED */
         /* @Sinan TODO: double check the sign of m_cTargetYawAngle with real tests */
          // tSetpoint.yaw = -m_cTargetYawAngle.GetValue() + cInitialOrientation.GetZ();
         tSetpoint.yaw_rate = -m_cTargetYawRate.GetValue(); 
         mavlink_message_t tMessage;
         mavlink_msg_set_position_target_local_ned_encode(unTargetSystem, 0, &tMessage, &tSetpoint);
         try {
            Write(tMessage);
         }
         catch(CARGoSException& ex) {
            LOGERR << "[ERROR] Could not write setpoint: " 
                   << ex.what()
                   << std::endl;
         }
      }
      else {
         LOGERR << "[WARNING] "
                << "Attempt to write setpoint before Pixhawk was ready"
                << std::endl;
      }
   }

   /****************************************/
   /****************************************/

   bool CDroneFlightSystemDefaultActuator::Ready() {
      return CRobot::GetInstance().GetPixhawk().Ready();
   }

   /****************************************/
   /****************************************/

   void CDroneFlightSystemDefaultActuator::Arm(bool b_arm, bool b_bypass_safety_checks) {
      if(CRobot::GetInstance().GetPixhawk().Ready()) {
         /* build commmand for arming/disarming the drone */
         mavlink_command_long_t tCommand = {0};
         tCommand.target_system    = CRobot::GetInstance().GetPixhawk().GetTargetSystem().value();
         tCommand.target_component = CRobot::GetInstance().GetPixhawk().GetTargetComponent().value();
         tCommand.command          = MAV_CMD_COMPONENT_ARM_DISARM;
         tCommand.confirmation     = 1;
         tCommand.param1           = b_arm ? 1.0f : 0.0f;
         tCommand.param2           = b_bypass_safety_checks ? 21196.0f : 0.0f;
         /* encode the message */
         mavlink_message_t tMessage;
         mavlink_msg_command_long_encode(CRobot::GetInstance().GetPixhawk().GetTargetSystem().value(),
                                         0,
                                         &tMessage,
                                         &tCommand);
         /* send the message */
         try {
            Write(tMessage);
         }
         catch(CARGoSException& ex) {
            std::string strError("Could not ");
            strError += b_arm ? "arm" : "disarm";
            strError += " drone";
            if(b_arm) {
               /* abort the program */
               THROW_ARGOSEXCEPTION_NESTED(strError, ex);
            }
            else {
               LOGERR << "[ERROR] " << strError << std::endl;
               LOGERR << "[ERROR] " << ex.what() << std::endl;
            }
         }
      }
      else {
         LOGERR << "[WARNING] "
                << "Attempt to arm/disarm drone before Pixhawk was ready"
                << std::endl;
      }
   }

   /****************************************/
   /****************************************/

   void CDroneFlightSystemDefaultActuator::SetOffboardMode(bool b_offboard_mode) {
      if(CRobot::GetInstance().GetPixhawk().Ready()) {
         /* build commmand for entering/exiting off-board mode */
         mavlink_command_long_t tCommand = {0};
         tCommand.target_system    = CRobot::GetInstance().GetPixhawk().GetTargetSystem().value();
         tCommand.target_component = CRobot::GetInstance().GetPixhawk().GetTargetComponent().value();
         tCommand.command          = MAV_CMD_NAV_GUIDED_ENABLE;
         tCommand.confirmation     = 1;
         tCommand.param1           = b_offboard_mode ? 1.0f : 0.0f;
         /* encode the message */
         mavlink_message_t tMessage;
         mavlink_msg_command_long_encode(CRobot::GetInstance().GetPixhawk().GetTargetSystem().value(),
                                         0,
                                         &tMessage,
                                         &tCommand);
         /* send the message */
         try {
            Write(tMessage);
         }
         catch(CARGoSException& ex) {
            std::string strError("Could not ");
            strError += b_offboard_mode ? "enter" : "exit";
            strError += " off-board mode";
            if(b_offboard_mode) {
               /* abort the program */
               THROW_ARGOSEXCEPTION_NESTED(strError, ex);
            }
            else {
               LOGERR << "[ERROR] " << strError << std::endl;
               LOGERR << "[ERROR] " << ex.what() << std::endl;
            }
         }
      }
      else {
         LOGERR << "[WARNING] "
                << "Attempt to enter/exit off-board mode before Pixhawk was ready"
                << std::endl;
      }
   }

   /****************************************/
   /****************************************/

   void CDroneFlightSystemDefaultActuator::Write(const mavlink_message_t& t_message) {
      /* write message to buffer */
      std::array<uint8_t, 256> arrBuffer;
      uint16_t unLength = ::mavlink_msg_to_send_buffer(arrBuffer.data(), &t_message);
      /* write message to Pixhawk */
      ssize_t nResult = 
         ::write(CRobot::GetInstance().GetPixhawk().GetFileDescriptor(),
                 reinterpret_cast<char*>(arrBuffer.data()),
                 unLength);
      if(nResult == unLength) {
         /* wait until all data has been written */
         ::tcdrain(CRobot::GetInstance().GetPixhawk().GetFileDescriptor());
      }
      else {
         THROW_ARGOSEXCEPTION("Could not write command to Pixhawk: " << ::strerror(errno));
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_ACTUATOR(CDroneFlightSystemDefaultActuator,
                     "drone_flight_system", "default",
                     "Michael Allwright [allsey87@gmail.com]",
                     "1.0",
                     "The drone flight system actuator.",
                     "This actuator writes data to a PX4 flight controller.",
                     "Usable"
   );

   /****************************************/
   /****************************************/
   
}
