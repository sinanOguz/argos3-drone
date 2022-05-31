/**
 * @file <argos3/plugins/robots/drone/hardware/drone_flight_system_default_sensor.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 * @author Sinan Oguz - <soguz.ankara@gmail.com>
 */

#include "drone_flight_system_default_sensor.h"
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/drone/hardware/robot.h>
#include <argos3/plugins/robots/drone/hardware/pixhawk.h>

namespace argos {

   /****************************************/
   /****************************************/ 

   void CDroneFlightSystemDefaultSensor::Init(TConfigurationNode& t_tree) {
      try {
         CCI_DroneFlightSystemSensor::Init(t_tree);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in the drone flight system sensor.", ex);
      }
   }

   /****************************************/
   /****************************************/
   
   void CDroneFlightSystemDefaultSensor::Update() {
      /* read and decode all messages */
      while(std::optional<mavlink_message_t> tMessage = Read()) {
         /* initialize the target system and component identifiers if not already done */
         if(!CRobot::GetInstance().GetPixhawk().GetTargetSystem()) {
            CRobot::GetInstance().GetPixhawk().GetTargetSystem().emplace(tMessage.value().sysid);
         }
         if(!CRobot::GetInstance().GetPixhawk().GetTargetComponent()) {
            CRobot::GetInstance().GetPixhawk().GetTargetComponent().emplace(tMessage.value().compid);
         }
         Decode(tMessage.value());
      }
      /* update readings with the latest data */
      if(m_tHighResImu) {
         const mavlink_highres_imu_t& tReading =
            m_tHighResImu.value();
         m_cAccelerometer.Set(tReading.xacc, tReading.yacc, tReading.zacc);
         m_cGyroscope.Set(tReading.xgyro, tReading.ygyro, tReading.zgyro);
         m_cMagnetometer.Set(tReading.xmag, tReading.ymag, tReading.zmag);
         m_fTemperature = tReading.temperature;
         /* clear out the read data */
         m_tHighResImu.reset();
      }
      if (m_tDistanceSensorData) {
         const mavlink_distance_sensor_t &tReading =
             m_tDistanceSensorData.value();
         m_fHeight = tReading.current_distance;
         /* clear out the read data */
         m_tDistanceSensorData.reset();
      }
      if(m_tLocalPositionNed) {
         const mavlink_local_position_ned_t& tReading =
            m_tLocalPositionNed.value();
         CVector3 cLocalPositionNed(tReading.x, tReading.y, tReading.z);
         CVector3 cLocalVelocityNed(tReading.vx, tReading.vy, tReading.vz);
         /* set the initial position if not already set, it should be in NED*/
         if(!CRobot::GetInstance().GetPixhawk().GetInitialPosition()) {
            CRobot::GetInstance().GetPixhawk().GetInitialPosition().emplace(cLocalPositionNed);
         }
         if(CRobot::GetInstance().GetPixhawk().GetInitialOrientation()) {
            CVector3& cInitialOrientation = CRobot::GetInstance().GetPixhawk().GetInitialOrientation().value();
            CVector3& cInitialPosition = CRobot::GetInstance().GetPixhawk().GetInitialPosition().value();
            /*ToDo: don't do the following, instead remove the initial position sum on the actuator side*/
            cLocalPositionNed = cLocalPositionNed - cInitialPosition;
            /* NED to ENU */
            cLocalPositionNed.RotateZ(CRadians(-cInitialOrientation.GetZ())); 
            cLocalVelocityNed.RotateZ(CRadians(-cInitialOrientation.GetZ()));
            m_cPosition.Set(cLocalPositionNed.GetX(), -cLocalPositionNed.GetY(), -cLocalPositionNed.GetZ());
            m_cVelocity.Set(cLocalVelocityNed.GetX(), -cLocalVelocityNed.GetY(), -cLocalVelocityNed.GetZ());
         }
         /* clear out the read data */
         m_tLocalPositionNed.reset();
      }
      if (m_tPositionTargetLocalNed) {
         if(CRobot::GetInstance().GetPixhawk().GetInitialOrientation()) {
            const mavlink_position_target_local_ned_t &tReading =
                m_tPositionTargetLocalNed.value();
            CVector3 cTargetPosition(tReading.x, tReading.y, tReading.z);
            CVector3 cTargetVelocity(tReading.vx, tReading.vy, tReading.vz);
            CVector3& cInitialOrientation = CRobot::GetInstance().GetPixhawk().GetInitialOrientation().value();
            /* NED to ENU */
            cTargetPosition.RotateZ(CRadians(-cInitialOrientation.GetZ()));
            cTargetVelocity.RotateZ(CRadians(-cInitialOrientation.GetZ()));
            m_cTargetPosition.Set(cTargetPosition.GetX(), -cTargetPosition.GetY(), -cTargetPosition.GetZ());
            m_cTargetVelocity.Set(cTargetVelocity.GetX(), -cTargetVelocity.GetY(), -cTargetVelocity.GetZ());
         }
         /* clear out the read data */
         m_tPositionTargetLocalNed.reset();
      }
      if(m_tAttitude) {
         const mavlink_attitude_t& tReading =
            m_tAttitude.value();
         m_cOrientation.Set(tReading.roll, tReading.pitch, tReading.yaw);
         /* set the initial orientation if not already set */
         if(!CRobot::GetInstance().GetPixhawk().GetInitialOrientation()) {
            CRobot::GetInstance().GetPixhawk().GetInitialOrientation().emplace(m_cOrientation);
         }
         /* NED to ENU */
         /* @Sinan TODO: double check the signs of m_cOrientation values with real tests */
         m_cOrientation.Set(tReading.roll, -tReading.pitch, -tReading.yaw);
         /* @Sinan TODO: double check the signs of m_cAngularVelocity values with real tests */
         m_cAngularVelocity.Set(tReading.rollspeed,
                                -tReading.pitchspeed,
                                -tReading.yawspeed);
         /* clear out the read data */
         m_tAttitude.reset();
      }
      if (m_tAttitudeTarget) {
         const mavlink_attitude_target_t &tReading =
             m_tAttitudeTarget.value();
         CQuaternion cTargetOrientation(tReading.q[0], tReading.q[1], tReading.q[2], tReading.q[3]);
         CRadians cYaw, cPitch, cRoll;
         cTargetOrientation.ToEulerAngles(cYaw, cPitch, cRoll);
         /* NED to ENU */
         m_cTargetOrientation.Set(cRoll.GetValue(), -cPitch.GetValue(), -cYaw.GetValue());
         /* clear out the read data */
         m_tAttitudeTarget.reset();
      }
      if(m_tBatteryStatus) {
         const mavlink_battery_status_t& tReading =
            m_tBatteryStatus.value();
         m_fBatteryVoltage = tReading.voltages[0];
         /* clear out the read data */
         m_tBatteryStatus.reset();
      }
   }

   /****************************************/
   /****************************************/

   void CDroneFlightSystemDefaultSensor::Decode(const mavlink_message_t& t_message) {
      switch (t_message.msgid) {
      case MAVLINK_MSG_ID_HEARTBEAT:
         m_tHeartbeat.emplace();
         ::mavlink_msg_heartbeat_decode(
            &t_message, &m_tHeartbeat.value());
         break;
      case MAVLINK_MSG_ID_SYS_STATUS:
         m_tSystemStatus.emplace();
         ::mavlink_msg_sys_status_decode(
            &t_message, &m_tSystemStatus.value());
         break;
      case MAVLINK_MSG_ID_BATTERY_STATUS:
         m_tBatteryStatus.emplace();
         ::mavlink_msg_battery_status_decode(
            &t_message, &m_tBatteryStatus.value());
         break;
      case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
         m_tLocalPositionNed.emplace();
         ::mavlink_msg_local_position_ned_decode(
            &t_message, &m_tLocalPositionNed.value());
         break;
      case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
         m_tPositionTargetLocalNed.emplace();
         ::mavlink_msg_position_target_local_ned_decode(
            &t_message, &m_tPositionTargetLocalNed.value());
         break;
      case MAVLINK_MSG_ID_HIGHRES_IMU:
         m_tHighResImu.emplace();
         ::mavlink_msg_highres_imu_decode(
            &t_message, &m_tHighResImu.value());
         break;
      case MAVLINK_MSG_ID_ATTITUDE:
         m_tAttitude.emplace();
         ::mavlink_msg_attitude_decode(
            &t_message, &m_tAttitude.value());
         break;
      case MAVLINK_MSG_ID_ATTITUDE_TARGET:
         m_tAttitudeTarget.emplace();
         ::mavlink_msg_attitude_target_decode(
             &t_message, &m_tAttitudeTarget.value());
         break;
      case MAVLINK_MSG_ID_DISTANCE_SENSOR:
         m_tDistanceSensorData.emplace();
         ::mavlink_msg_distance_sensor_decode(
             &t_message, &m_tDistanceSensorData.value());
         break;
      default:
         // LOG << "[INFO] Unknown message of type " << t_message.msgid << " received";
         break;
      }
   }

   /****************************************/
   /****************************************/

   std::optional<mavlink_message_t> CDroneFlightSystemDefaultSensor::Read() {
      /* only attempt to read if the connect is open */
      if(CRobot::GetInstance().GetPixhawk().GetFileDescriptor() >= 0) {
         mavlink_message_t tMessage;
         mavlink_status_t tStatus;
         uint8_t unRxChar;
         while(::read(CRobot::GetInstance().GetPixhawk().GetFileDescriptor(), &unRxChar, 1)) {
            switch(::mavlink_parse_char(::MAVLINK_COMM_1, unRxChar, &tMessage, &tStatus)) {
            case ::MAVLINK_FRAMING_INCOMPLETE:
               break;
            case ::MAVLINK_FRAMING_OK:
               return tMessage;
               break;
            case ::MAVLINK_FRAMING_BAD_CRC:
               LOGERR << "[WARNING] Corrupted MAVLink message received" << std::endl;
               break;
            }
         }
      }
      return std::nullopt;
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CDroneFlightSystemDefaultSensor,
                   "drone_flight_system", "default",
                   "Michael Allwright [allsey87@gmail.com]",
                   "1.0",
                   "The drone flight system sensor.",
                   "This sensor reads data from a PX4 flight controller.",
                   "Usable"
   );

   /****************************************/
   /****************************************/

}
