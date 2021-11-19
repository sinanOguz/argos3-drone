/**
 * @file <argos3/plugins/robots/drone/hardware/radios_default_sensor.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef RADIO_DEFAULT_SENSOR_H
#define RADIO_DEFAULT_SENSOR_H

namespace argos {
   class CRadiosDefaultSensor;
   class CTCPSocket;
   class CRobot;
}

#include <argos3/plugins/robots/drone/hardware/sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_radios_sensor.h>

namespace argos {

   class CRadiosDefaultSensor : public CPhysicalSensor,
                                public CCI_RadiosSensor {
   public:

      CRadiosDefaultSensor() {}

      virtual ~CRadiosDefaultSensor() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();
  
   };
}

#endif
