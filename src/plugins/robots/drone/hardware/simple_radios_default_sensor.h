/**
 * @file <argos3/plugins/robots/drone/hardware/simple_radios_default_sensor.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef SIMPLE_RADIOS_DEFAULT_SENSOR_H
#define SIMPLE_RADIOS_DEFAULT_SENSOR_H

namespace argos {
   class CSimpleRadiosDefaultSensor;
   class CTCPSocket;
   class CRobot;
}

#include <argos3/plugins/robots/drone/hardware/sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_simple_radios_sensor.h>

namespace argos {

   class CSimpleRadiosDefaultSensor : public CPhysicalSensor,
                                      public CCI_SimpleRadiosSensor {
   public:

      CSimpleRadiosDefaultSensor() {}

      virtual ~CSimpleRadiosDefaultSensor() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();
  
   };
}

#endif
