/**
 * @file <argos3/plugins/robots/drone/hardware/simple_radios_default_actuator.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef SIMPLE_RADIOS_DEFAULT_ACTUATOR_H
#define SIMPLE_RADIOS_DEFAULT_ACTUATOR_H

namespace argos {
   class CSimpleRadiosDefaultActuator;
   class CTCPSocket;
   class CRobot;
}

#include <argos3/plugins/robots/drone/hardware/actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_simple_radios_actuator.h>

namespace argos {

   class CSimpleRadiosDefaultActuator : public CPhysicalActuator,
                                        public CCI_SimpleRadiosActuator {
   public:

      CSimpleRadiosDefaultActuator() {}

      virtual ~CSimpleRadiosDefaultActuator() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

   };
}

#endif
