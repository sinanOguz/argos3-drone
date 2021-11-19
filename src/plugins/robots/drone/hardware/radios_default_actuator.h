/**
 * @file <argos3/plugins/robots/drone/hardware/radios_default_actuator.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef RADIO_DEFAULT_ACTUATOR_H
#define RADIO_DEFAULT_ACTUATOR_H

namespace argos {
   class CRadiosDefaultActuator;
   class CTCPSocket;
   class CRobot;
}

#include <argos3/plugins/robots/drone/hardware/actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_radios_actuator.h>

namespace argos {

   class CRadiosDefaultActuator : public CPhysicalActuator,
                                  public CCI_RadiosActuator {
   public:

      CRadiosDefaultActuator() {}

      virtual ~CRadiosDefaultActuator() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

   };
}

#endif
