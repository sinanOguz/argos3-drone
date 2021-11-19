/**
 * @file <argos3/plugins/robots/drone/hardware/pixhawk.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef PIXHAWK_H
#define PIXHAWK_H

#include <optional>

#include <argos3/core/utility/math/vector3.h>

namespace argos {

   class CPixhawk {
   public:

      CPixhawk() :
         m_nFileDescriptor(-1) {}

      void Open(const std::string& str_device, SInt32 n_baud);

      void Close();

      int GetFileDescriptor();

      std::optional<CVector3>& GetInitialPosition();

      std::optional<CVector3>& GetInitialOrientation();

      std::optional<uint8_t>& GetTargetSystem();

      std::optional<uint8_t>& GetTargetComponent();
      
      /* indicates that the sensor side is up and running
         and has initialized what is necessary for flight */
      bool Ready();

   private:

      int m_nFileDescriptor;

      std::optional<uint8_t> m_optTargetSystem;
      std::optional<uint8_t> m_optTargetComponent;
      std::optional<CVector3> m_optInitialPosition;
      std::optional<CVector3> m_optInitialOrientation;
   };
}

#endif
