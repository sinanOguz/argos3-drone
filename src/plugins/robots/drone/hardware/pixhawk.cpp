/**
 * @file <argos3/plugins/robots/drone/hardware/pixhawk.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <argos3/plugins/robots/drone/hardware/pixhawk.h>

namespace argos {

   /****************************************/
   /****************************************/

   void CPixhawk::Open(const std::string& str_device, SInt32 n_baud) {
      try {
         m_nFileDescriptor = 
            ::open(str_device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
         if (m_nFileDescriptor < 0)
            THROW_ARGOSEXCEPTION("Could not open " << str_device);
         ::fcntl(m_nFileDescriptor, F_SETFL, 0);
         if(!::isatty(m_nFileDescriptor)) {
            THROW_ARGOSEXCEPTION(str_device << " is not a serial port");
         }
         struct termios sPortConfiguration;
         if(::tcgetattr(m_nFileDescriptor, &sPortConfiguration) < 0) {
            THROW_ARGOSEXCEPTION("Could not read port configuration");
         }
         sPortConfiguration.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
         sPortConfiguration.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
#ifdef OLCUC
         sPortConfiguration.c_oflag &= ~OLCUC;
#endif
#ifdef ONOEOT
         sPortConfiguration.c_oflag &= ~ONOEOT;
#endif
         sPortConfiguration.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
         sPortConfiguration.c_cflag &= ~(CSIZE | PARENB);
         sPortConfiguration.c_cflag |= CS8;
         /* always return immediately regardless of whether a character was available or not */
         sPortConfiguration.c_cc[VMIN]  = 0;
         sPortConfiguration.c_cc[VTIME] = 0;
         /* apply baudrate */
         ::speed_t nBaudRate = B0;
         switch(n_baud) {
            case 50: nBaudRate = B50; break;
            case 75: nBaudRate = B75; break;
            case 110: nBaudRate = B110; break;
            case 134: nBaudRate = B134; break;
            case 150: nBaudRate = B150; break;
            case 200: nBaudRate = B200; break;
            case 300: nBaudRate = B300; break;
            case 600: nBaudRate = B600; break;
            case 1200: nBaudRate = B1200; break;
            case 1800: nBaudRate = B1800; break;
            case 2400: nBaudRate = B2400; break;
            case 4800: nBaudRate = B4800; break;
            case 9600: nBaudRate = B9600; break;
            case 19200: nBaudRate = B19200; break;
            case 38400: nBaudRate = B38400; break;
            case 57600: nBaudRate = B57600; break;
            case 115200: nBaudRate = B115200; break;
            case 230400: nBaudRate = B230400; break;
            case 460800: nBaudRate = B460800; break;
            case 500000: nBaudRate = B500000; break;
            case 576000: nBaudRate = B576000; break;
            case 921600: nBaudRate = B921600; break;
            case 1000000: nBaudRate = B1000000; break;
            case 1152000: nBaudRate = B1152000; break;
            case 1500000: nBaudRate = B1500000; break;
            case 2000000: nBaudRate = B2000000; break;
            case 2500000: nBaudRate = B2500000; break;
            case 3000000: nBaudRate = B3000000; break;
            case 3500000: nBaudRate = B3500000; break;
            case 4000000: nBaudRate = B4000000; break;
            default: THROW_ARGOSEXCEPTION(n_baud << " is not a recognized baudrate"); break;
         }
         if (::cfsetispeed(&sPortConfiguration, nBaudRate) < 0 || ::cfsetospeed(&sPortConfiguration, nBaudRate) < 0) {
            THROW_ARGOSEXCEPTION("Could not set baudrate to " << n_baud);
         }
         if(::tcsetattr(m_nFileDescriptor, TCSAFLUSH, &sPortConfiguration) < 0) {
            THROW_ARGOSEXCEPTION("Could not write port configuration");
         }
      }
      catch(CARGoSException &ex) {
         THROW_ARGOSEXCEPTION_NESTED("Could not open the MAVLink channel", ex);
      }
   }


   /****************************************/
   /****************************************/

   void CPixhawk::Close() {
      if(m_nFileDescriptor != -1) {
         ::close(m_nFileDescriptor);
         m_nFileDescriptor = -1;
      }
   }

   /****************************************/
   /****************************************/

   int CPixhawk::GetFileDescriptor() {
      return m_nFileDescriptor;
   }

   /****************************************/
   /****************************************/

   std::optional<CVector3>& CPixhawk::GetInitialPosition() {
      return m_optInitialPosition;
   }

   /****************************************/
   /****************************************/

   std::optional<CVector3>& CPixhawk::GetInitialOrientation() {
      return m_optInitialOrientation;
   }

   /****************************************/
   /****************************************/

   std::optional<uint8_t>& CPixhawk::GetTargetSystem() {
      return m_optTargetSystem;

   }

   /****************************************/
   /****************************************/

   std::optional<uint8_t>& CPixhawk::GetTargetComponent() {
      return m_optTargetComponent;
   }

   /****************************************/
   /****************************************/

   bool CPixhawk::Ready() {
      return (m_nFileDescriptor != -1) &&
             m_optInitialPosition &&
             m_optInitialOrientation &&
             m_optTargetSystem &&
             m_optTargetComponent;
   }

   /****************************************/
   /****************************************/

}
