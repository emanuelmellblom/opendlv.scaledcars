

#ifndef PROXY_H_
#define PROXY_H_

#include <map>
#include <memory>

#include "opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h"
//#include "opendavinci/odcore/data/Container.h"
//#include "opendavinci/odtools/recorder/Recorder.h"

#include <stdint.h>
#include <iostream>
#include <string>
#include <memory>


#include <opendavinci/odcore/base/Lock.h>
#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/wrapper/SharedMemory.h>
#include <opendavinci/odcore/wrapper/SharedMemoryFactory.h>



#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>
#include "SerialRead.h"


#define SERIAL_PORT "/dev/ttyACM"
#define BAUD_RATE 9600

#define BUFFER_SIZE 8

//#include "Camera.h"

namespace automotive {
    namespace miniature {

        using namespace std;
        using namespace odcore::wrapper;

        /**
         * This class wraps the software/hardware interface board.
         */
        class Proxy : public odcore::base::module::TimeTriggeredConferenceClientModule{
            private:
				
                Proxy(const Proxy &/*obj*/);

             
                Proxy& operator=(const Proxy &/*obj*/);

            public:
             
                Proxy(const int32_t &argc, char **argv);
                
                virtual ~Proxy();

                odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

            private:
                virtual void setUp();
                virtual void tearDown();

                void distribute(odcore::data::Container c);

            private:
				std::shared_ptr<SerialPort> serial;
				std::shared_ptr<SharedMemory> laneFollower;
				std::shared_ptr<SharedMemory> overtaking;
				
              //  unique_ptr<odtools::recorder::Recorder> m_recorder;
               // unique_ptr<Camera> m_camera;
        };

    }
} // automotive::miniature

#endif /*PROXY_H_*/
