/**
 * proxy - Sample application to encapsulate HW/SW interfacing with embedded systems.
 * Copyright (C) 2012 - 2015 Christian Berger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <ctype.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include <string>

#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/odcore/data/TimeStamp.h"


//#include "OpenCVCamera.h"

/*#ifdef HAVE_UEYE
    #include "uEyeCamera.h"
#endif*/

#include "Proxy.h"

namespace automotive {
    namespace miniature {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
      //  using namespace odtools::recorder;
        using namespace odcore;
		using namespace odcore::wrapper;

        Proxy::Proxy(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "proxy"),
            serial(0),
            laneFollower(0),
            overtaking(0)
            {
			//serial=NULL;
			//cerr << "go go " << endl;
			}

            //m_recorder(),
            //m_camera()

        Proxy::~Proxy() {
        }

        void Proxy::setUp() {
			for(int i=0;i<2;i++){
				try {

					std::stringstream portStream;
					portStream<<SERIAL_PORT<<i;
					std::string port=portStream.str();
					serial= std::shared_ptr<SerialPort> (SerialPortFactory::createSerialPort(port, BAUD_RATE));

					const uint32_t ONE_SECOND = 1000 * 1000;
					odcore::base::Thread::usleepFor( ONE_SECOND);
					break;
				}catch(string &exception) {
					cerr << "Serial port could not be created: " << exception << endl;
				}

			}


			try{
				laneFollower = std::shared_ptr<SharedMemory> (SharedMemoryFactory::createSharedMemory("1", BUFFER_SIZE));
				overtaking = std::shared_ptr<SharedMemory> (SharedMemoryFactory::createSharedMemory("sensorMemory", BUFFER_SIZE));
				{
					odcore::base::Lock o(overtaking);
					char *p = static_cast<char*>(overtaking->getSharedMemory());
					for(int i =0;i<BUFFER_SIZE;i++) 
						p[i]=0;
				}
				std::shared_ptr<SharedMemory> sharedMemory(SharedMemoryFactory::createSharedMemory("dsads", 10));
				if (overtaking->isValid()) {
					cerr<<"valid memory \n";

				}
			}catch(string &exception) {
				cerr << "Shared memory could not created: " << exception << endl;
			}


            // Get configuration data.
         //   KeyValueConfiguration kv = getKeyValueConfiguration();


        }

        void Proxy::tearDown() {
			cerr << "go go " << endl;
            // This method will be call automatically _after_ return from body().
           // delete serial;
            //delete laneFollower;
            //delete overtaking;
        }



        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Proxy::body() {
			//cerr << "go go " << endl;
            uint32_t captureCounter = 0;
				SerialRead handler;
				serial->setStringListener(&handler);

				/*std::stringstream bufferStream;
					bufferStream<<(char)captureCounter;
					std::string output=bufferStream.str();
					serial->send(output);
				if(captureCounter<31){
					captureCounter++;
				}else{
					captureCounter=0;
				}*/





			serial->start();


				while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
					{
					odcore::base::Lock o(overtaking);
					char *p = static_cast<char*>(overtaking->getSharedMemory());
					/*
					 * Do something with the serial data
					 * */
					string serialInput=handler.readstr();
					//char input;
					//if(serialInput.size()>0){ 
						
						for (unsigned int i = 1; i < 7; i++){
							if(handler.get(i) != 0){
							//input = serialInput.at(0);
							cerr<<"Id = " << (int)((handler.get(i) >> 5) & 0x07) << " Value = " << (int)(handler.get(i)&31)*2 << endl;
						
						//odcore::base::Lock o(overtaking); 
						//char *p = static_cast<char*>(overtaking->getSharedMemory());
						//if( (int)((input >> 5) & 0x07) == 1)
						//cerr << "input = " << (int)input << endl;

						//char *p = static_cast<char*>(overtaking->getSharedMemory());
						//cerr << "## Input = " << input << endl;		
						//if( (int)((input >> 5) & 0x07) == 1)			
						p[(int)((handler.get(i) >> 5) & 0x07)]=handler.get(i)&31;
						//}
						}
					}

					if(p[0]!=0){
						//odcore::base::Lock o(overtaking);
						std::stringstream stream;
						stream<<p[0];
						std::string value=stream.str();
						serial->send(value);
						//p[0]=0;
						cerr << "sending " << value << endl;
					}
					}//Scope lock
					odcore::base::Thread::usleepFor(1000);

				}

            serial->stop();
			serial->setStringListener(NULL);

            cout << "Proxy: Captured " << captureCounter << " frames." << endl;

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    }
} // automotive::miniature
