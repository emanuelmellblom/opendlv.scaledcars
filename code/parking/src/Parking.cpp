/**
 * boxparker - Sample application for realizing a box parking car.
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

#include <stdint.h>
#include <list>
#include <opendavinci/odcore/base/Thread.h>

#include <iostream>
#include <cstring>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <opendavinci/odcore/base/Lock.h>
#include <opendavinci/odcore/wrapper/SharedMemoryFactory.h>

#include <opendavinci/GeneratedHeaders_OpenDaVINCI.h>
#include <automotivedata/GeneratedHeaders_AutomotiveData.h>

#include <odvdscaledcarsdatamodel/generated/chalmersrevere/scaledcars/ExampleMessage.h>

#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "opendavinci/odcore/data/TimeStamp.h"
#include "Parking.h"

#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/data/Container.h"

//
#include <cstdio>
#include <cmath>
////


namespace scaledcars {

	using namespace std;
	using namespace odcore::base;
	using namespace odcore::data;
	using namespace odcore;
	using namespace odcore::wrapper;
	using namespace automotive;
	using namespace automotive::miniature;

	using namespace odcore;
	using namespace odcore::wrapper;

	Parking::Parking(const int32_t &argc, char **argv) :
		TimeTriggeredConferenceClientModule(argc, argv, "Parking"),
		m_foundGaps(),
		m_simulator(false) {} //Set m_simulator to true if simulator is used and false otherwise.

    Parking::~Parking() {}

    void Parking::setUp() {
        // This method will be call automatically _before_ running body().
    }

    void Parking::tearDown() {
        // This method will be call automatically _after_ return from body().
    }


	int Parking::readSensorData(int sensorId) {
		const string NAME = "sensorMemory";
		int returnValue;

		// We are using OpenDaVINCI's std::shared_ptr to automatically release any acquired resources.
		try {
			std::shared_ptr<SharedMemory> sharedMemory(SharedMemoryFactory::attachToSharedMemory(NAME));

			if (sharedMemory->isValid()) {
				uint32_t counter = 30;
				while (counter-- > 0) {
					int id;
					int value;
					{
						// Using a scoped lock to lock and automatically unlock a shared memory segment.
						odcore::base::Lock l(sharedMemory);
						char *p = static_cast<char*>(sharedMemory->getSharedMemory());

						//Extract the sensor ID from the received byte
						id = p[sensorId] >> 5 & 0x03;
						//Extract the sensor value from the received byte
						value = p[sensorId] & 31;
					}
					if (id == sensorId) {
						returnValue = value;
						break;
					}
					// Sleep some time.
					//const uint32_t ONE_SECOND = 1000 * 1000;
					odcore::base::Thread::usleepFor(1000);
				}
			}
		}
		catch (string &exception) {
			cerr << "Sensor memory could not created: " << exception << endl;
		}
		return returnValue;
	}

	void Parking::sendMotionData(double steeringAngle, int speed) {

		cerr << "org = " << steeringAngle << endl;
		double steeringAngleDegrees = ((steeringAngle * 180) / M_PI);
		cerr << "steeringAngle = " << steeringAngleDegrees << endl;
		char output = 0x00;
		output = (((int)(steeringAngleDegrees) / 4) + 15) & 31;

		// Speed must be 0 - 7
		output |= speed << 5;

		const string NAME = "sensorMemory";
		try {
			std::shared_ptr<SharedMemory> sharedMemory(SharedMemoryFactory::attachToSharedMemory(NAME));
			if (sharedMemory->isValid()) {
				{
					odcore::base::Lock l(sharedMemory);
					char *p = static_cast<char*>(sharedMemory->getSharedMemory());
					p[0] = output;
				}
			}
		}
		catch (string &exception) {
			cerr << "sharedMemory not created " << exception << endl;
		}
	}


    vector<double> Parking::getFoundGaps() const {
        return m_foundGaps;
    }
        
    // This method will do the main data processing job.
    odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Parking::body() {
        
        // Used to step through the hardcoded parking sequence
        // Measured in milliseconds
        double parkTimer = 0;
        TimeStamp lastTime;
        
        while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
            
                //const int32_t ULTRASONIC_FRONT_CENTER = 2;
                //const int32_t ULTRASONIC_FRONT_RIGHT = 3;
                const int32_t INFRARED_FRONT_RIGHT = 5;
                const int32_t INFRARED_REAR_RIGHT = 1;
                //const int32_t INFRARED_BACK = 1;
                const int32_t ODOMETER = 6;
            
            // 1. Get most recent vehicle data:
            //Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
            //VehicleData vd = containerVehicleData.getData<VehicleData> ();

            // 2. Get most recent sensor board data describing virtual sensor data:
            Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
            SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();
            TimeStamp currentTime;
            
            double deltaTime = (lastTime.toMicroseconds() - currentTime.toMicroseconds()) / 1000.0;
                    
            lastTime = currentTime;

            // Create vehicle control data.
            VehicleControl vc;

            // Moving state machine.
            if (state == Search) {
                // Go forward.
				/*if (m_simulator) {
					vc.setSpeed(1);
					vc.setSteeringWheelAngle(0);
				}
				else {*/
					sendMotionData(0, 5);
				//}
                    
                // Get odometer value - probably approx in cm
                currentSpaceSize += sbd.getValueForKey_MapOfDistances(ODOMETER) * deltaTime / 1000;
                    
                // Check if an object is blocking the space.
                // If it is, reset space size
                if(sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) < 7.2 && sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT) < 7.2){
                    //goForward = false;
                    cerr << "Object detected" << endl;
                    currentSpaceSize = 0;
                }
                    
                // If space size is big enough, start parking
                if(currentSpaceSize > minSpaceSize){
                    state = Park;
                }
            }
            // Parking
            else{               
                
                parkTimer += deltaTime;
                    
                // If 
                            
                // Stop completely
                if (parkTimer < 1000) {
					/*if (m_simulator) {
						vc.setSpeed(0);
						vc.setSteeringWheelAngle(0);
					}
					else {*/
						sendMotionData(0, 3);
						// sendSpeed(0)
					// }
                }
                // Backwards, steering wheel to the right.
                else if (parkTimer < 2000) {
					/*if (m_simulator) {
						vc.setSpeed(-2);
						vc.setSteeringWheelAngle(90);
					}
					else {*/
						sendMotionData(90, 0);
						// sendSpeed(-2)
					// }
                }
                else if (parkTimer < 3000) {
					/*if (m_simulator) {
						vc.setSpeed(-2);
						vc.setSteeringWheelAngle(-90);
					}
					else {*/
						sendMotionData(-30, 0);
						// sendSpeed(-2)
					// }
                }
                // Finally, stop again
                else if (parkTimer < 4000) {
					// if (m_simulator) {
					// 	vc.setSpeed(0);
					// 	vc.setSteeringWheelAngle(0);
					// }
					// else {
						sendMotionData(0, 3);
						// sendSpeed(0)
					// }
                }
				// Maybe move forward until front sensor detects something?
            }
            // Create container for finally sending the data.
            // Container c(vc);
            // // Send container.
            // getConference().send(c);
        }

        return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
    }

}
