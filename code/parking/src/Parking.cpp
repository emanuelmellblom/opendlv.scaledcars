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
					int value;
					{
						// Using a scoped lock to lock and automatically unlock a shared memory segment.
						odcore::base::Lock l(sharedMemory);
						char *p = static_cast<char*>(sharedMemory->getSharedMemory());

						//Extract the sensor value from the received byte
						value = (p[sensorId] & 31) * 2;
					}
					returnValue = value;

					break;
					// Sleep some time.
					//const uint32_t ONE_SECOND = 1000 * 1000;
				}
			}
		}
		catch (string &exception) {
			cerr << "Sensor memory could not created: " << exception << endl;
		}
		return returnValue;
	}

	void Parking::sendMotionData(double steeringAngle, int speed) {

		cerr << "steeringAngle = " << steeringAngle << endl;
		char output = 0x00;
		output = (((int)(steeringAngle) / 4) + 15) & 31;

		// Speed must be 0 - 7
		output |= (speed & 7) << 5;

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
        TimeStamp lastTime;
				double parkTimer = 0;
				sendMotionData(0, 3);
				bool parked = false;
        while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                //change these as needed- stage 2 and 4 should have same value for it to end up in a parallel position
                // const int stage1 = 200; // time needed for backing a bit
                // const int stage2 = 1200; //time needed for first curve
                // const int stage3 = 800; // time needed for the straight backwards (45 degree inclination to the lane) ( this is only needed if there is not enough y displacement)
                // const int stage4 = 1200; // time needed for second curve
                // const int stage5 = 500;
								// const int stage6 = 500;
								// working values above without extra stages for 85cm parking space
								// time needed for going straight back- later replace that with sensor reading telling it when to stop
								//optional stages for more turning
								//const int stage6,7,8 etc
								const int stage1 = 200; // adjust this when parking space is found //initially 200
                const int stage2 = 2400; //time needed for first curve	//initially 2200
                const int stage3 = 0; //initially 200
                const int stage4 = 1200; ////initially 1000// time needed for second curve
                const int stage5 = 1200;//initially 1200
								const int stage6 = 1000;//initially 1000
                //const int32_t ULTRASONIC_FRONT_CENTER = 2;
                //const int32_t ULTRASONIC_FRONT_RIGHT = 3;
                //const int32_t INFRARED_FRONT_RIGHT = 5;
                //const int32_t INFRARED_REAR_RIGHT = 1;
                const int32_t INFRARED_BACK = 4;
                //const int32_t ODOMETER = 6;

            TimeStamp currentTime;
            double deltaTime = (currentTime.toMicroseconds() - lastTime.toMicroseconds())/1000;
            lastTime = currentTime;
						parkTimer += deltaTime;

            if (parkTimer < stage1) {
              sendMotionData(0, 2);
            }
            else if (parkTimer < stage1+stage2) {
              sendMotionData(60, 2);
            }
            else if (parkTimer < stage1+stage2+stage3) {
              sendMotionData(0, 2);
            }
            else if (parkTimer < stage1+stage2+stage3+stage4) {
              sendMotionData(-60, 2);
            }
            else if (parkTimer < stage1+stage2+stage3+stage4+stage5) {
              sendMotionData(60, 4);
            }
            else if (parkTimer < stage1+stage2+stage3+stage4+stage5+stage6){
              sendMotionData(-60, 2);
            }
						else if(parkTimer > stage1+stage2+stage3+stage4+stage5+stage6 && parked == false) {
							int back = readSensorData(INFRARED_BACK);
							if (back > 10 || back == 0) {
								sendMotionData(0, 2);
							} else {
								sendMotionData(0,3);
								parked = true;
							}
						}
						else {
								sendMotionData(0,3);
						}

        }
        return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
    }
}
