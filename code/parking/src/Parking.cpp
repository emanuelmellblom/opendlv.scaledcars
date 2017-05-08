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

#include <iostream>

#include "opendavinci/odcore/data/Container.h"

#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"

#include "Parking.h"

namespace automotive {
    namespace miniature {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::base::module;
        using namespace odcore::data;
        using namespace automotive;

        Parking::Parking(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "Parking"),
            m_foundGaps() {}

        Parking::~Parking() {}

        void Parking::setUp() {
            // This method will be call automatically _before_ running body().
        }

        void Parking::tearDown() {
            // This method will be call automatically _after_ return from body().
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
            
                // 1. Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData> ();

                // 2. Get most recent sensor board data describing virtual sensor data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

                // Create vehicle control data.
                VehicleControl vc;

                // Moving state machine.
                if (state == Search) {
                    // Go forward.
                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(0);
                    
                    // Get odometer value
                    currentSpaceSize += 1; // 1 to be replaced with time traveled since last check
                    
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
                
                    TimeStamp currentTime;
                    
                    // Deltatime may be used in the search state 
                    // for distance if only velocity is available
                    double deltaTime = (lastTime.toMicroseconds() - currentTime.toMicroseconds()) / 1000.0;
                    parkTimer += deltaTime;
                    
                    // If 
                            
                    // Stop completely
                    if (parkTimer < 1000) {
                        vc.setSpeed(0);
                        vc.setSteeringWheelAngle(0);
                    }
                    // Backwards, steering wheel to the right.
                    else if (parkTimer < 2000) {
                        vc.setSpeed(-2);
                        vc.setSteeringWheelAngle(90);
                    }
                    else if (parkTimer < 3000) {
                        vc.setSpeed(-2);
                        vc.setSteeringWheelAngle(-90);
                    }
                    // Finally, stop again
                    else if (parkTimer < 4000) {
                        vc.setSpeed(0);
                        vc.setSteeringWheelAngle(0);
                    }
                    
                    lastTime = currentTime;
                }
                // Create container for finally sending the data.
                Container c(vc);
                // Send container.
                getConference().send(c);
            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    } // miniature
} // automotive

