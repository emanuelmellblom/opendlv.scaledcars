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
            double distanceOld = 0;
            double absPathStart = 0;
            double absPathEnd = 0;

            int stageMoving = 0;
            int stageMeasuring = 0;

            VehicleControl vc;

            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                // 1. Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData> ();

                // 2. Get most recent sensor board data describing virtual sensor data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

                // Create vehicle control data.

                // Moving state machine.
                if (stageMoving == 0) {
                    // Go forward.
                    vc.setSpeed(1.5);
                    vc.setSteeringWheelAngle(0);
                }
                if ((stageMoving > 0) && (stageMoving < 20)) {
                    // Move slightly forward.
                    vc.setSpeed(0);
                    vc.setSteeringWheelAngle(0);
                    stageMoving++;
                }
                if ((stageMoving >= 20) && (stageMoving < 25)) {
                    // Stop.
                    vc.setSpeed(0);
                    vc.setSteeringWheelAngle(0);
                    stageMoving++;
                }
                if ((stageMoving >= 25) && (stageMoving < 40)) {
                    // Backwards, steering wheel to the right.
                    vc.setSpeed(0);
                    vc.setSteeringWheelAngle(90);
                    stageMoving++;
                }
                if ((stageMoving >= 40) && (stageMoving < 80)) {
                    // Backwards, steering wheel to the right.
                    vc.setSpeed(-2);
                    vc.setSteeringWheelAngle(90);
                    stageMoving++;
                }
                if ((stageMoving >= 80) && (stageMoving < 108)) {
                    // Backwards, steering wheel to the right.
                    vc.setSpeed(-2);
                    vc.setSteeringWheelAngle(-90);
                    stageMoving++;
                }
                if (stageMoving >= 108) {
                    // Stop.
                    vc.setSpeed(0);
                    vc.setSteeringWheelAngle(0);

                    stageMoving++;
                }
                if (stageMoving >= 150) {
                    // End component.
                    break;
                }

                // Measuring state machine.
                switch (stageMeasuring) {
                    case 0:
                        {
                            // Initialize measurement.
                            distanceOld = sbd.getValueForKey_MapOfDistances(2);
                            stageMeasuring++;
                        }
                    break;
                    case 1:
                        {
                            // Checking for distance sequence +, -.
                            if ((distanceOld > 0) && (sbd.getValueForKey_MapOfDistances(2) < 0)) {
                                // Found distance sequence +, -.
                                stageMeasuring = 2;
                                absPathStart = vd.getAbsTraveledPath();
                            }
                            distanceOld = sbd.getValueForKey_MapOfDistances(2);
                        }
                    break;
                    case 2:
                        {
			    cerr << "stageMoving = " << stageMoving << endl;
                            // Checking for distance sequence -, +.
                            if ((distanceOld < 0) && (sbd.getValueForKey_MapOfDistances(2) > 0)) {
                                // Found distance sequence -, +.
                                stageMeasuring = 1;
                                absPathEnd = vd.getAbsTraveledPath();

                                const double GAP_SIZE = (absPathEnd - absPathStart);
                                cerr << "Size !!!! = " << GAP_SIZE << " | Turn " << vc.getSteeringWheelAngle() << endl;
                                m_foundGaps.push_back(GAP_SIZE);

                                if ((stageMoving < 1) && (GAP_SIZE > 3.5)) {
                                    stageMoving = 1;
                                }
                            }
                            distanceOld = sbd.getValueForKey_MapOfDistances(2);
                        }
                    break;
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

