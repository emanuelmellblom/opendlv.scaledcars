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

#ifndef BOXPARKER_H_
#define BOXPARKER_H_

#include <vector>

#include "opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h"

#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/wrapper/SharedMemory.h>

#include "opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h"
#include "opendavinci/odcore/data/TimeStamp.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"

namespace scaledcars {

        using namespace std;

        /**
         * This class is an example demonstrating how to park a simulated vehicle
         * using OpenDaVINCI's driving dynamics simulation.
         */

        class Parking : public odcore::base::module::TimeTriggeredConferenceClientModule {


            enum CarState { Search, Park };
            CarState state = Search;
            double currentSpaceSize = 0;
            const double minSpaceSize = 50;

            private:
                /**
                 * "Forbidden" copy constructor. Goal: The compiler should warn
                 * already at compile time for unwanted bugs caused by any misuse
                 * of the copy constructor.
                 *
                 * @param obj Reference to an object of this class.
                 */
                Parking(const Parking &/*obj*/);

                /**
                 * "Forbidden" assignment operator. Goal: The compiler should warn
                 * already at compile time for unwanted bugs caused by any misuse
                 * of the assignment operator.
                 *
                 * @param obj Reference to an object of this class.
                 * @return Reference to this instance.
                 */
                Parking& operator=(const Parking &/*obj*/);

            public:
                /**
                 * Constructor.
                 *
                 * @param argc Number of command line arguments.
                 * @param argv Command line arguments.
                 */
                Parking(const int32_t &argc, char **argv);

                virtual ~Parking();

                odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

                /**
                 * This method returns the gaps found during the parking process.
                 *
                 * return Gaps found during the parking process.
                 */
                vector<double> getFoundGaps() const;

            private:
                virtual void setUp();

                virtual void tearDown();

            private:
                vector<double> m_foundGaps;
                bool m_simulator;

		int readSensorData(int sensorId);

		void sendMotionData(double steeringAngle, int speed);
        };

}

#endif /*BOXPARKER_H_*/
