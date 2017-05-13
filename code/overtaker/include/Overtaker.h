/**
 * overtaker - Sample application for overtaking obstacles.
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

#ifndef OVERTAKER_H_
#define OVERTAKER_H_

#include "opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h"

//NEW FROM LANEFOLLOWER
#include <memory>

#include <opencv/cv.h>

#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/wrapper/SharedMemory.h>

#include "opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h"
#include "opendavinci/odcore/data/TimeStamp.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"


//namespace automotive {
    //namespace miniature {
namespace scaledcars{

        using namespace std;

        /**
         * This class is a skeleton to send driving commands to Hesperia-light's vehicle driving dynamics simulation.
         */
        class Overtaker : public odcore::base::module::TimeTriggeredConferenceClientModule {
            private:
                /**
                 * "Forbidden" copy constructor. Goal: The compiler should warn
                 * already at compile time for unwanted bugs caused by any misuse
                 * of the copy constructor.
                 *
                 * @param obj Reference to an object of this class.
                 */
                Overtaker(const Overtaker &/*obj*/);

                /**
                 * "Forbidden" assignment operator. Goal: The compiler should warn
                 * already at compile time for unwanted bugs caused by any misuse
                 * of the assignment operator.
                 *
                 * @param obj Reference to an object of this class.
                 * @return Reference to this instance.v $(SOURCE_TREE_PATH):/opt/$(PROJECT_NAME).so
                 */
                Overtaker& operator=(const Overtaker &/*obj*/);

            public:
                /**
                 * Constructor.
                 *
                 * @param argc Number of command line arguments.
                 * @param argv Command line arguments.
                 */
                Overtaker(const int32_t &argc, char **argv);

            virtual ~Overtaker();

                odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();
            protected:
                bool readSharedImage(odcore::data::Container &c);
                int readSensorData(int sensorId);
                void sendSteeringAngle(double steeringAngle, int speed);

            private:
                virtual void setUp();
                virtual void tearDown();

            private:
                void processImage();

            private:
                bool m_hasAttachedToSharedImageMemory;
                std::shared_ptr<odcore::wrapper::SharedMemory> m_sharedImageMemory;
                IplImage *m_image;
                //cv::Mat m_image;
                bool m_debug;
                bool m_simulator;
                CvFont m_font;
                odcore::data::TimeStamp m_previousTime;
                double m_eSum;
                double m_eOld;
                int m_speed;
                bool m_stopline;
                automotive::VehicleControl m_vehicleControl;
        };

    }
//} // automotive::miniature

#endif /*OVERTAKER_H_*/
