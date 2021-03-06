/**
 * Example - Example code.
 * Copyright (C) 2016 Christian Berger
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

#ifndef PERCEPTION_EXAMPLE_H
#define PERCEPTION_EXAMPLE_H

#include <memory>

#include <opencv/cv.h>

#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/wrapper/SharedMemory.h>

#include "opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h"
#include "opendavinci/odcore/data/TimeStamp.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"

namespace scaledcars {
namespace perception {

using namespace std;

/**
 * Time-triggered example.
 */
class Example : public odcore::base::module::TimeTriggeredConferenceClientModule {
   private:
    Example(const Example & /*obj*/) = delete;
    Example &operator=(const Example & /*obj*/) = delete;

   public:
    /**
     * Constructor.
     *
     * @param argc Number of command line arguments.
     * @param argv Command line arguments.
     */
    Example(const int &argc, char **argv);

    virtual ~Example();


  odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();
   protected:
   bool readSharedImage(odcore::data::Container &c);
   private:
    void setUp();
    void tearDown();

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
     automotive::VehicleControl m_vehicleControl;

  
};
}
} // scaledcars::perception

#endif /*PERCEPTION_EXAMPLE_H*/