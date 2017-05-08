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

#include <cstdio>
#include <cmath>

#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/data/Container.h"

#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"

#include "Overtaker.h"

//NEW FROM LANEFOLLOWING 
#include <stdint.h>
#include <list>
 #include <opendavinci/odcore/base/Thread.h>

#include <iostream>
#include <cstring>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <opendavinci/odcore/base/Lock.h>
#include <opendavinci/odcore/wrapper/SharedMemoryFactory.h>

#include "odvdscaledcarsdatamodel/generated/chalmersrevere/scaledcars/ExampleMessage.h"

#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "opendavinci/odcore/data/TimeStamp.h"
//----

namespace scaledcars{

    using namespace std;
    using namespace odcore::base;
    using namespace odcore::data;
    using namespace odcore;
    using namespace odcore::wrapper;
    using namespace automotive;
    using namespace automotive::miniature;

//NEW FROM LANEFOLLOWER
using namespace odcore::data::image;
double steering;
int32_t distance = 90; //280
//------


        //NEW FROM LANEFOLLOWING

        Overtaker::Overtaker(const int32_t &argc, char **argv) : TimeTriggeredConferenceClientModule(argc, argv, "Overtaker"), 
            m_hasAttachedToSharedImageMemory(false),
            m_sharedImageMemory(),
            m_image(NULL),
            m_debug(true), //Have true to show detected lane markings, debugging prints and debugging camera windows
            m_simulator(true), //Set m_simulator to true if simulator is used and false otherwise.
            m_font(),
            m_previousTime(),
            m_eSum(0),
            m_eOld(0),
            m_vehicleControl() {}
        

        Overtaker::~Overtaker() {}

        void Overtaker::setUp() {
            // This method will be call automatically _before_ running body().
            cerr << "setting up" << endl;
        }

        void Overtaker::tearDown() {
            // This method will be call automatically _after_ return from body().
        }



        bool Overtaker::readSharedImage(Container &c) {
            bool retVal = false;
            //cerr << "trying to read" << endl;
             if (c.getDataType() == odcore::data::image::SharedImage::ID()) {
                SharedImage si = c.getData<SharedImage> ();

                // Check if we have already attached to the shared memory containing the image from the virtual camera.
                if (!m_hasAttachedToSharedImageMemory) {
                    m_sharedImageMemory = odcore::wrapper::SharedMemoryFactory::attachToSharedMemory(si.getName());
                    m_hasAttachedToSharedImageMemory = true;
                }

                //debug
                if(!m_sharedImageMemory->isValid()){
                    //cerr << "SharedImageMemory not valid" << endl;
                }

                // Check if we could successfully attach to the shared memory.
                if (m_sharedImageMemory->isValid()) {
                    // Lock the memory region to gain exclusive access using a scoped lock.
                    //cerr << "got an image" << endl;
                    Lock l(m_sharedImageMemory);

                    //const uint32_t numberOfChannels = 3;
                    if(m_image == NULL) {
                        //cerr << "img is null" << endl;
                        m_image = cvCreateImage(cvSize(si.getWidth(), si.getHeight()), IPL_DEPTH_8U, si.getBytesPerPixel());

                    }

                    // Example: Simply copy the image into our process space.
                    if (m_image != NULL) {
                        //cerr << "img is not null" << endl;
                        memcpy(m_image->imageData, m_sharedImageMemory->getSharedMemory(), si.getWidth() * si.getHeight() * si.getBytesPerPixel());
                    }

                    if(m_simulator){
                        cvFlip(m_image, 0, -1);
                    }
                }

                //Procees the image
                processImage();
            }

            return retVal;
        }

        int Overtaker::readSensorData(int sensorId){
            const string NAME = "sensorMemory";
            int returnValue;

            // We are using OpenDaVINCI's std::shared_ptr to automatically release any acquired resources.
            try {
                std::shared_ptr<SharedMemory> sharedMemory(SharedMemoryFactory::attachToSharedMemory(NAME));

                if (sharedMemory->isValid()) {
                    uint32_t counter = 10;
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
                        value = p[sensorId] & 0xfe;
                        }
                        if(id == sensorId){
                            returnValue = value;
                            break;
                        }
                        // Sleep some time.
                        //const uint32_t ONE_SECOND = 1000 * 1000;
                        odcore::base::Thread::usleepFor(1000);
                    }
                }
            }
            catch(string &exception) {
                cerr << "Sensor memory could not created: " << exception << endl;
            }
            return returnValue;
        }

        void Overtaker::sendSteeringAngle(int steeringAngle){

            int steeringAngleDegrees = ((steeringAngle*180)/M_PI);
            char output = 0x00;
            output = (((int)(steeringAngleDegrees)/4)+15)& 31;

            const string NAME = "sensorMemory";
            try{
                std::shared_ptr<SharedMemory> sharedMemory(SharedMemoryFactory::attachToSharedMemory(NAME));
                if (sharedMemory->isValid()) {
                    {
                    odcore::base::Lock l(sharedMemory);
                    char *p = static_cast<char*>(sharedMemory->getSharedMemory());
                    p[0] = output;
                    }
                }
                
            }
            catch(string &exception){
            cerr << "sharedMemory not created " << exception << endl;
            }
        }

        void Overtaker::processImage() {

            static bool useRightLaneMarking = true;
            double e = 0;

            int32_t CONTROL_SCANLINE = 262;// 462, (372), 252 
            
            // if(m_simulator){
            //     CONTROL_SCANLINE = 462;
            // }else{
            //     CONTROL_SCANLINE = 222;
            // }

            //const int32_t CONTROL_SCANLINE = 462; // calibrated length to right: 280px
            //const int32_t distance = 150; //280

            cv::Mat grey_image;
            if(m_image!=NULL){
                cv::Mat blured;
                cv::Mat image = cv::cvarrToMat(m_image, true, true, 0);

                //Rezise image size(width<cols>, height<rows>)
                cv::Size size(640,400);
                resize(image,image,size);

                //Convert original image to black/white
                grey_image = cv::Mat(image.cols, image.cols, CV_8UC1);
                cvtColor(image, grey_image, cv::COLOR_BGR2GRAY);

                //original: blur(grey_image, blured, cv::Size(3,3) );
                //Add blur to the image. Blur determined by the kernel size.
                GaussianBlur( grey_image, grey_image, cv::Size( 5, 5 ),0,0);

                //Apply Canny edge detection 
                Canny(grey_image, grey_image, 50, 200, 3);

                //cerr << "Image Converted" << endl;

                //TEST Contours
                // vector<vector<cv::Point>> contours;
                // vector<Vec4i> hierarchy;
                // cv::findContours(grey_image, grey_image, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, cvPoint(0,0));
                // //for (unsigned int i = 0; i < contours.size(); i++){
                // for (unsigned int i = 0; i < sizeof(contours); i++){
                //     if(hierarchy[i][3] >= 0){
                //         cv::drawContours(grey_image, grey_image, i, cv::Scalar(2,55,0,0),1,8);
                //     }
                // }


                // vector<vector<cv::Point> > contours;
                // vector<cv::Vec4i> hierarchy;
                // cv::RNG rng(12345);
                // //Mat drawing = Mat::zeros( grey_image.size(), CV_8UC3 );
                // //cv::findContours( grey_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
             
                // //cv::Mat drawing = cv::Mat::zeros( sizeof(grey_image), CV_8UC3 );
                // cv::Mat m = grey_image.clone(); cv::findContours(m, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

                // for( unsigned int i = 0; i< sizeof(contours); i++ ){
                //     cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                //     cv::drawContours( m, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
                // }
                // //END TEST Contours

                // //Show the processed image if m_debug == true
                // if(!grey_image.empty() && m_debug){

                //     cv::imshow("Processed image", m);
                // }
                cvWaitKey(10);
            }

            TimeStamp beforeImageProcessing;

            //Convert MAT grey_image to IplImage named temp.
            IplImage pretemp = grey_image;
            IplImage* temp = &pretemp;

            //TEST VOTING
            //Right Lane
            int rightLaneCount = 0;
            int missingRightLane = 0;
            //int lastRightDistance = 0;

            //Left Lane
            int leftLaneCount = 0;
            int missingLeftLane = 0;
            //int lastLeftDistance = 0;

            //cerr << lastRightDistance << endl;


            //Lane detecting algorithm
            list<CvPoint> unsorted;
            list<CvPoint> unsortedLeft;
            list<list<CvPoint>>sorted;
            sorted.push_front(unsorted);

            for(int32_t y = temp->height - 8; y > CONTROL_SCANLINE -10; y -= 5) { //-10
                CvScalar pixelLeft;
                CvPoint left; //Changed
                left.y = y;
                left.x = -1;

                // Search from middle to the left:
                for (int x = temp->width / 2; x > 0; x--) {
                    pixelLeft = cvGet2D(temp, y, x);
                    if (pixelLeft.val[0] >= 200) {
                        left.x = x;
                        break;
                    }
                }

                // Search from middle to the right:
                CvScalar pixelRight;
                CvPoint right;
                right.y = y;
                right.x = -1;
                for (int x = temp->width / 2; x < temp->width; x++) {
                    pixelRight = cvGet2D(temp, y, x);
                    if (pixelRight.val[0] >= 200) {
                        right.x = x;
                        break;
                    }
                }

                //TEST VOTING Find correct poits detected ---------------------------------------
                //Right Lane
                if (right.x > 0) {
                    //cerr << "In the storing if--------------------" << endl;
                    rightLaneCount++;
                    //lastRightDistance = right.x;
                    //int point[2] = {right.x, y};
                    //CvPoint* point;
                    //point->x = right.x;
                    //point->y = y;
                    unsorted.push_front(right);
                } else {
                    missingRightLane++;
                }

                //Left Lane
                if (left.x > 0) {
                    leftLaneCount++;
                    //lastLeftDistance = left.x;
                    unsortedLeft.push_front(left);
                } else {
                    missingLeftLane++;
                }
                //-------------------------------------------------------------------------------

                if (m_debug) {
                    if (left.x > 0) {
                        //CvScalar green = CV_RGB(0, 255, 0);
                        //cvLine(temp, cvPoint(temp->width/2, y), left, green, 1, 8);

                        stringstream sstr;
                        sstr << (temp->width / 2 - left.x);
                        //cvPutText(temp, sstr.str().c_str(), cvPoint(temp->width/2 - 100, y - 2), &m_font, green);
                        //imgProc.line(temp,(temp->width/2 - 100), y-2, new cv::Scalar(0,255,0),3);

                        cv::line(grey_image, cv::Point2i(temp->width / 2, y), left, cv::Scalar(255, 0, 0), 1, 8);


                    }
                    if (right.x > 0) {
                        //cerr << "right" << endl;
                        //CvScalar red = CV_RGB(255, 0, 0);
                        //cvLine(temp, cvPoint(temp->width/2, y), right, red, 1, 8);

                        stringstream sstr;
                        sstr << (right.x - temp->width / 2);
                        //cvPutText(temp, sstr.str().c_str(), cvPoint(temp->width/2 + 100, y - 2), &m_font, red);

                        cv::line(grey_image, cv::Point2i(temp->width / 2, y), right, cv::Scalar(255, 0, 0), 1, 8);
                    }
                }


                if (y == CONTROL_SCANLINE) {
                    // Calculate the deviation error.
                    if (right.x > 0) {
                        if (!useRightLaneMarking) {
                            m_eSum = 0;
                            m_eOld = 0;
                        }

                        e = ((right.x - temp->width / 2.0) - distance) / distance;

                        useRightLaneMarking = true;
                    } else if (left.x > 0) {
                        if (useRightLaneMarking) {
                            m_eSum = 0;
                            m_eOld = 0;
                        }

                        e = (distance - (temp->width / 2.0 - left.x)) / distance;


                        useRightLaneMarking = false;
                    } else {
                        // If no measurements are available, reset PID controller.
                        m_eSum = 0;
                        m_eOld = 0;
                    }
                    /*
                     * Comparing Measurements
                     */
                    if (rightLaneCount >= missingRightLane) {
                        CvPoint h = unsorted.front();

                        if (unsorted.size() != 0) unsorted.pop_front();
                        CvPoint t = unsorted.front();

                        if (unsorted.size() != 0) unsorted.pop_front();

                        list <CvPoint> *listPointer = &sorted.front();

                        while (unsorted.size() != 0) {
                            //cerr << "First = " << (h.x+t.x)/2 << " Second = " << (h->y+t->y)/2 << " h0 = " << h->y << " t0 = " << t->y << " h1 = " << h->x << " t1 = " << t->x << endl;
                            //CvScalar pix = cvGet2D(temp, (h[0]+t[0])/2, (h[1]+t[1])/2);
                            CvScalar pix = cvGet2D(temp, (h.y + t.y) / 2, (h.x + t.x) / 2);
                            //cerr << "beforeif" << endl;
                            if (pix.val[0] >= 200) {
                                //cerr << "########Finds a white pixel in first if########" << endl;
                                //cerr << "in if" << endl;
                                listPointer->push_back(t);
                            } else {
                                //cerr << "##does not see a white##" << endl;
                                //cerr << "in else" << endl;
                                //listPointer->push_back(t);
                                unsigned int i;
                                for (i = 0; i < sorted.size(); i++) {
                                    //cerr << "First2 = " << (h->x+t->x)/2 << " Second = " << (h->y+t->y)/2 << " h0 = " << h->y << " t0 = " << t->y << " h1 = " << h->x << " t1 = " << t->x;
                                    pix = cvGet2D(temp, (h.y + t.y) / 2, (h.x + t.x) / 2);
                                    if (pix.val[0] >= 200) {
                                        //cerr << "########Finds a white pixel in for loop ########" << endl;

                                        listPointer->push_back(t);
                                        break;
                                    } else {
                                        //list<int*> tempList = sorted.front();
                                        list <CvPoint> tempList = sorted.front();
                                        sorted.pop_front();
                                        listPointer = &sorted.front();
                                        sorted.push_back(tempList);
                                    }
                                }
                                if (i == sorted.size()) {
                                    list <CvPoint> tempList2;
                                    tempList2.push_back(t);
                                    sorted.push_back(tempList2);
                                    listPointer = &sorted.back();
                                }
                            }
                            h = t;
                            t = unsorted.front();
                            if (unsorted.size() != 0)unsorted.pop_front();
                            //else t = 0;
                        }
                        //cerr << "out of first while" << endl;
                        //list<int*> l = sorted.front();
                        list <CvPoint> l = sorted.front();
                        //cerr << "in between " << endl;
                        sorted.pop_front();
                        while (sorted.size() != 0) {
                            //cerr << "in second while" << endl;
                            if (sorted.front().size() > l.size()) {
                                l = sorted.front();
                            }
                            sorted.pop_front();
                        }
                        //cerr << "out of second while" << endl;

                        e = ((l.front().x - temp->width / 2.0) - distance) / distance;
                        // else{
                        //     e = (distance - (temp->width/2.0 - left.x))/distance;
                        // }

                        /*
                         * LEFT LANE
                         */
                    } else if (leftLaneCount >= missingLeftLane) {
                        //cerr << "in the first if!!!!!" << endl;
                        CvPoint h = unsortedLeft.front();

                        if (unsortedLeft.size() != 0) unsortedLeft.pop_front();
                        CvPoint t = unsortedLeft.front();

                        if (unsortedLeft.size() != 0) unsortedLeft.pop_front();

                        list <CvPoint> *listPointer = &sorted.front();

                        while (unsortedLeft.size() != 0) {
                            //cerr << "First = " << (h.x+t.x)/2 << " Second = " << (h->y+t->y)/2 << " h0 = " << h->y << " t0 = " << t->y << " h1 = " << h->x << " t1 = " << t->x << endl;
                            //CvScalar pix = cvGet2D(temp, (h[0]+t[0])/2, (h[1]+t[1])/2);
                            CvScalar pix = cvGet2D(temp, (h.y + t.y) / 2, (h.x + t.x) / 2);
                            //cerr << "beforeif" << endl;
                            if (pix.val[0] >= 200) {
                                //cerr << "########Finds a white pixel in first if########" << endl;
                                //cerr << "in if" << endl;
                                listPointer->push_back(t);
                            } else {
                                //cerr << "##does not see a white##" << endl;
                                //cerr << "in else" << endl;
                                //listPointer->push_back(t);
                                unsigned int i;
                                for (i = 0; i < sorted.size(); i++) {
                                    //cerr << "First2 = " << (h->x+t->x)/2 << " Second = " << (h->y+t->y)/2 << " h0 = " << h->y << " t0 = " << t->y << " h1 = " << h->x << " t1 = " << t->x;
                                    pix = cvGet2D(temp, (h.y + t.y) / 2, (h.x + t.x) / 2);
                                    if (pix.val[0] >= 200) {
                                        //cerr << "########Finds a white pixel in for loop ########" << endl;

                                        listPointer->push_back(t);
                                        break;
                                    } else {
                                        //list<int*> tempList = sorted.front();
                                        list <CvPoint> tempList = sorted.front();
                                        sorted.pop_front();
                                        listPointer = &sorted.front();
                                        sorted.push_back(tempList);
                                    }
                                }
                                if (i == sorted.size()) {
                                    list <CvPoint> tempList2;
                                    tempList2.push_back(t);
                                    sorted.push_back(tempList2);
                                    listPointer = &sorted.back();
                                }
                            }
                            h = t;
                            t = unsortedLeft.front();
                            if (unsortedLeft.size() != 0)unsortedLeft.pop_front();
                            //else t = 0;
                        }
                        //cerr << "out of first while" << endl;
                        //list<int*> l = sorted.front();
                        list <CvPoint> l = sorted.front();
                        //cerr << "in between " << endl;
                        sorted.pop_front();
                        while (sorted.size() != 0) {
                            //cerr << "in second while" << endl;
                            if (sorted.front().size() > l.size()) {
                                l = sorted.front();
                            }
                            sorted.pop_front();
                        }
                        //cerr << "out of second while" << endl;

                        //e = ((l.front().x - temp->width/2.0) - distance)/distance;
                        e = (distance - (temp->width / 2.0 - l.front().x)) / distance;
                        // else{
                        //     e = (distance - (temp->width/2.0 - left.x))/distance;
                        // }
                    }

                }
            }


   
            TimeStamp afterImageProcessing;
            if(m_debug){
                //cerr << "Processing time: " << (afterImageProcessing.toMicroseconds() - beforeImageProcessing.toMicroseconds())/1000.0 << "ms." << endl;
            }

            //If debug is on show the lane detection distance lines
            if (m_debug) {
                if (temp != NULL) {
                    cv::imshow("image", grey_image);
                    cvWaitKey(10);
                }
            }

            


            TimeStamp currentTime;
            double timeStep = (currentTime.toMicroseconds() - m_previousTime.toMicroseconds()) / (1000.0 * 1000.0);
            m_previousTime = currentTime;



            if (fabs(e) < 1e-2) {
                m_eSum = 0;
            }
            else {
                m_eSum += e;
            }

            // The following values have been determined by Twiddle algorithm.
            const double Kp = 0.4482626884328734;
            const double Ki = 3.103197570937628;
            const double Kd = 0.030450210485408566;

            //const double Kp = 1.0;
            //const double Ki = 0.01;
            //const double Kd = 0.1;

             //cerr << "e = " << e << endl;
            const double p = Kp * e;
            const double i = Ki * timeStep * e;// * m_eSum; // * e
            const double d = Kd * (e - m_eOld)/timeStep;
            m_eOld = e;

            //const double y = p + i + d;
            const double y = p + i + d;

            double desiredSteering = 0;
            if (fabs(e) > 1e-2) {
                desiredSteering = y;

                if (desiredSteering > 25.0) {
                    //cerr << "in first if >25" << endl;
                    desiredSteering = 25.0;
                }
                if (desiredSteering < -25.0) {
                    //cerr << "in second if >25" << endl;

                    desiredSteering = -25.0;
                }
            }

            if(m_debug){
                //cerr << "PID: " << "e = " << e << ", eSum = " << m_eSum << ", desiredSteering = " << desiredSteering << ", y = " << y << endl;
            }

            // Go forward. Main speed of the car.
            //m_vehicleControl.setSpeed(2);
            //m_vehicleControl.setSteeringWheelAngle(desiredSteering);
            steering = desiredSteering;
        }


        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Overtaker::body() {
            
            //-const int32_t ULTRASONIC_FRONT_CENTER = 3;
            //const int32_t ULTRASONIC_FRONT_RIGHT = 4;
            //-const int32_t INFRARED_FRONT_RIGHT = 0;
            //-const int32_t INFRARED_REAR_RIGHT = 2;

             //Sensor Id's Real Car
            int INFRARED_FRONT_RIGHT = 5;
            int INFRARED_REAR_RIGHT = 1;
            int ULTRASONIC_FRONT_CENTER = 2;
            //int ULTRASONIC_FRONT_RIGHT = 3;
            //int INFRARED_BACK = 4;


            bool turnToLeftLane = false;
            bool turnToRightLane = false;
            bool goForward = true;
            bool driveOnLeftLane = false;
            //bool onRightLaneTurnLeft = false;
            int count=0;

            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {

                bool has_next_frame = false;

                // Get the most recent available container for a SharedImage.
                Container c = getKeyValueDataStore().get(odcore::data::image::SharedImage::ID());
                
                if(m_simulator){
                    m_vehicleControl.setSpeed(2); //1
                }
                if (c.getDataType() == odcore::data::image::SharedImage::ID()) {
                    has_next_frame = readSharedImage(c);
                }

                // Process the read image and calculate regular lane following set values for control algorithm.
                if (true == has_next_frame) {
                    processImage();
                }

                // 1. Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData> ();

                // 2. Get most recent sensor board data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

                // Create vehicle control data.
                VehicleControl vc;


                /*
                 * Main overtaking Algorithm
                */

                //Check for object Simulator
                //if(sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER) < 7.2 && sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER) > 0){ //5.5
                if(readSensorData(ULTRASONIC_FRONT_CENTER) < 7.2 && readSensorData(ULTRASONIC_FRONT_CENTER) > 0){ //5.5
                    cerr << "Object detected" << endl;
                    
                    if(m_simulator){
                        vc.setSpeed(1);
                        vc.setSteeringWheelAngle(-60); //-45
                    }else{
                        sendSteeringAngle(-60);
                    }

                    turnToLeftLane = true;
                    //driveOnLeftLane = true;
                    
                    //turnToLeftLane = true;
                    goForward = false;
                }

                else if(turnToLeftLane){  
                    cerr << "turn to the left lane" << endl;

                    if(m_simulator){
                        vc.setSpeed(1);
                        vc.setSteeringWheelAngle(-50);
                        Container cont(vc);
                        // Send container.
                        getConference().send(cont);
                        odcore::base::Thread::usleepFor(100000);
                        vc.setSpeed(1);
                        vc.setSteeringWheelAngle(0);
                        Container cont1(vc);
                        // Send container.
                        getConference().send(cont1);
                    }else{
                        sendSteeringAngle(-50);
                        odcore::base::Thread::usleepFor(100000);
                        sendSteeringAngle(0);
                    }

                    //double di = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                    //cerr << "IR front right = " << di << endl;
                    
                    //if(sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0 && sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) < 2.60){
                    if(readSensorData(INFRARED_FRONT_RIGHT) > 0 && readSensorData(INFRARED_FRONT_RIGHT) < 2.60){

                        driveOnLeftLane = true;
                        turnToLeftLane = false;
                    }
                }

                if(driveOnLeftLane){
                   
                    distance = 280;
                    cerr << "driving on the left lane" << endl;
                    //while((sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0 || sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT) > 0) && getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING){
                    while((readSensorData(INFRARED_FRONT_RIGHT) > 0 || readSensorData(INFRARED_REAR_RIGHT) > 0) && getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING){

                        double sensorF = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                        double sensorB = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);

                        cerr << "Front sensor = " << sensorF << " Back sensor = " << sensorB << endl;

                        c = getKeyValueDataStore().get(odcore::data::image::SharedImage::ID());
                
                        if (c.getDataType() == odcore::data::image::SharedImage::ID()) {
                            has_next_frame = readSharedImage(c);
                        }
                        if (true == has_next_frame){
                            processImage();
                        }

                        // Process the read image and calculate regular lane following set values for control algorithm.
                        if (true == has_next_frame) {
                            processImage();
                        }
                        cerr << "Go forward in while" << endl;

                        if(m_simulator){
                            vc.setSpeed(1);
                            vc.setSteeringWheelAngle(steering);
                            Container com(vc);
                            // Send container.
                            getConference().send(com);
                        }else{
                            sendSteeringAngle(steering);
                        }

                        //if(sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT)<0){
                        if(readSensorData(INFRARED_FRONT_RIGHT)<0){
                            cerr << "breakin out of while" << endl;
                            break;
                        }

                        //Get new sensor data
                        containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                        sbd = containerSensorBoardData.getData<SensorBoardData> ();
                       
                    }
                    driveOnLeftLane = false;
                    turnToRightLane = true;
                }

                else if(turnToRightLane){
                    
                    distance = 90;

                    cerr << "Turn back to right lane" << endl;
                    double inf3 = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);
                    cerr << "Infrared rear right = " <<  inf3 << endl;
                    double inf4 = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                    cerr << "Infrared front right = " <<  inf4 << endl;

                    //double v = sbd.getValueForKey_MapOfDistances(ULTRASONIC_REAR_RIGHT)
                             
                        //if(sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT) > 0 && sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) < 0){
                        if(readSensorData(INFRARED_REAR_RIGHT) > 0 && readSensorData(INFRARED_FRONT_RIGHT) < 0){
                            count++;
                            double inf = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);
                            cerr << "Infrared rear right = " <<  inf << endl;
                            double inf2 = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                            cerr << "Infrared front right = " <<  inf2 << endl;
                            
                            if(m_simulator){
                                vc.setSpeed(1);
                                vc.setSteeringWheelAngle(45);
                                Container cont(vc);
                                // Send container.
                                getConference().send(cont);
                                odcore::base::Thread::usleepFor(200000);
                                vc.setSpeed(1);
                                vc.setSteeringWheelAngle(0);
                                Container cont1(vc);
                                // Send container.
                                getConference().send(cont1);
                            }else{
                                sendSteeringAngle(45);
                                odcore::base::Thread::usleepFor(200000);
                                sendSteeringAngle(0);
                            }

                        }else{
                            if(m_simulator){
                                vc.setSpeed(1);
                                vc.setSteeringWheelAngle(-45);
                                Container cont2(vc);
                                // Send container.
                                getConference().send(cont2);
                                odcore::base::Thread::usleepFor(90000);
                            }else{
                                sendSteeringAngle(-45);
                                odcore::base::Thread::usleepFor(90000);
                            }

                            turnToRightLane = false;
                            goForward = true;
                            //onRightLaneTurnLeft = true;
                            count = 0;
                        }
                }

                else if(goForward){
                     //turnToRightLane = false;
                    //goForward = false;
                    cerr << "Go forward again -------- hello" << endl;

                    if(m_simulator){
                        vc.setSpeed(1);
                        vc.setSteeringWheelAngle(steering);
                    }else{
                        sendSteeringAngle(steering);
                    }
                }
                
                if(m_simulator){
                    Container control(vc);
                    //Send container.
                    getConference().send(control);
                }
             }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    }
//} // automotive::miniature
