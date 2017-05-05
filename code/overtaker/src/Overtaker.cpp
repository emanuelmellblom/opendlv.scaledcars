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
//----



//namespace automotive {
    //namespace miniature {
namespace scaledcars{


    using namespace std;
    using namespace odcore::base;
    using namespace odcore::data;
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
             const int32_t ULTRASONIC_FRONT_CENTER = 3;
             //const int32_t ULTRASONIC_FRONT_RIGHT = 4;
             const int32_t INFRARED_FRONT_RIGHT = 0;
             const int32_t INFRARED_REAR_RIGHT = 2;
             //const int32_t ULTRASONIC_REAR_RIGHT = 5;

            // const double OVERTAKING_DISTANCE = 5.5;
            // const double HEADING_PARALLEL = 0.04; //0.04
            
            // Overall state machines for moving and measuring.
            //enum StateMachineMoving { FORWARD, TO_LEFT_LANE_LEFT_TURN, TO_LEFT_LANE_RIGHT_TURN, CONTINUE_ON_LEFT_LANE, TO_RIGHT_LANE_RIGHT_TURN, TO_RIGHT_LANE_LEFT_TURN };
            //enum StateMachineMeasuring { DISABLE, FIND_OBJECT_INIT, FIND_OBJECT, FIND_OBJECT_PLAUSIBLE, HAVE_BOTH_IR, HAVE_BOTH_IR_SAME_DISTANCE, END_OF_OBJECT };

            //StateMachineMoving stageMoving = FORWARD;
            //StateMachineMeasuring stageMeasuring = FIND_OBJECT_INIT;

            // State counter for dynamically moving back to right lane.
            //int32_t stageToRightLaneRightTurn = 0;
            //int32_t stageToRightLaneLeftTurn = 0;

            // Distance variables to ensure we are overtaking only stationary or slowly driving obstacles.
            //double distanceToObstacle = 0;
            //double distanceToObstacleOld = 0;

            bool turnToLeftLane = false;
            bool turnToRightLane = false;
            bool goForward = true;
            bool driveOnLeftLane = false;
            bool onRightLaneTurnLeft = false;
            int count=0;
            //int counting =0;

            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                //cerr << "Running" << endl;

                bool has_next_frame = false;
                  //cerr << "In body" << endl;

                // Get the most recent available container for a SharedImage.
                Container c = getKeyValueDataStore().get(odcore::data::image::SharedImage::ID());
                
                 m_vehicleControl.setSpeed(2); //1
                if (c.getDataType() == odcore::data::image::SharedImage::ID()) {
                    //cerr << "is shared image" << endl;
                    // Example for processing the received container.
                    has_next_frame = readSharedImage(c);
                    //cerr << "read frame" << endl;
                }

                // Process the read image and calculate regular lane following set values for control algorithm.
                if (true == has_next_frame) {
                    //cerr << "has next frame" << endl;
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
                * TESTING STUFF
                */
                if(sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER) < 7.2 && sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER) > 0){ //5.5
                    //goForward = false;
                    cerr << "Object detected" << endl;
                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(-60); //-45

                    turnToLeftLane = true;
                    //driveOnLeftLane = true;
                    
                    //turnToLeftLane = true;
                    goForward = false;
                }

                else if(turnToLeftLane){  
                    cerr << "turn to the left lane" << endl;
                    //vc.setSpeed(1);
                    //vc.setSteeringWheelAngle(-45); //-45


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

                    double di = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                    cerr << "IR front right = " << di << endl;
                    
                    //if(sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT) < 0 && sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT)>1){
                    if(sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0 && sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) < 2.60){

                        driveOnLeftLane = true;
                        turnToLeftLane = false;
                    }
              
                    
                }

                if(driveOnLeftLane){
                   
                    distance = 280;
                    cerr << "driving on the left lane" << endl;
                    while((sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0 || sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT) > 0) && getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING){
                    //while(((sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0 && sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT) < 0) || (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0 && sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT) > 0)) && getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING){
                    //while(sbd.getValueForKey_MapOfDistances(ULTRASONIC_REAR_RIGHT) < 0 && getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING){

                        double sensorF = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                        double sensorB = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);

                        cerr << "Front sensor = " << sensorF << " Back sensor = " << sensorB << endl;

                        c = getKeyValueDataStore().get(odcore::data::image::SharedImage::ID());
                
                        if (c.getDataType() == odcore::data::image::SharedImage::ID()) {
                            has_next_frame = readSharedImage(c);
                        }
                        if (true == has_next_frame){
                            //cerr << "has next frame" << endl;
                            processImage();
                        }

                        // Process the read image and calculate regular lane following set values for control algorithm.
                        if (true == has_next_frame) {
                            //cerr << "has next frame" << endl;
                            processImage();
                        }
                        cerr << "Go forward in while" << endl;
                        vc.setSpeed(1);
                        vc.setSteeringWheelAngle(steering);
                        Container com(vc);
                        // Send container.
                        getConference().send(com);

                        if(sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT)<0){
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
              
                   //if(count<=46 && sbd.getValueForKey_MapOfDistances(ULTRASONIC_REAR_RIGHT) < 0 && sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT) > 0){                 
                        if(sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT) > 0 && sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) < 0){
                            count++;
                            double inf = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);
                            cerr << "Infrared rear right = " <<  inf << endl;
                            double inf2 = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                            cerr << "Infrared front right = " <<  inf2 << endl;
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
                            vc.setSpeed(1);
                            vc.setSteeringWheelAngle(-45);
                            Container cont2(vc);
                            // Send container.
                            getConference().send(cont2);
                            odcore::base::Thread::usleepFor(90000);
                            turnToRightLane = false;
                            //goForward = true;
                            onRightLaneTurnLeft = true;
                            count = 0;
                        }
                }

                //NEW TEST
                else if(onRightLaneTurnLeft){
                    // if((counting <= 95 && (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) < 0 && sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT) < 0))){
                    //     counting++;
                    //     vc.setSpeed(0.5);
                    //     vc.setSteeringWheelAngle(-65);
                    //     cerr << "####### TURNING #########" << endl;
                        
                    // }else{
                    //     onRightLaneTurnLeft = false;
                    //     goForward = true;
                    //     counting = 0;
                    // }
                    onRightLaneTurnLeft = false;
                    goForward = true;
                }


                else if(goForward){
                     //turnToRightLane = false;
                    //goForward = false;
                    cerr << "Go forward again -------- hello" << endl;
                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(steering);
                }

                Container control(vc);
                //Send container.
                getConference().send(control);



                // /*
                // * END OF TESTING STUFF
                // */

                    

                // Moving state machine.
                // if (stageMoving == FORWARD) {
                //     // Go forward.
                //     cerr << "Move Forward" << endl;
                //     vc.setSpeed(1);
                //     vc.setSteeringWheelAngle(steering);

                //     stageToRightLaneLeftTurn = 0;
                //     stageToRightLaneRightTurn = 0;
                // }
                // else if (stageMoving == TO_LEFT_LANE_LEFT_TURN) {
                //     // Move to the left lane: Turn left part until both IRs see something.
                //     cerr << "Move Left 1st step (left lane)" << endl;
                //     vc.setSpeed(1);
                //     vc.setSteeringWheelAngle(-25); //25

                //     // State machine measuring: Both IRs need to see something before leaving this moving state.
                //     stageMeasuring = HAVE_BOTH_IR;

                //     stageToRightLaneRightTurn++;
                // }
                // else if (stageMoving == TO_LEFT_LANE_RIGHT_TURN) {
                //     // Move to the left lane: Turn right part until both IRs have the same distance to obstacle.
                //     cerr << "Make right turn on the left lane" << endl;
                //     vc.setSpeed(1);
                //     vc.setSteeringWheelAngle(25);

                //     // State machine measuring: Both IRs need to have the same distance before leaving this moving state.
                //     stageMeasuring = HAVE_BOTH_IR_SAME_DISTANCE;

                //     stageToRightLaneLeftTurn++;
                // }
                // else if (stageMoving == CONTINUE_ON_LEFT_LANE) {
                //     // Move to the left lane: Passing stage.
                //     cerr << "Moves on left lane" << endl;
                //     vc.setSpeed(1);
                //     //vc.setSteeringWheelAngle(0);
                //     vc.setSteeringWheelAngle(25); //steering
                //     cerr << "Steering = " << steering << endl;

                //     // Find end of object.
                //     stageMeasuring = END_OF_OBJECT;
                //     //stageMoving = FORWARD;
                // }
                // /*
                //  * Probematic part
                // */

                // else if (stageMoving == TO_RIGHT_LANE_RIGHT_TURN) {
                //     cerr << "Move to right lane" << endl;
                //     vc.setSteeringWheelAngle(25); //-25
                //     vc.setSpeed(.5);
                //     stageToRightLaneRightTurn--;
                //     stageMoving = TO_RIGHT_LANE_LEFT_TURN;
                // }

                // else if (stageMoving == TO_RIGHT_LANE_LEFT_TURN) {
                //     cerr << "Turn left on the Right lane" << endl;
                //     vc.setSteeringWheelAngle(-65);
                //     vc.setSpeed(.5);
                //     distanceToObstacle = 0;
                //     distanceToObstacleOld = 0;
                //     stageToRightLaneLeftTurn--;
                //     if (stageToRightLaneLeftTurn == 0) {
                //         // Start over.
                //         stageMoving = FORWARD;
                //         stageMeasuring = FIND_OBJECT_INIT;

                //         distanceToObstacle = 0;
                //         distanceToObstacleOld = 0;
                //      }

                // }
                /*
                *  END PROBLEMATIC PART
                */

                // Measuring state machine.
            //     if (stageMeasuring == FIND_OBJECT_INIT) {
            //         cerr << "Init object detect" << endl;
            //         distanceToObstacleOld = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);
            //         stageMeasuring = FIND_OBJECT;
            //     }
            //     else if (stageMeasuring == FIND_OBJECT) {
            //         cerr << "Object found" << endl;
            //         distanceToObstacle = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);

            //         // Approaching an obstacle (stationary or driving slower than us).
            //         if ( (distanceToObstacle > 0) && (((distanceToObstacleOld - distanceToObstacle) > 0) || (fabs(distanceToObstacleOld - distanceToObstacle) < 1e-2)) ) {
            //             // Check if overtaking shall be started.
            //             cerr << "object plausible" << endl;
            //             stageMeasuring = FIND_OBJECT_PLAUSIBLE;
            //         }

            //         distanceToObstacleOld = distanceToObstacle;
            //     }
            //     else if (stageMeasuring == FIND_OBJECT_PLAUSIBLE) {
            //         if (sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER) < OVERTAKING_DISTANCE) {
            //             stageMoving = TO_LEFT_LANE_LEFT_TURN;
            //             cerr << "Start overtaking" << endl;

            //             // Disable measuring until requested from moving state machine again.
            //             stageMeasuring = DISABLE;
            //         }
            //         else {
            //             stageMeasuring = FIND_OBJECT;
            //         }
            //     }
            //     else if (stageMeasuring == HAVE_BOTH_IR) {
            //         cerr << "Both ir detects" << endl;
            //         // Remain in this stage until both IRs see something.
            //         if ( (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0) && (sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT) > 0) ) {
            //             // Turn to right.
            //             stageMoving = TO_LEFT_LANE_RIGHT_TURN;
            //         }
            //     }
            //     else if (stageMeasuring == HAVE_BOTH_IR_SAME_DISTANCE) {
            //         cerr << "Both ir same distance" << endl;
            //         // Remain in this stage until both IRs have the similar distance to obstacle (i.e. turn car)
            //         // and the driven parts of the turn are plausible.
            //         const double IR_FR = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
            //         const double IR_RR = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);

            //         cerr << "Ir front = " << IR_FR << endl;
            //         cerr << "Ir Back = " << IR_RR << endl;
            //         cerr << "SUM = " << IR_FR - IR_RR << endl;
                      
            //         // if(IR_FR < 0 || IR_RR < 0){
            //         //     stageMeasuring = END_OF_OBJECT;  
            //         // }
            //         if ((fabs(IR_FR - IR_RR) < HEADING_PARALLEL) && (stageToRightLaneLeftTurn - stageToRightLaneRightTurn) > 0){
            //             // Straight forward again.
            //             cerr << "In the if --------------------" << endl;
            //             stageMoving = CONTINUE_ON_LEFT_LANE;
            //             //stageMoving = TO_RIGHT_LANE_RIGHT_TURN;

            //         } 
            //     }
            //     else if (stageMeasuring == END_OF_OBJECT) {
            //         // Find end of object.
            //         cerr << "End of the object" << endl;
            //         distanceToObstacle = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT);
            //         //distanceToObstacle = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
            //         if (distanceToObstacle < 0) {
            //             // Move to right lane again.
            //             stageMoving = TO_RIGHT_LANE_RIGHT_TURN;
            //             stageMoving = TO_RIGHT_LANE_RIGHT_TURN;

            //             // Disable measuring until requested from moving state machine again.
            //             stageMeasuring = DISABLE;
            //         }
            //     }

            //     // Create container for finally sending the data.
            //     Container control(vc);
            //     // Send container.
            //     getConference().send(control);
             }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    }
//} // automotive::miniature
