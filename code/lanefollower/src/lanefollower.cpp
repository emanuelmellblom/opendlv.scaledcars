/**
 * LaneFollower
 * @authors Emanuel Mellblom, John Sundling
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

#include <iostream>
 #include <cstring>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <opendavinci/odcore/base/Lock.h>
#include <opendavinci/odcore/wrapper/SharedMemoryFactory.h>

#include <opendavinci/GeneratedHeaders_OpenDaVINCI.h>
#include <automotivedata/GeneratedHeaders_AutomotiveData.h>

//Serialport
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>
#include <sstream>
#include <string> 
#include <opendavinci/odcore/base/Thread.h>
#include "SerialRead.h"
#include <math.h>
#define _USE_MATH_DEFINES
//---

#include "odvdscaledcarsdatamodel/generated/chalmersrevere/scaledcars/ExampleMessage.h"

#include "lanefollower.h"

#define SERIAL_PORT "/dev/ttyACM0"
#define BAUD_RATE 9600

namespace scaledcars {
//namespace perception {
//namespace lanefollower {

using namespace std;
using namespace odcore::base;
using namespace odcore::data;
using namespace odcore::data::image;
using namespace automotive;
using namespace automotive::miniature;

//serial
//using namespace odtools::recorder;
using namespace odcore;
using namespace odcore::wrapper;
//---

bool serialOn = false;
std::shared_ptr<SerialPort> serial;
double steering;



lanefollower::lanefollower(const int &argc, char **argv): TimeTriggeredConferenceClientModule(argc, argv, "scaledcars-lanefollower"),
    m_hasAttachedToSharedImageMemory(false),
    m_sharedImageMemory(),
    m_image(NULL),
    m_debug(true), //Have true to show detected lane markings, debugging prints and debugging camera windows
    m_simulator(false), //Set m_simulator to true if simulator is used and false otherwise.
    m_font(),
    m_previousTime(),
    m_eSum(0),
    m_eOld(0),
    m_vehicleControl() {}

        lanefollower::~lanefollower() {}

    void lanefollower::setUp() {
        //cvNamedWindow("Camera Feed Image", CV_WINDOW_AUTOSIZE);
        //cvMoveWindow("Camera Feed Image", 300, 100);
        //Set SIMULATOR to true if simulator is used and false otherwise.
    }

    void lanefollower::tearDown() {
        if (m_image != NULL) {
            cvReleaseImage(&m_image);
        }
        //cvDestroyWindow("Camera Feed Image");
    }
    
    
    bool lanefollower::readSharedImage(Container &c) {
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
                    cerr << "SharedImageMemory not valid" << endl;
                }

                // Check if we could successfully attach to the shared memory.
                if (m_sharedImageMemory->isValid()) {
                    // Lock the memory region to gain exclusive access using a scoped lock.
                    Lock l(m_sharedImageMemory);

                    //const uint32_t numberOfChannels = 3;
                    if(m_image == NULL) {
                        m_image = cvCreateImage(cvSize(si.getWidth(), si.getHeight()), IPL_DEPTH_8U, si.getBytesPerPixel());
                    }

                    // Example: Simply copy the image into our process space.
                    if (m_image != NULL) {
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

    void lanefollower::processImage() {

            static bool useRightLaneMarking = true;
            double e = 0;

            int32_t CONTROL_SCANLINE = 262;// 462, (372), 252 
            
            // if(m_simulator){
            //     CONTROL_SCANLINE = 462;
            // }else{
            //     CONTROL_SCANLINE = 222;
            // }

            //const int32_t CONTROL_SCANLINE = 462; // calibrated length to right: 300px
            const int32_t distance = 300; //300

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
                //Canny(grey_image, grey_image, 50, 60, 3);


   
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
                                cerr << "########Finds a white pixel in first if########" << endl;
                                //cerr << "in if" << endl;
                                listPointer->push_back(t);
                            } else {
                                cerr << "##does not see a white##" << endl;
                                //cerr << "in else" << endl;
                                //listPointer->push_back(t);
                                unsigned int i;
                                for (i = 0; i < sorted.size(); i++) {
                                    //cerr << "First2 = " << (h->x+t->x)/2 << " Second = " << (h->y+t->y)/2 << " h0 = " << h->y << " t0 = " << t->y << " h1 = " << h->x << " t1 = " << t->x;
                                    pix = cvGet2D(temp, (h.y + t.y) / 2, (h.x + t.x) / 2);
                                    if (pix.val[0] >= 200) {
                                        cerr << "########Finds a white pixel in for loop ########" << endl;

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
                cerr << "Processing time: " << (afterImageProcessing.toMicroseconds() - beforeImageProcessing.toMicroseconds())/1000.0 << "ms." << endl;
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

             //cerr << "e = " << e << endl;
            const double p = Kp * e;
            const double i = Ki * timeStep * e;// * m_eSum; // * e
            const double d = Kd * (e - m_eOld)/timeStep;
            m_eOld = e;

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
                cerr << "PID: " << "e = " << e << ", eSum = " << m_eSum << ", desiredSteering = " << desiredSteering << ", y = " << y << endl;
            }

            // Go forward. Main speed of the car.
            m_vehicleControl.setSpeed(2);
            m_vehicleControl.setSteeringWheelAngle(desiredSteering);
            steering = desiredSteering;
        }

        // This method will do the main data processing job.
        // Therefore, it tries to open the real camera first. If that fails, the virtual camera images from camgen are used.
        
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode lanefollower::body() {
            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();

            // Initialize fonts.
            const double hscale = 0.4;
            const double vscale = 0.3;
            const double shear = 0.2;
            const int thickness = 1;
            const int lineType = 6;

            cvInitFont(&m_font, CV_FONT_HERSHEY_DUPLEX, hscale, vscale, shear, thickness, lineType);

            // Overall state machines for moving and measuring.
            //enum StateMachineMoving { FORWARD, TO_LEFT_LANE_LEFT_TURN, TO_LEFT_LANE_RIGHT_TURcd N, CONTINUE_ON_LEFT_LANE, TO_RIGHT_LANE_RIGHT_TURN, TO_RIGHT_LANE_LEFT_TURN };

            //StateMachineMoving stageMoving = FORWARD;
            //StateMachineMeasuring stageMeasuring = FIND_OBJECT_INIT;

            // State counter for dynamically moving back to right lane.
            //int32_t stageToRightLaneRightTurn = 0;
            //int32_t stageToRightLaneLeftTurn = 0;

            // Overall state machine handler.
            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                //cerr<<"nana"<<endl;
                bool has_next_frame = false;

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
                    //Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                    //automotive::VehicleData vd = containerVehicleData.getData<automotive::VehicleData> ();

                    //int steering = 0;

                /*
                    // Moving state machine.
                    if (stageMoving == FORWARD) {
                        // Use m_vehicleControl data from image processing.
                        stageToRightLaneLeftTurn = 0;
                        stageToRightLaneRightTurn = 0;
                    }
                    else if (stageMoving == TO_LEFT_LANE_LEFT_TURN) {
                        // Move to the left lane: Turn left part until both IRs see something.
                        m_vehicleControl.setSpeed(1);
                        m_vehicleControl.setSteeringWheelAngle(-25);

                        //First test
                        steering = -25;

                        stageToRightLaneRightTurn++;
                    }
                    else if (stageMoving == TO_LEFT_LANE_RIGHT_TURN) {
                        // Move to the left lane: Turn right part until both IRs have the same distance to obstacle.
                        m_vehicleControl.setSpeed(1);
                        m_vehicleControl.setSteeringWheelAngle(25);

                        //First test
                        steering = 25;

                        stageToRightLaneLeftTurn++;
                    }
                    else if (stageMoving == CONTINUE_ON_LEFT_LANE) {
                        // Move to the left lane: Passing stage.
                    }
                    else if (stageMoving == TO_RIGHT_LANE_RIGHT_TURN) {
                        // Move to the right lane: Turn right part.
                        m_vehicleControl.setSpeed(1.5);
                        m_vehicleControl.setSteeringWheelAngle(25);

                        //First test
                        steering = 25;

                        stageToRightLaneRightTurn--;
                        if (stageToRightLaneRightTurn == 0) {
                            stageMoving = TO_RIGHT_LANE_LEFT_TURN;
                        }
                    }
                    else if (stageMoving == TO_RIGHT_LANE_LEFT_TURN) {
                        // Move to the left lane: Turn left part.
                        m_vehicleControl.setSpeed(.9);
                        m_vehicleControl.setSteeringWheelAngle(-25);

                        //First test
                        steering = -25;

                        stageToRightLaneLeftTurn--;
                        if (stageToRightLaneLeftTurn == 0) {
                            // Start over.
                            stageMoving = FORWARD;
                            // Reset PID controller.
                            m_eSum = 0;
                            m_eOld = 0;
                        }
                    }
                */
                   
                   //Send to serial port


                    int one = ((steering*180)/M_PI);
                    //int one = steering;
                    // std::stringstream ss;
                    //ss << "a"<<one<<"&";
                    //std::string s = ss.str();
                    //Print
                   // cerr << s << endl;

                    //int one = floor(steering*100);
                   
                    cerr << "one =  " << one << endl;
                    cerr << "steering = " << steering << endl;


                    //char outputBytes = 0x00;
                    char con = (((int)(one)/4)+15)& 31; //Constant = 45/25=1,8

                    //Formula for setting speed = (7-3)/2
                     //con = 1;
                     std::stringstream ss;

                    ss << con;
                    std::string s = ss.str();

                    //cerr << "con = " <<((int)con-15)*3 << endl;
                    //cerr << "con2 = " << (int)con << endl;

                    //outputBytes & con << 5;

                    //cerr << std::to_string(steering) << endl;
                   
                try {
                    if(!serialOn){
                        serial = std::shared_ptr<SerialPort>(SerialPortFactory::createSerialPort(SERIAL_PORT, BAUD_RATE));
                        odcore::base::Thread::usleepFor(1000000);
                        serialOn = true;
                    }
            
                 for (int i = 0; i < 1; ++i) {
                        
                        //SerialRead handeler;
                        //serial->setStringListener(&handeler);
                        //serial->start();

                        serial->send(s);
                        //odcore::base::Thread::usleepFor(100);

                        //serial->stop();
                        //serial->setStringListener(NULL);
                    }

                 //  for(int i=0;i<10;i++){
                 //    serial->send("a30");
                 //    const uint32_t ONE_SECOND = 1000 * 1000;
                 //    odcore::base::Thread::usleepFor( ONE_SECOND);
                 // }
            }catch(string &exception) {
                cerr << "Serial port could not be created: " << exception << endl;
            }

            //End sending to serial port
    

                // Create container for finally sending the set values for the control algorithm.
                Container c2(m_vehicleControl);
                // Send container.
                getConference().send(c2);
            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        
    }


}

//} // scaledcars::lanefollower
