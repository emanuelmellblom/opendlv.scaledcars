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

    namespace scaledcars{

        using namespace std;
        using namespace cv;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace odcore;
        using namespace odcore::wrapper;
        using namespace automotive;
        using namespace automotive::miniature;

        //NEW FROM LANEFOLLOWER
        using namespace odcore::data::image;
        double steering;
        int32_t distance = 180; //280, 180

        Overtaker::Overtaker(const int32_t &argc, char **argv) : TimeTriggeredConferenceClientModule(argc, argv, "overtaker"),
            m_hasAttachedToSharedImageMemory(false),
            m_sharedImageMemory(),
            m_image(NULL),
            m_debug(true), //Have true to show detected lane markings, debugging prints and debugging camera windows
            m_simulator(true), //Set m_simulator to true if simulator is used and false otherwise.
            m_font(),
            m_previousTime(),
            m_eSum(0),
            m_eOld(0),
            m_speed(5),
            m_newStopLine(true),
            m_stopline(false),
            m_vehicleControl() {}



        Overtaker::~Overtaker() {}

        void Overtaker::setUp() {
            // This method will be call automatically _before_ running body().
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
                    // if(sharedMemory!=NULL){
                    //     char reset = 0x00;
                    //     odcore::base::Lock l(sharedMemory);
                    //     char *p = static_cast<char*>(sharedMemory->getSharedMemory());


                    // }

                    uint32_t counter = 30;
                    while (counter-- > 0) {
                        //int id;
                        char value;
                    {
                        // Using a scoped lock to lock and automatically unlock a shared memory segment.
                        {
                        odcore::base::Lock l(sharedMemory);
                        char *p = static_cast<char*>(sharedMemory->getSharedMemory());

                        char x = p[sensorId];
                        if(sensorId == 2)
                        cerr << "x = " << ((int)x&31)*2 << endl;
                        //Extract the sensor ID from the received byte
                       // id = (x >> 5 )& 0x07;
                        //Extract the sensor value from the received byte
                        value = (x & 31)*2;


                        //if(id == sensorId){
                            //p[sensorId] = 0x00;
                            // for(int i = 1; i < 8; i++){
                            //     p[i] = 0x00;
                            // }
                            returnValue = value;
                            break;
                        //}
                        }
                    }
                        // Sleep some time.
                        //const uint32_t ONE_SECOND = 1000 * 1000;
                        odcore::base::Thread::usleepFor(1000);
                    }
                }else{
                    cerr << "Invalid sharedMemory" << endl;
                }
            }
            catch(string &exception) {
                cerr << "Sensor memory could not created: " << exception << endl;
            }
            return returnValue;
        }

        char Overtaker::readOdometer(){
        
            try {
                std::shared_ptr<SharedMemory> sharedMemory(SharedMemoryFactory::attachToSharedMemory("odoMem"));
                
                if (sharedMemory->isValid()) {
                    odcore::base::Lock l(sharedMemory);
                    char *p = static_cast<char*>(sharedMemory->getSharedMemory());
                    char temp=p[0];
                    p[0]=0;
                    return temp;
                }
            }catch (string &exception) {
                cerr << "Sensor memory could not created: " << exception << endl;
            }
        
            return 0;
    
        }

        void Overtaker::resetOdometer(){
            try {
                std::shared_ptr<SharedMemory> sharedMemory(SharedMemoryFactory::attachToSharedMemory("odoMem"));

                    {
                    odcore::base::Lock l(sharedMemory);
                    char *p = static_cast<char*>(sharedMemory->getSharedMemory());
                    p[0] = 0; //output to the aruino, output is the byte we send to the arduino.
                    p[1] = 1;
                    }

                }catch (string &exception) {
                cerr << "Odometer memory could not created: " << exception << endl;
            }
}


        void Overtaker::sendSteeringAngle(double steeringAngle, int speed){ //speed is between 0 and 7. 

            cerr << "Original steeringAngle = " << steeringAngle << endl;

            //cerr << "org = " << steeringAngle << endl;
            int steeringAngleDegrees = ((steeringAngle*180)/M_PI);
            cerr << "steeringAngle = " << steeringAngleDegrees << endl;
            //char output = 0x00;

            char output = ((int)(round(steeringAngleDegrees/4))+15)& 31;
            output |= speed << 5;
            //m_speed
            //char output = ((29/4)+15)& 31;
            cerr << "Output steering = " << (int)output << endl;




            const string NAME = "sensorMemory";
            try{
                std::shared_ptr<SharedMemory> sharedMemory(SharedMemoryFactory::attachToSharedMemory(NAME));
                if (sharedMemory->isValid()) {
                    // if(sharedMemory!=NULL){
                    //     char reset = 0x00;
                    //     odcore::base::Lock l(sharedMemory);
                    //     char *p = static_cast<char*>(sharedMemory->getSharedMemory());
                    //     p[0] = reset;
                    // }
                    //cerr << "Writing (Angle) shared memory is valid" << endl;
                    {
                    odcore::base::Lock l(sharedMemory);
                    char *p = static_cast<char*>(sharedMemory->getSharedMemory());
                    p[0] = output; //output to the aruino, output is the byte we send to the arduino.
                    }
                }
            }
            catch(string &exception){
                cerr << "sharedMemory not Attached " << exception << endl;
            }
        }


        void Overtaker::processImage() {

            static bool useRightLaneMarking = true;
            double e = 0;

            int32_t CONTROL_SCANLINE = 262;// 462, (372), 252

            cv::Mat grey_image;
            if(m_image!=NULL){
                cv::Mat blured;

                cv::Mat image = cv::cvarrToMat(m_image, true, true, 0);

                GaussianBlur( image, image, cv::Size( 5, 5 ),0,0);

                //Rezise image size(width<cols>, height<rows>)
                cv::Size size(640,400);
                resize(image,image,size);

                //Convert original image to black/white
                grey_image = cv::Mat(image.cols, image.cols, CV_8UC1);
                cvtColor(image, grey_image, cv::COLOR_BGR2GRAY);

                //original: blur(grey_image, blured, cv::Size(3,3) );
                //Add blur to the image. Blur determined by the kernel size.

                //Apply Canny edge detection
                Canny(grey_image, grey_image, 50, 200, 3);

                //standard hough line transform
/*
                vector<Vec2f>lines;
                HoughLines(grey_image, lines, 1, CV_PI/180, 150, 0,0);
                   
                      for( size_t i = 0; i < lines.size(); i++ )
                              {
                                double rho = lines[i][0], theta = lines[i][1];
                                Point pt1, pt2;
                                double a = cos(theta), b = sin(theta);
                                double x0 = a*rho, y0 = b*rho;
                                pt1.x = cvRound(x0 + 1000*(-b));
                                pt1.y = cvRound(y0 + 1000*(a));
                                pt2.x = cvRound(x0 - 1000*(-b));
                                pt2.y = cvRound(y0 - 1000*(a));
                                line( grey_image, pt1, pt2, Scalar(255,0,0), 3, CV_AA);
                               
                               }
              
#else     */


      

    //     /*
    //     * ######################## Hough Lines Working ###########################
    //     */
    //         vector<cv::Vec4i> lines;
    //         HoughLinesP(grey_image, lines, 1, CV_PI/180, 5, 20, 30 );
    
    //         for( size_t i = 0; i < lines.size(); i++ ){
                // //line( grey_image, Point(lines[i][0], lines[i][1]),
                // //Point(lines[0]1], lines[2][3]), Scalar(255,0,0), 3, CV_AA);
                // Vec4i l = lines[i];
                // line( grey_image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 3, 8);     
    //         }
         
    //     /*
    //     * ################### END Hough lines Working ###########################
    //     */ 

    


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
            
            
            
            
            /*
                 * ############### CHECK STOPLINE ####################
                */
                CvPoint leftStopPoint, rightStopPoint;
                CvScalar leftPixel, rightPixel;

                int leftOffset = (temp->width/2)-50;
                int rightOffset = (temp->width/2)+50;

                leftStopPoint.x = leftOffset;
                leftStopPoint.y = 0;

                rightStopPoint.x = rightOffset;
                rightStopPoint.y = 0;

                //Find potential stopline pixels at left offset
                for (int i = temp->height-12; i > CONTROL_SCANLINE-40; i--){
                    leftPixel = cvGet2D(temp, i, leftOffset);
                    if(leftPixel.val[0] >= 200){
                        leftStopPoint.y = i;
                        break;
                    }
                }
                
                //Find potential stopline pixel at right offset
                for (int i = temp->height-12; i > CONTROL_SCANLINE-40; i--){
                    rightPixel = cvGet2D(temp, i, rightOffset);
                    if(rightPixel.val[0] >= 200){
                        rightStopPoint.y = i;
                        break;
                    }
                }
                
                    //Check the potential stopline line by using a range
                int range = 20; // max range between pixels is -10 to 10
                cerr<<"distance is measured at "<< (leftStopPoint.y+rightStopPoint.y)/2 <<" and img height is "<< temp->height<<endl;
                if((leftStopPoint.y - rightStopPoint.y > -range) && (leftStopPoint.y - rightStopPoint.y < range) && ((leftStopPoint.y+rightStopPoint.y)/2 < temp->height) && ((leftStopPoint.y+rightStopPoint.y)/2 > 325)){
                    if(this->m_newStopLine)
                        m_stopline = true;  
                    else
                        m_stopline = false;

                    this->m_newStopLine=false;
                        
                    cerr << "#################### Detected Stopline #################" << endl;
                }else{
                    m_stopline = false;
                    this->m_newStopLine=true;
                }   
            

                /*
                * ################ END OF STOPLINE CHECK ################# 
                */

            //TEST VOTING
            //Right Lane
            int rightLaneCount = 0;
            int missingRightLane = 0;
            //int lastRightDistance = 0;

            //Left Lane
            int leftLaneCount = 0;
            int missingLeftLane = 0;
            //int lastLeftDistance = 0;

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
                    rightLaneCount++;
                    unsorted.push_front(right);
                } else {
                    missingRightLane++;
                }

                //Left Lane
                if (left.x > 0) {
                    leftLaneCount++;
                    unsortedLeft.push_front(left);
                } else {
                    missingLeftLane++;
                }
                //-------------------------------------------------------------------------------

                 if (m_debug) {
                     if (left.x > 0) {//draw lines on left side
                         stringstream sstr;
                         sstr << (temp->width / 2 - left.x);
                         cv::line(grey_image, cv::Point2i(temp->width / 2, y), left, cv::Scalar(255, 0, 0), 1, 8);


                     }
                     
                     if (right.x > 0) {//draw line on right side
                         stringstream sstr;
                         sstr << (right.x - temp->width / 2);
                         cv::line(grey_image, cv::Point2i(temp->width / 2, y), right, cv::Scalar(255, 0, 0), 1, 8);
                     }
                 }


                if (y == CONTROL_SCANLINE) {//when maximum scna distance is readched lower values means that a bigger part of the image is scanned
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
                            CvScalar pix = cvGet2D(temp, (h.y + t.y) / 2, (h.x + t.x) / 2);
                            if (pix.val[0] >= 200) {
                                listPointer->push_back(t);
                            } else {
                                unsigned int i;
                                for (i = 0; i < sorted.size(); i++) {
                                    pix = cvGet2D(temp, (h.y + t.y) / 2, (h.x + t.x) / 2);
                                    if (pix.val[0] >= 200) {
                                        listPointer->push_back(t);
                                        break;
                                    } else {
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
                        }
                        list <CvPoint> l = sorted.front();
                        sorted.pop_front();
                        while (sorted.size() != 0) {
                            if (sorted.front().size() > l.size()) {
                                l = sorted.front();
                            }
                            sorted.pop_front();
                        }
                        int tempVar = 0;
                        int Lsize = l.size();
                        while(l.size() != 0){
                            tempVar += l.front().x;
                            l.pop_front();
                        }

                        e = ((round(tempVar/Lsize) - temp->width / 2.0) - distance) / distance;
                        
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
                            CvScalar pix = cvGet2D(temp, (h.y + t.y) / 2, (h.x + t.x) / 2);
                            if (pix.val[0] >= 200) {
                                listPointer->push_back(t);
                            } else {
                                unsigned int i;
                                for (i = 0; i < sorted.size(); i++) {
                                    pix = cvGet2D(temp, (h.y + t.y) / 2, (h.x + t.x) / 2);
                                    if (pix.val[0] >= 200) {
                                        listPointer->push_back(t);
                                        break;
                                    } else {
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
                        }
                        list <CvPoint> l = sorted.front();
                        sorted.pop_front();
                        while (sorted.size() != 0) {
                            if (sorted.front().size() > l.size()) {
                                l = sorted.front();
                            }
                            sorted.pop_front();
                        }

                        int tempVar = 0;
                        int Lsize = l.size();
                        while(l.size() != 0){
                            tempVar += l.front().x;
                            l.pop_front();
                        }

                        //e = ((distance - ((temp->width / 2.0) - (round(tempVar/Lsize))) / distance));
                        e = (((round(tempVar/Lsize) - temp->width / 2.0) - distance) / distance);
                    }

                }
            }
            
            
            if(m_debug){
                //cerr << "****** Stopline Distance = " << ((temp->height-8)-leftStopPoint.y) + ((temp->height-8)-rightStopPoint.y)/2 << endl; 
                cv::line(grey_image, cv::Point2i(leftOffset, temp->height-8), leftStopPoint, cv::Scalar(255, 0, 0), 1, 8);
                cv::line(grey_image, cv::Point2i(rightOffset, temp->height-8),rightStopPoint, cv::Scalar(255, 0, 0), 1, 8);
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

            const double p = Kp * e;
            const double i = Ki * timeStep * e;// * m_eSum; // * e
            const double d = Kd * (e - m_eOld)/timeStep;
            m_eOld = e;

            const double y = p + i + d;

            double desiredSteering = 0;
            if (fabs(e) > 1e-2) {
                desiredSteering = y;

            }

            if(m_debug){
                //cerr << "PID: " << "e = " << e << ", eSum = " << m_eSum << ", desiredSteering = " << desiredSteering << ", y = " << y << endl;
            }

            steering = desiredSteering;
            //cerr << "Original steering value = " << steering << endl;
}



        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Overtaker::body() {
            const int32_t ULTRASONIC_FRONT_CENTER = 3;
            const int32_t ULTRASONIC_FRONT_RIGHT = 4;
            const int32_t INFRARED_FRONT_RIGHT = 0;
            const int32_t INFRARED_REAR_RIGHT = 2;

            const double OVERTAKING_DISTANCE = 4.5;//5.5
            const double HEADING_PARALLEL = 1.30;//0.04

            // Overall state machines for moving and measuring.
            enum StateMachineMoving { FORWARD, TO_LEFT_LANE_LEFT_TURN, TO_LEFT_LANE_RIGHT_TURN, CONTINUE_ON_LEFT_LANE, TO_RIGHT_LANE_RIGHT_TURN, TO_RIGHT_LANE_LEFT_TURN };
            enum StateMachineMeasuring { DISABLE, FIND_OBJECT_INIT, FIND_OBJECT, FIND_OBJECT_PLAUSIBLE, HAVE_BOTH_IR, HAVE_BOTH_IR_SAME_DISTANCE, END_OF_OBJECT };

            StateMachineMoving stageMoving = FORWARD;
            StateMachineMeasuring stageMeasuring = FIND_OBJECT_INIT;

            // State counter for dynamically moving back to right lane.
            int32_t stageToRightLaneRightTurn = 0;
            int32_t stageToRightLaneLeftTurn = 0;

            // Distance variables to ensure we are overtaking only stationary or slowly driving obstacles.
            double distanceToObstacle = 0;
            double distanceToObstacleOld = 0;

            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                // 1. Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData> ();

                // 2. Get most recent sensor board data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

                // Create vehicle control data.
                VehicleControl vc;

                // Moving state machine.
                if (stageMoving == FORWARD) {
                    // Go forward.
                    vc.setSpeed(2);
                    vc.setSteeringWheelAngle(steering);

                    stageToRightLaneLeftTurn = 0;
                    stageToRightLaneRightTurn = 0;
                }
                else if (stageMoving == TO_LEFT_LANE_LEFT_TURN) {
                    // Move to the left lane: Turn left part until both IRs see something.
                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(-25);

                    // State machine measuring: Both IRs need to see something before leaving this moving state.
                    stageMeasuring = HAVE_BOTH_IR;

                    stageToRightLaneRightTurn++;
                }
                else if (stageMoving == TO_LEFT_LANE_RIGHT_TURN) {
                    // Move to the left lane: Turn right part until both IRs have the same distance to obstacle.
                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(15);

                    // State machine measuring: Both IRs need to have the same distance before leaving this moving state.
                    stageMeasuring = HAVE_BOTH_IR_SAME_DISTANCE;

                    stageToRightLaneLeftTurn++;
                }
                else if (stageMoving == CONTINUE_ON_LEFT_LANE) {
                    // Move to the left lane: Passing stage.
                    vc.setSpeed(2);
                    vc.setSteeringWheelAngle(0);

                    // Find end of object.
                    stageMeasuring = END_OF_OBJECT;
                }
                else if (stageMoving == TO_RIGHT_LANE_RIGHT_TURN) {
                    // Move to the right lane: Turn right part.
                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(6);
// reducéd the tuern distance on counter oto teh right
                    stageToRightLaneRightTurn--;
                    if (stageToRightLaneRightTurn == (stageToRightLaneRightTurn/2)) {
                        stageMoving = TO_RIGHT_LANE_LEFT_TURN;
                    }
                }
                else if (stageMoving == TO_RIGHT_LANE_LEFT_TURN) {
                    // Move to the left lane: Turn left part.
                    vc.setSpeed(.9);
                    vc.setSteeringWheelAngle(-25);
// reduced teh turn distance on  counter to the left

                    stageToRightLaneLeftTurn--;
                    if (stageToRightLaneLeftTurn == (stageToRightLaneLeftTurn/2)) {
                        // Start over.
                        stageMoving = FORWARD;
                        stageMeasuring = FIND_OBJECT_INIT;

                        distanceToObstacle = 0;
                        distanceToObstacleOld = 0;
                    }
                }

                // Measuring state machine.
                if (stageMeasuring == FIND_OBJECT_INIT) {
                    distanceToObstacleOld = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);
                    stageMeasuring = FIND_OBJECT;
                }
                else if (stageMeasuring == FIND_OBJECT) {
                    distanceToObstacle = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);

                    // Approaching an obstacle (stationary or driving slower than us).
                    if ( (distanceToObstacle > 0) && (((distanceToObstacleOld - distanceToObstacle) > 0) || (fabs(distanceToObstacleOld - distanceToObstacle) < 1e-2)) ) {
                        // Check if overtaking shall be started.
                        stageMeasuring = FIND_OBJECT_PLAUSIBLE;
                    }

                    distanceToObstacleOld = distanceToObstacle;
                }
                else if (stageMeasuring == FIND_OBJECT_PLAUSIBLE) {
                    if (sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER) < OVERTAKING_DISTANCE) {
                        stageMoving = TO_LEFT_LANE_LEFT_TURN;

                        // Disable measuring until requested from moving state machine again.
                        stageMeasuring = DISABLE;
                    }
                    else {
                        stageMeasuring = FIND_OBJECT;
                    }
                }
                else if (stageMeasuring == HAVE_BOTH_IR) {
                    // Remain in this stage until both IRs see something.
                    if ( (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0) && (sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT) > 0) ) {
                        // Turn to right.
                        stageMoving = TO_LEFT_LANE_RIGHT_TURN;
                    }
                }
                else if (stageMeasuring == HAVE_BOTH_IR_SAME_DISTANCE) {
                    // Remain in this stage until both IRs have the similar distance to obstacle (i.e. turn car)
                    // and the driven parts of the turn are plausible.
                    const double IR_FR = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                    const double IR_RR = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);

                    if ((fabs(IR_FR - IR_RR) < HEADING_PARALLEL) && ((stageToRightLaneLeftTurn - stageToRightLaneRightTurn) > 0)) {
                        // Straight forward again.
                        stageMoving = CONTINUE_ON_LEFT_LANE;
                    }
                }
                else if (stageMeasuring == END_OF_OBJECT) {
                    // Find end of object.
                    distanceToObstacle = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT);

                    if (distanceToObstacle < 0) {
                        // Move to right lane again.
                        stageMoving = TO_RIGHT_LANE_RIGHT_TURN;

                        // Disable measuring until requested from moving state machine again.
                        stageMeasuring = DISABLE;
                    }
                }

                // Create container for finally sending the data.
                Container c(vc);
                // Send container.
                getConference().send(c);
            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    }
 // automotive::miniature
