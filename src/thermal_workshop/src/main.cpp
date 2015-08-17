/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-2015
 *  Technische Hochschule Nürnberg Georg Simon Ohm
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Nuremberg Institute of Technology
 *     Georg Simon Ohm nor the authors names may be used to endorse
 *     or promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christian Pfitzner
 *********************************************************************/

// ros specific includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// std includes
#include <iostream>
#include <limits>
#include <vector>

// opencv includes
#include <opencv2/opencv.hpp>


//!!!Beginner!!!
// 1. Task:    Have a look at rqt_graph
// 2. Task:    Try to find the right topic for viewing an image in rqt_image_viewer
// 3. Task:    Have a look at the used bandwith of the thermal image with and without compression


//!!! Basic  !!!
// 1. Task:    Include a subscriber to adress the thermal image
// 2. Task:    Display the size of the image you receive
// 3. Task:    Convert the thermal image to an OpenCV compatible format
// 4. Task:    Write a function to convert energy data of the thermal camera to temperature in Celsius --> getPixelTemperature
// 5. Task:    Write a function to display the temperature of the central pixel
// 6. Task:    Find the coldest and hottest pixel in the thermal image and display it to terminal
// 7. Task:    Calculate the average temperature in the image. Consider that there are invalid pixels.


//!!!Advanced!!!
// 1. Task     Write a publisher for an binary image for pixels warmer than 30 celsius.
//             Consider a unique topic name, so no conficts with other participants occur.
// 2. Task:    Publish a grey scale image converted from thermal image
// 3. Task:    Write a function to find the hottest area in the image. Make use of common image processing from OpenCV.
//             BuzzWord to search: cv::findContours, cv::inRange
//             Modify our previously published binary image and publish only large areas



ros::Publisher  _binary_pub;
ros::Publisher  _grey_pub;

/**
 * Return the temperature in degree celsius
 * @param image      opencv Matrix
 * @param x          image coordinate | width
 * @param y          image coordinate | height
 * @return temperature in celsius
 */
float getPixelTemperature(const cv::Mat image, unsigned int x, unsigned y)
{
   // apply your conversion function





   return 0.0;
}

/**
 * Function to check if a pixel contains a valid temperature
 * @param image      opencv matrix
 * @param x          image coordinate | width
 * @param y          image coordinate | height
 * @return  true if pixel is valid
 */
bool isPixelValid(const cv::Mat image, unsigned int x, unsigned y)
{
   const float t = getPixelTemperature(image, x, y);
   if(t > -20.0f && t <50.0f) return true;
   else                       return false;
}


/**
 * Function to return temperature of center pixel
 * @param image
 * @return temperature in celsius
 */
float getCenterTemperature(const cv::Mat& image)
{
   const unsigned int x = 0;
   const unsigned int y = 0;
   return getPixelTemperature(image, x, y);
}


/**
 * Function to generate a binary image by threshold in celsius
 * @param image   opencv matrix
 * @param th      thermal threshold in celsius
 * @return  image as sensor_msgs::Image
 */
sensor_msgs::Image generateBinaryImageFromThreshold(const cv::Mat& image, const float& th)
{
   // fill header of image
   static unsigned int seq = 0;
   sensor_msgs::Image binary;
   binary.header.frame_id = "thermal_image_view";
   binary.header.seq      = ++seq;
   binary.header.stamp    = ros::Time::now();
   binary.height          = image.rows;
   binary.width           = image.cols;
   binary.step            = image.cols;
   binary.encoding        = "mono8";
   binary.data.resize(binary.height*binary.step);

   unsigned int i=0;
   for(   unsigned int x=0 ; x<image.rows; ++x) {
      for(unsigned int y=0 ; y<image.cols; y++)
      {








      }
   }

   return binary;
}

/**
 * Function to generate a grey scale image
 * @param image      opencv matrix
 * @param minTemperature      minimum temperature in celsius
 * @param maxTemperature      maximum temperature in celsius
 * @return  image as sensor_msgs::Image
 */
sensor_msgs::Image generateGreyImage(const cv::Mat& image, const float& minTemperature = 20, const float& maxTemperature = 35)
{
   // fill header of image
   // In case of any problems, have a look at function generateBinaryImageFromThreshold
   sensor_msgs::Image grey;



   return grey;
}

/**
 * Subscriber function for image from thermal camera
 * @param image
 */
void thermalImageCallback(const sensor_msgs::Image& image)
{
   // conversion from data of image to opencv
   const unsigned short* data = (const unsigned short*)&image.data[0];
   const cv::Mat mat;

   // fill matrix with data, take car of the right format







   float maxValue            = std::numeric_limits<unsigned short>::min();
   float minValue            = std::numeric_limits<unsigned short>::max();
   float average             = 0;
   unsigned int valid_pixels = 0;


   // check for maximum, minimum and average temperature in this loop
   for(   unsigned int w=0 ; w<image.width  ; ++w) {
      for(unsigned int h=0 ; h<image.height ; ++h)
      {
         const float t = getPixelTemperature(mat, w, h);
         if(isPixelValid(mat,w,h))
         {

         }
      }
   }


   ROS_INFO_STREAM("\t center: " << getCenterTemperature(mat) <<
                   "\t max.:   " << maxValue <<
                   "\t min.:   " << minValue <<
                   "\t avg.:   " << average          );



//   _binary_pub.publish(generateBinaryImageFromThreshold(mat, 33));           // Task Advanced 1;
//   _grey_pub.publish(generateGreyImage(mat));                                // Task Advanced 2;
}






int main(int argc, char* argv[])
{
   std::string unique_identifier = "chris";

   ros::init(argc, argv, unique_identifier + "_thermal_workshop_node");
   ros::NodeHandle nh;

   // define here your subscribers
   std::string thermal_topic  = "/optris/thermal_image";

   // place here your subscriber



   _binary_pub = nh.advertise<sensor_msgs::Image>(unique_identifier + "/thermal_binary" ,   1);
   _grey_pub   = nh.advertise<sensor_msgs::Image>(unique_identifier + "/thermal_grey"   ,   1);

   ros::Rate   _looprate(10);
   while(ros::ok())
   {
      ros::spinOnce();
      _looprate.sleep();
   }

   return 0;
}



