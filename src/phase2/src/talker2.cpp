/*This scirpt is able to detect up to 10(can be more, 
just need to change the drawing rectangle function)faces using the haarcascade
method which is a part of opencv. It uses the realsense library to access depth data and frames
I say it can handle up to 10 but I havent testested it. Its hard to find 10 faces to track
 */


/*Goals
Working towards getting the script to work with some use of ROS. 
To do this I will make two nodes communicate with eachother. 
One node will run the facial detection algorithm and send the information to another node
Another listener node will be listening to the depth data that is being sent out
This data will simply be how far away the detected face is. Hopefully we are able to get it working with multiple faces.
We will then use the rqt plotting function to plot how far away the face was over time. 

- make talker node(done)
- make listener node(done)
- make CMAKE (Done)
There seems to be an error with talker. Specifically it seems to be from realsense the 
error looks familiar to something we had before but I dont remember what  caused it. Once we fix that we 
should hopefully be able to catkin_make successfully then run the nodes
- make talker and listener function correctly
 */
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <sstream> 
//Include all necessary OpenCV header files
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include "opencv2/videoio.hpp"
//#include <opencv2/core/eigen.hpp>

// Include RealSense Library
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_processing.hpp>

using namespace cv;
int main(int argc, char **argv){
  //init ros

  ros::init(argc,argv,"talker2");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("chatter",1000);
  ros::Rate looprate(5);
  
  //facial recognition - load haarcascade xml file
  cv::CascadeClassifier faceCascade;
  std::string path = "/home/nvidia/opencv_build/tutorial/haarcascade_frontalface_default.xml";
  faceCascade.load(path);
  if(faceCascade.empty()){
    std::cout << "XML not loaded \n";
    ROS_INFO("Not Loaded");

  }
  std::vector<cv::Rect> faces;  
  //realsense prep
  rs2::pipeline pipe;
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_DEPTH);
  cfg.enable_stream(RS2_STREAM_COLOR);
  auto profile = pipe.start(cfg);
  rs2::align align_to_color(RS2_STREAM_COLOR);
  rs2::align align_to_depth(RS2_STREAM_DEPTH);//one of these two is reduntant
  while(ros::ok){
    std_msgs::Float32 msg;
    
    rs2::frameset fs = pipe.wait_for_frames();
    rs2::frameset fs_aligned = align_to_color.process(fs);
    rs2::frame color_frame = fs_aligned.get_color_frame();
    const int w2 = color_frame.as<rs2::video_frame>().get_width();
    const int h2 = color_frame.as<rs2::video_frame>().get_height();
    cv::Mat bgr_img(cv::Size(w2,h2), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat rgb_img;
    cv::cvtColor(bgr_img, rgb_img, cv::COLOR_BGR2RGB);
    rs2::depth_frame depth_frame = fs_aligned.get_depth_frame();
    cv::cvtColor(bgr_img, rgb_img, cv::COLOR_BGR2RGB);
    faceCascade.detectMultiScale(rgb_img,faces,1.1,10);
    for (int i = 0;i<faces.size();i++){
      //place point at the center of the face box which we will then use to give us the distance to the fac

      cv::rectangle(rgb_img,faces[i].tl(), faces[i].br(),(0,0,255),3);
      auto horizontalCenter=(faces[i].br().x + faces[i].tl().x)/2;
      auto verticalCenter = (faces[i].tl().y + faces[i].br().y)/2;
      auto center = cv::Point2f(horizontalCenter,verticalCenter);      
      cv::circle(rgb_img, center, 10,(255,0,0),-1);
      float dist_to_center = depth_frame.get_distance(horizontalCenter,verticalCenter);
      imshow("Color Frame", rgb_img);
      ROS_INFO("I am sending a distance of [%f]", dist_to_center);
      msg.data = dist_to_center;//diff type error maybe....well see    load message
	// std::cout << "The face "<< i << " " << dist_to_center << " meters away \n";
    }
    chatter_pub.publish(msg);//publish message
   

    ros::spinOnce();
    looprate.sleep();
  }
  return 0;
}
