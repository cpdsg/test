#include <ros/ros.h>
#include <stdlib.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>

//深度数据保存点
std::string filename_depthdata="/home/cp/gelsight/src/collecttest/src/1.png";
void pic_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
     ros::Time time = img_msg->header.stamp;
     std::string str;
     str = std::to_string(time.sec) + '.' + std::to_string(time.nsec) + ".jpg";
     cv_bridge::CvImageConstPtr ptr;
     ptr = cv_bridge::toCvCopy(img_msg, "32FC1");
     printf("%s\n", str.c_str());
     cv::imwrite(filename_depthdata,ptr->image);
     cv::imshow("depthlow", ptr->image);
     cv::waitKey(0);
   
}


int main(int argc, char **argv)
{
  //订阅者话题“img_save”
  ros::init(argc, argv, "img_save");
  ros::NodeHandle n;
  //实例化订阅者对象 参数1 话题
  ros::Subscriber sub_image = n.subscribe("/camera1/depth/image_raw", 50, pic_callback);     
 
  ros::spin();
  return 0;
}
