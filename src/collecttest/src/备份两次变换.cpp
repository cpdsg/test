#include <ros/ros.h>
#include <stdlib.h>
#include <unistd.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_listener.h>
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include <geometry_msgs/PoseStamped.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" 

#include <string>
#include <cmath>

#include<Eigen/Core>
#include<Eigen/Geometry>

 cv::Mat depth_im;
 cv::Mat depth_im1;
 bool flag;
 bool flag1;

 int img_centre_x_=800/2;
 int img_centre_y_=800/2;
//深度数据保存点
void pic_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
     cv_bridge::CvImagePtr cv_ptr;
     try
     {
      cv_ptr=cv_bridge::toCvCopy(img_msg);
     }
     catch(cv_bridge::Exception& e)
     {
      ROS_ERROR("cv_bridge expection: %s",e.what());
      return;
     }
     depth_im=cv_ptr->image;
     flag=1;  
}

void pic_callback1(const sensor_msgs::ImageConstPtr &img_msg)
{
     cv_bridge::CvImagePtr cv_ptr;
     try
     {
      cv_ptr=cv_bridge::toCvCopy(img_msg);
     }
     catch(cv_bridge::Exception& e)
     {
      ROS_ERROR("cv_bridge expection: %s",e.what());
      return;
     }
     
     depth_im1=cv_ptr->image;
     flag1=1;
   
}

int main(int argc, char **argv)
{
  //订阅者话题“img_save”
  ros::init(argc, argv, "img_save");
  ros::NodeHandle n;

  image_transport::ImageTransport it_(n);
  image_transport::ImageTransport it1_(n);
  image_transport::Subscriber image_sub_;
  ros::AsyncSpinner spinner(2);
  //实例化订阅者对象 参数1 话题
  image_transport::Subscriber  sub_image = it_.subscribe("/probot_anno/camera/depth/image_raw", 1, pic_callback);     
  image_transport::Subscriber  sub_image1 = it1_.subscribe("/probot_anno/camera1/depth/image_raw", 1, pic_callback1);     
  
  tf::StampedTransform transform_camera_to_world;
  tf::StampedTransform transform_camera1_to_world;

  tf::TransformListener tf_camera_to_camera1;

  tf::Vector3 obj_camera_frame,obj_rotbot_frame;
  tf::Vector3 obj_camera1_frame;
  /*
  *获取变换坐标
  */
  try
  {
    tf_camera_to_camera1.waitForTransform("/base_link", "/camera_link", ros::Time(0), ros::Duration(50.0));
    tf_camera_to_camera1.waitForTransform("/base_link", "/camera1_link", ros::Time(0), ros::Duration(50.0));

  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("[adventure_tf]: (wait) %s", ex.what());
    ros::Duration(1.0).sleep();
  }
  try
  {
    tf_camera_to_camera1.lookupTransform("/base_link", "/camera_link", ros::Time(0), transform_camera_to_world);
    tf_camera_to_camera1.lookupTransform("/base_link", "/camera1_link", ros::Time(0), transform_camera1_to_world);
    //参数4获得从cam ->base的变换 就是参数
  }

  catch (tf::TransformException &ex)
  {
    ROS_ERROR("[adventure_tf]: (lookup) %s", ex.what());
  }
    std::cout<<"frame_id_:"<<transform_camera_to_world.frame_id_<<std::endl;
    std::cout<<"transform.child_frame_id_:"<<transform_camera_to_world.child_frame_id_<<std::endl<<std::endl;
    // std::cout<<"robot到相机空间的平移 "<<std::endl;
    // std::cout<<"getOrigin x= "<< transform_camera_to_world.getOrigin().x() <<std::endl;
    // std::cout<<"getOrigin y= "<< transform_camera_to_world.getOrigin().y() <<std::endl; 
    // std::cout<<"getOrigin z= "<< transform_camera_to_world.getOrigin().z() <<std::endl<<std::endl;
  ros::Rate rate(10000);//指定频
  ros::Duration du(0.01);
    ros::spinOnce();
    du.sleep();
    ros::spinOnce();
       if(flag1&&flag)
    {
      float x,y,z;
      float pixels_permm_x=526.174276;
      float pixels_permm_y=526.174276;

      cv::imshow("depthlow0", depth_im1);
      cv::waitKey(0);
      cv::imshow("depth", depth_im);
      cv::waitKey(0);
        //获取像素点（400，400）的相机空间坐标
        // for(int i=0;i<800;i++)
        //   for(int j=0;j<800;j++)
         for(int i=400;i<800;i++)
          for(int j=100;j<600;j++)
        {
          x = (i - img_centre_x_) / pixels_permm_x;
	        y = (j- img_centre_y_) / pixels_permm_y;
          z = depth_im.at<float>(i,j);
          obj_camera_frame.setZ(z);
          obj_camera_frame.setY(y);
          obj_camera_frame.setX(x);

          std::cout<<"光源相机空间下该点的空间坐标 "<<std::endl;
          std::cout<<"obj_camera_frame x= "<< obj_camera_frame.getX() <<std::endl;
          std::cout<<"obj_camera_frame y= "<< obj_camera_frame.getY() <<std::endl; 
          std::cout<<"obj_camera_frame z= "<< obj_camera_frame.getZ() <<std::endl<<std::endl;

          obj_camera_frame.setZ(-y);
          obj_camera_frame.setY(-x);
          obj_camera_frame.setX(z);

          //在相机1坐标系下的坐标，
          obj_rotbot_frame = transform_camera_to_world*obj_camera_frame;
            
          std::cout<<"robot相机空间坐标 "<<std::endl;
          std::cout<<"obj_camera1_frame x= "<< obj_rotbot_frame.getX()<<std::endl;
          std::cout<<"obj_camera1_frame y= "<< obj_rotbot_frame.getY()<<std::endl; 
          std::cout<<"obj_camera1_frame z= "<< obj_rotbot_frame.getZ()<<std::endl<<std::endl;


          obj_camera1_frame=transform_camera1_to_world.inverse()*obj_camera_frame;

          std::cout<<"视觉相机的像素点位置是："<<std::endl;
          float new_x=(obj_camera1_frame.getZ())*pixels_permm_x+img_centre_x_;
          std::cout<<new_x<<std::endl;
          float new_y=obj_camera1_frame.getX()*pixels_permm_y+img_centre_y_;
          std::cout<<new_y<<std::endl;
          if(new_x>799||new_x==0||new_y>799||new_y<0)
            continue;
          std::cout<<"obj_camera1_frame x= "<< obj_camera1_frame.getZ()<<std::endl;
          std::cout<<"obj_camera1_frame y= "<< obj_camera1_frame.getX()<<std::endl;
          std::cout<<"obj_camera1_frame z= "<< obj_camera1_frame.getY()<<std::endl<<std::endl;
          std::cout<<"该点的高度是： = "<<depth_im1.at<float>(round(new_x),round(new_y)) <<std::endl; 

          if(obj_camera1_frame.getY()-depth_im1.at<float>(round(new_x),round(new_y))>0.4){
          depth_im1.at<float>(round(new_x),round(new_y))=255;
          cv::imshow("depthlow", depth_im1);
          cv::waitKey(0);  
          std::cout<<"depth_im该点的高度是： = "<<depth_im.at<float>(round(i),round(j)) <<std::endl;
          // pause();
          
          }

        }
      cv::imshow("depthlow", depth_im1);
      cv::waitKey(0);  
    }
  return 0;
}