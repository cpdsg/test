#include <stdlib.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>

int main(int argc, char **argv)
{
  //设置编码
  setlocale(LC_ALL,"");
  //订阅者话题“img_save”
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  //实例化订阅者对象 参数1 话题

  ros::Publisher pub = n.advertise<sensor_msgs::CameraInfo>("/camera/depth/camera_info", 50);   
 
  sensor_msgs::CameraInfo msg;
    // msg.data = "你好啊！！！";
    int count = 0; //消息计数器

    //逻辑(一秒10次)
    ros::Rate r(1);

    //节点不死
    while (ros::ok())
    {
      
     
        pub.publish(msg);
        //加入调试，打印发送的消息
        ROS_INFO("发送的高度消息:%d",msg.height);
        ROS_INFO("发送的计数消息:%d",count);


        //根据前面制定的发送贫频率自动休眠 休眠时间 = 1/频率；
        r.sleep();
        count++;//循环结束前，让 count 自增
        
        ros::spinOnce();
    }


    return 0;
}
