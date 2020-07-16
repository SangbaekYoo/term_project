#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/CompressedImage.h>
using namespace cv;
using namespace std;

void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
        ROS_INFO("recieved");
        try
        {
            Mat img;
            img = imdecode(Mat(msg->data),1);
            ROS_INFO("entered");
            imshow("image",img);
            waitKey(1);
        }
	catch (cv_bridge::Exception& e){
                ROS_ERROR("Could not convert to image!");
	}
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "law_image_subscriber");
        ros::NodeHandle nh;
        ros::Subscriber sub = nh.subscribe("/camera/color/image_raw/compressed",1,imageCallback);
        while(ros::ok()){
            ros::spinOnce();
        }
}

