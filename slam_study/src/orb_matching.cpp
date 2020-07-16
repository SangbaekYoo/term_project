#include <vector>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/CompressedImage.h>
#include "data_structure.cpp"

using namespace cv;
using namespace std;

Mat img_pre, img_cur;
//create ORB extractor, matcher
Ptr<ORB>orbF = ORB::create(1000);
vector<DMatch> matches;
BFMatcher matcher(NORM_HAMMING);

//frame start
bool start = true;

//frame number
int number=0;
//keyframe number
int frame_idx = 0;
//feature id
int feature_new_id = 0;
//keypoints1 : past, keypoints2 : current
vector<KeyPoint> keypoints1,  keypoints2;
Mat descriptors1, descriptors2;

//save whole keyframe set
vector<frame> frame_set;
//save whole feature set
vector<feature> feature_set;

//first intializing
void feature_extract_start(Mat img1){
    orbF->detectAndCompute(img1,noArray(),keypoints1,descriptors1);
}
//matching past and current feature
void feature_matching(Mat img2, vector<KeyPoint> keypoints, Mat descriptors){
    orbF->detectAndCompute(img2,noArray(),keypoints2,descriptors2);
    matcher.match(descriptors, descriptors2,matches);
}

//search whether feature is already exist in Frame
//using coordinate
int search_feature_coordinate(frame Frame, coordinate P){
    for(int i = 0;i<Frame.Arr_size;i++){
        if(Frame.coordinate_set[i].x==P.x && Frame.coordinate_set[i].y==P.y){
            return i;
        }
    }
    return -1;
}
//using id
int search_feature_id(frame Frame, int id){
    for(int i = 0;i<Frame.Arr_size;i++){
        if(Frame.feature_set[i].id==id){
            return i;
        }
    }
    return -1;
}

void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
    //extract keyframe when 20 frames are passed
    if(number%20==0){
        try
        {
            Mat img;
            img = imdecode(Mat(msg->data),1);
            //only first frame
            if(start){
                start=false;
                feature_extract_start(img);
            }
            //after
            else{
                feature_matching(img, keypoints1, descriptors1);
                Mat img_matches;
                drawMatches( img_pre, keypoints1, img, keypoints2, matches, img_matches, Scalar::all(-1),
                             Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
                //-- Show detected matches
                imshow("Matches", img_matches );
                waitKey(1);
                keypoints1 = keypoints2;
                descriptors1 = descriptors2;
            }
            img_pre = img.clone();
            frame frame(frame_idx);
            frame_set.push_back(frame);
            for (int i=0;i<matches.size();i++){
                int x_c,y_c;
                int x_p,y_p;
                x_p = keypoints1[matches[i].queryIdx].pt.x;
                y_p = keypoints1[matches[i].queryIdx].pt.y;
                x_c = keypoints2[matches[i].trainIdx].pt.x;
                x_c = keypoints2[matches[i].trainIdx].pt.y;
                coordinate coordinate_query(x_p,y_p);
                coordinate coordinate_train(x_c,y_c);
                int idx = search_feature_coordinate(frame_set[frame_idx-1],coordinate_query);
                if(idx != -1){
                    frame.add_feature(frame_set[frame_idx-1].feature_set[idx],coordinate_train);
                }
                else{
                    feature feature(feature_new_id);
                    feature_new_id+=1;
                    feature_set.push_back(feature);
                    frame.add_feature(feature,coordinate_train);
                }
            }
            frame_idx+=1;
            ROS_INFO("size : %d",frame.Arr_size);
            ROS_INFO("feature length : %d",feature_set.size());
        }
	catch (cv_bridge::Exception& e){
                ROS_ERROR("Could not convert to image!");
	}
    }
    number+=1;
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
