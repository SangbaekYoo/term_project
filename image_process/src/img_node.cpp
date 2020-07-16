#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/CompressedImage.h>
using namespace cv;
using namespace std;

std_msgs::Int16 steer;
std_msgs::Int16 throttle;

bool flag=false;//false : straight, true : P control
double Kpa=1000,Kpy=3;
double as=0.4,ys=300;
double yt = 340;
double M = 0.2;

double heaviside_pos(double x){
    if(x>0){return x;}
    else{return 0;}
}
double heaviside_neg(double x){
    if(x>0){return 0;}
    else{return x;}
}

void motion_planer(vector<Vec4i> L, const Mat src){
    //Estimate line slope, intecept and left and right end of y value
    double ar = ((double)(L[1][1]-L[1][3]))/((double)(L[1][0]-L[1][2]));
    double br = (double)L[1][1]-ar*(double)L[1][0];
    double yr = ar*src.size().width+br;
    double al = ((double)(L[0][1]-L[0][3]))/((double)(L[0][0]-L[0][2]));
    double bl = (double)L[0][1]-al*(double)L[0][0];
    double yl = bl;
    int steer_cmd;
    static int steer_past=0;
    int steer_cur;
    //Motion Decision
    if(!flag){
        if(ys<yl || ys<yr){flag=true;}
        else{steer_cmd=0;}
    }
    if(flag){
        if(ar>0 && al<0){
            if(ys<yr){steer_cmd = Kpa*heaviside_neg(ar-as)+Kpy*heaviside_neg(ys-yr);}
            else if(ys<yl){steer_cmd = Kpa*heaviside_pos(al+as)+Kpy*heaviside_pos(-ys+yl);}
            else{flag=false;steer_cmd=0;}
        }
        if(ar==al && ar>0){
            if(ys<yr){steer_cmd = Kpa*heaviside_neg(ar-as)+Kpy*heaviside_neg(ys-yr);}
            else{flag=false;steer_cmd=0;}
        }
        if(ar==al && al<0){
            if(ys<yl){steer_cur = Kpa*heaviside_pos(al+as)+Kpy*heaviside_pos(-ys+yl);}
            else{flag=false;steer_cmd=0;}
        }
    }
    //Low Pass Filter
    steer_cur = (int)(M/(M+1)*(double)steer_past+1/(M+1)*(double)steer_cmd);
    steer_past = steer_cur;
    //if steer approach max, min
    if (steer_cur>400){steer_cur=400;}
    if (steer_cur<-400){steer_cur=-400;}
    //curve of corner
    if (yt<yl){steer_cur=400;}
    if (yt<yr){steer_cur=-400;}
    steer.data=1500+(int)steer_cur;
    throttle.data = 1450;
}

vector<Vec4i> track_detection(const Mat src){
    //Line data
    vector<Vec4i> L;
    //Image Blurring
    Mat img_blur;
    GaussianBlur( src, img_blur, Size( 5, 5 ), 0, 0 );
    //Color Detection
    Mat hsv;
    cvtColor(img_blur,hsv,COLOR_BGR2HSV);
    Mat mask_yellow,mask_blue;
    Mat mask;
    inRange(hsv,Scalar(20,100,100),Scalar(30,255,255),mask_yellow);
    inRange(hsv,Scalar(98,109,50),Scalar(112,255,255),mask_blue);
    mask = mask_yellow|mask_blue;
    //Erosion
    Mat mask_erosion;
    Mat element = getStructuringElement(MORPH_RECT,Size(3,3),Point(1,1));
    erode(mask, mask_erosion, element);
    imshow("erode",mask_erosion);
    //Line Detection
    vector<Vec4i> linesP;
    HoughLinesP(mask_erosion, linesP, 1, CV_PI/180, 100, 50, 0 );
    //Lane Detection
    double yw_max = src.size().height/2, y0_max = src.size().height/2;
    double yw, y0;
    Vec4i lw, l0;
    for( size_t i = 0; i < linesP.size(); i++ )
    {
        Vec4i l = linesP[i];
        if (l[1]>cvRound(src.size().height/2) && l[3]>cvRound(src.size().height/2) && l[0] != l[2]){
            double a = ((double)(l[1]-l[3]))/((double)(l[0]-l[2]));
            double b = (double)l[1]-a*((double)l[0]);
            yw = a*src.size().width+b;
            y0 = b;
            if(yw>yw_max&&a>0){yw_max=yw;lw = linesP[i];}
            if(y0>y0_max&&a<0){y0_max=y0;l0 = linesP[i];}
        }
    }
    if(yw_max==src.size().height/2){
        L.push_back(l0);
        L.push_back(l0);
    }
    else if(y0_max==src.size().height/2){
        L.push_back(lw);
        L.push_back(lw);
    }
    else{
        L.push_back(l0);
        L.push_back(lw);
    }
    //Line data
    return L;
}

void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
        try
        {
            Mat img;
            img = imdecode(Mat(msg->data),1);
            motion_planer(track_detection(img),img);
            //image_show(img);
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
        ros::Publisher pub1 = nh.advertise<std_msgs::Int16>("/auto_cmd/steer",1);
        ros::Publisher pub2 = nh.advertise<std_msgs::Int16>("/auto_cmd/throttle",1);
        while(ros::ok()){
            ros::spinOnce();
            pub1.publish(steer);
            pub2.publish(throttle);
        }
}

