#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <boost/foreach.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <pwd.h>
#include <sys/types.h>
#include <boost/filesystem.hpp>

#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;


// About the x axis
double roll_angled = 0.0;
double roll_angler = 0.0;
int roll_angle_slider = 0;

// About the y axis
double pitch_angled = 0.0;
double pitch_angler = 0.0;
int pitch_angle_slider = 0;


// About the z axis
double yaw_angled = 0.0;
double yaw_angler = 0.0;
int yaw_angle_slider = 0;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

PointCloudRGB cloud;

ros::Subscriber pcl_sub;

double cloud_width = 0;
double cloud_height = 0;

double workspace_dist_min = 0.6, workspace_dist_max=0.8;

int workspace_min_x = 0;
int workspace_max_x = 0;
int workspace_min_y = 0;
int workspace_max_y = 0;

int table_topleft_y = 0;
int table_topleft_x = 0;

bool choose_top_left_anchor = false;
bool choose_bottom_right_anchor = false;
bool choose_table_topleft = false;

using namespace std;

string getHomePath()
{
    uid_t uid = getuid();
    struct passwd *pw = getpwuid(uid);

    if (pw == NULL) {
        ROS_ERROR("Failed to get homedir. Cannot save configuration file\n");
        return "";
    }

    // printf("%s\n", pw->pw_dir);
    string str(pw->pw_dir);
    return str;

}

void drawWorkspace(Mat img)
{

    Rect rect(workspace_min_x,workspace_min_y,workspace_max_x-workspace_min_x,workspace_max_y-workspace_min_y);

    rectangle(img,rect,cv::Scalar(0,0,255),2);

}


void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;

        if(cloud.points.size() > 0)
        {
            long index =y*cloud_width;
            index += x;


           Eigen::Affine3f transmat = pcl::getTransformation(0,0,0,roll_angler,pitch_angler,yaw_angler);

           // Apply roll
         /*  double x = cloud.points[index].x;
           double y = cloud.points[index].y*cos(roll_angler) - sin(roll_angler)*cloud.points[index].z;
           double z = cloud.points[index].z*cos(roll_angler) + sin(roll_angler)*cloud.points[index].y;*/

           pcl::transformPointCloud(cloud,cloud,transmat);



            cout<<"Metric coordinates: "<<cloud.points[index].x<<" "<<cloud.points[index].y<<" "<<cloud.points[index].z<<endl;

          //  cout<<"Metric coordinates after transformation: "<<x<<" "<<y<<" "<<z<<endl;


            /*
            //Apply pitch
            double x = cos(pitch_angler)*cloud.points[index].x;
            double y = cloud.points[index].y*cos(angler) - sin(angler)*cloud.points[index].z;
            double z = cloud.points[index].z*cos(angler) + sin(angler)*cloud.points[index].y;

            cout<<"Metric coordinates: "<<cloud.points[index].x<<" "<<cloud.points[index].y<<" "<<cloud.points[index].z<<endl;

            cout<<"Metric coordinates after roll rotation: "<<x<<" "<<y<<" "<<z<<endl;
*/

            if(choose_bottom_right_anchor)
            {
                workspace_max_x = x;
                workspace_max_y = y;

                cout<<"Bottom right anchor: "<<workspace_max_x<<" "<<workspace_max_y<<endl;
            }

            if(choose_top_left_anchor)
            {
                workspace_min_x = x;
                workspace_min_y = y;

                cout<<"Top left anchor: "<<workspace_min_x<<" "<<workspace_min_y<<endl;
            }

            if(choose_table_topleft)
            {
                table_topleft_x = x;
                table_topleft_y = y;

                cout<<"Table top left corner position : "<<table_topleft_x <<" "<<table_topleft_y<<endl;
            }


        }

    }
   /* else if  ( event == EVENT_RBUTTONDOWN )
    {
        cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if  ( event == EVENT_MBUTTONDOWN )
    {
        cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if ( event == EVENT_MOUSEMOVE )
    {
        cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;

    }*/
}


void saveConfig()
{

    string configpath = getHomePath();

    configpath += "/.ros/workspace_segmentation/";

    boost::filesystem::path dir(configpath);

    if(!(boost::filesystem::exists(dir)))
    {
        std::cout<<"Doesn't Exists"<<std::endl;
    }

    if (boost::filesystem::create_directory(dir))
        std::cout << "....Successfully Created !" << std::endl;

    configpath += "manual_workspace.txt";

    ofstream stream(configpath.data());



    if(stream.is_open()){

        stream<<table_topleft_x<<"\n"<<table_topleft_y<<"\n"<<workspace_min_x<<"\n"<<workspace_max_x<<"\n"<<workspace_min_y<<"\n"<<workspace_max_y<<"\n"<<roll_angler<<"\n"<<pitch_angler<<"\n"<<yaw_angler;

        stream.close();

        ROS_INFO("Workspace Dimensions Successfully Saved!");



    }
    else
    {
        ROS_ERROR("Configuration File cannot be opened!!");

    }



}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{



    if((char)cv::waitKey(1) == 27){


        ROS_INFO("Shutting down the node...");
        ros::shutdown();
    }
    else if((char)cv::waitKey(30) == 49) // key 1
    {

        ROS_INFO("Choosing the top left anchor...");

        choose_top_left_anchor = true;
        choose_bottom_right_anchor = false;
        choose_table_topleft = false;



    }
    else if((char)cv::waitKey(30) == 50) // key 2
    {

        ROS_INFO("Choosing the bottom right anchor...");

        choose_top_left_anchor = false;
        choose_bottom_right_anchor = true;
        choose_table_topleft = false;



    }
    else if((char)cv::waitKey(30) == 51) // key 2
    {

        ROS_INFO("Choosing the top left corner of the table...");

        choose_top_left_anchor = false;
        choose_bottom_right_anchor = false;
        choose_table_topleft = true;



    }
    else if((char)cv::waitKey(30) == 's')
    {

        ROS_INFO("Saving workspace dimensions...");

        saveConfig();



    }
    else if((char)cv::waitKey(30) > 50)
    {
        choose_top_left_anchor = false;
        choose_bottom_right_anchor = false;

    }

    if(workspace_max_x > workspace_min_x && workspace_max_y > workspace_min_y)
    {
        drawWorkspace(cv_bridge::toCvShare(msg, "bgr8")->image);
    }

    //Mat imgHSV;

    imshow("Workspace",cv_bridge::toCvShare(msg, "bgr8")->image);
}

void callback(PointCloudRGB::ConstPtr msg)
{
    //printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);

    cloud_width = msg->width;
    cloud_height = msg->height;

    cloud = *msg;


    //extractPlane2(msg);
    //BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
    // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}
void rollangleCallback( int , void* )
{
 roll_angled = (double) roll_angle_slider/4 ;

 roll_angler = roll_angled*M_PI/180;

 cout<<"Roll Angle in degrees"<<roll_angled<<" Roll Angle in radians"<<roll_angler<<endl;

}
void pitchangleCallback( int , void* )
{
 pitch_angled = (double) pitch_angle_slider/4 ;

 pitch_angler = pitch_angled*M_PI/180;

 cout<<"Pitch Angle in degrees"<<pitch_angled<<" Pitch Angle in radians"<<pitch_angler<<endl;

}
void yawangleCallback( int , void* )
{
 yaw_angled = (double) yaw_angle_slider/4 ;

 yaw_angler = yaw_angled*M_PI/180;

 cout<<"Yaw Angle in degrees"<<yaw_angled<<" Yaw Angle in radians"<<yaw_angler<<endl;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "workspace_segmentation_node");
    ros::NodeHandle nh;
    pcl_sub = nh.subscribe<PointCloudRGB>("/kinect2/hd/points", 1, callback);

    //Create a window
    namedWindow("Workspace", 1);

    //set the callback function for any mouse event
    setMouseCallback("Workspace", CallBackFunc, NULL);

    createTrackbar("Roll Angle","Workspace",&roll_angle_slider,1080,rollangleCallback);
    createTrackbar("Pitch Angle","Workspace",&pitch_angle_slider,1080,pitchangleCallback);
    createTrackbar("Yaw Angle","Workspace",&yaw_angle_slider,1080,yawangleCallback);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("kinect2/hd/image_color", 1, imageCallback);



    //show the image
    // imshow("My Window", img);



    //  ros::ServiceServer workspace_service = nh.advertiseService("workspace_segmentation/GetWorkspace",findWorkspace);
    ros::spin();
}


