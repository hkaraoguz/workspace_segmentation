#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <pwd.h>
#include <sys/types.h>
#include <boost/filesystem.hpp>


//#include <workspace_segmentation/Workspace.h>
#include <workspace_segmentation/GetWorkspaceRequest.h>
#include <workspace_segmentation/GetWorkspaceResponse.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

ros::Subscriber pcl_sub;

double cloud_width = 0;
double cloud_height = 0;

double workspace_dist_min = 0.6, workspace_dist_max=0.8;

int workspace_min_x = 0;
int workspace_max_x = 0;
int workspace_min_y = 0;
int workspace_max_y = 0;

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



void extractPlane(PointCloud::ConstPtr msg)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (msg);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.5, 1.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);
    // Create the filtering object: downsample the dataset using a leaf size of 1cm

    /*pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud (cloud_filtered);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered_downsampled);*/

    // *cloud_filtered_downsampled = *cloud_filtered;
    std::cout << "PointCloud after filtering has: " << cloud_filtered_downsampled->points.size ()  << " data points." << std::endl;

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.01);

    int i=0, nr_points = (int) cloud_filtered_downsampled->points.size ();
    while (cloud_filtered_downsampled->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered_downsampled);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered_downsampled);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;


        /*pass.setInputCloud (cloud_plane);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.5, 1.0);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*cloud_plane);*/


        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered_downsampled = *cloud_f;

        std::stringstream ss;
        ss << "plane_" << i << ".pcd";
        i++;
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_plane, false); //*



    }

    //return;

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_plane);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.01); // 2cm
    ec.setMinClusterSize (1000);
    ec.setMaxClusterSize (1000000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_plane);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            cloud_cluster->points.push_back(cloud_plane->points[*pit]);
            std::cout<<msg->points[*pit].z<<" "<<std::endl;
        }


        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";

        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
        j++;
    }

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

    configpath += "workspace.txt";

    ofstream stream(configpath.data());



    if(stream.is_open()){

        stream<<workspace_min_x<<"\n"<<workspace_max_x<<"\n"<<workspace_min_y<<"\n"<<workspace_max_y;

        stream.close();

        ROS_INFO("Workspace Dimensions Successfully Saved!");



    }
    else
    {
        ROS_ERROR("Configuration File cannot be opened!!");

    }



}



void extractPlaneGlobalIndices(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,float x_min,float x_max,float y_min,float y_max,float z_min,float z_max,int i)
{

    pcl::PCDWriter writer;

    /** \brief @b PassThrough passes points in a cloud based on constraints for one particular field of the point type.
      48     * \details Iterates through the entire input once, automatically filtering non-finite points and the points outside
      49     * the interval specified by setFilterLimits(), which applies only to the field specified by setFilterFieldName().
      50     * <br><br>
      51     * Usage example:
      52     * \code
      53     * pcl::PassThrough<PointType> ptfilter (true); // Initializing with true will allow us to extract the removed indices
      54     * ptfilter.setInputCloud (cloud_in);
      55     * ptfilter.setFilterFieldName ("x");
      56     * ptfilter.setFilterLimits (0.0, 1000.0);
      57     * ptfilter.filter (*indices_x);
      58     * // The indices_x array indexes all points of cloud_in that have x between 0.0 and 1000.0
      59     * indices_rem = ptfilter.getRemovedIndices ();
      60     * // The indices_rem array indexes all points of cloud_in that have x smaller than 0.0 or larger than 1000.0
      61     * // and also indexes all non-finite points of cloud_in
      62     * ptfilter.setIndices (indices_x);
      63     * ptfilter.setFilterFieldName ("z");
      64     * ptfilter.setFilterLimits (-10.0, 10.0);
      65     * ptfilter.setNegative (true);
      66     * ptfilter.filter (*indices_xz);
      67     * // The indices_xz array indexes all points of cloud_in that have x between 0.0 and 1000.0 and z larger than 10.0 or smaller than -10.0
      68     * ptfilter.setIndices (indices_xz);
      69     * ptfilter.setFilterFieldName ("intensity");
      70     * ptfilter.setFilterLimits (FLT_MIN, 0.5);
      71     * ptfilter.setNegative (false);
      72     * ptfilter.filter (*cloud_out);
      73     * // The resulting cloud_out contains all points of cloud_in that are finite and have:
      74     * // x between 0.0 and 1000.0, z larger than 10.0 or smaller than -10.0 and intensity smaller than 0.5.
      75     * \endcode
      76     * \author Radu Bogdan Rusu
      77     * \ingroup filters
      78     */

    pcl::PassThrough<pcl::PointXYZRGB> ptfilter;
    boost::shared_ptr<std::vector<int> > indices_x (new std::vector<int> ());
    boost::shared_ptr<std::vector<int> > indices_xz (new std::vector<int> ());
    boost::shared_ptr<std::vector<int> > indices_xzy (new std::vector<int> ());
    pcl::PointIndices::Ptr inliers_x (new pcl::PointIndices);
    pcl::PointIndices::Ptr inliers_xz (new pcl::PointIndices);
    pcl::PointIndices::Ptr inliers_xzy (new pcl::PointIndices);

    ptfilter.setInputCloud (cloud);
    ptfilter.setFilterFieldName ("x");
    ptfilter.setFilterLimits (x_min, x_max);
    ptfilter.filter (*indices_x);
    ptfilter.setIndices (indices_x);
    ptfilter.setFilterFieldName ("z");
    ptfilter.setFilterLimits (z_min, z_max);
    ptfilter.filter (*indices_xz);

    ptfilter.setIndices (indices_xz);
    ptfilter.setFilterFieldName ("y");
    ptfilter.setFilterLimits (y_min, y_max);
    ptfilter.filter (*indices_xzy);


    PointCloudRGB::Ptr result (new PointCloudRGB);


    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (indices_xzy);
    extract.setNegative (false);
    extract.filter (*result);

    std::vector<int> globalindices = *indices_xzy;
    std::sort (globalindices.begin(), globalindices.end());


   // std::cout<<globalindices[0]<<" Last:"<<globalindices[globalindices.size()-1]<<std::endl;

    int max = 0;
    long index = 0;

    workspace_min_y = floor((float)globalindices[0]/cloud->width);
    workspace_min_x = globalindices[0]%cloud->width;

    for(int k = 0; k< globalindices.size();k++)
    {
        int rem = globalindices[k]%cloud->width;

        if(rem > max)
        {
            max = rem;
        }

    }

    //std::cout<<"Max remainder "<<max<<std::endl;

    workspace_max_x = max;

    workspace_max_y = floor((float)globalindices.back()/cloud->width);

    ROS_INFO("Found workspace with x,y coordinates as %d %d %d %d \n",workspace_min_x,workspace_max_x,workspace_min_y,workspace_max_y);

    saveConfig();

    std::stringstream ss;
    ss << "result_" << i << ".pcd";
    //i++;
    // Save ascii must be true to get the color right
    writer.write<pcl::PointXYZRGB> (ss.str (), *result, true); //*



}


void extractPlane2(PointCloudRGB::ConstPtr msg)
{

    pcl::PCDWriter writer;
    PointCloudRGB::Ptr cloud_downsampled (new PointCloudRGB);
    PointCloudRGB::Ptr cloud_f (new PointCloudRGB);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (msg);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_downsampled);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud_downsampled);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.6, 0.8);
    pass.filter (*cloud_downsampled);

    //*cloud_downsampled = *msg;
    std::cout << "PointCloud after filtering has: " << cloud_downsampled->points.size ()  << " data points." << std::endl;

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    PointCloudRGB::Ptr cloud_plane (new PointCloudRGB());

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);

    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.01);

    int i=0, nr_points = (int) cloud_downsampled->points.size ();

    std::vector<PointCloudRGB> planes;

    while (cloud_downsampled->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_downsampled);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (cloud_downsampled);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_plane);

        /*cloud_plane->height = msg->height;
        cloud_plane->width = msg->width;
        cloud_plane->is_dense = false;
        cloud_plane->points.resize(cloud_plane->height * cloud_plane->width);*/

        pcl::PointXYZRGB minPt, maxPt;

        pcl::getMinMax3D(*cloud_plane,minPt, maxPt);

        std::cout << "Max x: " << maxPt.x << std::endl;
        std::cout << "Max y: " << maxPt.y << std::endl;
        std::cout << "Max z: " << maxPt.z << std::endl;
        std::cout << "Min x: " << minPt.x << std::endl;
        std::cout << "Min y: " << minPt.y << std::endl;
        std::cout << "Min z: " << minPt.z << std::endl;

        float avgDist = fabs((float)(minPt.z + maxPt.z)/2.0);

        // Get the points associated with the planar surface
        std::cout << "Height, width of the the planar component: " << cloud_plane->height  << " " <<cloud_plane->width<< std::endl;
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
        std::cout << "Planar coefficients: " << coefficients->values[0] << " " <<" "<<coefficients->values[1]<<" "<<coefficients->values[2]<<std::endl;

        // If the plane is orthogonal to the z direction with minimal drift in x and y
        if(coefficients->values[0] < 0.1 && coefficients->values[1] < 0.1 && coefficients->values[2] > 0.9 && avgDist>= workspace_dist_min && avgDist <= workspace_dist_max)
        {
            planes.push_back(*cloud_plane);
            std::stringstream ss;
            ss << "plane_" << i << ".pcd";
            i++;
            // Save ascii must be true to get the color right
            writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_plane, true);

        }

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_downsampled = *cloud_f;

    }

    // return;

    int j = 0;
    int max_size = 0;

    PointCloudRGB::Ptr workspace_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

    // For each extracted plane, try to get the individual connected plane surfaces, and output the largest one as the workspace
    for(int m = 0; m < planes.size(); m++)
    {

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud (planes[m].makeShared());

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance (0.01); // 2cm
        ec.setMinClusterSize (2000);
        ec.setMaxClusterSize (1000000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (planes[m].makeShared());
        ec.extract (cluster_indices);


        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            PointCloudRGB::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
                cloud_cluster->points.push_back(planes[m].points[*pit]);
                // std::cout<<(int)msg->points[*pit].r<<" "<<(int)msg->points[*pit].g<<" "<<(int)msg->points[*pit].b<<std::endl;
            }

            if(cloud_cluster->points.size() > max_size)
            {
                max_size = cloud_cluster->points.size();
                *workspace_cluster = *cloud_cluster;
            }


            j++;
        }
    }

    if(workspace_cluster->points.size() > 0)
    {
        pcl::PointXYZRGB minPt, maxPt;

        pcl::getMinMax3D(*workspace_cluster,minPt, maxPt);

        std::cout << "Max x: " << maxPt.x << std::endl;
        std::cout << "Max y: " << maxPt.y << std::endl;
        std::cout << "Max z: " << maxPt.z << std::endl;
        std::cout << "Min x: " << minPt.x << std::endl;
        std::cout << "Min y: " << minPt.y << std::endl;
        std::cout << "Min z: " << minPt.z << std::endl;

        extractPlaneGlobalIndices(msg,minPt.x,maxPt.x,minPt.y,maxPt.y,minPt.z,maxPt.z,j);

        workspace_cluster->width = workspace_cluster->points.size ();
        workspace_cluster->height = 1;
        workspace_cluster->is_dense = true;

        std::cout << "PointCloud representing the workspace has: " << workspace_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "workspace_cluster"<<".pcd";

        writer.write<pcl::PointXYZRGB> (ss.str (), *workspace_cluster, true);

        pcl_sub.shutdown();
        ros::shutdown();

    }

}


void findWorkspace(workspace_segmentation::GetWorkspaceRequest& request, workspace_segmentation::GetWorkspaceResponse& response)
{
    workspace_dist_max = request.max_distance;

    workspace_dist_min = request.min_distance;

   // pcl_sub = nh.subscribe<PointCloudRGB>("/kinect2/qhd/points", 1, callback);

}




void callback(PointCloudRGB::ConstPtr msg)
{
    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);

    cloud_width = msg->width;
    cloud_height = msg->height;

    extractPlane2(msg);
    //BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
    // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "workspace_segmentation_node");
    ros::NodeHandle nh;
    pcl_sub = nh.subscribe<PointCloudRGB>("/kinect2/hd/points", 1, callback);
  //  ros::ServiceServer workspace_service = nh.advertiseService("workspace_segmentation/GetWorkspace",findWorkspace);
    ros::spin();
}
