// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <math.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include<pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

typedef pcl::PointXYZI PointType;

int ndtNum = 0;

double timeLaserCloudCornerLast = 0;
double timeLaserCloudSurfLast = 0;
double timeLaserCloudFullRes = 0;

bool newLaserCloudCornerLast = false;
bool newLaserCloudSurfLast = false;
bool newLaserCloudFullRes = false;

bool firstRelocalization = false;
bool rotation_flag = false;

pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast_down(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast_down(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudOri(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr coeffSel(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurround_corner(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap_relo(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap_relo(new pcl::PointCloud<PointType>());

pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudALLFromMap_relo(new pcl::PointCloud<pcl::PointXYZRGB>());

pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap_relo_down(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap_relo_down(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr ndt_input_cloud(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr ndt_output_cloud(new pcl::PointCloud<PointType>);

std::vector< Eigen::Matrix4f > pose_map;
std::vector< Eigen::Matrix4f > add_pose_map;

pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes2(new pcl::PointCloud<PointType>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudFullResColor(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudFullResColor_pcd(new pcl::PointCloud<pcl::PointXYZRGB>());

//kd-tree
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

float transformTobeMapped[6] = {0};
float transformAftMapped[6] = {0};

// sub map for initial relocalization
pcl::PointCloud<PointType>::Ptr sub_corner_map(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr sub_surf_map(new pcl::PointCloud<PointType>());
//kd-tree
pcl::KdTreeFLANN<PointType>::Ptr kdtree_subcorner(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtree_subsurf(new pcl::KdTreeFLANN<PointType>());

pcl::PointCloud<PointType>::Ptr subCloudOri(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr subcoeffSel(new pcl::PointCloud<PointType>());

double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}
double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}
void transformUpdate()
{
    for (int i = 0; i < 6; i++) {
        transformAftMapped[i] = transformTobeMapped[i];
    }
}
//lidar coordinate sys to world coordinate sys
void pointAssociateToMap(PointType const * const pi, PointType * const po)
{
    //rot z（transformTobeMapped[2]）
    float x1 = cos(transformTobeMapped[2]) * pi->x
            - sin(transformTobeMapped[2]) * pi->y;
    float y1 = sin(transformTobeMapped[2]) * pi->x
            + cos(transformTobeMapped[2]) * pi->y;
    float z1 = pi->z;

    //rot x（transformTobeMapped[0]）
    float x2 = x1;
    float y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
    float z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

    //rot y（transformTobeMapped[1]）then add trans
    po->x = cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2
            + transformTobeMapped[3];
    po->y = y2 + transformTobeMapped[4];
    po->z = -sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2
            + transformTobeMapped[5];
    po->intensity = pi->intensity;
}

void RGBpointAssociateToMap(PointType const * const pi, pcl::PointXYZRGB * const po)
{
    float x1 = cos(transformTobeMapped[2]) * pi->x
            - sin(transformTobeMapped[2]) * pi->y;
    float y1 = sin(transformTobeMapped[2]) * pi->x
            + cos(transformTobeMapped[2]) * pi->y;
    float z1 = pi->z;

    float x2 = x1;
    float y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
    float z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

    po->x = cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2
            + transformTobeMapped[3];
    po->y = y2 + transformTobeMapped[4];
    po->z = -sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2
            + transformTobeMapped[5];
    //po->intensity = pi->intensity;

    float intensity = pi->intensity;
    intensity = intensity - std::floor(intensity);

    int reflection_map = intensity*10000;

    if (reflection_map < 30)
    {
        int green = (reflection_map * 255 / 30);
        po->r = 0;
        po->g = green & 0xff;
        po->b = 0xff;
    }
    else if (reflection_map < 90)
    {
        int blue = (((90 - reflection_map) * 255) / 60);
        po->r = 0x0;
        po->g = 0xff;
        po->b = blue & 0xff;
    }
    else if (reflection_map < 150)
    {
        int red = ((reflection_map-90) * 255 / 60);
        po->r = red & 0xff;
        po->g = 0xff;
        po->b = 0x0;
    }
    else
    {
        int green = (((255-reflection_map) * 255) / (255-150));
        po->r = 0xff;
        po->g = green & 0xff;
        po->b = 0;
    }
}

void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudCornerLast2)
{
    timeLaserCloudCornerLast = laserCloudCornerLast2->header.stamp.toSec();

    laserCloudCornerLast->clear();
    pcl::fromROSMsg(*laserCloudCornerLast2, *laserCloudCornerLast);

    newLaserCloudCornerLast = true;
}

void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudSurfLast2)
{
    timeLaserCloudSurfLast = laserCloudSurfLast2->header.stamp.toSec();

    laserCloudSurfLast->clear();
    pcl::fromROSMsg(*laserCloudSurfLast2, *laserCloudSurfLast);

    newLaserCloudSurfLast = true;
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2)
{
    timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();

    laserCloudFullRes->clear();
    laserCloudFullResColor->clear();
    pcl::fromROSMsg(*laserCloudFullRes2, *laserCloudFullRes);

    newLaserCloudFullRes = true;
}

//use icp/ndt to match
double initial_first_pose()
{
    Eigen::Matrix4f initial_pose;
    double min_score = 100;

    // // add rotation initial guess
    // Eigen::Matrix4f rotation_pose;
    // rotation_pose << -1,0,0,0,
    //                  0,-1,0,0,
    //                  0,0, 1,0,
    //                  0,0,0, 1;
    // Eigen::Matrix4f rotation_pose1;
    // rotation_pose1 << 0,-1,0,0,
    //                  1,0,0,0,
    //                  0,0, 1,0,
    //                  0,0,0, 1;
    // Eigen::Matrix4f rotation_pose2;
    // rotation_pose2 << 0,1,0,0,
    //                  -1,0,0,0,
    //                  0,0, 1,0,
    //                  0,0,0, 1;
    // for(auto kf : pose_map){
        
    //     Eigen::Matrix4f temp_kf = kf * rotation_pose;
    //     Eigen::Matrix4f temp_kf1 = kf * rotation_pose1;
    //     Eigen::Matrix4f temp_kf2 = kf * rotation_pose2;

    //     add_pose_map.push_back(kf);
    //     add_pose_map.push_back(temp_kf);
    //     add_pose_map.push_back(temp_kf1);
    //     add_pose_map.push_back(temp_kf2);
    // }

    //find the best kf pose
    for(auto kf : pose_map){
        //pcl::NormalDistributionsTransform<PointType,PointType> ndt;
        pcl::IterativeClosestPoint<PointType,PointType> ndt;

        //ndt_input_cloud = laserCloudSurfStack2;
        // ndt.setTransformationEpsilon(0.01);
        // ndt.setResolution(1);
        // ndt.setStepSize(0.1);
        ndt.setMaximumIterations(20); 

        //ndt.setInputCloud(ndt_input_cloud);
        ndt.setInputSource(ndt_input_cloud);
        ndt.setInputTarget(laserCloudCornerFromMap_relo);

        Eigen::Matrix4f init_guess;
        init_guess = kf;

        ndt.align(*ndt_output_cloud, init_guess);

        std::cout << "init_guess KF : " << std::endl << init_guess << std::endl;
        std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
        << " score: " << ndt.getFitnessScore() << std::endl;

        double score = ndt.getFitnessScore();

        if(score < min_score) {
            min_score = score;
            initial_pose = init_guess;
        }   
    }

    //match and transform
    pcl::IterativeClosestPoint<PointType,PointType> ndt_final;

    ndt_final.setMaximumIterations(100); 
    ndt_final.setInputSource(ndt_input_cloud);
    ndt_final.setInputTarget(laserCloudCornerFromMap_relo);

    std::cout<<" DEBUG initial_pose: "<< initial_pose << "min_score: "<<min_score << std::endl;

    ndt_final.align(*ndt_output_cloud, initial_pose);
    std::cout << "Normal Distributions Transform has converged:" << ndt_final.hasConverged()
    << " score: " << ndt_final.getFitnessScore() << std::endl;

    double temp_score = ndt_final.getFitnessScore();

    pcl::transformPointCloud(*ndt_input_cloud, *ndt_output_cloud, ndt_final.getFinalTransformation());

    std::cout<<" DEBUG : "<< ndt_final.getFinalTransformation() << std::endl;

    Eigen::Matrix4f relocal_pose = ndt_final.getFinalTransformation();

    Eigen::Matrix3f relocal_R = relocal_pose.block<3,3>(0,0);
    Eigen::Quaternionf relocal_Q(relocal_R);
    Eigen::Vector3f relocal_tt = relocal_pose.block<3,1>(0,3);
    Eigen::Vector3f euler_angles = relocal_R.eulerAngles(2,0,1);

    std::cout<<" DEBUG relocal_pose: "<< relocal_pose << std::endl;

    transformTobeMapped[0] = euler_angles(0);
    transformTobeMapped[1] = euler_angles(1);
    transformTobeMapped[2] = euler_angles(2);
    transformTobeMapped[3] = relocal_tt(0);
    transformTobeMapped[4] = relocal_tt(1);
    transformTobeMapped[5] = relocal_tt(2);

    transformAftMapped[0] = euler_angles(0);
    transformAftMapped[1] = euler_angles(1);
    transformAftMapped[2] = euler_angles(2);
    transformAftMapped[3] = relocal_tt(0);
    transformAftMapped[4] = relocal_tt(1);
    transformAftMapped[5] = relocal_tt(2);

    std::cout<<"DEBUG ICP END-----------------------------------------"<<std::endl;
    ndt_input_cloud->clear();

    return temp_score;
}
// void rotation_mapping() //TODO
// {
// }
int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>
            ("/laser_cloud_sharp", 2, laserCloudCornerLastHandler);

    ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>
            ("/laser_cloud_flat", 2, laserCloudSurfLastHandler);

    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>
            ("/livox_cloud", 2, laserCloudFullResHandler);

    ros::Publisher pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>
            ("/laser_cloud_surround", 1);
    ros::Publisher pubLaserCloudSurround_corner = nh.advertise<sensor_msgs::PointCloud2>
            ("/laser_cloud_surround_corner", 1);

    ros::Publisher pubNDTCloud = nh.advertise<sensor_msgs::PointCloud2>
            ("/ndt_out_put", 1);
    ros::Publisher puboldmap_allpoints = nh.advertise<sensor_msgs::PointCloud2>
            ("/oldmap_allpoints", 1);
    ros::Publisher puboldmap_corner = nh.advertise<sensor_msgs::PointCloud2>
            ("/oldmap_corner", 1);
    ros::Publisher puboldmap_surf = nh.advertise<sensor_msgs::PointCloud2>
           ("/oldmap_surf", 1); 

    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 1);
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "/camera_init";
    odomAftMapped.child_frame_id = "/aft_mapped";

    ros::Publisher pubRelocalPose = nh.advertise<nav_msgs::Odometry> ("/re_local_pose", 1);
    nav_msgs::Odometry RelocalPose;
    RelocalPose.header.frame_id = "/camera_init";
    RelocalPose.child_frame_id = "/aft_mapped";

    ros::Publisher pubOldMapPose = nh.advertise<nav_msgs::Odometry> ("/old_map_pose", 1);
    nav_msgs::Odometry OldMapPose;
    OldMapPose.header.frame_id = "/camera_init";
    OldMapPose.child_frame_id = "/aft_mapped";

    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform aftMappedTrans;
    aftMappedTrans.frame_id_ = "/camera_init";
    aftMappedTrans.child_frame_id_ = "/aft_mapped";

    ros::Publisher pubOdomInit = nh.advertise<nav_msgs::Odometry> ("/mapped_for_odo_init", 1);
    nav_msgs::Odometry odomInit;

    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);

    nav_msgs::Path laserPath;

    std::string map_file_path;
    ros::param::get("~map_file_path",map_file_path);
    bool use_map_update;
    ros::param::get("~use_map_update",use_map_update);

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    PointType pointOri, pointSel,pointSel_only, coeff;

    cv::Mat matA0(10, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matB0(10, 1, CV_32F, cv::Scalar::all(-1));
    cv::Mat matX0(10, 1, CV_32F, cv::Scalar::all(0));

    cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

    bool isDegenerate = false;
    cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));
    //VoxelGrid
    pcl::VoxelGrid<PointType> downSizeFilterCorner;

    downSizeFilterCorner.setLeafSize(0.1, 0.1, 0.1);

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    downSizeFilterSurf.setLeafSize(0.2, 0.2, 0.2);

    pcl::VoxelGrid<PointType> downSizeFilterMap;
    downSizeFilterMap.setLeafSize(0.1, 0.1, 0.1);

    pcl::VoxelGrid<PointType> downSizeFilterFull;
    downSizeFilterFull.setLeafSize(0.15, 0.15, 0.15);

    //------------------------------------load relo map -------------------------------
    if(pcl::io::loadPCDFile<PointType>(map_file_path + "/corner.pcd",*laserCloudCornerFromMap_relo) == -1){
        PCL_ERROR("Couldn't read file corner_map.pcd\n");
        return(-1);
    }
    if(pcl::io::loadPCDFile<PointType>(map_file_path + "/surf.pcd",*laserCloudSurfFromMap_relo) == -1){
        PCL_ERROR("Couldn't read file surf_map.pcd\n");
        return(-1);
    }
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(map_file_path + "/all_points.pcd",*laserCloudALLFromMap_relo) == -1){
        PCL_ERROR("Couldn't read file all_points_map.pcd\n");
        return(-1);
    }    
    std::ifstream kf_map(map_file_path + "/key_frame.txt");
    
    if(kf_map){
        while(kf_map){
            Eigen::Quaternionf q;
            Eigen::Vector3f t;
            kf_map >> q.x() >> q.y() >> q.z() >> q.w()
                   >> t(0) >> t(1) >> t(2);

            Eigen::Matrix3f R = q.matrix();
            Eigen::Matrix4f T;
            T << R(0,0) , R(0,1) , R(0,2) , t(0),
                 R(1,0) , R(1,1) , R(1,2) , t(1),
                 R(2,0) , R(2,1) , R(2,2) , t(2),
                 0 , 0 , 0 , 1;
            pose_map.push_back(T);

            geometry_msgs::PoseStamped laserPose;
            laserPose.header.frame_id = "/camera_init";
            laserPose.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);

            laserPose.pose.orientation.x = q.x();
            laserPose.pose.orientation.y = q.y();
            laserPose.pose.orientation.z = q.z();
            laserPose.pose.orientation.w = q.w();
            laserPose.pose.position.x = t(0);
            laserPose.pose.position.y = t(1);
            laserPose.pose.position.z = t(2);

            laserPath.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);
            laserPath.poses.push_back(laserPose);
            laserPath.header.frame_id = "/camera_init";
            pubLaserPath.publish(laserPath);

        }
    }
    else{
        std::cout<<"NO key_frame_map.txt !"<<std::endl;
    }

    sensor_msgs::PointCloud2 oldmap_allpoints;
    pcl::toROSMsg(*laserCloudALLFromMap_relo, oldmap_allpoints);
    oldmap_allpoints.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);
    oldmap_allpoints.header.frame_id = "/camera_init";
    puboldmap_allpoints.publish(oldmap_allpoints);

    sensor_msgs::PointCloud2 oldmap_corner;
    pcl::toROSMsg(*laserCloudCornerFromMap_relo, oldmap_corner);
    oldmap_corner.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);
    oldmap_corner.header.frame_id = "/camera_init";
    puboldmap_corner.publish(oldmap_corner);

    sensor_msgs::PointCloud2 oldmap_surf;
    pcl::toROSMsg(*laserCloudSurfFromMap_relo, oldmap_surf);
    oldmap_surf.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);
    oldmap_surf.header.frame_id = "/camera_init";
    puboldmap_surf.publish(oldmap_surf);
    //---------------------------------------------------------------------------------

    //------------------------------------------------------------------------------------------------------
    int frameCount = 0;

    ros::Rate rate(100);
    bool status = ros::ok();
    while (status) {
        ros::spinOnce();

        if (newLaserCloudCornerLast && newLaserCloudSurfLast && newLaserCloudFullRes &&
                fabs(timeLaserCloudSurfLast - timeLaserCloudCornerLast) < 0.005 &&
                fabs(timeLaserCloudFullRes - timeLaserCloudCornerLast) < 0.005) {

            newLaserCloudCornerLast = false;
            newLaserCloudSurfLast = false;
            newLaserCloudFullRes = false;

            frameCount++;
            //initial first pose use 10 frames
            if(!firstRelocalization){
                *ndt_input_cloud += *laserCloudCornerLast;
                if(frameCount >= 10){

                    double score = initial_first_pose();
                    
                    sensor_msgs::PointCloud2 ndt_cloud;
                    pcl::toROSMsg(*ndt_output_cloud, ndt_cloud);
                    ndt_cloud.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);
                    ndt_cloud.header.frame_id = "/camera_init";
                    pubNDTCloud.publish(ndt_cloud);


                    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                        (transformTobeMapped[2], transformTobeMapped[0], transformTobeMapped[1]);
    
                    RelocalPose.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);
                    RelocalPose.pose.pose.orientation.x = geoQuat.y;
                    RelocalPose.pose.pose.orientation.y = geoQuat.z;
                    RelocalPose.pose.pose.orientation.z = geoQuat.x;
                    RelocalPose.pose.pose.orientation.w = geoQuat.w; 
                    RelocalPose.pose.pose.position.x = transformTobeMapped[3];
                    RelocalPose.pose.pose.position.y = transformTobeMapped[4];
                    RelocalPose.pose.pose.position.z = transformTobeMapped[5];
                    pubRelocalPose.publish(RelocalPose);

                    firstRelocalization = true;

                    if(true){

                    }
                    else{
                        rotation_flag = true;
                        ndt_input_cloud->clear();
                        continue;
                    }


                }else{
                    continue;
                }
            }

            // if(rotation_flag){  //TODO
            // }

            std::cout<<"DEBUG mapping start "<<std::endl;
            frameCount = 0;

            int laserCloudCornerFromMapNum = laserCloudCornerFromMap_relo_down->points.size();
            int laserCloudSurfFromMapNum = laserCloudSurfFromMap_relo_down->points.size();

            laserCloudCornerLast_down->clear();
            downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
            downSizeFilterCorner.filter(*laserCloudCornerLast_down);
            int laserCloudCornerLast_downNum = laserCloudCornerLast_down->points.size();

            laserCloudSurfLast_down->clear();
            downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
            downSizeFilterSurf.filter(*laserCloudSurfLast_down);
            int laserCloudSurfLast_downNum = laserCloudSurfLast_down->points.size();

            if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 100) {
            //if (laserCloudSurfFromMapNum > 100) {
                kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap_relo_down);
                kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap_relo_down);

                for (int iterCount = 0; iterCount < 10; iterCount++) {
                    laserCloudOri->clear();
                    coeffSel->clear();

                    for (int i = 0; i < laserCloudCornerLast_down->points.size(); i++) {
                        pointOri = laserCloudCornerLast_down->points[i];

                        pointAssociateToMap(&pointOri, &pointSel);
                        //find the closest 5 points
                        kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                        if (pointSearchSqDis[4] < 1) {
                            float cx = 0;
                            float cy = 0;
                            float cz = 0;
                            for (int j = 0; j < 5; j++) {
                                cx += laserCloudCornerFromMap_relo_down->points[pointSearchInd[j]].x;
                                cy += laserCloudCornerFromMap_relo_down->points[pointSearchInd[j]].y;
                                cz += laserCloudCornerFromMap_relo_down->points[pointSearchInd[j]].z;
                            }
                            cx /= 5;
                            cy /= 5;
                            cz /= 5;
                            //mean square error
                            float a11 = 0;
                            float a12 = 0;
                            float a13 = 0;
                            float a22 = 0;
                            float a23 = 0;
                            float a33 = 0;
                            for (int j = 0; j < 5; j++) {
                                float ax = laserCloudCornerFromMap_relo_down->points[pointSearchInd[j]].x - cx;
                                float ay = laserCloudCornerFromMap_relo_down->points[pointSearchInd[j]].y - cy;
                                float az = laserCloudCornerFromMap_relo_down->points[pointSearchInd[j]].z - cz;

                                a11 += ax * ax;
                                a12 += ax * ay;
                                a13 += ax * az;
                                a22 += ay * ay;
                                a23 += ay * az;
                                a33 += az * az;
                            }
                            a11 /= 5;
                            a12 /= 5;
                            a13 /= 5;
                            a22 /= 5;
                            a23 /= 5;
                            a33 /= 5;

                            matA1.at<float>(0, 0) = a11;
                            matA1.at<float>(0, 1) = a12;
                            matA1.at<float>(0, 2) = a13;
                            matA1.at<float>(1, 0) = a12;
                            matA1.at<float>(1, 1) = a22;
                            matA1.at<float>(1, 2) = a23;
                            matA1.at<float>(2, 0) = a13;
                            matA1.at<float>(2, 1) = a23;
                            matA1.at<float>(2, 2) = a33;

                            cv::eigen(matA1, matD1, matV1);

                            if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                                float x0 = pointSel.x;
                                float y0 = pointSel.y;
                                float z0 = pointSel.z;
                                float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                                float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                                float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                                float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                                float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                                float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                                //OA = (x0 - x1, y0 - y1, z0 - z1),OB = (x0 - x2, y0 - y2, z0 - z2)，AB = （x1 - x2, y1 - y2, z1 - z2）
                                //cross:
                                //|  i      j      k  |
                                //|x0-x1  y0-y1  z0-z1|
                                //|x0-x2  y0-y2  z0-z2|
                                float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                                    * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                                    * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                                                    * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                                float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                                float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                            + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                                float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                                - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                                float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                                + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                                float ld2 = a012 / l12;
                                //if(fabs(ld2) > 1) continue;

                                float s = 1 - 0.9 * fabs(ld2);

                                coeff.x = s * la;
                                coeff.y = s * lb;
                                coeff.z = s * lc;
                                coeff.intensity = s * ld2;

                                if (s > 0.1) {
                                    laserCloudOri->push_back(pointOri);
                                    coeffSel->push_back(coeff);
                                }
                            }
                        }
                    }

                    for (int i = 0; i < laserCloudSurfLast_down->points.size(); i++) {
                        pointOri = laserCloudSurfLast_down->points[i];

                        pointAssociateToMap(&pointOri, &pointSel);
                        kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                        if (pointSearchSqDis[4] < 5.0) {

                            for (int j = 0; j < 5; j++) {
                                matA0.at<float>(j, 0) = laserCloudSurfFromMap_relo_down->points[pointSearchInd[j]].x;
                                matA0.at<float>(j, 1) = laserCloudSurfFromMap_relo_down->points[pointSearchInd[j]].y;
                                matA0.at<float>(j, 2) = laserCloudSurfFromMap_relo_down->points[pointSearchInd[j]].z;
                            }
                            //matA0*matX0=matB0
                            //AX+BY+CZ+D = 0 <=> AX+BY+CZ=-D <=> (A/D)X+(B/D)Y+(C/D)Z = -1
                            //(X,Y,Z)<=>mat_a0
                            //A/D, B/D, C/D <=> mat_x0
                
                            cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

                            float pa = matX0.at<float>(0, 0);
                            float pb = matX0.at<float>(1, 0);
                            float pc = matX0.at<float>(2, 0);
                            float pd = 1;

                            //ps is the norm of the plane normal vector
                            //pd is the distance from point to plane
                            float ps = sqrt(pa * pa + pb * pb + pc * pc);
                            pa /= ps;
                            pb /= ps;
                            pc /= ps;
                            pd /= ps;

                            bool planeValid = true;
                            for (int j = 0; j < 5; j++) {
                                if (fabs(pa * laserCloudSurfFromMap_relo_down->points[pointSearchInd[j]].x +
                                            pb * laserCloudSurfFromMap_relo_down->points[pointSearchInd[j]].y +
                                            pc * laserCloudSurfFromMap_relo_down->points[pointSearchInd[j]].z + pd) > 0.2) {
                                    planeValid = false;
                                    break;
                                }
                            }

                            if (planeValid) {
                                //loss fuction
                                float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                                //if(fabs(pd2) > 0.1) continue;

                                float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                                coeff.x = s * pa;
                                coeff.y = s * pb;
                                coeff.z = s * pc;
                                coeff.intensity = s * pd2;

                                if (s > 0.1) {
                                    laserCloudOri->push_back(pointOri);
                                    coeffSel->push_back(coeff);
                                }
                            }
                        }
                    }
                    //std::cout <<"DEBUG mapping select all points : " << coeffSel->size() << std::endl;

                    float srx = sin(transformTobeMapped[0]);
                    float crx = cos(transformTobeMapped[0]);
                    float sry = sin(transformTobeMapped[1]);
                    float cry = cos(transformTobeMapped[1]);
                    float srz = sin(transformTobeMapped[2]);
                    float crz = cos(transformTobeMapped[2]);

                    int laserCloudSelNum = laserCloudOri->points.size();
                    if (laserCloudSelNum < 50) {
                        continue;
                    }

                    //|c1c3+s1s2s3 c3s1s2-c1s3 c2s1|
                    //|   c2s3        c2c3      -s2|
                    //|c1s2s3-c3s1 c1c3s2+s1s3 c1c2|
                    //AT*A*x = AT*b
                    cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
                    cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
                    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
                    cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
                    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
                    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
                    float debug_distance = 0;
                    for (int i = 0; i < laserCloudSelNum; i++) {
                        pointOri = laserCloudOri->points[i];
                        coeff = coeffSel->points[i];

                        float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                                + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                                + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

                        float ary = ((cry*srx*srz - crz*sry)*pointOri.x
                                        + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                                + ((-cry*crz - srx*sry*srz)*pointOri.x
                                    + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

                        float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                                + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                                + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;

                        matA.at<float>(i, 0) = arx;
                        matA.at<float>(i, 1) = ary;
                        matA.at<float>(i, 2) = arz;

                        matA.at<float>(i, 3) = coeff.x;
                        matA.at<float>(i, 4) = coeff.y;
                        matA.at<float>(i, 5) = coeff.z;
                        matB.at<float>(i, 0) = -coeff.intensity;

                        debug_distance += fabs(coeff.intensity);
                    }
                    cv::transpose(matA, matAt);
                    matAtA = matAt * matA;
                    matAtB = matAt * matB;
                    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

                    //Deterioration judgment
                    if (iterCount == 0) {
                        cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
                        cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
                        cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

                        cv::eigen(matAtA, matE, matV);
                        matV.copyTo(matV2);

                        isDegenerate = false;
                        float eignThre[6] = {5, 5, 5, 5, 5, 5};
                        for (int i = 5; i >= 0; i--) {
                            if (matE.at<float>(0, i) < eignThre[i]) {
                                for (int j = 0; j < 6; j++) {
                                    matV2.at<float>(i, j) = 0;
                                }
                                isDegenerate = true;
                            } else {
                                break;
                            }
                        }
                        matP = matV.inv() * matV2;
                    }

                    if (isDegenerate) {
                        cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
                        matX.copyTo(matX2);
                        matX = matP * matX2;
                    }

                    transformTobeMapped[0] += matX.at<float>(0, 0);
                    transformTobeMapped[1] += matX.at<float>(1, 0);
                    transformTobeMapped[2] += matX.at<float>(2, 0);
                    transformTobeMapped[3] += matX.at<float>(3, 0);
                    transformTobeMapped[4] += matX.at<float>(4, 0);
                    transformTobeMapped[5] += matX.at<float>(5, 0);

                    float deltaR = sqrt(
                                pow(rad2deg(matX.at<float>(0, 0)), 2) +
                                pow(rad2deg(matX.at<float>(1, 0)), 2) +
                                pow(rad2deg(matX.at<float>(2, 0)), 2));
                    float deltaT = sqrt(
                                pow(matX.at<float>(3, 0) * 100, 2) +
                                pow(matX.at<float>(4, 0) * 100, 2) +
                                pow(matX.at<float>(5, 0) * 100, 2));

                    if (deltaR < 0.1 && deltaT < 0.1) {
                        break;
                    }
                }

                transformUpdate();
            }
            if(use_map_update){
                for (int i = 0; i < laserCloudCornerLast_down->points.size(); i++) {
                    pointAssociateToMap(&laserCloudCornerLast_down->points[i], &pointSel);
                    laserCloudCornerFromMap_relo->push_back(pointSel);
                }
                
                laserCloudSurfFromMap_relo_down->clear();
                laserCloudCornerFromMap_relo_down->clear();
                downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap_relo);
                downSizeFilterCorner.filter(*laserCloudCornerFromMap_relo_down);
                for (int i = 0; i < laserCloudSurfLast_down->points.size(); i++) {
                    pointAssociateToMap(&laserCloudSurfLast_down->points[i], &pointSel);
                    laserCloudSurfFromMap_relo->push_back(pointSel);
                }
                downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap_relo);
                downSizeFilterSurf.filter(*laserCloudSurfFromMap_relo_down);
            }
            else{
                laserCloudSurfFromMap_relo_down = laserCloudSurfFromMap_relo;
                laserCloudCornerFromMap_relo_down = laserCloudCornerFromMap_relo;
            }

            sensor_msgs::PointCloud2 laserCloudSurround3;
            pcl::toROSMsg(*laserCloudSurfFromMap_relo_down, laserCloudSurround3);
            laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);
            laserCloudSurround3.header.frame_id = "/camera_init";
            pubLaserCloudSurround.publish(laserCloudSurround3);

            sensor_msgs::PointCloud2 laserCloudSurround3_corner;
            pcl::toROSMsg(*laserCloudCornerFromMap_relo_down, laserCloudSurround3_corner);
            laserCloudSurround3_corner.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);
            laserCloudSurround3_corner.header.frame_id = "/camera_init";
            pubLaserCloudSurround_corner.publish(laserCloudSurround3_corner);
            
            laserCloudFullRes2->clear();
            *laserCloudFullRes2 = *laserCloudFullRes;

            int laserCloudFullResNum = laserCloudFullRes2->points.size();
            for (int i = 0; i < laserCloudFullResNum; i++) {

                pcl::PointXYZRGB temp_point;
                RGBpointAssociateToMap(&laserCloudFullRes2->points[i], &temp_point);
                laserCloudFullResColor->push_back(temp_point);
            }

            *laserCloudFullResColor_pcd += *laserCloudFullResColor;

            geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                    (transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]);

            odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);
            odomAftMapped.pose.pose.orientation.x = geoQuat.y;
            odomAftMapped.pose.pose.orientation.y = geoQuat.z;
            odomAftMapped.pose.pose.orientation.z = geoQuat.x;
            odomAftMapped.pose.pose.orientation.w = geoQuat.w;
            odomAftMapped.pose.pose.position.x = transformAftMapped[3];
            odomAftMapped.pose.pose.position.y = transformAftMapped[4];
            odomAftMapped.pose.pose.position.z = transformAftMapped[5];
            pubOdomAftMapped.publish(odomAftMapped);
       
        }

        status = ros::ok();
        rate.sleep();
    }
    return 0;
}

