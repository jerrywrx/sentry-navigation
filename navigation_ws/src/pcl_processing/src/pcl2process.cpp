//
// Created by KevinTC.
//
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

//#include <pcl/features/normal_3d.h>
//#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

tf2_ros::Buffer tfBuffer;
ros::Publisher pcl_publisher;

inline float float_abs(float x) {
    if (x > 0) {
        return x;
    } else {
        return -x;
    }
}

struct PointInt {
    int p_x, p_y, p_z;
};

float current_x = 0.0, current_y = 0.0;

void getcloud_vec(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {//法向量法投影转平面
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 ROSPCL_output;
    pcl::fromROSMsg(*laserCloudMsg, *pcl2cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    geometry_msgs::TransformStamped base2map;
    try {  // get newest transform from base_link to map
        base2map = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));

    } catch (tf2::TransformException &ex) {
        ROS_WARN("Pcl2process Get TF ERROR!");
        return;
    }
    current_x = base2map.transform.translation.x;
    current_y = base2map.transform.translation.y;

    if((pcl2cloud->points.size()) == 0){
        return;
    }
    else{
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::VoxelGrid<pcl::PointXYZ> filter;
        filter.setInputCloud(pcl2cloud);
        filter.setLeafSize(0.04f, 0.04f, 0.04f);  // set voxel size
        filter.filter(*pcl2cloud);  // apply voxel grid filter to downsample pcl2cloud
        ne.setInputCloud(pcl2cloud);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);   // using kd-tree for searching nearest neighbors
        
        // save output data
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        ne.setKSearch(10);          // search for 10 nearest neighbors
        ne.compute(*cloud_normals); // store computed normals in cloud_normals
        long point_num = 0;
        for (long i = 0; i <= pcl2cloud->points.size(); i = i + 1) {
            // calculate the gradient of the normal vector
            float gradient = (pow(cloud_normals->points[i].normal_x, 2) + pow(cloud_normals->points[i].normal_y, 2)) / pow(cloud_normals->points[i].normal_z, 2);
            // if the point lies on relatively flat surface
            if (gradient > 1.0f) {
                if(pcl2cloud->points[i].y > 6.6 or pcl2cloud->points[i].y < -6.6){
                    continue;
                }
                // remove points in the robot body (0.3m radius)
                // TODO: change according to robot size
                if(pow(pcl2cloud->points[i].x - current_x, 2) + pow(pcl2cloud->points[i].y - current_y, 2) > 0.09){
                    pcl2cloud->points[i].z = 0;  // project points to the plane
                    pcl2cloud_out->points.push_back(pcl2cloud->points[i]);
                    point_num = point_num + 1;
                }
            }
        }

        // define rectangular boundary
        // pcl::PointXYZ point4push;
        // for (float x = -7.00; x < 22.0; x = x + 0.05) {
        //     point4push.x = x;
        //     point4push.y = -7.85f;
        //     point4push.z = 0.2f;
        //     pcl2cloud_out->points.push_back(point4push);
        //     point4push.y = 7.85f;
        //     pcl2cloud_out->points.push_back(point4push);
        //     point_num = point_num + 2;
        // }
        // for (float y = -7.85; y < 7.85; y = y + 0.05) {
        //     point4push.x = -7.00f;
        //     point4push.y = y;
        //     point4push.z = 0.2f;
        //     pcl2cloud_out->points.push_back(point4push);
        //     point4push.x = 22.0f;
        //     pcl2cloud_out->points.push_back(point4push);
        //     point_num = point_num + 2;
        // }

        pcl2cloud_out->width = point_num;
        pcl2cloud_out->height = 1;
        pcl2cloud_out->points.resize(pcl2cloud_out->width * pcl2cloud_out->height);
        pcl::toROSMsg(*pcl2cloud_out, ROSPCL_output);
        ROSPCL_output.header.frame_id = "map";
        pcl_publisher.publish(ROSPCL_output);
    }


int main(int argc, char **argv) {
    ros::init(argc, argv, "pointcloud_process");
    ros::NodeHandle pnh("~");
    tf2_ros::TransformListener tfListener(tfBuffer);
    auto subCloud = pnh.subscribe<sensor_msgs::PointCloud2>("/pointcloud2_in", 1, getcloud_vec);
    pcl_publisher = pnh.advertise<sensor_msgs::PointCloud2>("/pointcloud2_out", 1);
    ros::spin();
    return 0;
}
