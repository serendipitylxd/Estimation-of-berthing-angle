//
// Created by luxiaodong on 2021/12/11.
//

#ifndef LXD_BERTHING_ANGLE_OUTLIER_NOISE_FILTERING_H
#define LXD_BERTHING_ANGLE_OUTLIER_NOISE_FILTERING_H

#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/convolution.h>
#include <pcl/search/kdtree.h>

//pointer
pcl::PointCloud<pcl::PointXYZ>::Ptr PCD_Lidar_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr PCD_Lidar1_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr PCD_Lidar2_ptr(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_port_filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_stray_filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr Lidar1_cloud_port_filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr Lidar2_cloud_port_filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr Lidar1_cloud_stray_filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr Lidar2_cloud_stray_filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr Lidar_cloud_stray_filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>);

class Outliner_noise_filtering{
public:
    // Remove point cloud data such as port and shore-based tower cranes
    void Berthing_filtering(pcl::PointCloud<pcl::PointXYZ>::Ptr curret_cloud_ptr){

        pcl::PassThrough<pcl::PointXYZ> pass1;
        pass1.setInputCloud (curret_cloud_ptr);//Set input point cloud
        pass1.setFilterFieldName ("x");// define the axes
        pass1.setFilterLimits (-500, 500);//　 Scope
        // pass.setKeepOrganized(true); // Keep Ordered Point Cloud Structure
        pass1.setFilterLimitsNegative (false);
        pass1.filter (*cloud_port_filtered_ptr);

        pcl::PassThrough<pcl::PointXYZ> pass2;
        pass2.setInputCloud (cloud_port_filtered_ptr);//Set input point cloud
        pass2.setFilterFieldName ("y");// define the axes
        pass2.setFilterLimits (5.0, 500.0);//　y
        // pass.setKeepOrganized(true); //
        pass2.setFilterLimitsNegative (false);
        pass2.filter (*cloud_port_filtered_ptr);
    }

    void RadiusOutlierRemoval_filtering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_port_filtered_ptr){
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius; //Create filter
        radius.setInputCloud(cloud_port_filtered_ptr); //Set input point cloud
        radius.setRadiusSearch(1.0); //Set radius 1 to find adjacent points
        radius.setMinNeighborsInRadius (2);//Delete the query point whose neighborhood set number is less than 2
// apply filter
        radius.filter (*cloud_stray_filtered_ptr);//Perform conditional filtering
    }



};

#endif //LXD_BERTHING_ANGLE_OUTLIER_NOISE_FILTERING_H
