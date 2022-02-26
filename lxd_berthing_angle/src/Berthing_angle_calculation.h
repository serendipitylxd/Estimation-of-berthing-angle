//
// Created by luxiaodong on 2021/12/12.
//

#ifndef LXD_BERTHING_ANGLE_BERTHING_ANGLE_CALCULATION_H
#define LXD_BERTHING_ANGLE_BERTHING_ANGLE_CALCULATION_H

#include "Cutting_point_cloud.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include<fstream>
#include<vector>
#include<string>

#include<pcl/point_types.h>
#include <math.h>

using namespace std;


using namespace Eigen;


pcl::PointCloud<pcl::PointXYZ>::Ptr Point_Cloud_Coordinate_system_transformation_ptr(new pcl::PointCloud<pcl::PointXYZ>);//去首去尾后的指针

class Berthing_angle_calculation{
public:

    //Calculate the variables required for regression
    double SumCalculation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr){
        sumxx = 0;
        sumxy = 0;
        sumyy = 0;
        sumx = 0;
        sumy = 0;
//        k = 0;
//        b = 0;
//        cout << "sum1 " << sumx << " " << sumy << " " << sumxy << " " << sumxx << " "<< "°" << endl;
        for(size_t j = 0; j < cloud_ptr->points.size(); ++j)
        {

            sumx += cloud_ptr->points[j].x;
            sumy += cloud_ptr->points[j].y;
            sumxx += cloud_ptr->points[j].x * cloud_ptr->points[j].x;
            sumxy += cloud_ptr->points[j].x * cloud_ptr->points[j].y;
            sumyy += cloud_ptr->points[j].y * cloud_ptr->points[j].y;
        }
//        cout << "sum2 " << sumx << " " << sumy << " " << sumxy << " " << sumxx << " "<< "°" << endl;
    }

    //Regression shows that W2
    double LinerRegressionK(int n,double Xmax_Ymax){
        if (Xmax_Ymax == 1){
            w2 = (sumxy - (sumx*sumy/n))/(sumxx-(sumx*sumx/n));
        } else if  (Xmax_Ymax == 2){
            w2 = (sumxy - (sumx*sumy/n))/(sumyy-(sumy*sumy/n));
        }

        return w2;
    }
    double LinerRegressionB(int n){
        b = sumy/n - w2*sumx/n;
        return b;
    }



private:
    double w2;
    double kup;
    double b;
    double sumyy;
    double sumxy;
    double sumxx;
    double sumx;
    double sumy;
    double Radians_Angle = 180.0/3.141592653;
    double Angle_Radians = 3.141592653/180.0;

};


#endif //LXD_BERTHING_ANGLE_BERTHING_ANGLE_CALCULATION_H
