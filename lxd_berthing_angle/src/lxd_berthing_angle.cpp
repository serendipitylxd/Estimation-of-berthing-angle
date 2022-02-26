#include <pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.
#include <sys/time.h>
#include <string>
#include "fstream"
#include "Outlier_noise_filtering.h"
#include "Cutting_point_cloud.h"
#include "Berthing_angle_calculation.h"



int main (int argc, char **argv)
{

    //Setting of output data,Set to your path
    ofstream out("/home/luxiaodong/lxd_ws/src/lxd_berthing_angle/data/output/lxd_output.txt");

    out << "heading" << "\t" << "rangex" << "\t" << "rangey" << "\t" << "pzhulla" << "\t" << "pzhullb" << "\t" << "pxmida" << "\t" << "pxmidb" << "\t" << "pymida" << "\t" << "pymidb" << "\t" << "berthingAngleDiff" << "\n";

    for(int i = 0;i < 1 ;i = i + 5 ){



        std::stringstream ss1;//Lidar1 data flow
        std::stringstream ss2;//Lidar2 data flow
        //Change to the file path of your point cloud data
        ss1 << "/home/luxiaodong/lxd_ws/src/lxd_berthing_angle/data/lidar_0_360/Lidar1/" << i <<"lidar1.pcd";
        ss2 << "/home/luxiaodong/lxd_ws/src/lxd_berthing_angle/data/lidar_0_360/Lidar2/" << i <<"lidar2.pcd";



        Cutting_point_cloud cuttingPointCloud;
        // Import the Lidar point cloud
        pcl::io::loadPCDFile (ss1.str(), *PCD_Lidar1_ptr);
        pcl::io::loadPCDFile (ss2.str(), *PCD_Lidar2_ptr);

        //timing
        struct timeval t1,t2;
        double timeuse;
        gettimeofday(&t1,NULL);

        // Remove the point cloud outside the port berth
        Outliner_noise_filtering outlinerNoiseFiltering;

        outlinerNoiseFiltering.Berthing_filtering(PCD_Lidar1_ptr);
        *Lidar1_cloud_port_filtered_ptr  = *cloud_port_filtered_ptr;
        outlinerNoiseFiltering.Berthing_filtering(PCD_Lidar2_ptr);
        *Lidar2_cloud_port_filtered_ptr  = *cloud_port_filtered_ptr;

        // Point cloud noise reduction
        outlinerNoiseFiltering.RadiusOutlierRemoval_filtering(Lidar1_cloud_port_filtered_ptr);
        *Lidar1_cloud_stray_filtered_ptr = *cloud_stray_filtered_ptr;
        outlinerNoiseFiltering.RadiusOutlierRemoval_filtering(Lidar2_cloud_port_filtered_ptr);
        *Lidar2_cloud_stray_filtered_ptr = *cloud_stray_filtered_ptr;

        //The point cloud data of two radars are spliced into one point cloud data
        *Lidar_cloud_stray_filtered_ptr = *Lidar1_cloud_stray_filtered_ptr;
        *Lidar_cloud_stray_filtered_ptr += *Lidar2_cloud_stray_filtered_ptr;

        //pcl::io::savePCDFileASCII("/home/luxiaodong/lxdch1_ws/src/lxd_berthing_angle/data/output/no_noise.pcd", *Lidar_cloud_stray_filtered_ptr);



        //Determine the rangexy of the point cloud
        double rangeX;
        double rangeY;
        double LidarminX;
        double LidarmaxX;
        double Lidar1minY;
        double Lidar1maxY;
        double Lidar2minY;
        double Lidar2maxY;
        double pxmida;
        double pxmidb;
        double pymida;
        double pymidb;


        cuttingPointCloud.CalcMinMaxPointXYZ(Lidar_cloud_stray_filtered_ptr);
        rangeX = cuttingPointCloud.LidarRangeX;
        rangeY = cuttingPointCloud.LidarRangeY;
        LidarminX = cuttingPointCloud.LidarMinX;
        LidarmaxX = cuttingPointCloud.LidarMaxX;
        pxmida = cuttingPointCloud.LidarMinX + (rangeX * 9 / 19);
        pxmidb = cuttingPointCloud.LidarMinX + (rangeX * 10 / 19);
        pymida = cuttingPointCloud.LidarMinY + (rangeY * 9 / 19);
        pymidb = cuttingPointCloud.LidarMinY + (rangeY * 10 / 19);

        cout << "rangeX: " << rangeX << " rangeY: " << rangeY << endl;

        double minHeight;
        double midHeight;
        double maxHeight;
        double range;

        minHeight = cuttingPointCloud.LidarMinZ;


//        cout << "minHeight: " << minHeight << " maxHeight: " << maxHeight << endl;


        maxHeight = 0;//Take the point cloud below the ship height 0
        range = maxHeight - minHeight;
        midHeight = 0.5 * (minHeight + maxHeight);

//        cout << "minHeight: " << minHeight << " maxHeight: " << maxHeight << " midHeight: " << midHeight << endl;

        int rangeNum = 1000000;
        int range1Num,range2Num;
        int Xmax_Ymax = 0;
        int lidar1Y_lidar2Y = 0;
        int lidar1_lidar2 = 0;
        string rangx_rangy;


        if (rangeX > rangeY){

            Xmax_Ymax = 1;
            rangx_rangy = "rangex";
            *Cutting_current_cloud_ptr = *Lidar_cloud_stray_filtered_ptr;
            //cout << "Number of point clouds in Cutting_current_cloud" << Cutting_current_cloud_ptr->size() << endl;
            //cout << "Number of point clouds in PCD_Lidar_ptr" << PCD_Lidar_ptr->size() << endl;
        }
        else {
            rangx_rangy = "rangey";
            Xmax_Ymax = 2;

            double Lidar1rangeY;
            //Determine the range of processing point clouds by the number of point clouds
            double Lidar1num;
            cuttingPointCloud.CalcMinMaxPointXYZ(Lidar1_cloud_stray_filtered_ptr);
            Lidar1rangeY = cuttingPointCloud.LidarRangeY;
            Lidar1num = Lidar1_cloud_stray_filtered_ptr->size();

            Lidar1minY = cuttingPointCloud.LidarMinY;
            Lidar1maxY = cuttingPointCloud.LidarMaxY;
//            cout << "Lidar1minY: " << Lidar1minY << " Lidar1maxY: " << Lidar1maxY << endl;
            double Lidar2rangeY;

            //Determine the range of processing point clouds by the number of point clouds
            double Lidar2num;

            cuttingPointCloud.CalcMinMaxPointXYZ(Lidar2_cloud_stray_filtered_ptr);
            Lidar2rangeY = cuttingPointCloud.LidarRangeY;

            Lidar2minY = cuttingPointCloud.LidarMinY;
            Lidar2maxY = cuttingPointCloud.LidarMaxY;

            Lidar2num = Lidar2_cloud_stray_filtered_ptr->size();
//            cout << "Lidar2minY: " << Lidar2minY << " Lidar2maxY: " << Lidar2maxY << endl;
            //cout << "Lidar1rangeY:" << Lidar1rangeY << " Lidar2rangeY:" << Lidar2rangeY <<endl;

            if(Lidar1num > Lidar2num){
                lidar1_lidar2 = 1;
                *Cutting_current_cloud_ptr = *Lidar1_cloud_stray_filtered_ptr;
            }
            else{
                lidar1_lidar2 = 2;
                *Cutting_current_cloud_ptr = *Lidar2_cloud_stray_filtered_ptr;
            }

/*
            if (Lidar1rangeY > Lidar2rangeY){
                lidar1Y_lidar2Y = 1;
                *Cutting_current_cloud_ptr = *PCD_Lidar1_ptr;
            }
            else{
                lidar1Y_lidar2Y = 2;
                *Cutting_current_cloud_ptr = *PCD_Lidar2_ptr;
            }
*/

        }

        //Dichotomous point cloud strip cutting method
        while (range > 1 && rangeNum > 500){


            cuttingPointCloud.Point_two_cutting(Cutting_current_cloud_ptr,minHeight,midHeight,maxHeight);
            range1Num = cuttingPointCloud.range1PointNumCal(Range1_cloud_ptr);
            //cout << "Number of point clouds in range1:" << Range1_cloud_ptr->size() << "个" << endl ;

            range2Num = cuttingPointCloud.range2PointNumCal(Range2_cloud_ptr);
            //cout << "Number of point clouds in range2:" << Range2_cloud_ptr->size() << "个" << endl ;

            if (range1Num > range2Num){

                *Cutting_current_cloud_ptr = *Range1_cloud_ptr;
                maxHeight = midHeight;
                midHeight = 0.5 * (minHeight + maxHeight );
            }
            else{
                *Cutting_current_cloud_ptr = *Range2_cloud_ptr;
                minHeight = midHeight;
                midHeight = 0.5 * (minHeight + maxHeight );
            }

            range = maxHeight - minHeight;

            rangeNum = cuttingPointCloud.rangePointNumCal(Cutting_current_cloud_ptr);
            //cout << "It has been cut, and the number of point clouds after cutting is："<< rangeNum <<",The range size of point cloud is：" << range << endl;
        }

        //Extract point cloud data near the ship
//        cout << "The z direction has been cut, and the number of point clouds after cutting is："<< Cutting_current_cloud_ptr->size() <<",The range size of point cloud is：" << range << endl;
//        cout << "minHeight: " << minHeight << " maxHeight: " << maxHeight << " midHeight: " << midHeight << endl;

//        pcl::io::savePCDFileASCII("/home/luxiaodong/lxd_ws/src/lxd_berthing_angle/data/output/1cutting.pcd", *Cutting_current_cloud_ptr);
        cout << "The z direction has been cut, and the number of point clouds after cutting is："<< rangeNum <<",The range size of point cloud is：" << range << endl;
//        cout << "Xmax_Ymax:" << Xmax_Ymax << " lidar1Y_lidar2Y:" << lidar1Y_lidar2Y << endl;
        if (Xmax_Ymax == 1){
            //Cutting_current_cloud_ptr is the fused data
            cuttingPointCloud.XamidshipFiltering(Cutting_current_cloud_ptr,LidarminX,LidarmaxX);
        } else if (Xmax_Ymax == 2){
            if (lidar1_lidar2 == 1){
//                cout << "Lidar1minY: " << Lidar1minY << " Lidar1maxY: " << Lidar1maxY << endl;
                //Cutting_current_cloud_ptr is Lidar1 data
                cuttingPointCloud.YamidshipFiltering(Cutting_current_cloud_ptr,Lidar1minY,Lidar1maxY);
            } else if (lidar1_lidar2 == 2){
                //Cutting_current_cloud_ptr is Lidar2 data
//                cout << "Lidar2minY: " << Lidar2minY << " Lidar2maxY: " << Lidar2maxY << endl;
                cuttingPointCloud.YamidshipFiltering(Cutting_current_cloud_ptr,Lidar2minY,Lidar2maxY);
//                cout << "There are several points" << Cutting_current_cloud_ptr->size() << endl;
            }
        }

//      pcl::io::savePCDFileASCII("/home/luxiaodong/lxd_ws/src/lxd_berthing_angle/data/output/2cutting.pcd", *Amidship_cloud_ptr);

        double w2;
        double b1;
        double Radians_Angle = 180.0/3.141592653;
        double berthingAngle;
        Berthing_angle_calculation berthingAngleCalculation;
        berthingAngleCalculation.SumCalculation(Amidship_cloud_ptr);
        w2 = berthingAngleCalculation.LinerRegressionK(Amidship_cloud_ptr->points.size(), Xmax_Ymax);

        b1 = berthingAngleCalculation.LinerRegressionB(Amidship_cloud_ptr->points.size());

        if (Xmax_Ymax == 1){
            berthingAngle = Radians_Angle * atan(w2);
        } else if (Xmax_Ymax == 2 ){
            double YberthingAngle;
//            cout << "---------------------------" << endl;
            YberthingAngle = Radians_Angle * atan(w2);
            berthingAngle = -(YberthingAngle - 90);
            if  (berthingAngle > 90){
                berthingAngle = berthingAngle -180 ;
            }

        }

       cout << "berthingAngle:" << berthingAngle << "°" << " w2:" << w2 << "." << endl;


        //To store data
        double trueBerthingAngle;

        if( i <  91){
            trueBerthingAngle = i;
        } else if (  i < 269 ){
            trueBerthingAngle = i - 180;
        } else {
            trueBerthingAngle = i - 360;
        }

        double berthingAngleDiff;
        berthingAngleDiff = berthingAngle - trueBerthingAngle;

        if (berthingAngleDiff < - 175){
            berthingAngleDiff +=  180;
        }

        if ( 175 < berthingAngleDiff ){
            berthingAngleDiff +=  - 180;
        }


        gettimeofday(&t2,NULL);
        timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0;
        cout<<"heading = " << i <<"; berthingAngleDiff = "<<berthingAngleDiff<<"; timeuse = "<<timeuse<<"s"<<endl;  //Output time (unit: s)

//        out << i << "\t" << berthingAngleDiff << "\t" << headingAngleDiff << "\t" << timeuse << "\t" << rangx_rangy << "\n";
        out << i << "\t" << rangeX << "\t" << rangeY << "\t" << minHeight << "\t" << maxHeight<< "\t" << pxmida << "\t" << pxmidb << "\t" << pymida << "\t" << pymidb << "\t" << berthingAngleDiff << "\n";
        //Point cloud cycle
    }
    out.close();

    return 0;
}
