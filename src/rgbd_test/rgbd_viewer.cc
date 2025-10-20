/**
* This file is part of https://github.com/JingwenWang95/DSP-SLAM
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <sys/resource.h>
#include <iostream>
#include <ros/ros.h>

// zhjd
// #include <ellipsoid-version/Ellipsoid.h>
// #include <ellipsoid-version/Geometry.h>
// #include <Map.h>
// #include <ellipsoid-version/BasicEllipsoidEdges.h>

// #include <src/symmetry/PointCloudFilter.h>
// #include <src/symmetry/Symmetry.h>
// #include <src/symmetry/SymmetrySolver.h>

// #include <Eigen/Core>

// #include <pcl/io/io.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>	

// rviz
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>

using namespace std;
typedef pcl::PointXYZ PointType;


/**
 * @brief 加载图像
 * 
 * @param[in] strAssociationFilename     关联文件的访问路径
 * @param[out] vstrImageFilenamesRGB     彩色图像路径序列
 * @param[out] vstrImageFilenamesD       深度图像路径序列
 * @param[out] vTimestamps               时间戳
 */


void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps, bool order_rgb_depth = 1);

void view(cv::Mat& depth_image);

void publishDepthAsPointCloud(const cv::Mat& depth, ros::Publisher& pub, const std_msgs::Header& header,
                               float fx, float fy, float cx, float cy, float scale = 1000.0f);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();
    ros::NodeHandle nh;

    //(1)从ros param和yaml中获取参数
    if(argc != 6)
    {
        cerr << endl << "Usage: ./dsp_slam_rgbd path_to_vocabulary path_to_settings path_to_sequence path_to_association path_to_saved_trajectory" << endl;
        return 1;
    }
    string strAssociationFilename = string(argv[4]);

    string strSettingsFile = argv[2];

    //  path_to_dataset [path_to_map]
    string dataset_path = argv[3];
    string dataset_path_savedir = dataset_path + "/result/";

    std::string save_map_dir = std::string(argv[5]);
    std::string data_source_dir = std::string(argv[3]);

    std::cout << "- settings file: " << strSettingsFile << std::endl;
    std::cout << "- dataset_path: " << dataset_path << std::endl;

    cv::FileStorage fSettings(string(argv[2]), cv::FileStorage::READ);

    //(2)启动SLAM系统
    ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/depth/points", 1);
    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("/depth/image_raw", 1);
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "world";
    // 相机内参
    float fx = 481.20f;
    float fy = 480.00f;
    float cx = 319.50f;
    float cy = 239.50f;

    //(3)获取图片：从本地读取
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;

    std::cout<< "System Init 6-2: "<<strAssociationFilename << std::endl;

    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps, false);

    std::cout<< "System Init 6" << std::endl;

    int nImages = vstrImageFilenamesRGB.size();

    if(vstrImageFilenamesRGB.empty()){
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size()){
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    std::cout<< "System Init 7" << std::endl;

    // 每一帧的track耗时
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);


    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;

    bool frame_by_frame = true;
    if(frame_by_frame) {
        std::cout << "*****************************" << std::endl;
        std::cout << "input image: Press [ENTER] to continue ... , [y] to autonomous mode" << std::endl;
        std::cout << "*****************************" << std::endl;
        char key = getchar();
        if (key=='y')
        {
            frame_by_frame = false;
        }
    }

    for(int ni = 0; ni < nImages; ni++)
    {
        std::cout << "\n========================================" << std::endl;
        std::cout << "=> Inputting Image " << ni << "/" << nImages << std::endl;

        std::chrono::steady_clock::time_point t1_read = std::chrono::steady_clock::now();

        //! 读取图像
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni], CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni], CV_LOAD_IMAGE_UNCHANGED);

        // cv::imshow ("RGB", imRGB);
        // cv::imshow ("D", imD);
        // cv::waitKey(5000);

        std::chrono::steady_clock::time_point t2_read = std::chrono::steady_clock::now();
        double t_read = std::chrono::duration_cast<std::chrono::duration<double> >(t2_read - t1_read).count();

        cout << " 读取 Image 耗时: " << t_read << "s" << endl;

        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;/*  */
        }


        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // cout << "imRGB.type() = " << imRGB.type() << endl;
        // cout << "imD.type() = " << imD.type() << endl;

        assert(imRGB.type()==16);
        assert(imD.type()==2);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        cout << " - [ total_frame: " << (double) ttrack  << "s ]" << endl;

        vTimesTrack[ni]=ttrack;

        // // Wait to load the next frame
        // double T = 0.0;
        // if(ni<nImages-1)
        //     T = vTimestamps[ni+1]-tframe;
        // else if(ni>0)
        //     T = tframe-vTimestamps[ni-1];

        // if(ttrack<T)
        // {
        //     std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>((T- ttrack)*1e6)));
        // }

        
        publishDepthAsPointCloud(imD, pc_pub, header, fx, fy, cx, cy);

        if(frame_by_frame) {
            std::cout << "*****************************" << std::endl;
            std::cout << "Press [ENTER] to continue ... , [y] to autonomous mode, [e] to quit" << std::endl;
            std::cout << "*****************************" << std::endl;
            char key = getchar();
            if (key=='y')
            {
                frame_by_frame = false;
            }
            else if (key=='e'){
                break;
            }
        }
        
    }


    cv::destroyAllWindows();

    cout << "End." << endl;
    return 0;
}

//从关联文件中提取这些需要加载的图像的路径和时间戳
void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps, bool order_rgb_depth)
{
    std::cout<< "System Init 6-3: "<<strAssociationFilename << std::endl;

    //输入文件流
    ifstream fAssociation;
    //打开关联文件
    fAssociation.open(strAssociationFilename.c_str());

    std::cout<< "System Init 6-4 "<< std::endl;

    //一直读取,知道文件结束
    while(!fAssociation.eof())
    {
        string s;
        //读取一行的内容到字符串s中
        getline(fAssociation,s);
        //如果不是空行就可以分析数据了
        if(!s.empty())
        {
            //字符串流
            stringstream ss;
            if (order_rgb_depth)
            {
                ss << s;
                //字符串格式:  时间戳 rgb图像路径 时间戳 深度图像路径
                double t;
                string sRGB, sD;
                ss >> t;
                vTimestamps.push_back(t);
                ss >> sRGB;
                vstrImageFilenamesRGB.push_back(sRGB);
                ss >> t;
                ss >> sD;
                vstrImageFilenamesD.push_back(sD);
            }
            else
            {
                //字符串格式:  时间戳 深度图像路径 时间戳 rgb图像路径
                ss << s;
                double t;
                string sD, sRGB;
                ss >> t;
                vTimestamps.push_back(t);
                ss >> sD;
                vstrImageFilenamesD.push_back(sD);
                ss >> t;
                ss >> sRGB;
                vstrImageFilenamesRGB.push_back(sRGB);
            }
        }
    }
}


void view(cv::Mat& depth){
    // pcl::PointCloud<PointType>::Ptr pCloudPCL = ExtractPointCloud(depth,bbox,pose,camera);

    // 直接显示深度图像
}

void publishDepthAsPointCloud(const cv::Mat& depth, ros::Publisher& pub, const std_msgs::Header& header,
                               float fx, float fy, float cx, float cy, float scale) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (int v = 0; v < depth.rows; ++v) {
        for (int u = 0; u < depth.cols; ++u) {
            ushort d = depth.at<ushort>(v, u);
            if (d == 0) continue;

            float z = d / scale;
            float x = (u - cx) * z / fx;
            float y = (v - cy) * z / fy;

            cloud->points.push_back(pcl::PointXYZ(x, y, z));
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header = header;

    pub.publish(cloud_msg);
}