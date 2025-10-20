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

#include "System.h"
#include <boost/filesystem.hpp>
#include <sys/resource.h>
#include <iostream>
#include<ros/ros.h>


using namespace std;

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

bool check_order_rgb_depth();

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

    auto msensor = ORB_SLAM2::System::RGBD;

    cv::FileStorage fSettings(string(argv[2]), cv::FileStorage::READ);

    //(2)启动SLAM系统
    ORB_SLAM2::System SLAM(argv[1], argv[2], argv[3], msensor);
    
    std::cout<< "System Init 6-1" << std::endl;

    //(3)获取图片：从本地读取
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;

    std::cout<< "System Init 6-2: "<<strAssociationFilename << std::endl;


    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps, check_order_rgb_depth());

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

    SLAM.SetImageNames(vstrImageFilenamesRGB);

    std::cout<< "System Init 7" << std::endl;

    // 每一帧的track耗时
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);


    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;

    int images_numbers_to_pass_over = 1;
    bool frame_by_frame = true;
    if(frame_by_frame) {
        std::cout << "*****************************" << std::endl;
        std::cout << "input image: Press [ENTER] to continue ... , [y] to autonomous mode, [e] to quit." << std::endl;
        std::cout << "*****************************" << std::endl;
        char key = getchar();
        if (key=='y')
        {
            frame_by_frame = false;
        }
        else if (key=='e'){
            return 0;
        }
        else if (key > '0' && key <= '9') {
            images_numbers_to_pass_over = key - '0';
        }
        else if (key=='a'){
            images_numbers_to_pass_over = 10;
        }
        else if (key=='s'){
            images_numbers_to_pass_over = 50;
        }
        else if (key=='d'){
            images_numbers_to_pass_over = 100;
        }
        else if (key=='f'){
            images_numbers_to_pass_over = 300;
        }
        else if (key=='g'){
            images_numbers_to_pass_over = 450;
        }
    }

    double sleep_time = ORB_SLAM2::Config::Get<double>("Dataset.sleep_time");

    for(int ni = 0; ni < nImages; ni++)
    {
        std::cout << "\n========================================" << std::endl;
        std::cout << "=> Inputting Image " << ni << "/" << nImages << std::endl;

        std::chrono::steady_clock::time_point t1_read = std::chrono::steady_clock::now();
        //! 读取图像
        std::cout<< " 读取 RGB   Image: "<<string(argv[3])+"/"+vstrImageFilenamesRGB[ni] << std::endl;
        // imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni], CV_LOAD_IMAGE_UNCHANGED);
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni], CV_LOAD_IMAGE_COLOR);
        std::cout << " main imRGB size: " << imRGB.size() << ", channels: " << imRGB.channels() << std::endl;

        std::cout<< " 读取 Depth Image: "<<string(argv[3])+"/"+vstrImageFilenamesD[ni] << std::endl;
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni], CV_LOAD_IMAGE_UNCHANGED);
        std::cout<< " 读取 Image Done." << std::endl;
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

        cout << "imRGB.type() = " << imRGB.type() << endl;
        cout << "imD.type() = " << imD.type() << endl;

        assert(imRGB.type()==16);
        assert(imD.type()==2);

        SLAM.TrackRGBD(imRGB, imD, tframe);


        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        cout << " - [ total_frame: " << (double) ttrack  << "s ]" << endl;


        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T = 0.0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];
        if(ttrack<T)
        {
            // std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>((T- ttrack)*1e6)));
        }
        // std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>(0.5*1e6)));

        images_numbers_to_pass_over --;
        if(images_numbers_to_pass_over<=0)
            if(frame_by_frame) {
                std::cout << "*****************************" << std::endl;
                std::cout << "Press [ENTER] to continue ... , [y] to autonomous mode, [e] to quit, [photo numbers] to pass over." << std::endl;
                std::cout << "*****************************" << std::endl;
                char key = getchar();
                if (key=='y')
                {
                    frame_by_frame = false;
                }
                else if (key=='e'){
                    break;
                }
                else if (key > '0' && key <= '9') {
                    images_numbers_to_pass_over = key - '0';
                }
                else if (key=='a'){
                    images_numbers_to_pass_over = 10;
                }
                else if (key=='s'){
                    images_numbers_to_pass_over = 50;
                }
                else if (key=='d'){
                    images_numbers_to_pass_over = 100;
                }
                else if (key=='f'){
                    images_numbers_to_pass_over = 300;
                }
                else if (key=='g'){
                    images_numbers_to_pass_over = 450;
                }
            }
    }

    SLAM.SaveEntireMap(save_map_dir);

    string traj_path = data_source_dir  + "/eval/temp/";
    
    
    // Save Objects and Points
    int SaveLocalObjects = fSettings["saveobjects"];
    bool success_save_objects = false;
    if (SaveLocalObjects){
        bool move_to_origin = true;
        success_save_objects = SLAM.SaveObjects( data_source_dir  + "/eval/temp/objects/", move_to_origin);
        while(!success_save_objects){
            std::cout << "*****************************" << std::endl;
            std::cout << "请创建用于保存物体的文件夹, 并 Press [ENTER] to continue." << std::endl;
            std::cout << "*****************************" << std::endl;
            char key = getchar();
            success_save_objects = SLAM.SaveObjects( data_source_dir  + "/eval/temp/objects/", move_to_origin);
        }
    }
    
    SLAM.SaveKeyFrameTrajectoryTUM(traj_path);
    
    int SavePoints = fSettings["savepoints"];
    if (SavePoints)
        SLAM.SavePoints( data_source_dir  + "/eval/temp/points/");
    int SavePCDMap = fSettings["savePCDMap"];
    if (SavePCDMap)
        SLAM.SavePCDMap( data_source_dir  + "/map/dataset.pcd");

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    cout << "Press [e] to quit." << endl;
    char key;
    key = getchar();

    while(key!='e' && key!='E'){
        cout << "Press [e] to quit." << endl;
        key = getchar();
    }

    
    cv::destroyAllWindows();

    // Stop all threads
    SLAM.Shutdown();
    ros::shutdown();
    
    cout << "End." << endl;
    return 0;
}

//从关联文件中提取这些需要加载的图像的路径和时间戳
void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps, bool order_rgb_depth)
{
    std::cout<< "Load Images From: "<<strAssociationFilename << std::endl;

    int step_default = ORB_SLAM2::Config::Get<int>("Dataset.step");

    //输入文件流
    ifstream fAssociation;
    //打开关联文件
    fAssociation.open(strAssociationFilename.c_str());

    //一直读取,知道文件结束
    int step = 0;
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation, s);

        if(!s.empty())
        {
            step++;  // 每读取一行就递增计数

            //  每隔 n 行处理一次
            if (step % step_default == 0)
            {
                stringstream ss;
                double t;
                string sRGB, sD;

                if (order_rgb_depth)
                {
                    // 格式: 时间戳 rgb图像路径 时间戳 深度图像路径
                    ss << s;
                    ss >> t >> sRGB >> t >> sD;
                }
                else
                {
                    // 格式: 时间戳 深度图像路径 时间戳 rgb图像路径
                    ss << s;
                    ss >> t >> sD >> t >> sRGB;
                }

                vTimestamps.push_back(t);
                vstrImageFilenamesRGB.push_back(sRGB);
                vstrImageFilenamesD.push_back(sD);

                // 可选调试输出
                // std::cout << "Line " << step << ": " << sRGB << ", " << sD << std::endl;
            }
        }
    }
}


bool check_order_rgb_depth(){
    string dataset_type = ORB_SLAM2::Config::Get<string>("Dataset.Type");

    if(dataset_type == "ICL-NUIM" || dataset_type == "REPLICA"){
            return false;
    }
    else if(dataset_type == "AllObjectsOnGround")// TUM 
    {
            return true;
    }
    else{
        std::cerr << "Error: check_order_rgb_depth" <<std::endl;
        std::exit(0);
    }
}