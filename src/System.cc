/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include "System.h"
#include "Converter.h"
#include <chrono>
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <opencv2/core/eigen.hpp>
#include <time.h>
#include "src/config/Config.h"

// ellipsoid-version
ORB_SLAM2::Map* expMap;

bool has_suffix(const std::string &str, const std::string &suffix) {
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const string &strSequencePath,
               const eSensor sensor) :
               mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)),
               mbReset(false),mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
    "DSP-SLAM: Object Oriented SLAM with Deep Shape Priors." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";
    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    // ellipsoid-version
    Config::Init();
    Config::SetParameterFile(strSettingsFile);

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();

    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(strVocFile, ".txt"))
        bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    else
        bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);

    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    // Set Object-related variables
    py::initialize_interpreter();
    cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);
    py::module sys = py::module::import("sys");
    sys.attr("path").attr("append")("./");
    py::module io_utils = py::module::import("reconstruct.utils");
    string pyCfgPath = fSettings["DetectorConfigPath"].string();
    pyCfg = io_utils.attr("get_configs")(pyCfgPath);
    
    // 原程序： 单物体dsp模型导入, 弃用
    // pyDecoder = io_utils.attr("get_decoder")(pyCfg);

    // 多物体dsp模型导入
    mbChair2counch = fSettings["Chair2counch"];  //控制，在localmapping物体建模时，是否将椅子转换为沙发

    mnComputeCuboidType = fSettings["ComputeCuboidType"]; 
    mnNumKFsPassedSinceLastRecon_thresh = fSettings["NumKFsPassedSinceLastRecon_thresh"];
    mnNumKFsPassedSinceInit_thresh = fSettings["NumKFsPassedSinceInit_thresh"];
    py::module deep_sdf_utils = py::module::import("deep_sdf.workspace");
    vector<int> yolo_classes;
    fSettings["YoloClasses"] >> yolo_classes;
    vector<std::string> decoder_paths;
    fSettings["DecoderPaths"] >> decoder_paths;
    std::cout << "多物体dsp模型导入："<<std::endl;
    for (const std::string& path : decoder_paths) 
        std::cout << path << " "<<std::endl;
    py::module optim  = py::module::import("reconstruct.optimizer");
    for (int i = 0; i < yolo_classes.size(); i++){
        int class_id = yolo_classes[i];
        cout << "mmPyDecoders Add class: " << class_id << std::endl;
        py::object decoder = deep_sdf_utils.attr("config_decoder")(decoder_paths[i]);
        // py::object* decoder_ptr = &decoder;
        mmPyDecoders[class_id] = std::move(decoder);
    }

    pySequence = py::module::import("reconstruct").attr("get_sequence")(strSequencePath, pyCfg);
    InitThread();

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);
    mpMapPublisher = new MapPublisher(mpMap, strSettingsFile);\
    mpObjectDrawer = new ObjectDrawer(mpMap, mpMapDrawer, strSettingsFile);
    mpMapDrawer->SetObjectDrawer(mpObjectDrawer);
    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer, mpMapPublisher,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(this, mpMap, mpObjectDrawer, mSensor==MONOCULAR);
    mbMapInSameThread = fSettings["Mapping.LocalMappingInSameThread"];
    if (!mbMapInSameThread) {
        mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);
    }
    else {
        mpLocalMapper->InitSet();
    }


    //Initialize the Loop Closing thread and launch
    // Only enable loop closing for KITTI
    if (mSensor == STEREO)
    {
        mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
        mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);
    }
    else
    {
        mpLoopCloser = nullptr;
    }


    //Initialize the Viewer thread and launch
    mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpMapPublisher, mpObjectDrawer, mpTracker,strSettingsFile);
    mptViewer = new thread(&Viewer::Run, mpViewer);
    mpTracker->SetViewer(mpViewer);


    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);


    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);


    if (mpLoopCloser)
    {
        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);
    }
    // Release GIL
    PyEval_ReleaseThread(PyThreadState_Get());

    // ellipsoid-version
    expMap = mpMap;
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft, imRight, timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);
    
    mpTracker->DenseBuild();

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    if (mbMapInSameThread) {
        mpLocalMapper->RunOneTime();
    }

    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    bool bLoopCloserStopped = false;
    if (mpLoopCloser)
        mpLoopCloser->RequestFinish();
    else
        bLoopCloserStopped = true;

    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !bLoopCloserStopped)
    {
        if (!bLoopCloserStopped)
        {
            usleep(5000);
            bLoopCloserStopped = (mpLoopCloser->isFinished() && !mpLoopCloser->isRunningGBA());
        }
        usleep(5000);
    }


    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");

    PyGILState_Ensure();
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << "trajectory saved in: "<< filename << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filepath)
{
    cout << endl << "Saving keyframe trajectory in:  " << filepath << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();
    std::string filename_head = filepath + "/KeyFrameTrajectory"; 
    std::string filename = generateFileName(filename_head);

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


bool System::SaveObjects(const string &filepath , bool move_to_origin) {
    cout << endl << "Saving Objects in: " << filepath << endl;
    
    char *yolo_id[] = {
        "person",  //0
        "bicycle", "car", "motorcycle", "airplane", "bus",   //1
        "train", "truck", "boat", "traffic light",   "firehydrant", //6
        "stopsign", "parkingmeter", "bench", "bird", "cat", //11
        "dog", "horse", "sheep", "cow",  "elephant", //16
        "bear", "zebra", "giraffe", "backpack", "umbrella", //21
        "handbag", "tie", "suitcase", "frisbee",  "skis", //26
        "snowboard", "sportsball", "kite", "baseballbat", "baseballglove", //31
        "skateboard", "surfboard",  "tennis_racket", "bottle", "wine_glass", //36
        "cup", "fork", "knife", "spoon", "bowl", //41
        "banana", "apple",   "sandwich", "orange", "broccoli", //46
        "carrot", "hot_dog", "pizza", "donut", "cake", //51
        "chair", "couch",  "potted_plant", "bed", "dining_table",//56
        "toilet", "tv", "laptop", "mouse", "remote", //61
        "keyboard", "cell_phone",  "microwave", "oven", "toaster", //66
        "sink", "refrigerator", "book", "clock", "vase", //71
        "scissors", "teddy_bear",  "hair_drier", "toothbrush"};//76

    auto mvpMapObjects = mpMap->GetAllMapObjects();

    for (auto pMO: mvpMapObjects) {

        if (!pMO)
            continue;
        if (pMO->isBad())
            continue;

        if(pMO->faces.rows() == 0)
            continue;

        //  生成文件  例如 "filepath/file_1.txt"
        std::string filename_head = filepath + "/object_" + yolo_id[pMO->label] + "_" + std::to_string(pMO->mnId); 
        std::string filename = generateFileName(filename_head);
        ofstream file;
        file.open(filename.c_str());
        if (!file) {
            std::cerr << "文件创建失败: " << filename << std::endl;
            return false;
        }
        file << fixed;
        
        // 提取旋转部分, 转换为四元数
        Eigen::Matrix3f R = pMO->SE3Two.block<3,3>(0, 0);
        Eigen::Quaternionf q(R);
    
        //只存储物体
        file    
            << pMO->mnId << " "
            << pMO->label << "    "
            << pMO->SE3Two(0, 3) << " "
            << pMO->SE3Two(1, 3) << " "
            << pMO->SE3Two(2, 3)<< "     "
            << q.x() << " "
            << q.y() << " "
            << q.z() << " "
            << q.w() << "     "
            << pMO->w << " "
            << pMO->l << " "
            << pMO->h << "       "
            // ss >> degree;
            << "0   "
            // ss >> scale_x; ss >> scale_y; ss >> scale_z;
            << "1 1 1   "
            // ss >> color;  //设定颜色的种类
            << "0   "
            << "#" <<yolo_id[pMO->label]
            << endl;

        std::cout    
            << pMO->mnId << " "
            << pMO->label << "    "
            << pMO->SE3Two(0, 3) << " "
            << pMO->SE3Two(1, 3) << " "
            << pMO->SE3Two(2, 3) << "     "
            << "0 0 0 1     "
            << pMO->w << " "
            << pMO->l << " "
            << pMO->h << "       "
            << "0   "
            << "1 1 1   "
            << "0   "
            << "#" << yolo_id[pMO->label]
            << std::endl;

        // 使用 Eigen 矩阵来表示 verts 和 faces
        auto verts = pMO->vertices;
        auto faces = pMO->faces;

        Eigen::Matrix4f Sim3Two = pMO->GetPoseSim3();

        // 遍历每个 face，并从 verts 中提取顶点
        for (int i = 0; i < faces.rows(); ++i) {
            for (int j = 0; j < 3; ++j) {
                int vertex_idx = faces(i, j); // 获取顶点索引

                const Eigen::Vector3f &vertex = verts.row(vertex_idx); // 获取顶点的坐标

                Eigen::Vector4f local_vertex(vertex(0), vertex(1), vertex(2), 1.0);  // 齐次坐标

                // 将顶点转换到 world 坐标系
                Eigen::Vector4f world_vertex = Sim3Two * local_vertex;

                double px, py, pz;
                if(move_to_origin){
                    px = world_vertex[0] - pMO->SE3Two(0, 3);
                    py = world_vertex[1] - pMO->SE3Two(1, 3);
                    pz = world_vertex[2] - pMO->SE3Two(2, 3);
                }
                else{
                    px = world_vertex[0];
                    py = world_vertex[1];
                    pz = world_vertex[2];
                }

                file   
                    << px << " "
                    << py << " "
                    << pz << " "
                    << endl;
            }
        }
        file.close();
        cout << "Saving Object in: " << filename << endl;
    }

    
    cout << endl << "Object saved!" << endl;
    return true;
}

void System::SavePoints(const string &filepath ) {
    cout << endl << "Saving Points in: " << filepath << endl;
    
    std::string filename_head = filepath + "/points"; 
    std::string filename = generateFileName(filename_head);
    ofstream file;
    file.open(filename.c_str());
    if (!file) {
        std::cerr << "文件创建失败: " << filename << std::endl;
    }
    file << fixed;

    vector<MapPoint*> vpMP = mpMap->GetAllMapPoints();

    for (auto pMP : vpMP) {
        if (!pMP)
            continue;
        if (pMP->isBad())
            continue;
        // if (pMP->isOutlier())
        //     continue;

        cv::Mat x3Dw = pMP->GetWorldPos();
        float xc = x3Dw.at<float>(0);
        float yc = x3Dw.at<float>(1);
        float zc = x3Dw.at<float>(2);

        file   
            << xc << " "
            << yc << " "
            << zc << " "
            << endl;
    }

    file.close();
        
    cout << "Points saved in: "<< filename << endl;

}


// DatasetPathRoot

void System::SavePCDMap(const string &filepath ) {
    cout << endl << "Saving PCDMap in: " << filepath << endl;
    
    mpTracker->mpBuilder->saveMap(filepath);
        
    // cout << "PCDMap saved in: "<< filepath << endl;

}


int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

std::string System::generateFileName(std::string head) {
    // 获取当前系统时间
    std::time_t now = std::time(0);
    char timestamp[80];

    // 格式化系统时间为年-月-日_时-分-秒
    std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H-%M-%S", std::localtime(&now));

    // 生成文件名：cam_traj_当前时间.txt
    std::string filename = head + "-" +std::string(timestamp) + ".txt";

    // 如果文件名已存在，确保不会覆盖
    int counter = 1;
    while (fileExists(filename)) {
        filename = "cam_traj_" + std::string(timestamp) + "_" + std::to_string(counter) + ".txt";
        counter++;
    }

    return filename;
}

bool System::fileExists(const std::string &filename) {
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}


// ellipsoid-version

void System::SetImageNames(vector<string>& vstrImageFilenamesRGB)
{
    mvstrImageFilenamesRGB.resize(vstrImageFilenamesRGB.size());
    mvstrImageFilenamesRGB = std::vector<string>(vstrImageFilenamesRGB.begin(), vstrImageFilenamesRGB.end());
    mpTracker->SetImageNames(vstrImageFilenamesRGB);
}

// ellipsoid-version
Map* System::getMap() {
    return mpMap;
}




} //namespace ORB_SLAM
