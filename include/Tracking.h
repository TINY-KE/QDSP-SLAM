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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"
#include "MapObject.h"
#include "MapPublisher.h"

#include <mutex>

// ellipsoid-version
#include "utils/dataprocess_utils.h"
#include <src/pca/EllipsoidExtractor.h>
#include <src/Relationship/Relationship.h>
#include <src/plane/PlaneExtractorManhattan.h>
#include <src/config/Config.h>
#include <src/dense_builder/builder.h>
#include <src/Relationship/Relationship.h>
#include "include/ellipsoid-version/ConstrainPlane.h"
// #include <src/plane/PlaneExtractor.h>


typedef pcl::PointXYZ PointType;

namespace g2o
{   
    class plane;
}

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class MapPublisher;
class Map;
class LocalMapping;
class LoopClosing;
class System;
class MapObject;

enum OBJECT_MODEL
{
    POINT_MODEL = 0,
    QUADRIC_MODEL = 1
};

class Tracking
{  

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, MapPublisher*  pMapPublisher, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);

    // Object SLAM by Jingwen
    // KITTI (stereo+LiDAR)
    void GetObjectDetectionsLiDAR(KeyFrame *pKF);
    void ObjectDataAssociation_onlyforStereo(KeyFrame *pKF);
    // Freiburg Cars and Redwood (Mono)
    int maskErrosion;
    std::string detection_path;  // path to associated detected instances
    cv::Mat GetCameraIntrinsics();
    void GetObjectDetectionsMono(KeyFrame *pKF);
    void GetObjectDetectionsRGBD(KeyFrame *pKF);

private:
    void AssociateObjectsByProjection(KeyFrame *pKF);  // assocating detection to object by projecting map points
    void AssociateObjectsByDistance(ORB_SLAM2::KeyFrame *pKF);
    void UpdateAssociatedObjectPoseAndScale(MapObject* pMO);
    std::vector<Vector3d> getVerticesOfEllipsoid(ellipsoid* pEllipsoid, int num, double verticles_degree);

    bool mb_use_depth_pcd_to_reconstruct;

public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;
    cv::Mat mImColor;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset();

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    string DetectorConfigFile;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    MapPublisher*  mpMapPublisher;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;

public:
    // 物体检测框和掩码，用于可视化
    vector<cv::Mat> mvImObjectMasks;
    vector<vector<int>> mvImObjectBboxs;
    
private:
    std::string mStrSettingPath;
    int mbUseRos;
    string mDatasetPathRoot;
    int mMinimux_Points_To_Judge_Good;

// ellipsoid-version
private:
    bool miGroundPlaneState = false;
    g2o::plane mGroundPlane;
    // 设置地面的真值
    void SetGroundPlaneMannually(const Eigen::Vector4d &param);

    // 设置相机的真实位姿,但只用于第一帧
    void SetRealPose();

public:
    // Add by Lj
    vector<string> mvstrImageFilenamesRGB;
    void SetImageNames(vector<string>& vstrImageFilenamesRGB);
    int mCols, mRows;


    // 用于生成椭球体模型
public:
    EllipsoidExtractor* mpEllipsoidExtractor;  //椭球体提取器
private:
    RelationExtractor* mpRelationExtractor;   //空间关系提取器
    PlaneExtractorManhattan* pPlaneExtractorManhattan; // Manhattan平面（目前只有水平面，例如地面、桌面）提取器
    void UpdateObjectEllipsoidObservation(ORB_SLAM2::Frame *pFrame, KeyFrame* pKF);
    void ExtractManhattanPlanes(ORB_SLAM2::Frame *pFrame);
    void UpdateDepthEllipsoidEstimation(ORB_SLAM2::Frame* pFrame, KeyFrame* pKF);
    void TaskRelationship(ORB_SLAM2::Frame* pFrame, KeyFrame* pKF);
    void RefineObjectsWithRelations(ORB_SLAM2::Frame *pFrame, KeyFrame* pKF);
    // bool calibrateMeasurement(Eigen::Vector4d &measure , int rows, int cols, int config_boarder = 10, int config_size = 100); 
    
    camera_intrinsic mCamera; // 相机内参
    
// 用于椭球体数据关联
private:
    double mf_associate_IoU_thresold;
    double mf_associate_Dis_thresold;
    bool mb_associate_debug;
    bool mb_associate_object_with_ellipsold;
    int associateDetWithObject(ORB_SLAM2::KeyFrame *pKF, MapObject* pMO, int d_i, ObjectDetection* detKF1, vector<MapPoint*>& mvpMapPoints);
    
//深度点云可视化
public:
    void DenseBuild();
    Builder* mpBuilder;     // a dense pointcloud builder from visualization

private:
    void LoadPointcloud(const string& strPcdDir, const string& strPointcloud_name);   //g2o::SE3Quat Ttrans=g2o::SE3Quat()
    bool mb_global_map_input = false;
private:
    Eigen::Matrix3d mCalib;
    vector<vector<double> > mvManualObjectDetect;  //用于存储 手动标定的物体坐标检测标签
    double mfManualObjectDetectDisThresold = 1.0; // 手动标定的物体坐标检测标签的距离阈值
    int CheckManualLabel(ellipsoid* pEllipsoid);
    int CheckManualDirection(MapObject* pMO);

};

} //namespace ORB_SLAM

#endif // TRACKING_H
