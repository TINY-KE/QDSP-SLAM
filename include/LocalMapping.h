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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "System.h"
#include <mutex>
#include "Converter.h"
// LJ修改
#define PYBIND11_NO_ASSERT_GIL_HELD_INCREF_DECREF
#include <pybind11/embed.h>
#include <pybind11/eigen.h>
namespace py = pybind11;

namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class Map;
class System;
class MapObject;

class LocalMapping
{
public:
    LocalMapping(System *pSys, Map* pMap, ObjectDrawer* pObjectDrawer, const float bMonocular);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    void RequestStop();
    void RequestReset();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    // Object SLAM by Jingwen
    KeyFrame* mpLastKeyFrame;
    std::list<MapObject*> mlpRecentAddedMapObjects;
    void GetNewObservations();
    void CreateNewMapObjects();
    void MapObjectCulling();
    void CreateNewObjectsFromDetections();
    void Create_Multi_NewObjectsFromDetections();
    void ProcessDetectedObjects_byPythonReconstruct();
    void Process_Multi_DetectedObjects_byPythonReconstruct();
    
    map<int, py::object> mmPyOptimizers;
    map<int, py::object> mmPyMeshExtractors;

    // 单物体的dsp模型相关， 弃用
    // py::object pyOptimizer;
    // py::object pyMeshExtractor;


    int nLastReconKFID;

protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();

    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    bool mbMonocular;

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;
    ObjectDrawer* mpObjectDrawer;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

// zhjd
private:
    std::vector<MapPoint*> AddCubePointsToMapObject(std::vector<MapPoint*> points);
    
    void AssociateObjects3D();
    void MergeMapObject(MapObject* pMO_i, MapObject* pMO_j); // Merge pMO_j into pMO_i

    int mbChair2counch = 0;  //控制，在localmapping物体建模时，是否将椅子转换为沙发
    int mbObjectInit = 1;  //控制是否全局物体初始化
    int mbSDFConstruct = 1;   //控制是否DeepSDF建模

    int mnComputeCuboidType = 0; //  // 0: 原版dspslam中的PCA,  1: 指定方向, 2: 椭球体
    int mnNumKFsPassedSinceLastRecon_thresh = 8;  // 自上次重建后经过的最少关键帧数量，如果小于阈值，则跳过重建，从而节约运算资源
    int mnNumKFsPassedSinceInit_thresh = 5;  // 一个物体被检测的次数，大于该值，才进行一次重建，从而节约运算资源

    // 处理完检测到的物体之后，要把它们更新到地图中
    void UpdateObjectsToMap();
    bool DeepSDFObjectConstruction(ObjectDetection *det, MapObject *pMO, int det_i);
    bool DeepSDFObjectConstruction_PcdCloud(ObjectDetection *det, MapObject *pMO, int det_i);
    bool DeepSDFObjectConstruction_PcdCloud_new(ObjectDetection *det, MapObject *pMO, int det_i);
    
    int mb_use_depth_pcd_to_reconstruct;

public:
    void InitSet();
    bool RunOneTime();
    std::vector<Vector3d> getVerticesOfEllipsoid(ellipsoid* pEllipsoid, int num);
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
