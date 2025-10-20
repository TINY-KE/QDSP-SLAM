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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>
#include <mutex>

// ellipsoid version
#include "ellipsoid-version/Ellipsoid.h"
#include "ellipsoid-version/Geometry.h"
#include "ellipsoid-version/Plane.h"
// #include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>	


using namespace g2o;

namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;
class MapObject;

enum ADD_POINT_CLOUD_TYPE
{
    REPLACE_POINT_CLOUD = 0,
    ADD_POINT_CLOUD = 1
};
enum DELETE_POINT_CLOUD_TYPE
{
    COMPLETE_MATCHING = 0,
    PARTIAL_MATCHING = 1
};

class Arrow
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vector3d center;
    Vector3d norm;
    Vector3d color;    
};

class Map
{
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    vector<KeyFrame*> mvpKeyFrameOrigins;  //没用？

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    int mnDynamicObj;

    // Object SLAM
    void AddMapObject(MapObject* pMO);
    void EraseMapObject(MapObject* pMO);
    MapObject* GetMapObject(int object_id);
    std::vector<MapObject*> GetAllMapObjects();

    // // ellipsoid-version 
    // void AddGlobalObjectDetections(ObjectDetection* pOD);
    // std::vector<ObjectDetection*> GetGlobalObjectDetections();

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    std::set<MapObject*> mspMapObjects;   //用于椭球体的数据关联
    
    // std::vector<ObjectDetection*> mvpGlobalObjectDetections; 

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;
    std::mutex mMutexRefinedMap;




// ellipsoid-version
public:
    bool AddPointCloudList(const string& name, std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& vCloudPCL, g2o::SE3Quat& Twc, int type = REPLACE_POINT_CLOUD);
    bool AddPointCloudList(const string& name, PointCloud* pCloud, int type = REPLACE_POINT_CLOUD);   // type 0: replace when exist,  type 1: add when exist
    bool DeletePointCloudList(const string& name, int type = 0);    // [废弃方案] type 0: complete matching, 1: partial matching
    bool ClearPointCloudLists();
    std::map<string, PointCloud*> mmPointCloudLists; // name-> pClouds

    // plane
    void addPlane(plane* pPlane, int visual_group = -1);
    std::vector<plane*> GetAllPlanes();
    std::set<plane*> mspPlanes;
    void clearPlanes();

    
protected:
    std::vector<ellipsoid*> mspEllipsoidsVisual;
    std::vector<ellipsoid*> mspRefinedEllipsoidsVisual;
    std::vector<ellipsoid*> mspEllipsoidsObjects;

public:
    // 用于可视化的椭球体，并没用参与优化
    // those visual ellipsoids are for visualization only and DO NOT join the optimization
    void addEllipsoidVisual(ellipsoid* pObj);
    std::vector<ellipsoid*> GetAllEllipsoidsVisual();
    void ClearEllipsoidsVisual();
    
    void addRefinedEllipsoidVisual(ellipsoid* pObj);
    std::vector<ellipsoid*> GetAllRefinedEllipsoidsVisual();
    // void ClearEllipsoidsVisual();
    
    void addEllipsoidObjects(ellipsoid* pObj);
    std::vector<ellipsoid*> GetAllEllipsoidsObjects();
    void ClearEllipsoidsObjects();
    
    // 深度点云
    std::map<string, PointCloud *>  GetPointCloudList();  //用户提取椭球体的深度点云，用于可视化debug
    PointCloud GetPointCloudInList(const string& name);

    // relations
public:
    void addArrow(const Vector3d& center, const Vector3d& norm, const Vector3d& color);
    std::vector<Arrow> GetArrows();
    void clearArrows();
protected:
    std::vector<Arrow> mvArrows;
    
};

} //namespace ORB_SLAM

#endif // MAP_H
