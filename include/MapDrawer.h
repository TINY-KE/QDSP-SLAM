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

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Converter.h"
#include "ObjectDrawer.h"
#include <pangolin/pangolin.h>
#include <GL/glu.h>
#include<mutex>

namespace ORB_SLAM2
{

class ObjectDrawer;

class MapDrawer
{
public:
    MapDrawer(Map* pMap, const string &strSettingPath);
    Map* mpMap;
    void DrawMapPoints();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const cv::Mat &Tcw);

    // By Jingwen
    std::vector<std::tuple<float, float, float>> mvObjectColors;
    ObjectDrawer* mpObjectDrawer;
    void SetObjectDrawer(ObjectDrawer *pObjectDrawer);
    Eigen::Matrix4f GetCurrentCameraMatrix();
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

private:

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    cv::Mat mCameraPose;

    std::mutex mMutexCamera;

// ellipsoid-version
public:
    bool drawEllipsoidsVisual(double prob_thresh, double ellipsoidLineWidth=1.0);
    bool drawGlobalEllipsoids(double prob_thresh, double ellipsoidLineWidth=1.0, double MinEllipsoidSize = 0.0);
    bool drawLastestEllipsoidsVisual(double prob_thresh, double ellipsoidLineWidth=1.0);
    bool drawLastestRefinedEllipsoidsVisual(double prob_thresh, double ellipsoidLineWidth=1.0);
    bool drawEllipsoidVertices(double prob_thresh, double pointcloudSize=1.0);

    bool drawVerticesOfEllipsoid(ellipsoid* pEllipsoid, int num=10,  double pointSize=1);
    
    void drawAllEllipsoidsInVector(std::vector<ellipsoid*>& ellipsoids, int color_mode = 0, double ellipsoidLineWidth=1.0);
    void drawEllipsoidInVector(ellipsoid* e, int color_mode = 0, double ellipsoidLineWidth=1.0); // 0: Red, 1: Green, 2:Blue
    void SE3ToOpenGLCameraMatrix(g2o::SE3Quat &matIn, pangolin::OpenGlMatrix &M); // inverse matIn
    void drawAxisNormal();
    void drawPointCloudLists(float pointSize);
    void drawPointCloudWithOptions(const std::map<std::string,bool> &options, float pointcloudSize=1, float sink_dis=0.0); // draw the point cloud lists with options opened

    // 绘制平面
    Eigen::Matrix3d calibRotMatAccordingToAxis(Matrix3d& rotMat, const Vector3d& normal);
    void drawLine(const Vector3d& start, const Vector3d& end, const Vector3d& color, double width, double alpha = 1.0);
    bool drawPlanes(g2o::MANHATTAN_PLANE_TYPE visual_group, float PlaneLineWidth = 1.0);
    void drawPlaneWithEquation(plane* p, float PlaneLineWidth);

    // relations
    void drawArrows();
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
