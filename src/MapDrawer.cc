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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>
#include <tuple>
namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

    mvObjectColors.push_back(std::tuple<float, float, float>({230. / 255., 0., 0.}));	 // red  0
    mvObjectColors.push_back(std::tuple<float, float, float>({60. / 255., 180. / 255., 75. / 255.}));   // green  1
    mvObjectColors.push_back(std::tuple<float, float, float>({0., 0., 255. / 255.}));	 // blue  2
    mvObjectColors.push_back(std::tuple<float, float, float>({255. / 255., 0, 255. / 255.}));   // Magenta  3
    mvObjectColors.push_back(std::tuple<float, float, float>({255. / 255., 165. / 255., 0}));   // orange 4
    mvObjectColors.push_back(std::tuple<float, float, float>({128. / 255., 0, 128. / 255.}));   //purple 5
    mvObjectColors.push_back(std::tuple<float, float, float>({0., 255. / 255., 255. / 255.}));   //cyan 6
    mvObjectColors.push_back(std::tuple<float, float, float>({210. / 255., 245. / 255., 60. / 255.}));  //lime  7
    mvObjectColors.push_back(std::tuple<float, float, float>({250. / 255., 190. / 255., 190. / 255.})); //pink  8
    mvObjectColors.push_back(std::tuple<float, float, float>({0., 128. / 255., 128. / 255.}));   //Teal  9

}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    // const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    // set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || vpMPs[i]->in_any_object)
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    // Draw object points (only for mono)
    int n_objects = mpMap->GetAllMapObjects().size();
    for (int o_i = 0 ; o_i < n_objects; o_i++)
    {
        glPointSize(2 * mPointSize);
        glBegin(GL_POINTS);
        float r, g, b;
        r = get<0>(mvObjectColors[o_i % 10]);
        g = get<1>(mvObjectColors[o_i % 10]);
        b = get<2>(mvObjectColors[o_i % 10]);
        glColor3f(r, g, b);

        for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
        {
            if(!vpMPs[i] || vpMPs[i]->isBad() || !vpMPs[i]->in_any_object || vpMPs[i]->object_id != o_i || vpMPs[i]->isOutlier())
                continue;
            cv::Mat pos = vpMPs[i]->GetWorldPos();
            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
        }
        glEnd();
    }
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(1.0f,0.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

void MapDrawer::SetObjectDrawer(ObjectDrawer *pObjectDrawer)
{
    mpObjectDrawer = pObjectDrawer;
}

void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
    mpObjectDrawer->SetCurrentCameraPose(Converter::toMatrix4f(mCameraPose));
}

Eigen::Matrix4f MapDrawer::GetCurrentCameraMatrix()
{
    unique_lock<mutex> lock(mMutexCamera);
    Eigen::Matrix4f Tcw = Converter::toMatrix4f(mCameraPose);
    return Tcw;
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}


// ellipsoid-version
void MapDrawer::drawPointCloudLists(float pointSize)
{
    auto pointLists = mpMap->GetPointCloudList();

    // cout << "pointLists.size() = " << pointLists.size() << std::endl;

    glPushMatrix();
    std::cout<<"[debug]MapDrawer::drawPointCloudLists start:  pointLists.size() = " << pointLists.size() << std::endl;
    for(auto pair:pointLists){
        auto strpoints = pair.first;
        // std::cout << "strpoints = " << strpoints << std::endl;
        auto pPoints = pair.second;
        if( pPoints == NULL ) continue;
        for(int i=0; i<pPoints->size(); i=i+1)
        {
            PointXYZRGB &p = (*pPoints)[i];
            // std::cout << "pPoints->size() = " << pPoints->size() << std::endl;
            // std::cout << "(&p) == NULL = " << ((&p)==NULL ) << std::endl;
            // std::cout << "p.x = " << p.x << std::endl;

            glPointSize( pointSize );
            // glPointSize( p.size );
            glBegin(GL_POINTS);

            // std::cout<<"[debug]MapDrawer::drawPointCloudLists: size:"<< pPoints->size() <<" p.x = " << p.x << ", p.y = " << p.y << ", p.z = " << p.z;
            // std::cout<<" --------------- color = " << p.r << ", " << p.g << ", " << p.b;
            glColor3d(p.r/255.0, p.g/255.0, p.b/255.0);
            // std::cout<<"------------end"<<std::endl;
            glVertex3d(p.x, p.y, p.z);
            glEnd();

        }
    }
    glPointSize( pointSize );

    glPopMatrix();
    std::cout<<"[debug]MapDrawer::drawPointCloudLists end:  pointLists.size() = " << pointLists.size() << std::endl;
}


void MapDrawer::drawPointCloudWithOptions(const std::map<std::string,bool> &options, float pointcloudSize, float sink_dis)
{
    auto pointLists = mpMap->GetPointCloudList();
    if(pointLists.size() < 1) return;
    glPushMatrix();

    for(auto pair:pointLists){
        auto pPoints = pair.second;
        if( pPoints == NULL ) continue;
        
        auto iter = options.find(pair.first);
        if(iter == options.end()) {
            continue;  // not exist
        }
        if(iter->second == false) continue; // menu is closed

        // 拷贝指针指向的点云. 过程中应该锁定地图. (理应考虑对性能的影响)
        PointCloud cloud = mpMap->GetPointCloudInList(pair.first); 
        for(int i=0; i<cloud.size(); i=i+1)
        {
            PointXYZRGB& p = cloud[i];
            // glPointSize( p.size );
            glPointSize( pointcloudSize );
            glBegin(GL_POINTS);
            glColor3d(p.r/255.0, p.g/255.0, p.b/255.0);
            glVertex3d(p.x, p.y, p.z-sink_dis);
            glEnd();
        }
    }
    glPointSize( 1 );
    glPopMatrix();        
}


bool MapDrawer::drawEllipsoidsVisual(double prob_thresh, double ellipsoidLineWidth) {

    std::vector<ellipsoid*> ellipsoidsVisual = mpMap->GetAllEllipsoidsVisual();

    std::vector<ellipsoid*> ellipsoids_prob;
    for(auto& pE : ellipsoidsVisual)
    {
        if(pE->prob > prob_thresh ){
            ellipsoids_prob.push_back(pE);
            // std::cout << "[MapDrawer::drawEllipsoidsVisual] Ellipsoid with prob: " << pE->prob << std::endl;
        }
        else{
            // std::cout << "[MapDrawer::drawEllipsoidsVisual] Ellipsoid with prob: " << pE->prob << " is filtered out." << std::endl;
        }
    }
    
    drawAllEllipsoidsInVector(ellipsoids_prob, 4, ellipsoidLineWidth);

    return true;
}

bool MapDrawer::drawLastestEllipsoidsVisual(double prob_thresh, double ellipsoidLineWidth) {

    std::vector<ellipsoid*> ellipsoidsVisual = mpMap->GetAllEllipsoidsVisual();

    std::vector<ellipsoid*> ellipsoids_prob;
    if(ellipsoidsVisual.size() < 1) return false;
    auto pE = ellipsoidsVisual.back();
    if(pE->prob > prob_thresh ){
        ellipsoids_prob.push_back(pE);
        // std::cout << "[MapDrawer::drawEllipsoidsVisual] Ellipsoid with prob: " << pE->prob << std::endl;
        // drawVerticesOfEllipsoid(pE,10,ellipsoidLineWidth);
    }
    else{
        // std::cout << "[MapDrawer::drawEllipsoidsVisual] Ellipsoid with prob: " << pE->prob << " is filtered out." << std::endl;
    }
    
    drawAllEllipsoidsInVector(ellipsoids_prob, 2, ellipsoidLineWidth);

    return true;
}

bool MapDrawer::drawVerticesOfEllipsoid(ellipsoid* pEllipsoid, int num, double pointSize) {
    // // std::cout<<"[debug] MapDrawer::drawVerticesOfEllipsoid start."<<std::endl;
    // // 半轴长度
    // double a = pEllipsoid->scale(0);    // x轴半轴长度，物体正向
    // double b = pEllipsoid->scale(1);    // y轴半轴长度，物体侧向
    // double c = pEllipsoid->scale(2);    // z轴半轴长度，物体竖向
    // // std::cout<<"[debug] MapDrawer::drawVerticesOfEllipsoid 1."<<std::endl;

    // // 顶点
    // std::vector<Vector3d> vertices;
    // Vector3d front = Vector3d(a, 0, 0);   vertices.push_back(front);// 前
    // Vector3d back = Vector3d(-a, 0, 0);   vertices.push_back(back);// 后
    // Vector3d left = Vector3d(0, b, 0);    vertices.push_back(left); // 左
    // Vector3d right = Vector3d(0, -b, 0);  vertices.push_back(right); // 右
    // Vector3d top = Vector3d(0, 0, c);   vertices.push_back(top);  // 上
    // Vector3d bottom = Vector3d(0, 0, -c); vertices.push_back(bottom);   // 下
    // // std::cout<<"[debug] MapDrawer::drawVerticesOfEllipsoid 2."<<std::endl;

    std::vector<Vector3d> vertices;
    Eigen::Matrix4d S = Eigen::Matrix4d::Identity();
    S(0,0) = pEllipsoid->scale(0);
    S(1,1) = pEllipsoid->scale(1);
    S(2,2) = pEllipsoid->scale(2);

    double max_angle_deg = 20.0;
    double max_angle_rad = max_angle_deg * M_PI / 180.0;
    // 顶面
    for (int i = 0; i < num; ++i) {
        double u = drand48();  // in [0,1)
        double v = drand48();

        double theta = 2 * M_PI * u;    //// 方位角（绕 z 轴），与x轴的夹角
        double phi = max_angle_rad * (v-1);   // 极角（与 z 轴夹角），限制在顶点附近的锥体区域

        double x = sin(phi) * cos(theta);
        double y = sin(phi) * sin(theta);
        double z = cos(phi);

        Eigen::Vector4d p_unit(x, y, z, 1.0);
        Eigen::Vector4d p_e = S * p_unit;

        // 椭球表面点
        Eigen::Vector3d sampled_point = p_e.head<3>();
        vertices.push_back(sampled_point);
    }
    // 地面
    for (int i = 0; i < num; ++i) {
        double u = drand48();  // in [0,1)
        double v = drand48();

        double theta = 2 * M_PI * u;    //// 方位角（绕 z 轴），与x轴的夹角
        double phi = M_PI - max_angle_rad * (v-1);   // 极角（与 z 轴夹角），限制在顶点附近的锥体区域

        double x = sin(phi) * cos(theta);
        double y = sin(phi) * sin(theta);
        double z = cos(phi);

        Eigen::Vector4d p_unit(x, y, z, 1.0);
        Eigen::Vector4d p_e = S * p_unit;

        // 椭球表面点
        Eigen::Vector3d sampled_point = p_e.head<3>();
        vertices.push_back(sampled_point);
    }
    // 前面
    for (int i = 0; i < num; ++i) {
        double u = drand48();  // in [0,1)
        double v = drand48();

        double theta = max_angle_rad * (u-0.5);    //// 方位角（绕 z 轴），与x轴的夹角
        double phi = max_angle_rad * (v-0.5) + M_PI_2;   // 极角（与 z 轴夹角），限制在顶点附近的锥体区域

        double x = sin(phi) * cos(theta);
        double y = sin(phi) * sin(theta);
        double z = cos(phi+M_PI);

        Eigen::Vector4d p_unit(x, y, z, 1.0);
        Eigen::Vector4d p_e = S * p_unit;

        // 椭球表面点
        Eigen::Vector3d sampled_point = p_e.head<3>();
        vertices.push_back(sampled_point);
    }
    // 后面
    for (int i = 0; i < num; ++i) {
        double u = drand48();  // in [0,1)
        double v = drand48();

        double theta = max_angle_rad * (u-0.5) + M_PI;    //// 方位角（绕 z 轴），与x轴的夹角
        double phi = max_angle_rad * (v-0.5) + M_PI_2;   // 极角（与 z 轴夹角），限制在顶点附近的锥体区域

        double x = sin(phi) * cos(theta);
        double y = sin(phi) * sin(theta);
        double z = cos(phi+M_PI);

        Eigen::Vector4d p_unit(x, y, z, 1.0);
        Eigen::Vector4d p_e = S * p_unit;

        // 椭球表面点
        Eigen::Vector3d sampled_point = p_e.head<3>();
        vertices.push_back(sampled_point);
    }
    // 右面
    for (int i = 0; i < num; ++i) {
        double u = drand48();  // in [0,1)
        double v = drand48();

        double theta = max_angle_rad * (u-0.5);    //// 方位角（绕 z 轴），与x轴的夹角
        double phi = max_angle_rad * (v-0.5) + M_PI_2;   // 极角（与 z 轴夹角），限制在顶点附近的锥体区域

        double x = sin(phi) * cos(theta);
        double y = sin(phi) * sin(theta);
        double z = cos(phi+M_PI);

        Eigen::Vector4d p_unit(x, y, z, 1.0);
        Eigen::Vector4d p_e = S * p_unit;

        // 椭球表面点
        Eigen::Vector3d sampled_point = p_e.head<3>();
        vertices.push_back(sampled_point);
    }
    // 左面
    for (int i = 0; i < num; ++i) {
        double u = drand48();  // in [0,1)
        double v = drand48();

        double theta = max_angle_rad * (u-0.5) + M_PI_2;    //// 方位角（绕 z 轴），与x轴的夹角
        double phi = max_angle_rad * (v-0.5) + M_PI_2;   // 极角（与 z 轴夹角），限制在顶点附近的锥体区域

        double x = sin(phi) * cos(theta);
        double y = sin(phi) * sin(theta);
        double z = cos(phi+M_PI);

        Eigen::Vector4d p_unit(x, y, z, 1.0);
        Eigen::Vector4d p_e = S * p_unit;

        // 椭球表面点
        Eigen::Vector3d sampled_point = p_e.head<3>();
        vertices.push_back(sampled_point);
    }

    // 绘制
    glPushMatrix();
    for(auto p:vertices){
        // 世界坐标系下的点
        Vector3d pw = pEllipsoid->pose * p;
        glPointSize( pointSize *20.0 );
        glBegin(GL_POINTS);
        glColor3d(1.0, 0.0, 0.0);
        glVertex3d(pw(0), pw(1), pw(2));
        glEnd();
    }
    // std::cout<<"[debug] MapDrawer::drawVerticesOfEllipsoid 3."<<std::endl;

    glPointSize( 1 );
    glPopMatrix(); 
    // std::cout<<"[debug] MapDrawer::drawVerticesOfEllipsoid 4."<<std::endl;
    return true;
}


bool MapDrawer::drawLastestRefinedEllipsoidsVisual(double prob_thresh, double ellipsoidLineWidth) {

    std::vector<ellipsoid*> ellipsoidsVisual = mpMap->GetAllRefinedEllipsoidsVisual();

    std::vector<ellipsoid*> ellipsoids_prob;
    if(ellipsoidsVisual.size() < 1) return false;
    auto pE = ellipsoidsVisual.back();
    if(pE->prob > prob_thresh ){
        ellipsoids_prob.push_back(pE);
        // std::cout << "[MapDrawer::drawEllipsoidsVisual] Ellipsoid with prob: " << pE->prob << std::endl;
    }
    else{
        // std::cout << "[MapDrawer::drawEllipsoidsVisual] Ellipsoid with prob: " << pE->prob << " is filtered out." << std::endl;
    }
    
    drawAllEllipsoidsInVector(ellipsoids_prob, 4, ellipsoidLineWidth);

    return true;
}
bool MapDrawer::drawGlobalEllipsoids(double prob_thresh, double ellipsoidLineWidth, double MinEllipsoidSize) {

    auto mvpMapObjects = mpMap->GetAllMapObjects();
    std::vector<ellipsoid*> ellipsoids_prob;

    for (MapObject *pMO : mvpMapObjects)
    {
        bool not_exist = !pMO ? true : false;
        // std::cout<<"not exist:"<< not_exist << ", bad:" << pMO->isBad()<< std::endl;

        if (!pMO)
            continue;
        if (pMO->isBad())
            continue;

        auto pE = pMO->GetEllipsold();

        if(pE->scale(0) <= MinEllipsoidSize || pE->scale(1) <= MinEllipsoidSize || pE->scale(2) <= MinEllipsoidSize)
            continue;

        if(pE->prob > prob_thresh ){
            // std::cout << std::endl;
            ellipsoids_prob.push_back(pE);
            // std::cout<<"[debug] drawGlobalEllipsoids 3 Ellipsoid with prob: " << pE->prob << ", scale:" << pE->scale.transpose() << std::endl;
        }
        else{
            // std::cout << ", low prob:" << pE->prob << std::endl;
            // std::cout<<"[debug] drawGlobalEllipsoids 3 Ellipsoid with prob: " << pE->prob << " is filtered out."  << ", scale:" << pE->scale.transpose()<< std::endl;
        }
    }
    
    // std::cout<<"[debug] MapDrawer::drawGlobalEllipsoids, Number of success objects / ALL map objects: " << ellipsoids_prob.size() << "/" << mvpMapObjects.size() << std::endl;

    drawAllEllipsoidsInVector(ellipsoids_prob, 0, ellipsoidLineWidth);

    return true;
}

bool MapDrawer::drawEllipsoidVertices(double prob_thresh, double pointcloudSize) {

    auto mvpMapObjects = mpMap->GetAllMapObjects();
    std::vector<ellipsoid*> ellipsoids_prob;

    for (MapObject *pMO : mvpMapObjects)
    {
        bool not_exist = !pMO ? true : false;
        // std::cout<<"not exist:"<< not_exist << ", bad:" << pMO->isBad()<< std::endl;

        if (!pMO)
            continue;
        if (pMO->isBad())
            continue;

        auto pE = pMO->GetEllipsold();

        if(pE->prob < prob_thresh ){
            continue;
        }

        // 绘制
        std::vector<Eigen::Vector3f>  vertices = pMO->GetEllipsoidVertices();
        glPushMatrix();
        for(auto pw:vertices){
            // 世界坐标系下的点
            glPointSize( pointcloudSize *2.0 );
            glBegin(GL_POINTS);
            glColor3d(0.0, 0.0, 0.0);
            glVertex3d(pw(0), pw(1), pw(2));
            glEnd();
        }
        // std::cout<<"[debug] MapDrawer::drawVerticesOfEllipsoid 3."<<std::endl;

        glPointSize( 1 );
        glPopMatrix(); 
    }
    
    return true;
}

// 加入了 transform
void MapDrawer::drawAllEllipsoidsInVector(std::vector<ellipsoid*>& ellipsoids, int color_mode, double ellipsoidLineWidth)
{    
    for( size_t i=0; i<ellipsoids.size(); i++)
    {
        drawEllipsoidInVector(ellipsoids[i], color_mode, ellipsoidLineWidth);
    }
    return;
}


void MapDrawer::drawEllipsoidInVector(ellipsoid* e, int color_mode, double ellipsoidLineWidth)
{
    
    SE3Quat TmwSE3 = e->pose.inverse();

    Vector3d scale = e->scale;

    // std::cout << "TmwSE3 = " << TmwSE3.to_homogeneous_matrix().matrix() << std::endl;
    // std::cout << "Ellipsoid scale = " << scale.transpose().matrix() << std::endl; 

    glPushMatrix();

    glLineWidth(mCameraLineWidth/3.0*ellipsoidLineWidth);

    // // glColor3f(0.0f,0.0f,1.0f);
    
    if (color_mode == 4) {
        if(e->isColorSet()){
            Vector4d color = e->getColorWithAlpha();
            glColor4d(color(0),color(1),color(2),color(3));
        }
        else
            // glColor3f(0.2f, 0.7f, 0.5f);
            glColor3f(1.0f, 0.0f, 0.0f);
    }
    else {
        if (color_mode == 0)
            // 红色
            glColor3f(1.0f,0.0f,0.0f);  
        else if (color_mode == 1)
            glColor3f(0.0f,1.0f,0.0f);
        else if(color_mode == 2)
            glColor3f(0.0f,0.0f,1.0f);
        else if(e->isColorSet()){
            Vector4d color = e->getColorWithAlpha();
            // std::cout << "color = " << color.matrix() << std::endl;
            glColor4d(color(0),color(1),color(2),color(3));
        }
        else
            glColor3f(0.0f,0.0f,1.0f);
    }
    
    GLUquadricObj *pObj;
    pObj = gluNewQuadric();
    gluQuadricDrawStyle(pObj, GLU_LINE);

    pangolin::OpenGlMatrix Twm;   // model to world

    SE3ToOpenGLCameraMatrix(TmwSE3, Twm);

    glMultMatrixd(Twm.m);  
    glScaled(scale[0],scale[1],scale[2]);
    gluSphere(pObj, 1.0, 26, 13); // draw a sphere with radius 1.0, center (0,0,0), slices 26, and stacks 13.

    drawAxisNormal();

    glPopMatrix();
}


// In : Tcw
// Out: Twc
void MapDrawer::SE3ToOpenGLCameraMatrix(g2o::SE3Quat &matInSe3, pangolin::OpenGlMatrix &M)
{
    // eigen to cv
    Eigen::Matrix4d matEigen = matInSe3.to_homogeneous_matrix();
    cv::Mat matIn;

    matIn = Converter::toCvMat(matEigen);

    // std::cout << "matIn = " << matIn << std::endl;
    // cv::eigen2cv(matEigen, matIn);

    if(!matIn.empty())
    {
        cv::Mat Rwc(3,3,CV_64F);
        cv::Mat twc(3,1,CV_64F);

        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = matIn.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*matIn.rowRange(0,3).col(3);
        }

        // 原来是 double, 发现有问题

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

void MapDrawer::drawAxisNormal()
{
    float length = 2.0;

    glLineWidth(2);
    
    // x
    glColor3f(1.0,0.0,0.0); // red x
    glBegin(GL_LINES);
    glVertex3f(0.0, 0.0f, 0.0f);
    glVertex3f(length, 0.0f, 0.0f);
    glEnd();

    // y 
    glColor3f(0.0,1.0,0.0); // green y
    glBegin(GL_LINES);
    glVertex3f(0.0, 0.0f, 0.0f);
    glVertex3f(0.0, length, 0.0f);

    glEnd();

    // z 
    glColor3f(0.0,0.0,1.0); // blue z
    glBegin(GL_LINES);
    glVertex3f(0.0, 0.0f ,0.0f );
    glVertex3f(0.0, 0.0f ,length );

    glEnd();
}

// draw all the planes
bool MapDrawer::drawPlanes(g2o::MANHATTAN_PLANE_TYPE type, float PlaneLineWidth) {
    std::vector<plane*> planes = mpMap->GetAllPlanes();
    // std::cout << "plane_num = " << planes.size() << std::endl;
    // bool success_debug = false;
    for( size_t i=0; i<planes.size(); i++) {
        g2o::plane* ppl = planes[i];
        if(ppl->miMHType == type) {
            // if(ppl->miMHType == g2o::MANHATTAN_PLANE_TYPE::BBOX)
            //     i+=5;
            // std::cout << "drawPlaneWithEquation : " << ppl->param.transpose().matrix() << std::endl;
            drawPlaneWithEquation(ppl, PlaneLineWidth);
            // success_debug = true;
        }
    }
    return true;
}


// A sparse version.
void MapDrawer::drawPlaneWithEquation(plane *p, float PlaneLineWidth) {
    if( p == NULL ) return;
    Vector3d center;            // 平面上一点!!
    double size;
    
    Vector3d color = p->color;
    Vector3d normal = p->normal(); 
    if(normal.norm()>0)
        normal.normalize();
    if(!p->mbLimited)
    {
        // an infinite plane, us default size
        center = p->SampleNearAnotherPoint(Vector3d(0,0,0));
        size = 25;
    }
    else
    {
        // todo: this make only ground drew
        // std::cout << "return because p->mbLimited" << std::endl;
        // return;
        size = p->mdPlaneSize;
        center = p->SampleNearAnotherPoint(p->mvPlaneCenter);
    }

    // draw the plane
    Matrix3d rotMat = Matrix3d::Identity();
    // 将其z轴旋转到 normal 方向.
    Matrix3d rotMatCalib = calibRotMatAccordingToAxis(rotMat, normal);

    Vector3d basis_x = rotMatCalib.col(0);
    Vector3d basis_y = rotMatCalib.col(1);

    // const Vector3d v1(center - (basis_x * size) - (basis_y * size));
    // const Vector3d v2(center + (basis_x * size) - (basis_y * size));
    // const Vector3d v3(center + (basis_x * size) + (basis_y * size));
    // const Vector3d v4(center - (basis_x * size) + (basis_y * size));

    // // Draw wireframe plane quadrilateral:

    // // 外轮廓.??
    // drawLine(v1, v2, color, line_width);
    // drawLine(v2, v3, color, line_width);
    // drawLine(v3, v4, color, line_width);
    // drawLine(v4, v1, color, line_width);

    // 绘制内部线条.
    Vector3d point_ld = center - size/2.0 * basis_x - size/2.0 * basis_y;

    double line_width = 2.0 * PlaneLineWidth;
    double alpha = 0.8;
    // int sample_num = 15; // 格子数量
    int sample_num = max(15, (int)(size/0.5)); // 格子数量
    double sample_dis = size / sample_num;
    for(int i=0;i<sample_num+1;i++)
    {
        // 从起始到结束, 包含起始和结束的等距离sample
        Vector3d v1(point_ld + i*sample_dis*basis_x);
        Vector3d v2(v1 + size*basis_y);
        drawLine(v1, v2, color, line_width, alpha);
    }
    for(int i=0;i<sample_num+1;i++)
    {
        // 从起始到结束, 包含起始和结束的等距离sample
        Vector3d v1(point_ld + i*sample_dis*basis_y);
        Vector3d v2(v1 + size*basis_x);
        drawLine(v1, v2, color, line_width, alpha);
    }

    bool bDrawDirection = true; // 绘制法向量方向
    if(p->miMHType==g2o::MANHATTAN_PLANE_TYPE::GROUND)
        bDrawDirection = false; // 地面不绘制法向量方向
    double direction_length = size / 3;
    if(bDrawDirection)
    {
        Vector3d end_point = center + normal * direction_length;
        drawLine(center, end_point, color/2.0, line_width/1.5, alpha);

        Vector3d end_point2 = end_point - normal * (direction_length / 4);
        drawLine(end_point, end_point2, color*2.0, line_width*2, alpha);// 绘制末端
    }

    return;
}


void MapDrawer::drawLine(const Vector3d& start, const Vector3d& end, const Vector3d& color, double width, double alpha)
{
    glLineWidth(width);

    glPushMatrix();

    glColor4d(color[0], color[1], color[2], alpha);
    // 先tm画很粗的 Line 吧.
    glBegin(GL_LINES);
    // opengl 画箭头.
    glVertex3d(start[0], start[1], start[2]);
    glVertex3d(end[0], end[1], end[2]);
    glEnd();

    glPopMatrix();
}


// from EllipsoidExtractor::calibRotMatAccordingToGroundPlane
Eigen::Matrix3d MapDrawer::calibRotMatAccordingToAxis(Matrix3d& rotMat, const Vector3d& normal){
    // in order to apply a small rotation to align the z axis of the object and the normal vector of the groundplane,
    // we need calculate the rotation axis and its angle.

    // first get the rotation axis
    Vector3d ellipsoid_zAxis = rotMat.col(2);
    Vector3d rot_axis = ellipsoid_zAxis.cross(normal); 
    if(rot_axis.norm()>0)
        rot_axis.normalize();

    // then get the angle between the normal of the groundplane and the z axis of the object
    double norm1 = normal.norm();
    double norm2 = ellipsoid_zAxis.norm();
    double vec_dot = normal.transpose() * ellipsoid_zAxis;
    double cos_theta = vec_dot/norm1/norm2;
    double theta = acos(cos_theta);     

    // generate the rotation vector
    AngleAxisd rot_angleAxis(theta,rot_axis);

    Matrix3d rotMat_calibrated = rot_angleAxis * rotMat;

    return rotMat_calibrated;
}


void MapDrawer::drawArrows()
{
    std::vector<Arrow> vArs = mpMap->GetArrows();

    for(int i=0;i<vArs.size();i++)
    {
        Arrow& ar = vArs[i];
        Vector3d norm = ar.norm; 
        Vector3d start = ar.center - norm;
        Vector3d end = ar.center;

        drawLine(start, end, ar.color, mCameraLineWidth * 100000, 0.8);
    }
}

} //namespace ORB_SLAM
