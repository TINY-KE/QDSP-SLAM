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

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, MapPublisher*  pMapPublisher,  ObjectDrawer *pObjectDrawer, Tracking *pTracking, const string &strSettingPath):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpObjectDrawer(pObjectDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false), mpMapPublisher(pMapPublisher)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];

    mUsePangolin =  fSettings["Viewer.UsePangolin"];
}

cv::Mat Viewer::GetRGBFrame()
{
    return mpFrameDrawer->DrawFrame();
    // return mpFrameDrawer->mIm;
}

cv::Mat Viewer::GetDepthFrame()
{
    return mpFrameDrawer->DrawDepthFrame();
    // return mpFrameDrawer->mmDepth;
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;
    int w = 1024;
    int h = 576;
    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",w,h);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",false,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowCurrentFrame("menu.Show CurrentFrame",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    // ellipsoid-version
    pangolin::Var<bool> menuShowEllipsoids("menu.Show Ellipsoids One Frame Visual", false, true);
    pangolin::Var<bool> menuShowLastestEllipsoids("menu.Show Newest Ellipsoids One Frame Visual", true, true);
    pangolin::Var<bool> menuShowLastestRefinedEllipsoids("menu.Show Refined Ellipsoids One Frame Visual", true, true);
    pangolin::Var<bool> menuShowGlobalEllipsoids("menu.Show Global Ellipsoids", true, true);
    pangolin::Var<bool> menuShowEllipsoidVertices("menu.Show Ellipsoid Vertices", true, true);
    pangolin::Var<double> SliderEllipsoidProbThresh("menu.Ellipsoid Prob", 0.01, 0.0, 1.0);
    pangolin::Var<double> SliderEllipsoidLineWidth("menu.EllipsoidLine Width", 1.0, 0.5, 3.0);
    pangolin::Var<double> SliderEllipsoidMiniSize("menu.Ellipsoid MiniSize", 0.0, 0.0, 1.0);
    
    pangolin::Var<bool> menuShowSdfObjects("menu.Show SDF Objects",true,true);
    // 深度点云
    pangolin::Var<float> SliderPointCloudListSize("menu.Pointcloud Size", 3.0, 0.5, 10.0);
    pangolin::Var<float> SliderPointCloudSink("menu.World Sink", 0.0, -0.3, 0.3);

    // pangolin::Var<bool> menuShowDepthPoints("menu.Show Depth Points",false,true);
    // 地面
    pangolin::Var<float> SlidermPlaneLineWidth("menu.PlaneLine Width", 1.0, 0.5, 3.0);
    pangolin::Var<bool> menuShowGroundPlane("menu.Show GroundPlane",true,true);
    pangolin::Var<bool> menuShowBackingPlane("menu.Show BackingPlane",true,true);
    pangolin::Var<bool> menuShowSupportingPlane("menu.Show SupportingPlane",true,true);
    // 最新帧中的bbox平面
    pangolin::Var<bool> menuShowBboxPlane("menu.Show Bbox Plane",false,true);
    // relations
    pangolin::Var<bool> menuShowRelationArrow("menu.Relation Arrow",false,true);
    
    // 图片
    pangolin::Var<bool> menuShowFrameImg("menu.Show FrameImg", false, true);
    pangolin::GlTexture imageTexture(mImageWidth,mImageHeight,GL_RGB,false,0,GL_BGR,GL_UNSIGNED_BYTE);
    pangolin::View& rgb_image = pangolin::Display("rgb")
        .SetBounds(0,0.3,0.2,0.5,float(mImageWidth) / float(mImageHeight))
        .SetLock(pangolin::LockLeft, pangolin::LockBottom);
    pangolin::View& depth_image = pangolin::Display("depth")
        .SetBounds(0,0.3,0.5,0.8,float(mImageWidth) / float(mImageHeight))
        .SetLock(pangolin::LockLeft, pangolin::LockBottom);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(w,h, mViewpointF,mViewpointF,w / 2,h / 2,0.1,5000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -float(w) / float(h))
            .SetHandler(new pangolin::Handler3D(s_cam));

    auto mpRenderer = new ObjectRenderer(w, h);
    mpRenderer->SetupCamera(mViewpointF, mViewpointF, double(w) / 2, double(h) / 2, 0.1, 5000);
    mpObjectDrawer->SetRenderer(mpRenderer);

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    cv::namedWindow("DSP-SLAM: Current Frame");

    bool bFollow = true;
    bool bLocalizationMode = false;
    Eigen::Matrix4f Tec;

    while(1)
    {
        mpMapPublisher->Refresh();

        if(mUsePangolin)
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

            if(menuFollowCamera && bFollow)
            {
                s_cam.Follow(Twc);
            }
            else if(menuFollowCamera && !bFollow)
            {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
                s_cam.Follow(Twc);
                bFollow = true;
            }
            else if(!menuFollowCamera && bFollow)
            {
                bFollow = false;
            }

            if(menuLocalizationMode && !bLocalizationMode)
            {
                mpSystem->ActivateLocalizationMode();
                bLocalizationMode = true;
            }
            else if(!menuLocalizationMode && bLocalizationMode)
            {
                mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
            }

            d_cam.Activate(s_cam);
            // Used for object drawer
            Tec = s_cam.GetModelViewMatrix();
            Tec.row(1) = -Tec.row(1);
            Tec.row(2) = -Tec.row(2);
            glClearColor(1.0f,1.0f,1.0f,1.0f);
            if(menuShowCurrentFrame)
                mpMapDrawer->DrawCurrentCamera(Twc);
            if(menuShowKeyFrames || menuShowGraph)
                mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
            if(menuShowPoints)
                mpMapDrawer->DrawMapPoints();

            
            if(menuShowEllipsoids){
                double ellipsoidProbThresh = SliderEllipsoidProbThresh;
                double ellipsoidLineWidth = SliderEllipsoidLineWidth;
                mpMapDrawer->drawEllipsoidsVisual(ellipsoidProbThresh,ellipsoidLineWidth);
            }
            if(menuShowLastestEllipsoids){
                double ellipsoidProbThresh = SliderEllipsoidProbThresh;
                double ellipsoidLineWidth = SliderEllipsoidLineWidth;
                mpMapDrawer->drawLastestEllipsoidsVisual(ellipsoidProbThresh,ellipsoidLineWidth);
            }
            if(menuShowLastestRefinedEllipsoids){
                double ellipsoidProbThresh = SliderEllipsoidProbThresh;
                double ellipsoidLineWidth = SliderEllipsoidLineWidth;
                mpMapDrawer->drawLastestRefinedEllipsoidsVisual(ellipsoidProbThresh,ellipsoidLineWidth);
            }
            mpObjectDrawer->ProcessNewObjects();
            if(menuShowSdfObjects){
                mpObjectDrawer->DrawObjects(bFollow, Tec);
            }
            if(menuShowGlobalEllipsoids){
                double ellipsoidProbThresh = SliderEllipsoidProbThresh;
                double ellipsoidLineWidth = SliderEllipsoidLineWidth;
                double ellipsoidMiniSize = SliderEllipsoidMiniSize;
                mpMapDrawer->drawGlobalEllipsoids(ellipsoidProbThresh,ellipsoidLineWidth, ellipsoidMiniSize);
            }

            // if(menuShowDepthPoints)
            // {
            //     float pointcloudSize = SliderPointCloudListSize;
            //     mpMapDrawer->drawPointCloudLists(pointcloudSize);
            // }

            // 展示图片
            if (menuShowFrameImg) 
            // if(mpTracker->mState != Tracking::NOT_INITIALIZED)
            {
                cv::Mat rgb = GetRGBFrame();
                if(!rgb.empty())
                {
                    imageTexture.Upload(rgb.data,GL_BGR,GL_UNSIGNED_BYTE);
                    //display the image
                    rgb_image.Activate();
                    glColor3f(1.0,1.0,1.0);
                    imageTexture.RenderToViewportFlipY();
                }

                cv::Mat depth = GetDepthFrame();
                if(!depth.empty())
                {
                    imageTexture.Upload(depth.data,GL_BGR,GL_UNSIGNED_BYTE);
                    //display the image
                    depth_image.Activate();
                    glColor3f(1.0,1.0,1.0);
                    imageTexture.RenderToViewportFlipY();
                }
            }

            // 地面
            if(menuShowGroundPlane)
                mpMapDrawer->drawPlanes(g2o::MANHATTAN_PLANE_TYPE::GROUND); 

            float PlaneLineWidth = SlidermPlaneLineWidth;
            // 倚靠面
            if(menuShowBackingPlane)
                mpMapDrawer->drawPlanes(g2o::MANHATTAN_PLANE_TYPE::VERTICAL, PlaneLineWidth); 
            // 支撑面
            if(menuShowSupportingPlane)
                mpMapDrawer->drawPlanes(g2o::MANHATTAN_PLANE_TYPE::HORIZONTAL, PlaneLineWidth); 
            // 最新帧中的观测切面
            if(menuShowBboxPlane)
                mpMapDrawer->drawPlanes(g2o::MANHATTAN_PLANE_TYPE::BBOX, PlaneLineWidth); 

            // relations
            if(menuShowRelationArrow)
                mpMapDrawer->drawArrows();

            // 展示Tracking::DenseBuild()中生成的点云
            RefreshMenuForDepthPointCloud();
            RefreshPointCloudOptions();
            float pointcloudSize = SliderPointCloudListSize;
            float pointcloudSink = SliderPointCloudSink;
            mpMapDrawer->drawPointCloudWithOptions(mmPointCloudOptionMap, pointcloudSize, pointcloudSink);
            // end

            if(menuShowEllipsoidVertices){
                float pointcloudSize = SliderPointCloudListSize;
                double ellipsoidProbThresh = SliderEllipsoidProbThresh;
                mpMapDrawer->drawEllipsoidVertices(ellipsoidProbThresh,pointcloudSize);
            }

            pangolin::FinishFrame();
        }

        cv::Mat im = GetRGBFrame();
        // // double scale = float(w) / im.size().width;
        // // cv::Mat scaled_im;
        // // cv::resize(im, scaled_im, cv::Size(0, 0), scale, scale);
        cv::imshow("DSP-SLAM: Current Frame", im);
        cv::waitKey(mT);

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->Reset();
            menuReset = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

// 
void Viewer::RefreshMenuForDepthPointCloud(){
    // unique_lock<mutex> lock(mMutexFinish);

    // 以名称为单位，给 pointcloud list 中的每个点云设置菜单
    auto pointLists = mpSystem->getMap()->GetPointCloudList();

    // Iterate over the menu and delete the menu if the corresponding clouds are no longer available
    // 遍历菜单，如果对应的点云没有了则删除菜单
    for( auto menuPair = mmDepthPointCloudOptionMenus.begin(); menuPair!=mmDepthPointCloudOptionMenus.end();)
    {
        if(pointLists.find(menuPair->first) == pointLists.end())
        {
            if( menuPair->second !=NULL ){
                delete menuPair->second;        // destroy the dynamic menu 
                menuPair->second = NULL;
            }
            menuPair = mmDepthPointCloudOptionMenus.erase(menuPair);  
            continue;
        }
        menuPair++;
    }

    // Iterate over the cloud lists to add new menu.
    // 遍历点云列表，添加新菜单
    for( auto cloudPair: pointLists )
    {
        if(mmDepthPointCloudOptionMenus.find(cloudPair.first) == mmDepthPointCloudOptionMenus.end())
        {
            pangolin::Var<bool>* pMenu = new pangolin::Var<bool>(string("menu.") + cloudPair.first, false, true);
            mmDepthPointCloudOptionMenus.insert(make_pair(cloudPair.first, pMenu));            
        }
    }
}

void Viewer::RefreshPointCloudOptions()
{
    // generate options from mmPointCloudOptionMenus, pointclouds with names will only be drawn when their options are activated.
    std::map<std::string,bool> options;
    for( auto pair : mmDepthPointCloudOptionMenus)
        options.insert(make_pair(pair.first, pair.second->Get()));
    
    mmPointCloudOptionMap.clear();
    mmPointCloudOptionMap = options;
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
