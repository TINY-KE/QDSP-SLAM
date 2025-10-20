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

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;

    double ImageWidth = Config::Get<int>("Camera.width");
    double ImageHeight = Config::Get<int>("Camera.height");

    mIm = cv::Mat(ImageHeight,ImageWidth,CV_8UC3, cv::Scalar(0,0,0));
    mmDepth = cv::Mat(ImageHeight,ImageWidth,CV_8UC3, cv::Scalar(0,0,0));

}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    // cvshow中显示mask和bbox
    vector<cv::Mat> vmasks;
    vector<vector<int>> vbboxs;

    // std::cout<<"[debug] FrameDrawer::DrawFrame() 1: "<<std::endl;
    // std::cout << "  mIm size: " << mIm.size() << ", channels: " << mIm.channels() << std::endl;

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;

        // cvshow中显示mask和bbox
        for (auto mask: mvImObjectMasks) {
            vmasks.push_back(mask);
        }
        for (auto bbox: mvImObjectBboxs) {
            vbboxs.push_back(bbox);
        }


        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    // cvshow中显示mask和bbox
    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);
    // std::cout<<"[debug] FrameDrawer::DrawFrame() 8"<<std::endl;


    for (auto &mask: vmasks) {
        cv::Mat mask_rgb = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC3);
        cv::Mat mask_grey = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC1);
        // 1、掩膜转换为单通道灰度图：
        mask.convertTo(mask, CV_8U);
        // cout << "       im info : " << im.rows << "," << im.cols << "," << im.type() << endl;
        // cout << "     mask info : " << mask.rows << "," << mask.cols << "," << mask.type() << endl;
        // cout << " mask_rgb info : " << mask_rgb.rows << "," << mask_rgb.cols << "," << mask_rgb.type() << endl;
        // cout << "mask_grey info : " << mask_grey.rows << "," << mask_grey.cols << "," << mask_grey.type() << endl;
        // 2、生成三通道的彩色掩膜：
        vector<cv::Mat> channels;
        channels.push_back(mask_grey);
        channels.emplace_back(mask_grey);
        channels.emplace_back(mask);
        merge(channels, mask_rgb);
        // 3、将掩膜叠加到原图：
        // std::cout << "  im size: " << im.size() << ", channels: " << im.channels() << std::endl;
        // std::cout << "  mask_rgb size: " << mask_rgb.size() << ", channels: " << mask_rgb.channels() << std::endl;
        cv::addWeighted(im, 1, mask_rgb, 0.2, 0.0, im);
        // std::cout<<"[debug] FrameDrawer::DrawFrame() 8-9"<<std::endl;
    }
    // std::cout<<"[debug] FrameDrawer::DrawFrame() 9"<<std::endl;
    for (auto &bbox: vbboxs) {
        int x1 = bbox[0], y1 = bbox[1], x2 = bbox[2], y2 = bbox[3];
        cv::rectangle(im, cv::Point2f(float(x1), float(y1)), cv::Point2f(float(x2), float(y2)), \
                      cv::Scalar(0,0,200));
    }

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }        
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                bool not_draw_orb_points = Config::Get<int>("Viewer.not_draw_orb_points");
                if(!not_draw_orb_points){
                    // This is a match to a MapPoint in the map
                    if(vbMap[i])
                    {
                        cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                        cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                        mnTracked++;
                    }
                    else // This is match to a "visual odometry" MapPoint created in the last frame
                    {
                        cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                        cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                        mnTrackedVO++;
                    }
                }
                
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);
    // std::cout<<"[debug] FrameDrawer::DrawFrame() 10"<<std::endl;

    // mIm = imWithInfo.clone();
    return imWithInfo;
}

// cv::Mat FrameDrawer::DrawDepthFrame(){
//     return mmDepth;
// }


cv::Mat FrameDrawer::DrawDepthFrame() {
    Frame* frame = &(mpTracker->mCurrentFrame);

    if(frame->pointcloud_img.empty())
        return mmDepth;

    cv::Mat I = frame->pointcloud_img;   // U16C1 , ushort
    cv::Mat im,R,G,B;

    double min;
    double max;
    cv::minMaxIdx(I, &min, &max);

    I.convertTo(im,CV_8UC1, 255 / (max-min), -min/(max-min)*255); 

    Vector3d color1(51,25,0);
    Vector3d color2(255,229,204);
    // Vector3d color1(255,20,0);
    // Vector3d color2(0,20,255);

    Vector3d scale_color = (color2-color1) / 255.0;
    // r = color1 + value/255*(color2-color1)
    // (255-51)/255   51
    // 220-25

    // ********************************************
    // Vector3d color1(0,204,102);
    // Vector3d color2(255,204,102);
    // double depth_thresh = 8;
    // double scale = Config::Get<double>("Camera.scale"); 

    // Vector3d scale_param = (color2-color1) / scale / depth_thresh;
    // // I / scale / depth_thresh   (0-1)   * (color2-color1) + color 1

    // I.convertTo(B, CV_8UC1, scale_param[0], color1[0]);
    // I.convertTo(G, CV_8UC1, scale_param[1], color1[1]);
    // I.convertTo(R, CV_8UC1, scale_param[2], color1[2]);
    // *********************************************

    im.convertTo(R, CV_8UC1, scale_color[0], color1[0]);
    im.convertTo(G, CV_8UC1, scale_color[1], color1[1]);
    im.convertTo(B, CV_8UC1, scale_color[2], color1[2]);

    std::vector<cv::Mat> array_to_merge;
    array_to_merge.push_back(B);
    array_to_merge.push_back(G);
    array_to_merge.push_back(R);
    cv::merge(array_to_merge, im);

    mmDepth = im.clone();

    // cv::Mat out = drawObservationOnImage(im, false);
    // mmDepth = out.clone();

    return mmDepth;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    mpTracker = pTracker;
    if (pTracker->mImGray.empty()) {
        std::cerr << "The cv::Mat is empty!" << std::endl;
        std::exit(EXIT_FAILURE);
    } 
    pTracker->mImColor.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;

    // cvshow中显示mask和bbox
    mvImObjectMasks.clear();
    mvImObjectBboxs.clear();
    for (auto img: pTracker->mvImObjectMasks) {
        mvImObjectMasks.push_back(img);
    }
    for (auto bbox: pTracker->mvImObjectBboxs) {
        mvImObjectBboxs.push_back(bbox);
    }

    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
    
    // // 绘制
    // DrawFrame();
    // DrawDepthFrame();

}

} //namespace ORB_SLAM
