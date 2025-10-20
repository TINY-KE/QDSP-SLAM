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

#include "MapPoint.h"

namespace ORB_SLAM2
{

void MapPoint::SetOutlierFlag()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    mbOutlier = true;
}

bool MapPoint::isOutlier()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbOutlier;
}


// zhjd
// 根据已有point的min和max xy，将0~maxz的点都加入pMO->GetMapPointsOnObject()中。
void MapObject::GetMapPointsWithinBoundingCubeToGround(){
    // 应该与什么其他程序不要冲突呢？？
    // unique_lock<mutex> lock(mMutexFeatures);
    
    std::cout<<"[GetMapPointsWithinBoundingCubeToGround] begin"<<std::endl;

    // 计算points的float minX, float maxX, float minY, float maxY, float maxZ
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();
    float maxZ = std::numeric_limits<float>::lowest();
    std::cout<<"[GetMapPointsWithinBoundingCubeToGround] 0"<<std::endl;

    std::vector<MapPoint*> map_points_copy = GetMapPointsOnObject();

    for (auto pMP : map_points_copy) {
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

        // 更新 minX 和 maxX
        if (xc < minX) minX = xc;
        if (xc > maxX) maxX = xc;

        // 更新 minY 和 maxY
        if (yc < minY) minY = yc;
        if (yc > maxY) maxY = yc;

        // 更新 maxZ
        if (zc > maxZ) maxZ = zc;
    }
    std::cout<<"[GetMapPointsWithinBoundingCubeToGround] 1"<<std::endl;

    vector<MapPoint*> vpMP = mpMap->GetAllMapPoints();

    std::cout<<"[GetMapPointsWithinBoundingCubeToGround] 2"<<std::endl;

    // 存储到新的vector中
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

        if (xc >= minX && xc <= maxX && yc >= minY && yc <= maxY && zc <= maxZ) {
            map_points.insert(pMP);
        }
    }
    std::cout<<"[GetMapPointsWithinBoundingCubeToGround] 3"<<std::endl;

}


}