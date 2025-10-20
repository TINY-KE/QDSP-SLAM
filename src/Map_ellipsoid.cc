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

#include "Map.h"

#include <mutex>

namespace ORB_SLAM2
{
void Map::addArrow(const Vector3d &center, const Vector3d &norm, const Vector3d &color) {
    unique_lock<mutex> lock(mMutexMap);
    Arrow ar;
    ar.center = center;
    ar.norm = norm;
    ar.color = color;
    mvArrows.push_back(ar);

    return;
}

std::vector<Arrow> Map::GetArrows() {
    unique_lock<mutex> lock(mMutexMap);
    return mvArrows;
}

void Map::clearArrows() {
    unique_lock<mutex> lock(mMutexMap);
    mvArrows.clear();
    return;
}

bool Map::AddPointCloudList(const string &name, std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &vCloudPCL, g2o::SE3Quat &Twc, int type) {
    // std::cout<<"[debug] AddPointCloudList -- "<<name<< " 1" << std::endl;
    if (type == REPLACE_POINT_CLOUD) {
        DeletePointCloudList(name, COMPLETE_MATCHING);
    }
    srand(time(0));
    
    // std::cout<<"[debug] AddPointCloudList -- "<<name<< " 2" << std::endl;
    for (auto &cloud : vCloudPCL) {
        ORB_SLAM2::PointCloud cloudQuadri = pclToQuadricPointCloud(cloud);
        ORB_SLAM2::PointCloud *pCloudGlobal = new ORB_SLAM2::PointCloud(cloudQuadri);
        transformPointCloudSelf(pCloudGlobal, &Twc);

        int r = rand() % 155;
        int g = rand() % 155;
        int b = rand() % 155;
        SetPointCloudProperty(pCloudGlobal, r, g, b, 4);
        bool result = AddPointCloudList(name, pCloudGlobal, ADD_POINT_CLOUD);
        if (!result) {
            delete pCloudGlobal;
            pCloudGlobal = NULL;
        }
    }
    
    // std::cout<<"[debug] AddPointCloudList -- "<<name<< " 3" << std::endl;
    return true;
}

bool Map::AddPointCloudList(const string& name, PointCloud* pCloud, int type){  //默认是 REPLACE_POINT_CLOUD（0）
    // std::cout<<"[debug] Map::AddPointCloudList Start, name: "<<name<<std::endl;
    unique_lock<mutex> lock(mMutexMap);
    // std::cout<<"[debug] Map::AddPointCloudList 1, name: "<<name<<std::endl;
    if(pCloud == NULL)
    {
        std::cout << "NULL point cloud." << std::endl;
        return false;
    }
    // std::cout<<"[debug] Map::AddPointCloudList 2, name: "<<name<<std::endl;
    // Check repetition
    if(mmPointCloudLists.find(name) != mmPointCloudLists.end() )
    {
        // Exist
        // std::cout<<"[debug] Map::AddPointCloudList 3-1, name: "<<name<<std::endl;
        auto pCloudInMap = mmPointCloudLists[name];
        if(pCloudInMap==NULL){
            std::cout << "Error: the cloud " << name << " has been deleted." << std::endl;
            return false;
        }

        if( type == REPLACE_POINT_CLOUD){
            // std::cout<<"[debug] Map::AddPointCloudList 4-1, name: "<<name<<", cloud size: "<<pCloud->size()<<std::endl;
            // replace it.
            pCloudInMap->clear(); // release it
            mmPointCloudLists[name] = pCloud;
        }
        else if( type == ADD_POINT_CLOUD )
        {
            // std::cout<<"[debug] Map::AddPointCloudList 4-2, name: "<<name<<", cloud size: "<<pCloud->size()<<std::endl;
            // add together
            for( auto &p : *pCloud ){
                // std::cout<<"[debug] Map::AddPointCloudList 4-2-1, name: "<<name<<", point: "<<p.x<<","<<p.y<<","<<p.z<<std::endl;
                pCloudInMap->push_back(p);
            }
        }
        else 
        {
            // std::cout<<"[debug] Map::AddPointCloudList 4-3, name: "<<name<<std::endl;
            std::cout << "Wrong type : " << type << std::endl;
        }
        // std::cout<<"[debug] Map::AddPointCloudList End, name: "<<name<<std::endl;
        return false;
    }
    else{
        // std::cout<<"[debug] Map::AddPointCloudList 3-2, name: "<<name<<", cloud size: "<<pCloud->size()<<std::endl;
        mmPointCloudLists.insert(make_pair(name, pCloud));
        // std::cout<<"[debug] Map::AddPointCloudList End, name: "<<name<<", cloud size: "<<pCloud->size()<<std::endl;
        return true;
    }
        
}

// 删除点云
bool Map::DeletePointCloudList(const string& name, int type){
    // std::cout << "[debug] Map address: " << this << std::endl;  // 检查this是否合法
    
    unique_lock<mutex> lock(mMutexMap);

    if( type == 0 ) // complete matching: the name must be the same
    {
        auto iter = mmPointCloudLists.find(name);
        if (iter != mmPointCloudLists.end() )
        {
            PointCloud* pCloud = iter->second;
            if(pCloud!=NULL)
            {
                delete pCloud;
                pCloud = NULL;
            }
            mmPointCloudLists.erase(iter);
            return true;
        }
        else{
            std::cerr << "PointCloud name " << name << " doesn't exsit. Can't delete it." << std::endl;
            return false;
        }
    }
    else if ( type == 1 ) // partial matching
    {
        bool deleteSome = false;
        for( auto iter = mmPointCloudLists.begin();iter!=mmPointCloudLists.end();)
        {
            auto strPoints = iter->first;
            if( strPoints.find(name) != strPoints.npos )
            {
                PointCloud* pCloud = iter->second;
                if(pCloud!=NULL)
                {
                    delete pCloud;
                    pCloud = NULL;
                }
                iter = mmPointCloudLists.erase(iter);
                deleteSome = true;
                continue;
            }
            iter++;
        }
        return deleteSome;
    }
    
    return false;
}


bool Map::ClearPointCloudLists(){
    unique_lock<mutex> lock(mMutexMap);

    mmPointCloudLists.clear();
    return true;
}



/**
 * Plane
 */

void Map::addPlane(plane *pPlane, int visual_group) {
    unique_lock<mutex> lock(mMutexMap);
    // pPlane->miVisualGroup = visual_group;
    mspPlanes.insert(pPlane);
}

vector<plane *> Map::GetAllPlanes() {
    unique_lock<mutex> lock(mMutexMap);
    return vector<plane *>(mspPlanes.begin(), mspPlanes.end());
}

void Map::clearPlanes() {
    unique_lock<mutex> lock(mMutexMap);
    mspPlanes.clear();
}



void Map::addRefinedEllipsoidVisual(ellipsoid *pObj) {
    unique_lock<mutex> lock(mMutexRefinedMap);
    mspRefinedEllipsoidsVisual.push_back(pObj);
}

vector<ellipsoid *> Map::GetAllRefinedEllipsoidsVisual() {
    unique_lock<mutex> lock(mMutexRefinedMap);
    return mspRefinedEllipsoidsVisual;
}


// 用于可视化的椭球体，并没用参与优化
// 添加真值/单帧生成的椭球体

void Map::addEllipsoidVisual(ellipsoid *pObj) {
    unique_lock<mutex> lock(mMutexMap);
    mspEllipsoidsVisual.push_back(pObj);
}

vector<ellipsoid *> Map::GetAllEllipsoidsVisual() {
    unique_lock<mutex> lock(mMutexMap);
    return mspEllipsoidsVisual;
}

void Map::ClearEllipsoidsVisual() {
    unique_lock<mutex> lock(mMutexMap);
    // cout << "!!!! Map::ClearEllipsoidsVisual !!!!" << endl;
    mspEllipsoidsVisual.clear();
}


// 向地图中添加 多帧优化后的椭球体
void Map::addEllipsoidObjects(ellipsoid *pObj) {
    unique_lock<mutex> lock(mMutexMap);
    mspEllipsoidsObjects.push_back(pObj);
}

void Map::ClearEllipsoidsObjects() {
    unique_lock<mutex> lock(mMutexMap);
    // cout << "!!!! Map::ClearEllipsoidsObjects !!!!" << endl;
    mspEllipsoidsObjects.clear();
}

vector<ellipsoid *> Map::GetAllEllipsoidsObjects() {
    unique_lock<mutex> lock(mMutexMap);
    return mspEllipsoidsObjects;
}

// 深度点云
std::map<string, PointCloud *> Map::GetPointCloudList() {
    unique_lock<mutex> lock(mMutexMap);
    return mmPointCloudLists;
}

PointCloud Map::GetPointCloudInList(const string &name) {
    unique_lock<mutex> lock(mMutexMap);

    if (mmPointCloudLists.find(name) != mmPointCloudLists.end())
        return *mmPointCloudLists[name];
    else
        return PointCloud(); // 空
}


// ellipsoid-version
// void Map::AddGlobalObjectDetections(ObjectDetection* pOD){
//     unique_lock<mutex> lock(mMutexMap);
//     // if (pOD == NULL) {
//     //     std::cout << "NULL ObjectDetection." << std::endl;
//     //     return;
//     // }
//     mvpGlobalObjectDetections.push_back(pOD);
// }


// std::vector<ObjectDetection*> Map::GetGlobalObjectDetections(){
//     unique_lock<mutex> lock(mMutexMap);
//     return mvpGlobalObjectDetections;
// }

} //namespace ORB_SLAM
