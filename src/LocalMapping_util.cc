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

#include <LocalMapping.h>
#include <ORBmatcher.h>

using namespace std;

namespace ORB_SLAM2
{

/*
 * Tracking utils for stereo+lidar on KITTI
 */
void LocalMapping::MapObjectCulling()
{
    // Check Recent Added MapObjects
    list<MapObject*>::iterator lit = mlpRecentAddedMapObjects.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    const int cnThObs = 2;

    // Treat static and dynamic objects differently
    while(lit != mlpRecentAddedMapObjects.end())
    {
        MapObject* pMO = *lit;
        if (pMO->isDynamic())
        {
            if ((int) nCurrentKFid - (int) pMO->mpNewestKF->mnId  >= 2)
            {
                pMO->SetBadFlag();
                lit = mlpRecentAddedMapObjects.erase(lit);
                mpMap->mnDynamicObj--;
            }
        }

        if(pMO->isBad())
        {
            lit = mlpRecentAddedMapObjects.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMO->mnFirstKFid) >= 2 && pMO->Observations() <= cnThObs)
        {
            pMO->SetBadFlag();
            lit = mlpRecentAddedMapObjects.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMO->mnFirstKFid) >= 3)
            lit = mlpRecentAddedMapObjects.erase(lit);
        else
            lit++;
    }

    // Dynamic objects that aren't recently added
    if (mpMap->mnDynamicObj > 0)
    {
        std::vector<MapObject*> pMOs = mpMap->GetAllMapObjects();
        for (MapObject *pMO : pMOs)
        {
            if (pMO->isDynamic())
            {
                if ((int) nCurrentKFid - (int) pMO->mpNewestKF->mnId  >= 2)
                {
                    pMO->SetBadFlag();
                    mpMap->mnDynamicObj--;
                }
            }
        }
    }
}

// 用于双目模式
void LocalMapping::GetNewObservations()
{

}
// 用于双目模式
void LocalMapping::CreateNewMapObjects()
{

}

/*
 * Tracking utils for monocular input on Freiburg Cars and Redwood OS
 */
void LocalMapping::CreateNewObjectsFromDetections()   // 用于单目模式
{
    
}

void LocalMapping::ProcessDetectedObjects_byPythonReconstruct()
{
    
}




// 处理数据关联失败的物体detection
void LocalMapping::Create_Multi_NewObjectsFromDetections()  // 用于RGBD模式
{
    // cout << "LocalMapping: Started new objects creation" << endl;

    cv::Mat Rcw = mpCurrentKeyFrame->GetRotation();
    cv::Mat tcw = mpCurrentKeyFrame->GetTranslation();
    auto mvpObjectDetections = mpCurrentKeyFrame->GetObjectDetections();
    auto mvpGlobalEllipsolds = mpCurrentKeyFrame->GetEllipsoldsGlobal();
    
    // Create new objects first, otherwise data association might fail
    for (int det_i = 0; det_i < mvpObjectDetections.size(); det_i++)
    {
        auto det = mvpObjectDetections[det_i];

        // If the detection is a new object, create a new map object.
        if (!det->isNew)
            continue;
        if (!det->isGood_OrbPointsEnough && !mb_use_depth_pcd_to_reconstruct)
            continue;
        if (mvpGlobalEllipsolds[det_i] == NULL) 
            continue;

        // Create object with associated feature points
        int class_id = det->label;
        auto pNewObj = new MapObject(mpCurrentKeyFrame, mpMap, class_id);
        mpCurrentKeyFrame->AddMapObject(pNewObj, det_i);
        mpMap->AddMapObject(pNewObj);

        auto mvpMapPoints = mpCurrentKeyFrame->GetMapPointMatches();
        int n_valid_points = 0;
        for (int k_i : det->GetFeaturePoints())
        {
            auto pMP = mvpMapPoints[k_i];
            if (!pMP)
                continue;
            if (pMP->isBad())
                continue;
            pMP->in_any_object = true;
            pMP->object_id = pNewObj->mnId;
            pMP->keyframe_id_added_to_object = int(mpCurrentKeyFrame->mnId);
            pNewObj->AddMapPoints(pMP);
            n_valid_points++;
        }
        // pNewObj->GetMapPointsWithinBoundingCubeToGround();
        // std::cout<<"[GetMapPointsWithinBoundingCubeToGround] end"<<std::endl;
        
        double scale = Config::Get<double>("Mapping.ObjectScale");

        pNewObj->SetPoseByEllipsoid(mvpGlobalEllipsolds[det_i], scale);
    }
}

// 处理数据关联成功的物体detection
void LocalMapping::Process_Multi_DetectedObjects_byPythonReconstruct()
{
    auto SE3Twc = Converter::toMatrix4f(mpCurrentKeyFrame->GetPoseInverse());
    auto SE3Tcw = Converter::toMatrix4f(mpCurrentKeyFrame->GetPose());
    cv::Mat Rcw = mpCurrentKeyFrame->GetRotation();
    cv::Mat tcw = mpCurrentKeyFrame->GetTranslation();
    auto mvpObjectDetections = mpCurrentKeyFrame->GetObjectDetections();
    auto mvpAssociatedObjects = mpCurrentKeyFrame->GetMapObjectMatches();
    auto mvpGlobalEllipsolds = mpCurrentKeyFrame->GetEllipsoldsGlobal();
    
    for (int det_i = 0; det_i < mvpObjectDetections.size(); det_i++)
    {
        auto det = mvpObjectDetections[det_i];

        std::cout<< "[zhjd-debug] Process_Multi_DetectedObjects, KeyFrame id: "<< mpCurrentKeyFrame->mnId <<", detection: "<<det_i<<", isNew:"<< det->isNew<< ", isGood_OrbPointsEnough:"<< det->isGood_OrbPointsEnough<< std::endl;
        // If the detection is associated with an existing map object, we consider 2 different situations:
        // 1. the object has been reconstructed: update observations 2. the object has not been reconstructed:
        // check if it's ready for reconstruction, reconstruct if it's got enough points
        if (det->isNew)   //只有track中数据关联上的物体才会被重建
            continue;
        if (!det->isGood_OrbPointsEnough && !mb_use_depth_pcd_to_reconstruct)
            continue;

        MapObject *pMO = mvpAssociatedObjects[det_i];  
        std::cout<< "[zhjd-debug] Process_Multi_DetectedObjects 检查是否存在关联物体: ";                   
        if (!pMO){
            std::cout<< "pMO 不存在" << std::endl;
            continue;
        }
        else
            std::cout<< "pMO 存在" << std::endl;

        int numKFsPassedSinceInit = int(mpCurrentKeyFrame->mnId - pMO->mpRefKF->mnId);

        // 把深度点云加到地图物体中
        bool success_contruct = false;
        if (mb_use_depth_pcd_to_reconstruct==1) {
            if ( mvpGlobalEllipsolds[det_i] != NULL ) {
                std::cout<<"[debug] 开启基于PCD点云的DeepSDF建模"<<std::endl;
                success_contruct = DeepSDFObjectConstruction_PcdCloud_new(det, pMO, det_i);
            }
        }
        else if(mb_use_depth_pcd_to_reconstruct==0) { 
            if ( mvpGlobalEllipsolds[det_i] != NULL ) {
                std::cout<<"[debug] 开启基于ORB点云的DeepSDF建模"<<std::endl;
                success_contruct = DeepSDFObjectConstruction(det, pMO, det_i);
            }
        }

        std::cout<<"[debug] 完成DeepSDF建模,结果："<<pMO->reconstructed<<std::endl;
        if(pMO->reconstructed){
            pMO->AddObjectObservation(mpCurrentKeyFrame, det_i);
            mpCurrentKeyFrame->AddMapObject(pMO, det_i);
            mpObjectDrawer->AddObject(pMO);
            mlpRecentAddedMapObjects.push_back(pMO);
            nLastReconKFID = int(mpCurrentKeyFrame->mnId);
        }
    }
}

// 2D渲染损失用的是深度点云 
bool LocalMapping::DeepSDFObjectConstruction_PcdCloud_new(ObjectDetection *det, MapObject *pMO, int det_i){

        auto SE3Twc = Converter::toMatrix4f(mpCurrentKeyFrame->GetPoseInverse());
        auto SE3Tcw = Converter::toMatrix4f(mpCurrentKeyFrame->GetPose());
        cv::Mat Rcw = mpCurrentKeyFrame->GetRotation();
        cv::Mat tcw = mpCurrentKeyFrame->GetTranslation();


        // 获取Global PCD Cloud，用于3D形状损失项
        int n_valid_points = 0;  //有效点的数量
        std::shared_ptr<PointCloud> mPointsPtr = pMO->GetPointCloud();
        PointCloud* pPoints = mPointsPtr.get();
        n_valid_points = pPoints->size();

        // 获取当前观测帧中PCD Cloud，用于2D渲染损失项
        int n_rays = 0;
        pcl::PointCloud<PointType>::Ptr mPointsKeyFramePtr= det->getPcdPtr();
        n_rays = mPointsKeyFramePtr->size();  //直接用点云的点数作为ray的数量


        int n_background_ray = det->background_rays.rows();
        
        int min_valid_points = Config::Get<int>("Mapping.MinValidPoints");
        int min_valid_rays = Config::Get<int>("Mapping.MinValidRays");
        bool use_ellipsoid_verticles = Config::Get<int>("Mapping.use_ellipsoid_verticles");
        std::cout<< "[debug] DeepSDFObjectConstruction_PcdCloud_new, points/thresh: "<< n_valid_points << " / "<<min_valid_points<<", rays/thresh: "<< n_rays <<  " / "<< min_valid_rays << std::endl;
        
        if (n_valid_points >= min_valid_points && n_rays > min_valid_rays)
        // if (n_valid_points >= min_valid_points)  //这个判断有必要吗？  因为点云非常稠密
        {
            if(use_ellipsoid_verticles)
                n_valid_points += pMO->GetEllipsoidVertices().size();

            //！获取surface_points_cam
            Eigen::MatrixXf surface_points_cam = Eigen::MatrixXf::Zero(n_valid_points, 3);
            int p_i = 0;

            std::shared_ptr<PointCloud> mPointsPtr = pMO->GetPointCloud();
            PointCloud* pPoints = mPointsPtr.get();
            
            // （1）将PCD点云，转换为DSP表面的点
            for(int i=0; i<pPoints->size(); i=i+1)
            {
                PointXYZRGB &p = (*pPoints)[i];
                cv::Mat x3Dw = (cv::Mat_<float>(3,1) << p.x, p.y, p.z);
                cv::Mat x3Dc = Rcw * x3Dw + tcw;
                float xc = x3Dc.at<float>(0);
                float yc = x3Dc.at<float>(1);
                float zc = x3Dc.at<float>(2);
                surface_points_cam(p_i, 0) = xc;
                surface_points_cam(p_i, 1) = yc;
                surface_points_cam(p_i, 2) = zc;
                p_i++;
            }

            // （2）将椭球体顶点，转换为DSP表面的点
            std::cout<< "[debug] ellipsoid_verticles Start" << std::endl;
            if(use_ellipsoid_verticles){

                std::vector<Eigen::Vector3f>  vertices = pMO->GetEllipsoidVertices();
                
                for(int i=0; i<vertices.size(); i=i+1)
                {
                    Eigen::Vector3f v = vertices[i];
                    cv::Mat x3Dw = (cv::Mat_<float>(3,1) << v[0], v[1], v[2]);
                    cv::Mat x3Dc = Rcw * x3Dw + tcw;
                    float xc = x3Dc.at<float>(0);
                    float yc = x3Dc.at<float>(1);
                    float zc = x3Dc.at<float>(2);
                    surface_points_cam(p_i, 0) = xc;
                    surface_points_cam(p_i, 1) = yc;
                    surface_points_cam(p_i, 2) = zc;
                    p_i++;
                }

            }
            std::cout<< "[debug] ellipsoid_verticles End" << std::endl;

            // （3）获取ray_pixels和depth_obs
            Eigen::MatrixXf ray_pixels = Eigen::MatrixXf::Zero(n_rays, 2);  //ray_pixels: 一个 n_rays × 2 的矩阵，用于存储每条射线在像素坐标系中的位置（x 和 y 坐标）。
            Eigen::VectorXf depth_obs = Eigen::VectorXf::Zero(n_rays);  // depth_obs: 一个长度为 n_rays 的向量，用于存储每条射线对应的深度（即从相机到 3D 点的距离）。
            int k_i = 0;
            pcl::PointCloud<PointType>::Ptr mPointsKeyFramePtr= det->getPcdPtr();
            for (const auto& point : *mPointsKeyFramePtr) 
            {
                cv::Mat x3Dw(3,1, CV_32F);  
                x3Dw.at<float>(0) = point.x;
                x3Dw.at<float>(1) = point.y;
                x3Dw.at<float>(2) = point.z;
                cv::Mat x3Dc = Rcw * x3Dw + tcw;
                depth_obs(k_i) = x3Dc.at<float>(2);

                // 将x3Dc转到像素坐标系
                cv::Mat x2D = mpTracker->GetCameraIntrinsics() * x3Dc;
                ray_pixels(k_i, 0) = x2D.at<float>(0) / x2D.at<float>(2); // 像素坐标 x = u
                ray_pixels(k_i, 1 ) = x2D.at<float>(1) / x2D.at<float>(2); // 像素坐标 y = v

                k_i++;
            }
            // 像素点的归一化的向量 [[x,y,1], ... ]
            Eigen::MatrixXf u_hom(n_rays, 3);
            u_hom << ray_pixels, Eigen::MatrixXf::Ones(n_rays, 1);
            // 转换到相机坐标系的射线向量
            Eigen::MatrixXf fg_rays(n_rays, 3);
            Eigen::Matrix3f invK = Converter::toMatrix3f(mpTracker->GetCameraIntrinsics()).inverse();
            for (int i = 0; i  < n_rays; i++)
            {
                auto x = u_hom.row(i).transpose();
                fg_rays.row(i) = (invK * x).transpose();
            }
            Eigen::MatrixXf rays(fg_rays.rows() + det->background_rays.rows(), 3);
            rays << fg_rays, det->background_rays;



            /**
             * 表面点与射线数据准备完毕，下面进行物体重建
             * 
            */
            PyThreadStateLock PyThreadLock;

            auto Sim3Two_pMO = pMO->Sim3Two;

            int class_id = det->label;
            py::object* optimizer_ptr;
            std::cout<< "[debug] DeepSDFObjectConstruction_PcdCloud, 4"<< std::endl;

            if(mmPyOptimizers.count(class_id) > 0) {
                py::object* optimizer_ptr_local = &(mmPyOptimizers[class_id]);
                optimizer_ptr = optimizer_ptr_local;
            }
            else{
                cout << " [ProcessDetectedObjects_byPythonReconstruct] class " << class_id << " is not in yolo_classes" << endl;
                int default_class_id = 60;  //默认物体设置为桌子
                py::object* optimizer_ptr_local = &(mmPyOptimizers[default_class_id]);
                optimizer_ptr = optimizer_ptr_local;
            }

            // cout << " [debug] Before reconstruct_object from detection ["<< det_i << "]"<< std::endl;

            auto pyMapObject = optimizer_ptr->attr("reconstruct_object")
                    (SE3Tcw * pMO->Sim3Two, surface_points_cam, rays, depth_obs, pMO->vShapeCode);

            cout << " [debug] reconstruct_object 5, class id = "<<  class_id << std::endl;

            // If not initialized, duplicate optimization to resolve orientation ambiguity
            // 翻转物体朝向。这对椭球体来时是非常有必要的
            auto flipped_Two = pMO->Sim3Two;
            flipped_Two.col(0) *= -1;   // 翻转x方向
            flipped_Two.col(2) *= -1;   // 翻转z方向
            // y方向是与地面垂直的，所以不用翻转方向。
            auto pyMapObjectFlipped = optimizer_ptr->attr("reconstruct_object")
                    (SE3Tcw * flipped_Two, surface_points_cam, rays, depth_obs, pMO->vShapeCode);

            double loss = pyMapObject.attr("loss").cast<float>();
            double loss_flipped = pyMapObjectFlipped.attr("loss").cast<float>();
            std::cout<< "[debug] DeepSDFObjectConstruction_PcdCloud_new, object id: "<< pMO->mnId<< ", loss: "<< loss << ", loss_flipped: "<< loss_flipped << std::endl;
            if (loss > loss_flipped)
                pyMapObject = pyMapObjectFlipped;
            
            // cout << " [debug] reconstruct_object 2, class id = "<<  class_id << std::endl;
            
            auto Sim3Tco = pyMapObject.attr("t_cam_obj").cast<Eigen::Matrix4f>();

            det->SetPoseMeasurementSim3(Sim3Tco);
            // // Sim3, SE3, Sim3
            // // std::cbrt(Sim3Two.topLeftCorner<3, 3>().determinant());
            // std::cout << "Sim3Two  scale old = " << std::cbrt(pMO->Sim3Two.topLeftCorner<3, 3>().determinant()) << std::endl;
            Eigen::Matrix4f Sim3Two = SE3Twc * Sim3Tco;
            // // Sim3Two.topLeftCorner<3, 3>() *= 1.2;
            // std::cout << "Sim3Two scale new = " << std::cbrt(Sim3Two.topLeftCorner<3, 3>().determinant())  << std::endl;
            // std::cout << "Sim3Two scale cube = " << sqrt(pMO->w*pMO->w + pMO->h*pMO->h + pMO->l*pMO->l)/2.0 << std::endl;

            int code_len = optimizer_ptr->attr("code_len").cast<int>();
            Eigen::Matrix<float, 64, 1> code = Eigen::VectorXf::Zero(64);
            if (code_len == 32)
            {
                auto code_32 = pyMapObject.attr("code").cast<Eigen::Matrix<float, 32, 1>>();
                code.head(32) = code_32;
            }
            else
            {
                code = pyMapObject.attr("code").cast<Eigen::Matrix<float, 64, 1>>();
            }

            
            // cout << " [debug] Before extract_mesh_from_code for object labe:"<< class_id << ", labe:"<< class_id << std::endl;

            // 获取mesh提取器
            py::object* mesh_extracter_ptr;
            if(mmPyOptimizers.count(class_id) > 0) {
                // cout << " [debug] ProcessDetectedObjects_byPythonReconstruct class " << class_id << " is in yolo_classes" << endl;
                py::object* mesh_extracter_ptr_local = &(mmPyMeshExtractors[class_id]);
                mesh_extracter_ptr = mesh_extracter_ptr_local;
            }
            else{
                cerr << " [debug] ProcessDetectedObjects_byPythonReconstruct class " << class_id << " is NOT in yolo_classes" << endl;
                int default_class_id = 60;  //默认物体设置为桌子
                py::object* mesh_extracter_ptr_local = &(mmPyMeshExtractors[default_class_id]);
                mesh_extracter_ptr = mesh_extracter_ptr_local;
            }

            // cout << " [debug] reconstruct_object 3, class id = "<<  class_id << std::endl;

            pMO->UpdateReconstruction(Sim3Two, code);
            // cout << " [debug] reconstruct_object 3-1, class id = "<<  class_id << std::endl;
            auto pyMesh = mesh_extracter_ptr->attr("extract_mesh_from_code")(code);
            // cout << " [debug] reconstruct_object 3-2, class id = "<<  class_id << std::endl;
            pMO->vertices = pyMesh.attr("vertices").cast<Eigen::MatrixXf>();
            double temp = pyMapObject.attr("loss").cast<float>();
            if(temp < pMO->loss){  //重建失败
                pMO->loss = temp;
            }
            // cout << " [debug] reconstruct_object 3-3, class id = "<<  class_id << std::endl;
            pMO->faces = pyMesh.attr("faces").cast<Eigen::MatrixXi>();
            // cout << " [debug] reconstruct_object 3-4, class id = "<<  class_id << std::endl;
            pMO->reconstructed = true;
        }
        // cout << " [debug] End DeepSDFObjectConstruction_PcdCloud" << std::endl;

        return true;
}


bool LocalMapping::DeepSDFObjectConstruction_PcdCloud(ObjectDetection *det, MapObject *pMO, int det_i){

        auto SE3Twc = Converter::toMatrix4f(mpCurrentKeyFrame->GetPoseInverse());
        auto SE3Tcw = Converter::toMatrix4f(mpCurrentKeyFrame->GetPose());
        cv::Mat Rcw = mpCurrentKeyFrame->GetRotation();
        cv::Mat tcw = mpCurrentKeyFrame->GetTranslation();

        int n_valid_points = 0;  //有效点的数量

        // 获取PCD Cloud
        std::shared_ptr<PointCloud> mPointsPtr = pMO->GetPointCloud();
        PointCloud* pPoints = mPointsPtr.get();
        n_valid_points = pPoints->size();

        // TODO: 此处还是用的ORB feature points， 但似乎也没什么好的解决方法
        // 记录物体上的关键点的数量  2D feature points inside mask 
        int n_rays = 0;
        auto map_points_vector = mpCurrentKeyFrame->GetMapPointMatches();
        for (auto idx : det->GetFeaturePoints())
        {
            auto pMP = map_points_vector[idx];
            if (!pMP)
                continue;
            if (pMP->isBad())
                continue;
            if (pMP->object_id != pMO->mnId)
                continue;
            if (pMP->isOutlier())
                continue;
            n_rays++;
        }


        int n_background_ray = det->background_rays.rows();
        
        int min_valid_points = Config::Get<int>("Mapping.MinValidPoints");
        int min_valid_rays = Config::Get<int>("Mapping.MinValidRays");
        bool use_ellipsoid_verticles = Config::Get<int>("Mapping.use_ellipsoid_verticles");
        std::cout<< "[debug] DeepSDFObjectConstruction_PcdCloud, points/thresh: "<< n_valid_points << " / "<<min_valid_points<<", rays/thresh: "<< n_rays <<  " / "<< min_valid_rays << std::endl;
        
        if (n_valid_points >= min_valid_points && n_rays > min_valid_rays)
        // if (n_valid_points >= min_valid_points)  //这个判断有必要吗？  因为点云非常稠密
        {
            if(use_ellipsoid_verticles)
                n_valid_points += pMO->GetEllipsoidVertices().size();

            //！获取surface_points_cam
            Eigen::MatrixXf surface_points_cam = Eigen::MatrixXf::Zero(n_valid_points, 3);
            int p_i = 0;

            std::shared_ptr<PointCloud> mPointsPtr = pMO->GetPointCloud();
            PointCloud* pPoints = mPointsPtr.get();
            
            // （1）将PCD点云，转换为DSP表面的点
            for(int i=0; i<pPoints->size(); i=i+1)
            {
                PointXYZRGB &p = (*pPoints)[i];
                cv::Mat x3Dw = (cv::Mat_<float>(3,1) << p.x, p.y, p.z);
                cv::Mat x3Dc = Rcw * x3Dw + tcw;
                float xc = x3Dc.at<float>(0);
                float yc = x3Dc.at<float>(1);
                float zc = x3Dc.at<float>(2);
                surface_points_cam(p_i, 0) = xc;
                surface_points_cam(p_i, 1) = yc;
                surface_points_cam(p_i, 2) = zc;
                p_i++;
            }

            // （2）将椭球体顶点，转换为DSP表面的点
            std::cout<< "[debug] ellipsoid_verticles Start" << std::endl;
            if(use_ellipsoid_verticles){

                std::vector<Eigen::Vector3f>  vertices = pMO->GetEllipsoidVertices();
                
                for(int i=0; i<vertices.size(); i=i+1)
                {
                    Eigen::Vector3f v = vertices[i];
                    cv::Mat x3Dw = (cv::Mat_<float>(3,1) << v[0], v[1], v[2]);
                    cv::Mat x3Dc = Rcw * x3Dw + tcw;
                    float xc = x3Dc.at<float>(0);
                    float yc = x3Dc.at<float>(1);
                    float zc = x3Dc.at<float>(2);
                    surface_points_cam(p_i, 0) = xc;
                    surface_points_cam(p_i, 1) = yc;
                    surface_points_cam(p_i, 2) = zc;
                    p_i++;
                }

            }
            std::cout<< "[debug] ellipsoid_verticles End" << std::endl;

            // （3）获取ray_pixels和depth_obs
            Eigen::MatrixXf ray_pixels = Eigen::MatrixXf::Zero(n_rays, 2);  //ray_pixels: 一个 n_rays × 2 的矩阵，用于存储每条射线在像素坐标系中的位置（x 和 y 坐标）。
            Eigen::VectorXf depth_obs = Eigen::VectorXf::Zero(n_rays);  // depth_obs: 一个长度为 n_rays 的向量，用于存储每条射线对应的深度（即从相机到 3D 点的距离）。
            int k_i = 0;
            for (auto point_idx : det->GetFeaturePoints())
            {
                auto pMP = map_points_vector[point_idx];
                if (!pMP)
                    continue;
                if(pMP->isBad())
                    continue;
                if(pMP->object_id != pMO->mnId)
                    continue;
                if (pMP->isOutlier())
                    continue;

                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw * x3Dw + tcw;
                depth_obs(k_i) = x3Dc.at<float>(2);
                ray_pixels(k_i, 0) = mpCurrentKeyFrame->mvKeysUn[point_idx].pt.x;
                ray_pixels(k_i, 1 ) = mpCurrentKeyFrame->mvKeysUn[point_idx].pt.y;
                k_i++;
            }
            // ray_pixels转为齐次坐标
            Eigen::MatrixXf u_hom(n_rays, 3);
            u_hom << ray_pixels, Eigen::MatrixXf::Ones(n_rays, 1);
            // u_hom转换到相机坐标系
            Eigen::MatrixXf fg_rays(n_rays, 3);
            Eigen::Matrix3f invK = Converter::toMatrix3f(mpTracker->GetCameraIntrinsics()).inverse();
            for (int i = 0; i  < n_rays; i++)
            {
                auto x = u_hom.row(i).transpose();
                fg_rays.row(i) = (invK * x).transpose();
            }
            Eigen::MatrixXf rays(fg_rays.rows() + det->background_rays.rows(), 3);
            rays << fg_rays, det->background_rays;



            /**
             * 表面点与射线数据准备完毕，下面进行物体重建
             * 
            */
            PyThreadStateLock PyThreadLock;

            auto Sim3Two_pMO = pMO->Sim3Two;

            int class_id = det->label;
            py::object* optimizer_ptr;
            std::cout<< "[debug] DeepSDFObjectConstruction_PcdCloud, 4"<< std::endl;

            if(mmPyOptimizers.count(class_id) > 0) {
                py::object* optimizer_ptr_local = &(mmPyOptimizers[class_id]);
                optimizer_ptr = optimizer_ptr_local;
            }
            else{
                cout << " [ProcessDetectedObjects_byPythonReconstruct] class " << class_id << " is not in yolo_classes" << endl;
                int default_class_id = 60;  //默认物体设置为桌子
                py::object* optimizer_ptr_local = &(mmPyOptimizers[default_class_id]);
                optimizer_ptr = optimizer_ptr_local;
            }

            // cout << " [debug] Before reconstruct_object from detection ["<< det_i << "]"<< std::endl;

            auto pyMapObject = optimizer_ptr->attr("reconstruct_object")
                    (SE3Tcw * pMO->Sim3Two, surface_points_cam, rays, depth_obs, pMO->vShapeCode);

            cout << " [debug] reconstruct_object 5, class id = "<<  class_id << std::endl;

            // If not initialized, duplicate optimization to resolve orientation ambiguity
            // 翻转物体朝向。这对椭球体来时是非常有必要的
            auto flipped_Two = pMO->Sim3Two;
            flipped_Two.col(0) *= -1;   // 翻转x方向
            flipped_Two.col(2) *= -1;   // 翻转z方向
            // y方向是与地面垂直的，所以不用翻转方向。
            auto pyMapObjectFlipped = optimizer_ptr->attr("reconstruct_object")
                    (SE3Tcw * flipped_Two, surface_points_cam, rays, depth_obs, pMO->vShapeCode);

            if (pyMapObject.attr("loss").cast<float>() > pyMapObjectFlipped.attr("loss").cast<float>())
                pyMapObject = pyMapObjectFlipped;
            
            // cout << " [debug] reconstruct_object 2, class id = "<<  class_id << std::endl;
            
            auto Sim3Tco = pyMapObject.attr("t_cam_obj").cast<Eigen::Matrix4f>();

            det->SetPoseMeasurementSim3(Sim3Tco);
            // // Sim3, SE3, Sim3
            // // std::cbrt(Sim3Two.topLeftCorner<3, 3>().determinant());
            // std::cout << "Sim3Two  scale old = " << std::cbrt(pMO->Sim3Two.topLeftCorner<3, 3>().determinant()) << std::endl;
            Eigen::Matrix4f Sim3Two = SE3Twc * Sim3Tco;
            // // Sim3Two.topLeftCorner<3, 3>() *= 1.2;
            // std::cout << "Sim3Two scale new = " << std::cbrt(Sim3Two.topLeftCorner<3, 3>().determinant())  << std::endl;
            // std::cout << "Sim3Two scale cube = " << sqrt(pMO->w*pMO->w + pMO->h*pMO->h + pMO->l*pMO->l)/2.0 << std::endl;

            int code_len = optimizer_ptr->attr("code_len").cast<int>();
            Eigen::Matrix<float, 64, 1> code = Eigen::VectorXf::Zero(64);
            if (code_len == 32)
            {
                auto code_32 = pyMapObject.attr("code").cast<Eigen::Matrix<float, 32, 1>>();
                code.head(32) = code_32;
            }
            else
            {
                code = pyMapObject.attr("code").cast<Eigen::Matrix<float, 64, 1>>();
            }

            
            // cout << " [debug] Before extract_mesh_from_code for object labe:"<< class_id << ", labe:"<< class_id << std::endl;

            // 获取mesh提取器
            py::object* mesh_extracter_ptr;
            if(mmPyOptimizers.count(class_id) > 0) {
                // cout << " [debug] ProcessDetectedObjects_byPythonReconstruct class " << class_id << " is in yolo_classes" << endl;
                py::object* mesh_extracter_ptr_local = &(mmPyMeshExtractors[class_id]);
                mesh_extracter_ptr = mesh_extracter_ptr_local;
            }
            else{
                cerr << " [debug] ProcessDetectedObjects_byPythonReconstruct class " << class_id << " is NOT in yolo_classes" << endl;
                int default_class_id = 60;  //默认物体设置为桌子
                py::object* mesh_extracter_ptr_local = &(mmPyMeshExtractors[default_class_id]);
                mesh_extracter_ptr = mesh_extracter_ptr_local;
            }

            // cout << " [debug] reconstruct_object 3, class id = "<<  class_id << std::endl;

            pMO->UpdateReconstruction(Sim3Two, code);
            // cout << " [debug] reconstruct_object 3-1, class id = "<<  class_id << std::endl;
            auto pyMesh = mesh_extracter_ptr->attr("extract_mesh_from_code")(code);
            // cout << " [debug] reconstruct_object 3-2, class id = "<<  class_id << std::endl;
            pMO->vertices = pyMesh.attr("vertices").cast<Eigen::MatrixXf>();
            // cout << " [debug] reconstruct_object 3-3, class id = "<<  class_id << std::endl;
            pMO->faces = pyMesh.attr("faces").cast<Eigen::MatrixXi>();
            // cout << " [debug] reconstruct_object 3-4, class id = "<<  class_id << std::endl;
            pMO->reconstructed = true;
        }
        // cout << " [debug] End DeepSDFObjectConstruction_PcdCloud" << std::endl;

        return true;
}

bool LocalMapping::DeepSDFObjectConstruction(ObjectDetection *det, MapObject *pMO, int det_i){
        
        std::cout<< "[debug] DeepSDFObjectConstruction, 1"<< std::endl;
        auto SE3Twc = Converter::toMatrix4f(mpCurrentKeyFrame->GetPoseInverse());
        auto SE3Tcw = Converter::toMatrix4f(mpCurrentKeyFrame->GetPose());
        cv::Mat Rcw = mpCurrentKeyFrame->GetRotation();
        cv::Mat tcw = mpCurrentKeyFrame->GetTranslation();
        int numKFsPassedSinceInit = int(mpCurrentKeyFrame->mnId - pMO->mpRefKF->mnId);
        std::cout<< "[debug] DeepSDFObjectConstruction, 2"<< std::endl;

        // 一个物体被检测到五次，才进行一次重建，从而节约运算资源
        if ((numKFsPassedSinceInit - 15) % mnNumKFsPassedSinceInit_thresh != 0) {
            std::cout << "  Conitinue because (numKFsPassedSinceInit - 15) % 5 != 0" << std::endl;
            return false;
        }
        std::cout<< "[debug] DeepSDFObjectConstruction, 3"<< std::endl;

        // 如果自上次重建后经过的关键帧数量少于8个，则跳过重建，从而节约运算资源
        int numKFsPassedSinceLastRecon = int(mpCurrentKeyFrame->mnId) - nLastReconKFID;
        if (numKFsPassedSinceLastRecon  < mnNumKFsPassedSinceLastRecon_thresh)
            return false;
        std::cout<< "[debug] DeepSDFObjectConstruction, 4"<< std::endl;
        
        // 1. 统计物体 pMO 上有效（三维）地图点的数量，存储在变量 n_valid_points 中。
        std::vector<MapPoint*> points_on_object = pMO->GetMapPointsOnObject();
        int n_points = points_on_object.size();
        int n_valid_points = 0;
        for (auto pMP : points_on_object)
        {
            if (!pMP)
                continue;
            if (pMP->isBad())
                continue;
            if (pMP->isOutlier())
                continue;
            n_valid_points++;
        }

        // 2. 统计与检测到的（二维）特征点（FeaturePoints）相关联的、属于目标物体的有效射线数量，存储在变量 n_rays 中。
        int n_rays = 0;
        auto map_points_vector = mpCurrentKeyFrame->GetMapPointMatches();
        for (auto idx : det->GetFeaturePoints())
        {
            auto pMP = map_points_vector[idx];
            if (!pMP)
                continue;
            if (pMP->isBad())
                continue;
            if (pMP->object_id != pMO->mnId)
                continue;
            if (pMP->isOutlier())
                continue;
            n_rays++;
        }
        cout << "Object " << pMO->mnId << ": " << n_points << " points observed, " << "with " << n_valid_points << " valid points, and " << n_rays << " rays" << endl;

        // Surface points
        // if (n_valid_points >= 50 && n_rays > 20)
        if (n_valid_points >= 10 && n_rays > 10)
        {
            Eigen::MatrixXf surface_points_cam = Eigen::MatrixXf::Zero(n_valid_points, 3);
            // 3. （三维）sdf表面的点，存储在 surface_points_cam 中。
            int p_i = 0;
            for (auto pMP : points_on_object)
            {
                if (!pMP)
                    continue;
                if (pMP->isBad())
                    continue;
                if (pMP->isOutlier())
                    continue;

                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw * x3Dw + tcw;
                float xc = x3Dc.at<float>(0);
                float yc = x3Dc.at<float>(1);
                float zc = x3Dc.at<float>(2);
                surface_points_cam(p_i, 0) = xc;
                surface_points_cam(p_i, 1) = yc;
                surface_points_cam(p_i, 2) = zc;
                p_i++;
            }

            // 4. Rays 初始化射线和深度变量
            Eigen::MatrixXf ray_pixels = Eigen::MatrixXf::Zero(n_rays, 2);
            Eigen::VectorXf depth_obs = Eigen::VectorXf::Zero(n_rays);
            
            // 5. 遍历（二维）特征点并筛选有效的地图点
            int k_i = 0;
            for (auto point_idx : det->GetFeaturePoints())
            {
                auto pMP = map_points_vector[point_idx];
                if (!pMP)
                    continue;
                if(pMP->isBad())
                    continue;
                if(pMP->object_id != pMO->mnId)
                    continue;
                if (pMP->isOutlier())
                    continue;

                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw * x3Dw + tcw;
                depth_obs(k_i) = x3Dc.at<float>(2);
                ray_pixels(k_i, 0) = mpCurrentKeyFrame->mvKeysUn[point_idx].pt.x;
                ray_pixels(k_i, 1 ) = mpCurrentKeyFrame->mvKeysUn[point_idx].pt.y;
                k_i++;
            }

            // 6. 利用（二维）特征点ray_pixels，计算前景射线。将射线转换为相机坐标系下的射线
            Eigen::MatrixXf u_hom(n_rays, 3);
            u_hom << ray_pixels, Eigen::MatrixXf::Ones(n_rays, 1);
            Eigen::MatrixXf fg_rays(n_rays, 3);
            Eigen::Matrix3f invK = Converter::toMatrix3f(mpTracker->GetCameraIntrinsics()).inverse();
            for (int i = 0; i  < n_rays; i++)
            {
                auto x = u_hom.row(i).transpose();
                fg_rays.row(i) = (invK * x).transpose();
            }

            // 7. 背景射线。通过python程序get_detections获得
            
            // 8. 合并前景射线和背景射线。 推测：前景是二维特征点对应的线，背景是物体mask中每个像素对应的线
            Eigen::MatrixXf rays(fg_rays.rows() + det->background_rays.rows(), 3);
            rays << fg_rays, det->background_rays;

            PyThreadStateLock PyThreadLock;
            

            // object_class_table = {
            //     "cars": [2],
            //     "benches": [13], # 板凳
            //     "backpack": [24], # 背包
            //     "chairs": [56], # 椅子
            //     "counchs": [57], # 沙发
            //     "bottles": [39], # 瓶子
            //     "wine_glasses": [40], # 酒杯
            //     "cups": [41], # 杯子
            //     "bowls": [45], # 碗
            //     "bananas": [46], "apples": [47], "oranges": [49],
            //     "potted_plants": [58], # 盆栽植物
            //     "beds": [59],
            //     "dining_tables": [60], #桌子
            //     "tv_monitor": [62],
            //     "laptop": [63],
            //     "mouse": [64],
            //     "keyboard": [66],
            //     "microwave": [68], "oven":[69], "toaster": [70], "refrigerator": [72],
            //     "book": [73], "clock": [74], "vase": [75], "teddy_bears": [77]
            // }
            // 获取dsp优化器
            int class_id = det->label;  //临时设置为60table，用于debug
            // class_id = 57; //counchs 沙发
            // class_id = 75; //vase 花瓶
            // class_id = 60;  //桌子
            // class_id = 56;  //椅子
            // class_id = 62;  //显示器
            // class_id = 63;  //笔记本 laptop

            std::cout<<"[zhjd-debug] Process_Multi_DetectedObjects class_id: "<<class_id<<std::endl;
            py::object* optimizer_ptr;
            // chair2counch
            if(mmPyOptimizers.count(class_id) > 0) {
                if(class_id==56 && mbChair2counch)  //56是椅子
                    class_id = 57;  //57是沙发
                // if(class_id==62)  //62是电视
                //     class_id = 2;  //2是汽车
                py::object* optimizer_ptr_local = &(mmPyOptimizers[class_id]);
                optimizer_ptr = optimizer_ptr_local;
            }
            else{
                cout << " [ProcessDetectedObjects_byPythonReconstruct] class " << class_id << " is not in yolo_classes" << endl;
                int default_class_id = 60;  //默认物体设置为桌子
                py::object* optimizer_ptr_local = &(mmPyOptimizers[default_class_id]);
                optimizer_ptr = optimizer_ptr_local;
            }

            cout << "Before reconstruct_object" << std::endl;

            auto pyMapObject = optimizer_ptr->attr("reconstruct_object")
                    (SE3Tcw * pMO->Sim3Two, surface_points_cam, rays, depth_obs, pMO->vShapeCode);
            cout << "reconstruct_object 1, class id = "<<  class_id << std::endl;

            // If not initialized, duplicate optimization to resolve orientation ambiguity
            if (!pMO->reconstructed)
            {
                auto flipped_Two = pMO->Sim3Two;
                flipped_Two.col(0) *= -1;
                flipped_Two.col(2) *= -1;
                auto pyMapObjectFlipped = optimizer_ptr->attr("reconstruct_object")
                        (SE3Tcw * flipped_Two, surface_points_cam, rays, depth_obs, pMO->vShapeCode);

                if (pyMapObject.attr("loss").cast<float>() > pyMapObjectFlipped.attr("loss").cast<float>())
                    pyMapObject = pyMapObjectFlipped;
            }
            cout << "reconstruct_object 2, class id = "<<  class_id << std::endl;
            
            auto Sim3Tco = pyMapObject.attr("t_cam_obj").cast<Eigen::Matrix4f>();

            det->SetPoseMeasurementSim3(Sim3Tco);
            // Sim3, SE3, Sim3
            // std::cbrt(Sim3Two.topLeftCorner<3, 3>().determinant());
            std::cout << "Sim3Two  scale old = " << std::cbrt(pMO->Sim3Two.topLeftCorner<3, 3>().determinant()) << std::endl;
            Eigen::Matrix4f Sim3Two = SE3Twc * Sim3Tco;
            // Sim3Two.topLeftCorner<3, 3>() *= 1.2;
            std::cout << "Sim3Two scale new = " << std::cbrt(Sim3Two.topLeftCorner<3, 3>().determinant())  << std::endl;
            std::cout << "Sim3Two scale cube = " << sqrt(pMO->w*pMO->w + pMO->h*pMO->h + pMO->l*pMO->l)/2.0 << std::endl;

            int code_len = optimizer_ptr->attr("code_len").cast<int>();
            Eigen::Matrix<float, 64, 1> code = Eigen::VectorXf::Zero(64);
            if (code_len == 32)
            {
                auto code_32 = pyMapObject.attr("code").cast<Eigen::Matrix<float, 32, 1>>();
                code.head(32) = code_32;
            }
            else
            {
                code = pyMapObject.attr("code").cast<Eigen::Matrix<float, 64, 1>>();
            }

            
            cout << "Before extract_mesh_from_code" << std::endl;

            // 获取mesh提取器
            py::object* mesh_extracter_ptr;
            if(mmPyOptimizers.count(class_id) > 0) {
                py::object* mesh_extracter_ptr_local = &(mmPyMeshExtractors[class_id]);
                mesh_extracter_ptr = mesh_extracter_ptr_local;
            }
            else{
                cout << " [ProcessDetectedObjects_byPythonReconstruct] class " << class_id << " is not in yolo_classes" << endl;
                int default_class_id = 60;  //默认物体设置为桌子
                py::object* mesh_extracter_ptr_local = &(mmPyMeshExtractors[default_class_id]);
                mesh_extracter_ptr = mesh_extracter_ptr_local;
            }

            pMO->UpdateReconstruction(Sim3Two, code);
            auto pyMesh = mesh_extracter_ptr->attr("extract_mesh_from_code")(code);
            pMO->vertices = pyMesh.attr("vertices").cast<Eigen::MatrixXf>();
            pMO->faces = pyMesh.attr("faces").cast<Eigen::MatrixXi>();
            pMO->reconstructed = true;


            return true;
            // pMO->AddObjectObservation(mpCurrentKeyFrame, det_i);
            // mpCurrentKeyFrame->AddMapObject(pMO, det_i);
            // mpObjectDrawer->AddObject(pMO);
            // mlpRecentAddedMapObjects.push_back(pMO);

            // nLastReconKFID = int(mpCurrentKeyFrame->mnId);
        }
}
 
// 根据已有point的min和max xy，将0~maxz的点都加入pMO->GetMapPointsOnObject()中。
std::vector<MapPoint*> LocalMapping::AddCubePointsToMapObject(std::vector<MapPoint*> points){
    // 计算points的float minX, float maxX, float minY, float maxY, float maxZ
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();
    float maxZ = std::numeric_limits<float>::lowest();

    for (auto pMP : points) {
        if (!pMP)
            continue;
        if (pMP->isBad())
            continue;
        if (pMP->isOutlier())
            continue;

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

    vector<MapPoint*> vpMP = mpMap->GetAllMapPoints();

    // 存储到新的vector中
    std::vector<MapPoint*> cube_points_on_object;
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
            cube_points_on_object.push_back(pMP);
        }
    }

    return cube_points_on_object;
}




void LocalMapping::AssociateObjects3D()
{
    cout << "\n [LocalMapping_utils.cc] AssociateObjects3D" << endl;
    auto allMapObjects = mpMap->GetAllMapObjects();
    int obj_num = allMapObjects.size();

    for (int i = 0; i < obj_num-1; i++) {
        for (int j = obj_num - 1; j > i; j--) {

            auto pMO_i = allMapObjects[i], pMO_j = allMapObjects[j];
            

            bool c0 = (pMO_i->label == pMO_j->label);

            auto SE3Two_i = pMO_i->GetPoseSE3();
            auto SE3Two_j = pMO_j->GetPoseSE3();


            // TODO：这个距离是在世界坐标系中的，但是世界坐标系并不与地面对齐
            Eigen::Vector3f dist3D = SE3Two_i.topRightCorner<3, 1>() - SE3Two_j.topRightCorner<3, 1>();
            float dist3D_norm = dist3D.norm();

            // FIXME: 这里的0.5有待改成配置文件中进行设置
            // bool c1 = (dist3D_norm < dist_filt_param * dist_limit);
            bool c1 = (dist3D_norm < 1);
            if (c0 && c1) {
            // if (c1) {
                // 这里应该把 pMO_j 合并到 pMO_i 中
                MergeMapObject(pMO_i, pMO_j);
            }
        }
    }
}


// 先将j中的点加入到i物体中
// 同时将每个点的object_id设置为i的id
// 然后将j设置为bad

// 物体对应的观测还没有处理？？？？？？？  pMO->AddObservation(pKF, i);    参考雷达点云建图中的ObjectDataAssociation(KeyFrame *pKF)
// 关键帧对应的物体还没有处理？？？？？？？  pKF->AddMapObject(pMO, d_i);  参考单目建图中的AssociateObjectsByProjection(ORB_SLAM2::KeyFrame *pKF)

// 如果物体融合了，会不会因此在图优化中引入错误mask误差项。
void LocalMapping::MergeMapObject(MapObject* pMO_i, MapObject* pMO_j)
{   
    std::cout<<"[MergeMapObject] pMO_i->mnId: "<<pMO_i->mnId<<", pMO_j->mnId: "<<pMO_j->mnId<<std::endl;
    std::vector<MapPoint*> points_on_object_j = pMO_j->GetMapPointsOnObject();
    for (auto pMP : points_on_object_j)
    {
        if (!pMP)
            continue;
        if (pMP->isBad())
            continue;
        if (pMP->isOutlier())
            continue;
        
        pMP->object_id = pMO_i->mnId;
        pMO_i->AddMapPoints(pMP);
    }

    pMO_i->ComputeCuboidPCA(true);  //测试完，感觉没用

    pMO_j->SetBadFlag();
}



void LocalMapping::UpdateObjectsToMap()
{
    // cout << "\n[LocalMapping::UpdateObjectsToMap]" << endl;
    
    // mpMap->ShowMapInfo();

    // 每次都会重新更新一遍地图中的“椭球体”
    mpMap->ClearEllipsoidsObjects();
    mpMap->DeletePointCloudList("ObjectPCDCloud - Global", 0);
    int ellip_num_valid = 0;
    int pc_num_valid = 0;

    auto mapObjects = mpMap->GetAllMapObjects();
    for (auto &pMO: mapObjects){
        if (pMO->isBad()) {
            continue;
        }

        // FIXME：这里添加了一个没有初始化的ellipsold导致显示错误，暂时通过判断e的概率小于0.001
        auto e = pMO->GetEllipsold();
        if (e != NULL) {
            mpMap->addEllipsoidObjects(e);
            ellip_num_valid++;
        }
        else{
            continue;
        }

        // TODO: 下一步考虑如何将所有物体的点云都添加到地图中，并且可以在每个新帧进行更新
        if (pMO->hasValidDepthPointCloud()){

            pc_num_valid++;
            // std::shared_ptr<PointCloud> mPointsPtr = pMO->GetPointCloud();
            // PointCloud* pPoints = mPointsPtr.get();
            auto pcl_ptr= pMO->GetDepthPointCloudPCL();
            auto pPoints = pclXYZToQuadricPointCloudPtr(pcl_ptr);
            // auto pcd = pMO->GetPointCloud();

            mpMap->AddPointCloudList("ObjectPCDCloud - Global", pPoints, 1);

            // int n_valid_points = pPoints->size();
            // cout << "MapObject PointCloud size = " << pPoints->size() << endl;
        }

    }
    // cout << " - pc_num_valid = " << pc_num_valid << endl;
    // cout << " - ellip_num_valid = " << ellip_num_valid << endl;
}




std::vector<Vector3d> LocalMapping::getVerticesOfEllipsoid(ellipsoid* pEllipsoid, int num) {

    std::vector<Vector3d> vertices;
    Eigen::Matrix4d S = Eigen::Matrix4d::Identity();
    S(0,0) = pEllipsoid->scale(0);
    S(1,1) = pEllipsoid->scale(1);
    S(2,2) = pEllipsoid->scale(2);

    double max_angle_deg = 20.0;
    double max_angle_rad = max_angle_deg * M_PI / 180.0;
    // // 顶面
    // for (int i = 0; i < num; ++i) {
    //     double u = drand48();  // in [0,1)
    //     double v = drand48();

    //     double theta = 2 * M_PI * u;    //// 方位角（绕 z 轴），与x轴的夹角
    //     double phi = max_angle_rad * (v-1);   // 极角（与 z 轴夹角），限制在顶点附近的锥体区域

    //     double x = sin(phi) * cos(theta);
    //     double y = sin(phi) * sin(theta);
    //     double z = cos(phi);

    //     Eigen::Vector4d p_unit(x, y, z, 1.0);
    //     Eigen::Vector4d p_e = S * p_unit;

    //     // 椭球表面点
    //     Eigen::Vector3d sampled_point = p_e.head<3>();
    //     vertices.push_back(sampled_point);
    // }
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

    return vertices;
}


}
