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

#include "Tracking.h"
#include "ObjectDetection.h"
#include "ORBmatcher.h"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

using namespace std;

namespace ORB_SLAM2 {

/*
 * Tracking utils for stereo+lidar on KITTI
 */
void Tracking::GetObjectDetectionsLiDAR(KeyFrame *pKF) {

    PyThreadStateLock PyThreadLock;

    py::list detections = mpSystem->pySequence.attr("get_frame_by_id")(pKF->mnFrameId);
    for (auto det : detections) {
        auto pts = det.attr("surface_points").cast<Eigen::MatrixXf>();
        auto Sim3Tco = det.attr("T_cam_obj").cast<Eigen::Matrix4f>();
        auto rays = det.attr("rays");
        Eigen::MatrixXf rays_mat;
        Eigen::VectorXf depth;

        if (rays.is_none()) {
            // std::cout << "No 2D masks associated!" << std::endl;
            rays_mat = Eigen::Matrix<float, 0, 0>::Zero();
            depth = Eigen::Matrix<float, 0, 1>::Zero();
        } else {
            rays_mat = rays.cast<Eigen::MatrixXf>();
            depth = det.attr("depth").cast<Eigen::VectorXf>();
        }
        // Create C++ detection instance
        auto o = new ObjectDetection(Sim3Tco, pts, rays_mat, depth);
        pKF->mvpDetectedObjects.push_back(o);
    }
    pKF->nObj = pKF->mvpDetectedObjects.size();
    pKF->mvpMapObjects = vector<MapObject *>(pKF->nObj, static_cast<MapObject *>(NULL));
}

void Tracking::ObjectDataAssociation_onlyforStereo(KeyFrame *pKF)
{
    vector<MapObject *> vpLocalMapObjects;
    // Loop over all the local frames to find matches
    for (KeyFrame *plKF : mvpLocalKeyFrames)
    {
        vector<MapObject *> vpMOs = plKF->GetMapObjectMatches();
        for (MapObject *pMO : vpMOs)
        {
            if (pMO)
            {
                // Prevent multiple association to the same object
                if (pMO->mnAssoRefID != pKF->mnId)
                {
                    vpLocalMapObjects.push_back(pMO);
                    pMO->mnAssoRefID = pKF->mnId;
                }
            }
        }
    }
    if (vpLocalMapObjects.empty())
        return;

    Eigen::Matrix4f Tcw = Converter::toMatrix4f(mCurrentFrame.mTcw);
    Eigen::Matrix3f Rcw = Tcw.topLeftCorner<3, 3>();
    Eigen::Vector3f tcw = Tcw.topRightCorner<3, 1>();
    auto vDetections = pKF->mvpDetectedObjects;
    // loop over all the detections.
    for (int i = 0; i < pKF->nObj; i++)
    {
        auto det = vDetections[i];
        Eigen::Vector3f transDet = det->tco;
        vector<float> dist;

        for (auto pObj : vpLocalMapObjects)
        {
            if (!pObj || pObj->isBad())
            {
                dist.push_back(1000.0);
                continue;
            }

            if (!pObj->isDynamic()) {
                Eigen::Vector3f dist3D = Rcw * pObj->two + tcw - transDet;
                Eigen::Vector2f dist2D;
                dist2D << dist3D[0], dist3D[2];
                dist.push_back((dist2D).norm());
            }
            else
            {
                float deltaT = (float) (mCurrentFrame.mnId - mpLastKeyFrame->mnFrameId);
                auto twoPredicted = pObj->two + pObj->velocity * deltaT;
                Eigen::Vector3f dist3D = Rcw * twoPredicted + tcw - transDet;
                Eigen::Vector2f dist2D;
                dist2D << dist3D[0], dist3D[2];
                dist.push_back((dist2D).norm());
            }
        }
        float minDist = *min_element(dist.begin(), dist.end());

        // Start with a loose threshold
        if (minDist < 5.0)
        {
            det->isNew = false;
            if (det->nPts < 25)
                det->isGood_OrbPointsEnough = false;

            int idx = min_element(dist.begin(), dist.end()) - dist.begin();
            MapObject *pMO = vpLocalMapObjects[idx];
            if (!pKF->mdAssociatedObjects.count(pMO)) {
                pKF->mdAssociatedObjects[pMO] = minDist;
                pKF->AddMapObject(pMO, i);
                pMO->AddObjectObservation(pKF, i);
            } else // Another detection is associated with pMO, compare distance
            {
                if (minDist < pKF->mdAssociatedObjects[pMO]) {
                    // cout << "Associated to: " << pMO->mnId << ", Distance: " << minDist << endl;
                    pKF->mdAssociatedObjects[pMO] = minDist;
                    int detId = pMO->GetObservations()[pKF];
                    pKF->EraseMapObjectMatch(detId);
                    vDetections[detId]->isNew = true;
                    pKF->AddMapObject(pMO, i);
                    pMO->AddObjectObservation(pKF, i);
                }
            }
        }
        else
        {
            det->isNew = true;
            if (det->nPts < 50)
                det->isGood_OrbPointsEnough = false;
        }
    }
}

/*
 * Tracking utils for monocular input on Freiburg Cars and Redwood OS
 */
cv::Mat Tracking::GetCameraIntrinsics()
{
    return mK;
}


void Tracking::GetObjectDetectionsRGBD(KeyFrame *pKF)
{
    PyThreadStateLock PyThreadLock;
    mvImObjectMasks.clear();
    mvImObjectBboxs.clear();


    // 调用python接口获取，物体检测结果
    py::list detections;
    if (mbUseRos) {
        string file_name = "ros.png";
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        cv::imwrite(mDatasetPathRoot+"/"+file_name, pKF->color_img);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        std::cout << "[zhjd-debug] 存储图片花费了 " << ttrack << std::endl;
        detections= mpSystem->pySequence.attr("get_frame_by_name")(pKF->mnFrameId, file_name);
    }
    else {
        std::string frame_name = mvstrImageFilenamesRGB[pKF->mnFrameId];
        detections= mpSystem->pySequence.attr("get_frame_by_name")(pKF->mnFrameId, frame_name);
    }

    int num_dets = detections.size();
    // No detections, return immediately
    if (num_dets == 0)
        return;

    for (int detected_idx = 0; detected_idx < num_dets; detected_idx++)
    {
        auto det = new ObjectDetection();
        auto py_det = detections[detected_idx];
        det->background_rays = py_det.attr("background_rays").cast<Eigen::MatrixXf>();
        auto mask = py_det.attr("mask").cast<Eigen::MatrixXf>();
        det->bbox = py_det.attr("bbox").cast<Eigen::Vector4d>();
        det->label = py_det.attr("label").cast<int>();
        det->prob = py_det.attr("prob").cast<double>();

        cv::Mat mask_cv;
        cv::eigen2cv(mask, mask_cv);
        // cv::imwrite("mask.png", mask_cv);
        cv::Mat mask_erro = mask_cv.clone();
        cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE,
                                               cv::Size(2 * maskErrosion + 1, 2 * maskErrosion + 1),
                                               cv::Point(maskErrosion, maskErrosion));
        
        cv::erode(mask_cv, mask_erro, kernel);

        // 物体检测框和掩码，用于可视化
        int x1 = (int)(det->bbox(0)), y1 = (int)(det->bbox(1)), \
            x2 = (int)(det->bbox(2)), y2 = (int)(det->bbox(3));
        mvImObjectMasks.push_back(std::move(mask_cv));
        mvImObjectBboxs.push_back({x1,y1,x2,y2});


        // get 2D feature points inside mask
        for (int i = 0; i < pKF->mvKeys.size(); i++)
        {
            int val = (int)mask_erro.at<float>(pKF->mvKeys[i].pt.y, pKF->mvKeys[i].pt.x);
            if (val > 0)  // inside the mask
            {
                det->AddFeaturePoint(i);
            }
        }

        // Reject the detection if too few keypoints are extracted
        // todo: 设置点数量数量而不是isGood
        if (det->NumberOfPoints() < mMinimux_Points_To_Judge_Good)
        {
            std::cout << "\033[31m" << "     物体class为"<< det->label<<"的Detection包含point较少，设置为bad。" << "\033[0m" << std::endl;
            det->isGood_OrbPointsEnough = false;
        }
        pKF->mvpDetectedObjects.push_back(det);

    }

    pKF->nObj = pKF->mvpDetectedObjects.size();
    pKF->mvpMapObjects = vector<MapObject *>(pKF->nObj, static_cast<MapObject *>(NULL));
    // 将det结果保存到关键帧对应的普通帧的mmObservations中. 是否要这样做呢？
    mCurrentFrame.SetObservations(pKF);
}


void Tracking::GetObjectDetectionsMono(KeyFrame *pKF)
{
    PyThreadStateLock PyThreadLock;
    mvImObjectMasks.clear();
    mvImObjectBboxs.clear();

    // 调用python接口获取，物体检测结果
    py::list detections;
    if (mbUseRos) {
        string file_name = "ros.png";
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        cv::imwrite(mDatasetPathRoot+"/"+file_name, pKF->color_img);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        std::cout << "[zhjd-debug] 存储图片花费了 " << ttrack << std::endl;
        detections= mpSystem->pySequence.attr("get_frame_by_name")(pKF->mnFrameId, file_name);
    }
    // py::list detections = mpSystem->pySequence.attr("get_frame_by_id")(pKF->mnFrameId);


    int num_dets = detections.size();
    // No detections, return immediately
    if (num_dets == 0)
        return;

    for (int detected_idx = 0; detected_idx < num_dets; detected_idx++)
    {
        auto det = new ObjectDetection();
        auto py_det = detections[detected_idx];
        det->background_rays = py_det.attr("background_rays").cast<Eigen::MatrixXf>();
        auto mask = py_det.attr("mask").cast<Eigen::MatrixXf>();
        det->bbox = py_det.attr("bbox").cast<Eigen::Vector4d>();
        det->label = py_det.attr("label").cast<int>();
        det->prob = py_det.attr("prob").cast<double>();


        cv::Mat mask_cv;
        cv::eigen2cv(mask, mask_cv);
        // cv::imwrite("mask.png", mask_cv);
        cv::Mat mask_erro = mask_cv.clone();
        cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE,
                                               cv::Size(2 * maskErrosion + 1, 2 * maskErrosion + 1),
                                               cv::Point(maskErrosion, maskErrosion));
        cv::erode(mask_cv, mask_erro, kernel);
        
        // 物体检测框和掩码，用于可视化
        int x1 = (int)(det->bbox(0)), y1 = (int)(det->bbox(1)), \
            x2 = (int)(det->bbox(2)), y2 = (int)(det->bbox(3));
        mvImObjectMasks.push_back(std::move(mask_cv));
        mvImObjectBboxs.push_back({x1,y1,x2,y2});


        // get 2D feature points inside mask
        for (int i = 0; i < pKF->mvKeys.size(); i++)
        {
            int val = (int) mask_erro.at<float>(pKF->mvKeys[i].pt.y, pKF->mvKeys[i].pt.x);
            if (val > 0)  // inside the mask
            {
                det->AddFeaturePoint(i);
            }
        }

        // Reject the detection if too few keypoints are extracted
        if (det->NumberOfPoints() < 20)
        {
            det->isGood_OrbPointsEnough = false;
        }
        pKF->mvpDetectedObjects.push_back(det);
    }

    pKF->nObj = pKF->mvpDetectedObjects.size();
    pKF->mvpMapObjects = vector<MapObject *>(pKF->nObj, static_cast<MapObject *>(NULL));

}

void Tracking::AssociateObjectsByProjection(ORB_SLAM2::KeyFrame *pKF)
{
    // 获取地图中的物体和点
    auto mapObjects = mpMap->GetAllMapObjects();
    auto mvpMapPoints = pKF->GetMapPointMatches();
    // Try to match and triangulate key-points with last key-frame
    auto detectionsKF1 = pKF->mvpDetectedObjects;
    for (int d_i = 0; d_i < detectionsKF1.size(); d_i++)
    {
        // cout << "Detection: " << d_i + 1 << endl;
        auto detKF1 = detectionsKF1[d_i];
        // mpMap->AddGlobalObjectDetections(detKF1);
        
        if(mb_associate_object_with_ellipsold){    
            cout << "Tracking::AssociateObjectsByProjection" << endl;

            // 当前观测的bbox框
            auto bbox_det = detKF1->bbox;
            cv::Rect r2_bbox(cv::Point(bbox_det[0], bbox_det[1]), cv::Point(bbox_det[2], bbox_det[3]));
            auto label_bbox = detKF1->label;


            // 与global椭球体的投影IoU评分
            vector<double> iou_stats;
            bool has_associate = false;
            std::vector<std::pair<double, MapObject*>> objIoUVec;

            for (auto pMO: mapObjects){
                // FIXME：这里暂时对于 e 为 NULL 的情况跳过处理
                cv::Mat img_show = mCurrentFrame.color_img.clone();
                auto e = pMO->GetEllipsold();

                if (e==NULL){
                    continue;
                }

                auto label_obj = pMO->label;
                auto campose_cw = mCurrentFrame.cam_pose_Tcw;
                auto ellipse = e->projectOntoImageEllipse(campose_cw, mCalib);
                e->drawEllipseOnImage(ellipse, img_show);
                
                // draw bbox of object
                Vector4d rect = e->getBoundingBoxFromProjection(campose_cw, mCalib); 

                // 与bbox求IoU
                cv::Rect r1_proj(cv::Point(rect[0], rect[1]), cv::Point(rect[2], rect[3]));
                // 红色是椭球体投影
                cv::rectangle(img_show, r1_proj, cv::Scalar(0, 0, 255), 2);

                // draw bbox of det
                // cv::rectangle(img_show, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 0, 0), 2);  // Scalar(255, 0, 0) is for blue color, 2 is the thickness
                // 蓝色是当前检测框
                cv::rectangle(img_show, r2_bbox, cv::Scalar(255, 0, 0), 2);  // Scalar(255, 0, 0) is for blue color, 2 is the thickness

                // LZW版本
                // cv::Rect r_and = r1_proj | r2_bbox;
                // cv::Rect r_U = r1_proj & r2_bbox;
                // double iou = r_U.area()*1.0/r_and.area();

                // zhjd版本
                cv::Rect inter = r1_proj & r2_bbox;
                int interArea = inter.area();
                int area1 = r1_proj.area();
                int area2 = r2_bbox.area();
                int unionArea = area1 + area2 - interArea;
                double iou = unionArea > 0 ? (double)interArea / unionArea : 0.0;
                iou_stats.push_back(iou);
                // iou_stats.push_back(make_pair(dis, pPlane));
                if (iou > mf_associate_IoU_thresold && label_bbox==label_obj)
                    objIoUVec.push_back(make_pair(iou, pMO));

                //TODO: 没关联上可能是因为椭球体的参数没有及时更新
                if (mb_associate_debug)
                {
                    if (iou > mf_associate_IoU_thresold && label_bbox==label_obj){
                        std::cout << "!! Associated !!" << std::endl;
                    }
                    else {
                        std::cout << "Not Associated" << std::endl;
                    }
                    std::cout << "class(bbox/obj)/IoU: " << label_bbox << "/" \
                            << label_obj << "/" << iou << std::endl;

                    cv::imshow("[Debug] Global Ellipse Projection in Current Frame", img_show);
                    cv::waitKey(10);

                    std::cout << "Press any key to continue" << endl;
                    char key = getchar();
                }

            }
                
            // 找出最大IoU的物体
            std::sort(objIoUVec.begin(), objIoUVec.end(),
                [](const std::pair<double, MapObject*>& a, const std::pair<double, MapObject*>& b) {
                    return a.first > b.first; // 降序排序，IOU 最大的在前面
                });
            if (!objIoUVec.empty()) {
                MapObject* bestObject = objIoUVec.front().second;
                double maxIou = objIoUVec.front().first;
                // 使用 bestObject 和 maxIou
                if (maxIou > mf_associate_IoU_thresold){
                    cout << "[debug] Associate" << std::endl;
                    // 这里有一个问题，被关联过的物体可能在下一个det再次被遍历到
                    has_associate = true;
                    associateDetWithObject(pKF, bestObject, d_i, detKF1, mvpMapPoints);
                    break;
                }
            }

            std::cout << "Detection " << d_i << ", class " << label_bbox ;
            
            if (has_associate) std::cout << ", associated successfully: ";
            else std::cout << " associated failed: ";
            for (auto &iou : iou_stats) {
                cout << iou << ", ";
            }
            cout << endl;
        }
        else {
            map<int, int> observed_object_id;
            int nOutliers = 0;
            for (int k_i : detKF1->GetFeaturePoints()) {
                auto pMP = mvpMapPoints[k_i];
                if (!pMP)
                    continue;
                if (pMP->isOutlier())
                {
                    nOutliers++;
                    continue;
                }
                
                // 如果pMP->object_id小于0，说明该点还没有被分配到任何物体
                if (pMP->object_id < 0)
                    continue;

                // 根据det中的feature point，统计潜在关联的object_id（的数量）
                if (observed_object_id.count(pMP->object_id))
                    observed_object_id[pMP->object_id] += 1;
                else
                    observed_object_id[pMP->object_id] = 1;
            }

            // If associated with an object
            if (!observed_object_id.empty())
            {
                // Find object that has the most matches
                int object_id_max_matches = 0;  // global object id
                int max_matches = 0;
                for (auto it = observed_object_id.begin(); it != observed_object_id.end(); it++) {
                    if (it->second > max_matches) {
                        max_matches = it->second;
                        object_id_max_matches = it->first;
                    }
                }

                // associated object
                auto pMO = mpMap->GetMapObject(object_id_max_matches);
                pKF->AddMapObject(pMO, d_i);  //// Associated objects
                detKF1->isNew = false;

                // add newly detected feature points to object
                int newly_matched_points = 0;
                for (int k_i : detKF1->GetFeaturePoints()) {
                    auto pMP = mvpMapPoints[k_i];
                    if (pMP)
                    {
                        if (pMP->isBad())
                            continue;
                        // new map points
                        if (pMP->object_id < 0)
                        {
                            pMP->in_any_object = true;
                            pMP->object_id = object_id_max_matches;
                            pMO->AddMapPoints(pMP);
                            newly_matched_points++;
                        }
                        else
                        {
                            // if pMP is already associate to a different object, set bad flag
                            if (pMP->object_id != object_id_max_matches)
                                pMP->SetBadFlag();
                        }
                    }
                }
                // pMO->GetMapPointsWithinBoundingCubeToGround();
                /*cout <<  "Matches: " << max_matches << ", New points: " << newly_matched_points << ", Keypoints: " <<
                    detKF1->mvKeysIndices.size() << ", Associated to object by projection " << object_id_max_matches
                    << endl << endl;*/
            }

        }
    }
}




void Tracking::AssociateObjectsByDistance(ORB_SLAM2::KeyFrame *pKF)
{
    // 获取地图中的物体和点
    auto mapObjects = mpMap->GetAllMapObjects();
    auto mvpMapPoints = pKF->GetMapPointMatches();
    // Try to match and triangulate key-points with last key-frame
    auto detectionsKF1 = pKF->mvpDetectedObjects;
    auto mvpGlobalEllipsolds = pKF->GetEllipsoldsGlobal();

    for (int d_i = 0; d_i < detectionsKF1.size(); d_i++)
    {
        // cout << "Detection: " << d_i + 1 << endl;
        auto detKF1 = detectionsKF1[d_i];
        g2o::ellipsoid* local_e = mvpGlobalEllipsolds[d_i];
        if (local_e==NULL){
            continue;
        }

        cout << "[debug] Tracking::AssociateObjectsByDistance" << endl;
        bool has_associate = false;

        // 当前观测的label
        auto local_label = detKF1->label;

        // 与global椭球体的投影IoU评分
        std::vector<std::pair<double, MapObject*>> objDisVec;


        for (auto pMO: mapObjects){
            
            auto global_e = pMO->GetEllipsold();

            if (global_e==NULL){
                continue;
            }

            auto global_label = pMO->label;
            
            Eigen::Vector3d global_center = global_e->pose.translation();
            Eigen::Vector3d local_center = local_e->pose.translation();
            // 计算两者的距离
            double dis = (global_center - local_center).norm();

            if (dis < mf_associate_Dis_thresold && local_label==global_label)
                objDisVec.push_back(make_pair(dis, pMO));

        }
        
        std::cout << "[debug] Associate Detection " << d_i << ", class " << local_label ;
        if (!objDisVec.empty()) {
            std::sort(objDisVec.begin(), objDisVec.end(),
                [](const std::pair<double, MapObject*>& a, const std::pair<double, MapObject*>& b) {
                    return a.first < b.first; 
                });
            
            MapObject* bestObject = objDisVec.front().second;
            double minDis = objDisVec.front().first;

            associateDetWithObject(pKF, bestObject, d_i, detKF1, mvpMapPoints);
    
            std::cout << ", associated successfully, minDis: "<< minDis << endl;

        }
        else 
            std::cout << ", associated failed " << endl;
    }
}


int Tracking::associateDetWithObject(ORB_SLAM2::KeyFrame *pKF, MapObject* pMO, int d_i, ObjectDetection* detKF1, vector<MapPoint*>& mvpMapPoints)
{
    // 设置该帧的某个观测对应的物体
    pKF->AddMapObject(pMO, d_i);
    pMO->AddObjectObservation(pKF, d_i);
    // pMO->AddmessutionsId(d_i);   此函数内自动加上原有的size

    // 设置物体所包含的观测
    detKF1->isNew = false;

    int associate_object_id = pMO->mnId;
    // pMO

    // 将新观测的特征点，添加到物体中
    int newly_matched_points = 0;
    for (int k_i : detKF1->GetFeaturePoints()) {
        auto pMP = mvpMapPoints[k_i];
        if (pMP && !pMP->isBad())
        {
            // new map points
            if (pMP->object_id < 0)
            {
                pMP->in_any_object = true;
                pMP->object_id = associate_object_id;
                pMO->AddMapPoints(pMP);
                newly_matched_points++;
            }
            else
            {
                // if pMP is already associate to a different object, set bad flag
                // 一个特征点在不同帧可以在不同物体的mask内
                if (pMP->object_id != associate_object_id)
                    pMP->SetBadFlag();
            }
        }
    }

    if(mb_use_depth_pcd_to_reconstruct){
        // 融合PCD点云
        pMO->AddDepthPointCloudFromObjectDetection(detKF1->getPcdPtr());
        // 更新数据关联后的物体的位姿形状
        bool recompute_merged_ellipsoid = Config::Get<int>("Debug.Tracking.recompute_merged_ellipsoid");
        if(recompute_merged_ellipsoid)
            UpdateAssociatedObjectPoseAndScale(pMO);

        bool use_ellipsoid_verticles = Config::Get<int>("Mapping.use_ellipsoid_verticles");
        double ellipsoid_verticles_scale = Config::Get<double>("Mapping.ellipsoid_verticles_scale");
        
        if(use_ellipsoid_verticles){
            pMO->EraseEllipsoidVertices();
            auto SE3Two = pMO->GetEllipsold()->pose;
            double verticles_num = std::ceil(pMO->GetPointCloud()->size()*ellipsoid_verticles_scale);
            double verticles_degree = Config::Get<double>("Mapping.ellipsoid_verticles_degree");

            std::vector<Vector3d> verticles = getVerticesOfEllipsoid(pMO->GetEllipsold(), verticles_num, verticles_degree);
            int num = 1;
            std::vector<Vector3f> verticles_in_camera;
            for(auto p_o: verticles){
                Vector3d p_w = SE3Two * p_o;
                // Eigen::Vector3f p_w_3 = p_w.head<3>();
                pMO->AddEllipsoidVertices(p_w.cast<float>());
                num++;
            }
        }
    }
    
    return newly_matched_points;
}




void Tracking::UpdateAssociatedObjectPoseAndScale(MapObject* pMO){
    
    mpEllipsoidExtractor->ClearPointCloudList(); 

    pcl::PointCloud<PointType>::Ptr pcd_ptr = pMO->GetDepthPointCloudPCL();

    g2o::ellipsoid e_merged = mpEllipsoidExtractor->EstimateEllipsoidFromPCDCloud(pcd_ptr, &mGroundPlane, CheckManualDirection(pMO));

    double scale = Config::Get<double>("Mapping.ObjectScale");
    // std::cout << "[debug] UpdateAssociatedObjectPoseAndScale, 融合后的椭球体 scale:"<< e_merged.scale.transpose() << endl;
    // std::cout << "[debug] UpdateAssociatedObjectPoseAndScale, 融合后的椭球体 Pose:"<< e_merged.pose.translation().transpose() << endl;
    e_merged.prob_3d = pMO->GetEllipsold()->prob_3d;
    e_merged.prob = pMO->GetEllipsold()->prob;    // measurement_prob * symmetry_prob
    e_merged.miLabel = pMO->GetEllipsold()->miLabel;
    e_merged.bbox = pMO->GetEllipsold()->bbox;
    e_merged.bPointModel = false;

    // 通过曼哈顿平面优化
    const map<KeyFrame*,size_t> observations = pMO->GetObservations();
    bool bSupportingPlaneDefined = false;
    ConstrainPlane* pSupportingPlane;
    bool bBackingPlaneDefined = false;
    ConstrainPlane* pBackingPlane;
    int num = 0;
    for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++){
        // 关键帧中所有的椭球体
        auto mvpGlobalEllipsolds = it->first->GetEllipsoldsGlobal();


        // std::cout<<"[debug] UpdateAssociatedObjectPoseAndScale, 关键帧["<< it->first->mnId << "]的观测：" << it->second <<std::endl;
        // 对应的椭球体
        g2o::ellipsoid* e = mvpGlobalEllipsolds[it->second];
        // std:cout<< "            支撑面："<<e->mbSupportingPlaneDefined <<",倚靠面："<<e->mbBackingPlaneDefined<<std::endl;

        // 提前曼哈顿平面，当前考虑到现在提取的MHP都挺好，直接随便选一个
        if(e->mbSupportingPlaneDefined){
            pSupportingPlane = e->mpSupportingPlane;
            bSupportingPlaneDefined = true;
        }

        if(e->mbBackingPlaneDefined){
            pBackingPlane = e->mpBackingPlane;
            bBackingPlaneDefined = true;
        }

        num++;
    }

    // std::cout<<"[debug] UpdateAssociatedObjectPoseAndScale1, 关键帧数量: "<< num <<std::endl;
    // std::cout<<"[debug] UpdateAssociatedObjectPoseAndScale2, flag: "<< bSupportingPlaneDefined << "/"<< bBackingPlaneDefined <<std::endl;
    
    int opt_num = Config::ReadValue<int>("EllipsoidExtractor.Optimizer.Number");
    
    if(bSupportingPlaneDefined && bBackingPlaneDefined){
        
        double supproting_weight = Config::ReadValue<double>("EllipsoidExtractor.Optimizer.SupportingWeight");
        g2o::plane* pSupPlane = pSupportingPlane->pPlane;
        // std::cout<<"[debug] UpdateAssociatedObjectPoseAndScale3-1, pSupPlane: "<< pSupPlane->param.transpose() << ", supproting_weight:" << supproting_weight <<std::endl;
        double backing_weight = Config::ReadValue<double>("EllipsoidExtractor.Optimizer.BackingWeight");
        g2o::plane* pBackPlane = pBackingPlane->pPlane;
        // std::cout<<"[debug] UpdateAssociatedObjectPoseAndScale3-2, pBackPlane: "<< pBackPlane->param.transpose() << ", backing_weight:" << backing_weight <<std::endl;

        



        g2o::ellipsoid e_refined = mpEllipsoidExtractor->OptimizeEllipsoidWithMHPlanes(
                e_merged, opt_num, *pSupPlane, supproting_weight, *pBackPlane, backing_weight);
        
        // 可视化 Refined Object，并变换到世界坐标系下
        // Visualize estimated ellipsoid
        g2o::ellipsoid* pObjRefined = new g2o::ellipsoid(e_refined);
        // pObjRefined->setColor(Vector3d(189/255.0, 183/255.0, 107/255.0), 1); 
        // pObjRefined->setColor(Vector3d(255/255.0, 255/255.0, 0/255.0), 1); 
        pObjRefined->setColor(Vector3d(255/255.0, 0/255.0, 0/255.0), 1); //用于论文做图
        mpMap->addRefinedEllipsoidVisual(pObjRefined);
        
        
        
        // 可视化
        Vector3d center = e_merged.pose.translation();
        Vector4d planeVec_1 = pSupPlane->param.head(4);
        Vector3d color; double plane_size = 0.5;
        color = Vector3d(0.7,0,0);  // 边界颜色
        g2o::plane *pPlane_1 = new g2o::plane(planeVec_1, color);
        pPlane_1->InitFinitePlane(center, plane_size);
        pPlane_1->miMHType = g2o::MANHATTAN_PLANE_TYPE::MERGE_REFINE;
        mpMap->addPlane(pPlane_1);
        Vector4d planeVec_2 = pBackPlane->param.head(4);
        color = Vector3d(0,0,0.7);  // 边界颜色
        g2o::plane *pPlane_2 = new g2o::plane(planeVec_2, color);
        pPlane_2->InitFinitePlane(center, plane_size);
        pPlane_2->miMHType = g2o::MANHATTAN_PLANE_TYPE::MERGE_REFINE;
        mpMap->addPlane(pPlane_2);
        

        // std::cout<<"[debug] UpdateAssociatedObjectPoseAndScale3-3, e_merged: "<< e_merged.scale.transpose() <<std::endl;
        // std::cout<<"[debug] UpdateAssociatedObjectPoseAndScale3-4, pObjRefined: "<< pObjRefined->scale.transpose() <<std::endl;

        pMO->SetPoseByEllipsoid(pObjRefined, scale);
        // std::cout<<"[debug] UpdateAssociatedObjectPoseAndScale3-5, ENd"<<std::endl;
        
    }
    // 只使用支撑面进行优化
    else if(bSupportingPlaneDefined){
        double supproting_weight = Config::ReadValue<double>("EllipsoidExtractor.Optimizer.SupportingWeight");
        g2o::plane* pSupPlane = &mGroundPlane;
        // std::cout<<"[debug] UpdateAssociatedObjectPoseAndScale4, pSupPlane: "<< pSupPlane->param.transpose() <<std::endl;

        g2o::ellipsoid e_refined = mpEllipsoidExtractor->OptimizeEllipsoidWithSupportingPlanes( e_merged, opt_num, *pSupPlane, supproting_weight);
        
        // 可视化 Refined Object，并变换到世界坐标系下
        // Visualize estimated ellipsoid
        g2o::ellipsoid* pObjRefined = new g2o::ellipsoid(e_refined);
        // pObjRefined->setColor(Vector3d(189/255.0, 183/255.0, 107/255.0), 1); 
        // pObjRefined->setColor(Vector3d(255/255.0, 255/255.0, 0/255.0), 1); 
        pObjRefined->setColor(Vector3d(255/255.0, 0/255.0, 0/255.0), 1); //用于论文做图
        mpMap->addRefinedEllipsoidVisual(pObjRefined);
        
        // 用优化后的
        // (*pFrame->mpLocalObjects[i]) = e_refined;

        // g2o::ellipsoid e_global = e_refined.transform_from(pFrame->cam_pose_Twc);
        // pKF->ReplaceEllipsoldsGlobal(i, &e_global);

        pMO->SetPoseByEllipsoid(pObjRefined, scale);
    }


    
}



std::vector<Vector3d> Tracking::getVerticesOfEllipsoid(ellipsoid* pEllipsoid, int num, double verticles_degree) {

    std::vector<Vector3d> vertices;
    Eigen::Matrix4d S = Eigen::Matrix4d::Identity();
    S(0,0) = pEllipsoid->scale(0);
    S(1,1) = pEllipsoid->scale(1);
    S(2,2) = pEllipsoid->scale(2);

    double max_angle_deg = verticles_degree;
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
    // // 前面
    // for (int i = 0; i < num; ++i) {
    //     double u = drand48();  // in [0,1)
    //     double v = drand48();

    //     double theta = max_angle_rad * (u-0.5);    //// 方位角（绕 z 轴），与x轴的夹角
    //     double phi = max_angle_rad * (v-0.5) + M_PI_2;   // 极角（与 z 轴夹角），限制在顶点附近的锥体区域

    //     double x = sin(phi) * cos(theta);
    //     double y = sin(phi) * sin(theta);
    //     double z = cos(phi+M_PI);

    //     Eigen::Vector4d p_unit(x, y, z, 1.0);
    //     Eigen::Vector4d p_e = S * p_unit;

    //     // 椭球表面点
    //     Eigen::Vector3d sampled_point = p_e.head<3>();
    //     vertices.push_back(sampled_point);
    // }
    // // 后面
    // for (int i = 0; i < num; ++i) {
    //     double u = drand48();  // in [0,1)
    //     double v = drand48();

    //     double theta = max_angle_rad * (u-0.5) + M_PI;    //// 方位角（绕 z 轴），与x轴的夹角
    //     double phi = max_angle_rad * (v-0.5) + M_PI_2;   // 极角（与 z 轴夹角），限制在顶点附近的锥体区域

    //     double x = sin(phi) * cos(theta);
    //     double y = sin(phi) * sin(theta);
    //     double z = cos(phi+M_PI);

    //     Eigen::Vector4d p_unit(x, y, z, 1.0);
    //     Eigen::Vector4d p_e = S * p_unit;

    //     // 椭球表面点
    //     Eigen::Vector3d sampled_point = p_e.head<3>();
    //     vertices.push_back(sampled_point);
    // }
    // 右面
    for (int i = 0; i < num; ++i) {
        double u = drand48();  // in [0,1)
        double v = drand48();

        double theta = max_angle_rad * (u-0.5) - M_PI_2;    //// 方位角（绕 z 轴），与x轴的夹角
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


int Tracking::CheckManualLabel(ellipsoid* pEllipsoid){
    // 获取椭球体的位姿矩阵（4x4）
    Eigen::Matrix4f Two = Converter::toMatrix4f(pEllipsoid->pose);

    // 提取椭球体的中心位置（平移部分）
    double e_x = Two(0, 3);
    double e_y = Two(1, 3);
    double e_z = Two(2, 3);

    int label = pEllipsoid->miLabel;
    for (auto ml : mvManualObjectDetect) {
        // 假设 ml 是一个 vector<int> 或 vector<double>，格式为 {x, y, z, label}
        double ml_x = ml[0];
        double ml_y = ml[1];
        double ml_z = ml[2];
        int ml_label = ml[3];

        // 计算欧氏距离
        double dis = std::sqrt(
            std::pow(e_x - ml_x, 2) +
            std::pow(e_y - ml_y, 2) +
            std::pow(e_z - ml_z, 2)
        );

        if(dis<mfManualObjectDetectDisThresold){
            label = ml_label;
        }
    }
    return label;
}



int Tracking::CheckManualDirection(MapObject* pMO){
    // 获取椭球体的位姿矩阵（4x4）
    Eigen::Matrix4f Two = pMO->SE3Two;

    // 提取椭球体的中心位置（平移部分）
    double e_x = Two(0, 3);
    double e_y = Two(1, 3);
    double e_z = Two(2, 3);

    int flag = 0;
    int index = -1;
    for (int i=0; i<mvManualObjectDetect.size(); i++) {
        auto ml = mvManualObjectDetect[i];
        // 假设 ml 是一个 vector<int> 或 vector<double>，格式为 {x, y, z, label}
        double ml_x = ml[0];
        double ml_y = ml[1];
        double ml_z = ml[2];

        // 计算欧氏距离
        double dis = std::sqrt(
            std::pow(e_x - ml_x, 2) +
            std::pow(e_y - ml_y, 2) +
            std::pow(e_z - ml_z, 2)
        );

        if(dis<mfManualObjectDetectDisThresold){
            flag = ml[4];
            index = i;
        }
    }
    std::cout<<"[debug] CheckManualDirection, index: "<< index << ", flag: "<< flag <<std::endl;
    return flag;
}



}