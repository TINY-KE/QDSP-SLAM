/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapPublisher.h"


namespace ORB_SLAM2
{


MapPublisher::MapPublisher(Map* pMap, const string &strSettingPath):mpMap(pMap), mbCameraUpdated(false)
{

    //Configure MapPoints
    fPointSize=0.01;
    mPoints.header.frame_id = MAP_FRAME_ID;
    mPoints.ns = POINTS_NAMESPACE;
    mPoints.id=0;
    mPoints.type = visualization_msgs::Marker::POINTS;
    mPoints.scale.x=fPointSize;
    mPoints.scale.y=fPointSize;
    mPoints.pose.orientation.w=1.0;
    mPoints.action=visualization_msgs::Marker::ADD;
    mPoints.color.a = 1.0;
    
    mObjectPoints.header.frame_id = MAP_FRAME_ID;
    mObjectPoints.ns = OBJECTPOINTS_NAMESPACE;
    mObjectPoints.id=0;
    mObjectPoints.type = visualization_msgs::Marker::POINTS;
    mObjectPoints.scale.x=fPointSize*2.0;
    mObjectPoints.scale.y=fPointSize*2.0;
    mObjectPoints.pose.orientation.w=1.0;
    mObjectPoints.action=visualization_msgs::Marker::ADD;
    mObjectPoints.color.a = 1.0;


    //Configure KeyFrames
    fCameraSize=0.04;
    mKeyFrames.header.frame_id = MAP_FRAME_ID;
    mKeyFrames.ns = KEYFRAMES_NAMESPACE;
    mKeyFrames.id=1;
    mKeyFrames.type = visualization_msgs::Marker::LINE_LIST;
    mKeyFrames.scale.x=0.005;
    mKeyFrames.pose.orientation.w=1.0;
    mKeyFrames.action=visualization_msgs::Marker::ADD;

    mKeyFrames.color.b=1.0f;
    mKeyFrames.color.a = 1.0;

    //Configure Covisibility Graph 共视图
    mCovisibilityGraph.header.frame_id = MAP_FRAME_ID;
    mCovisibilityGraph.ns = GRAPH_NAMESPACE;
    mCovisibilityGraph.id=2;
    mCovisibilityGraph.type = visualization_msgs::Marker::LINE_LIST;
    mCovisibilityGraph.scale.x=0.002;
    mCovisibilityGraph.pose.orientation.w=1.0;
    mCovisibilityGraph.action=visualization_msgs::Marker::ADD;
    mCovisibilityGraph.color.b=0.7f;
    mCovisibilityGraph.color.g=0.7f;
    mCovisibilityGraph.color.a = 0.3;

    //Configure KeyFrames Spanning Tree  关键帧中心的连线
    mMST.header.frame_id = MAP_FRAME_ID;
    mMST.ns = GRAPH_NAMESPACE;
    mMST.id=3;
    mMST.type = visualization_msgs::Marker::LINE_LIST;
    mMST.scale.x=0.005;
    mMST.pose.orientation.w=1.0;
    mMST.action=visualization_msgs::Marker::ADD;
    mMST.color.b=0.0f;
    mMST.color.g=1.0f;
    mMST.color.a = 1.0;

    //Configure Current Camera
    mCurrentCamera.header.frame_id = MAP_FRAME_ID;
    mCurrentCamera.ns = CAMERA_NAMESPACE;
    mCurrentCamera.id=4;
    mCurrentCamera.type = visualization_msgs::Marker::LINE_LIST;
    mCurrentCamera.scale.x=0.01;//0.2; 0.03
    mCurrentCamera.pose.orientation.w=1.0;
    mCurrentCamera.action=visualization_msgs::Marker::ADD;
    mCurrentCamera.color.g=1.0f;
    mCurrentCamera.color.a = 1.0;

    //Configure Reference MapPoints
    mReferencePoints.header.frame_id = MAP_FRAME_ID;
    mReferencePoints.ns = POINTS_NAMESPACE;
    mReferencePoints.id=6;
    mReferencePoints.type = visualization_msgs::Marker::POINTS;
    mReferencePoints.scale.x=fPointSize;
    mReferencePoints.scale.y=fPointSize;
    mReferencePoints.pose.orientation.w=1.0;
    mReferencePoints.action=visualization_msgs::Marker::ADD;
    mReferencePoints.color.r =1.0f;
    mReferencePoints.color.a = 1.0;
    

    //Configure Publisher
    publisher = nh.advertise<visualization_msgs::Marker>("Point", 1000);
    publisher_curframe = nh.advertise<visualization_msgs::Marker>("CurFrame", 1000);
    publisher_KF = nh.advertise<visualization_msgs::Marker>("KeyFrame", 1000);
    publisher_CoView = nh.advertise<visualization_msgs::Marker>("CoView", 1000);
    // publisher_object = nh.advertise<visualization_msgs::Marker>("objectmap", 1000);
    // publisher_object_points = nh.advertise<visualization_msgs::Marker>("objectPoints", 1000);
    // publisher_IE_maindirection = nh.advertise<visualization_msgs::Marker>("object_ie_maindirection", 1000);
    // publisher_IE_cylinder = nh.advertise<visualization_msgs::Marker>("object_ie_cylinder", 1000);
    // publisher_IE_ellipse = nh.advertise<visualization_msgs::Marker>("object_ie_ellipse", 1000);
    // publisher_IE_half = nh.advertise<visualization_msgs::Marker>("object_ie_half", 1000);
    // publisher_MainDirection = nh.advertise<visualization_msgs::Marker>("object_MainDirection", 1000);
    // publisher_SumMainDirection = nh.advertise<visualization_msgs::Marker>("object_SumMainDirection", 1000);
    //publisher_robotpose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);
    //publisher_mam_rviz = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/local_nbv", 1000);
    // publisher_IEtext = nh.advertise<visualization_msgs::Marker>("IEtext", 1);
    publisher_ObjectInfo = nh.advertise<std_msgs::Float32MultiArray>("/objects_info", 10);
    publisher_SdfObject = nh.advertise<visualization_msgs::Marker>("/objects", 10);
    publisher_CubeObject = nh.advertise<visualization_msgs::Marker>("/cubeobjects", 10);
    publisher_ObjectPoints = nh.advertise<visualization_msgs::Marker>("/objectpoint", 1000);
    publisher_ellipsoid = nh.advertise<visualization_msgs::Marker>("/ellipsoid_slam", 1000);
    publisher_depth_current_frame = nh.advertise<sensor_msgs::PointCloud2>("/depth/points", 1);

    publisher.publish(mPoints);
    publisher.publish(mReferencePoints);
    publisher_CoView.publish(mCovisibilityGraph);
    publisher_KF.publish(mKeyFrames);
    publisher_curframe.publish(mCurrentCamera);

    mvObjectColors.push_back(std::tuple<float, float, float>({210. / 255., 245. / 255., 60. / 255.}));  //lime  0
    mvObjectColors.push_back(std::tuple<float, float, float>({60. / 255., 180. / 255., 75. / 255.}));   // green  1
    mvObjectColors.push_back(std::tuple<float, float, float>({0., 0., 255. / 255.}));	 // blue  2
    mvObjectColors.push_back(std::tuple<float, float, float>({255. / 255., 0, 255. / 255.}));   // Magenta  3
    mvObjectColors.push_back(std::tuple<float, float, float>({255. / 255., 165. / 255., 0}));   // orange 4
    mvObjectColors.push_back(std::tuple<float, float, float>({128. / 255., 0, 128. / 255.}));   //purple 5
    mvObjectColors.push_back(std::tuple<float, float, float>({0., 255. / 255., 255. / 255.}));   //cyan 6
    mvObjectColors.push_back(std::tuple<float, float, float>({230. / 255., 0., 0.}));	 // red  7
    mvObjectColors.push_back(std::tuple<float, float, float>({250. / 255., 190. / 255., 190. / 255.})); //pink  8
    mvObjectColors.push_back(std::tuple<float, float, float>({0., 128. / 255., 128. / 255.}));   //Teal  9
}

void MapPublisher::Refresh()
{
    {
        PublishCurrentCamera(mCameraPose);
    }
    {
        vector<KeyFrame*> vKeyFrames = mpMap->GetAllKeyFrames();
        vector<MapPoint*> vMapPoints = mpMap->GetAllMapPoints();
        vector<MapPoint*> vRefMapPoints = mpMap->GetReferenceMapPoints();
        //vector<MapPlane*> vMapPlanes = mpMap ->GetAllMapPlanes();
        // vector<MapObject*> vMapObjects = mpMap ->GetAllMapObjects();
        // vector<ellipsoid*> vMapEllipsoids = mpMap ->GetAllEllipsoidsVisual();
        PublishMapPoints(vMapPoints, vRefMapPoints);
        PublishKeyFrames(vKeyFrames);
        //PublishPlane(vMapPlanes);
        // PublishEllipsoidInfo(vMapEllipsoids);

        auto mvpMapObjects = mpMap->GetAllMapObjects();
        PublishMapObjects(mvpMapObjects);
    }
}

void MapPublisher::PublishMapPoints(const vector<MapPoint*> &vpMPs, const vector<MapPoint*> &vpRefMPs)
{
    mPoints.points.clear();
    mReferencePoints.points.clear();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        geometry_msgs::Point p;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        p.x=pos.at<float>(0);
        p.y=pos.at<float>(1);
        p.z=pos.at<float>(2);

        mPoints.points.push_back(p);
    }

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        geometry_msgs::Point p;
        cv::Mat pos = (*sit)->GetWorldPos();
        p.x=pos.at<float>(0);
        p.y=pos.at<float>(1);
        p.z=pos.at<float>(2);

        mReferencePoints.points.push_back(p);
    }

    mPoints.header.stamp = ros::Time::now();
    mReferencePoints.header.stamp = ros::Time::now();
    publisher.publish(mPoints);
    publisher.publish(mReferencePoints);
}

void MapPublisher::PublishKeyFrames(const vector<KeyFrame*> &vpKFs)
{
    mKeyFrames.points.clear();
    mCovisibilityGraph.points.clear();
    mMST.points.clear();

    float d = fCameraSize;

    //Camera is a pyramid. Define in camera coordinate system
    cv::Mat o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
    cv::Mat p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
    cv::Mat p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
    cv::Mat p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
    cv::Mat p4 = (cv::Mat_<float>(4,1) << -d, d*0.8, d*0.5, 1);

    for(size_t i=0, iend=vpKFs.size() ;i<iend; i++)
    {
        cv::Mat Tcw = vpKFs[i]->GetPose();
//        std::cout<<"[rviz debug] frame id： "<< vpKFs[i]->mnId <<", Tcw:"<<Tcw<< std::endl;
        cv::Mat Twc = Tcw.inv();
        cv::Mat ow = vpKFs[i]->GetCameraCenter();
        cv::Mat p1w = Twc*p1;
        cv::Mat p2w = Twc*p2;
        cv::Mat p3w = Twc*p3;
        cv::Mat p4w = Twc*p4;

        geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
        msgs_o.x=ow.at<float>(0);
        msgs_o.y=ow.at<float>(1);
        msgs_o.z=ow.at<float>(2);
        msgs_p1.x=p1w.at<float>(0);
        msgs_p1.y=p1w.at<float>(1);
        msgs_p1.z=p1w.at<float>(2);
        msgs_p2.x=p2w.at<float>(0);
        msgs_p2.y=p2w.at<float>(1);
        msgs_p2.z=p2w.at<float>(2);
        msgs_p3.x=p3w.at<float>(0);
        msgs_p3.y=p3w.at<float>(1);
        msgs_p3.z=p3w.at<float>(2);
        msgs_p4.x=p4w.at<float>(0);
        msgs_p4.y=p4w.at<float>(1);
        msgs_p4.z=p4w.at<float>(2);

        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p1);
        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p2);
        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p3);
        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p4);
        mKeyFrames.points.push_back(msgs_p1);
        mKeyFrames.points.push_back(msgs_p2);
        mKeyFrames.points.push_back(msgs_p2);
        mKeyFrames.points.push_back(msgs_p3);
        mKeyFrames.points.push_back(msgs_p3);
        mKeyFrames.points.push_back(msgs_p4);
        mKeyFrames.points.push_back(msgs_p4);
        mKeyFrames.points.push_back(msgs_p1);

        // Covisibility Graph
        vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
        if(!vCovKFs.empty())
        {
            for(vector<KeyFrame*>::iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
            {
                if((*vit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Ow2 = (*vit)->GetCameraCenter();
                geometry_msgs::Point msgs_o2;
                msgs_o2.x=Ow2.at<float>(0);
                msgs_o2.y=Ow2.at<float>(1);
                msgs_o2.z=Ow2.at<float>(2);
                mCovisibilityGraph.points.push_back(msgs_o);
                mCovisibilityGraph.points.push_back(msgs_o2);
            }
        }

        // MST
        KeyFrame* pParent = vpKFs[i]->GetParent();
        if(pParent)
        {
            cv::Mat Owp = pParent->GetCameraCenter();
            geometry_msgs::Point msgs_op;
            msgs_op.x=Owp.at<float>(0);
            msgs_op.y=Owp.at<float>(1);
            msgs_op.z=Owp.at<float>(2);
            mMST.points.push_back(msgs_o);
            mMST.points.push_back(msgs_op);
        }
        set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
        for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
        {
            if((*sit)->mnId<vpKFs[i]->mnId)
                continue;
            cv::Mat Owl = (*sit)->GetCameraCenter();
            geometry_msgs::Point msgs_ol;
            msgs_ol.x=Owl.at<float>(0);
            msgs_ol.y=Owl.at<float>(1);
            msgs_ol.z=Owl.at<float>(2);
            mMST.points.push_back(msgs_o);
            mMST.points.push_back(msgs_ol);
        }
    }

    mKeyFrames.header.stamp = ros::Time::now();
    mCovisibilityGraph.header.stamp = ros::Time::now();
    mMST.header.stamp = ros::Time::now();

    publisher_KF.publish(mKeyFrames);
    publisher_CoView.publish(mCovisibilityGraph);
    publisher_KF.publish(mMST);
}

void MapPublisher::PublishCurrentCamera(const cv::Mat &Tcw) {
    if (Tcw.empty())
        return;

    mCurrentCamera.points.clear();

    float d = fCameraSize;
    //(1) 相机的几何模型
    //Camera is a pyramid. Define in camera coordinate system
    cv::Mat o = (cv::Mat_<float>(4, 1) << 0, 0, 0, 1);
    cv::Mat p1 = (cv::Mat_<float>(4, 1) << d, d * 0.8, d * 0.5, 1);
    cv::Mat p2 = (cv::Mat_<float>(4, 1) << d, -d * 0.8, d * 0.5, 1);
    cv::Mat p3 = (cv::Mat_<float>(4, 1) << -d, -d * 0.8, d * 0.5, 1);
    cv::Mat p4 = (cv::Mat_<float>(4, 1) << -d, d * 0.8, d * 0.5, 1);

    cv::Mat Twc = Tcw.inv();
    cv::Mat ow = Twc * o;
    cv::Mat p1w = Twc * p1;
    cv::Mat p2w = Twc * p2;
    cv::Mat p3w = Twc * p3;
    cv::Mat p4w = Twc * p4;

    geometry_msgs::Point msgs_o, msgs_p1, msgs_p2, msgs_p3, msgs_p4;
    msgs_o.x = ow.at<float>(0);
    msgs_o.y = ow.at<float>(1);
    msgs_o.z = ow.at<float>(2);
    msgs_p1.x = p1w.at<float>(0);
    msgs_p1.y = p1w.at<float>(1);
    msgs_p1.z = p1w.at<float>(2);
    msgs_p2.x = p2w.at<float>(0);
    msgs_p2.y = p2w.at<float>(1);
    msgs_p2.z = p2w.at<float>(2);
    msgs_p3.x = p3w.at<float>(0);
    msgs_p3.y = p3w.at<float>(1);
    msgs_p3.z = p3w.at<float>(2);
    msgs_p4.x = p4w.at<float>(0);
    msgs_p4.y = p4w.at<float>(1);
    msgs_p4.z = p4w.at<float>(2);

    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p1);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p1);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p1);

    mCurrentCamera.header.stamp = ros::Time::now();

    publisher_curframe.publish(mCurrentCamera);

}


void MapPublisher::PublishMapObjects(const vector<MapObject *> &vObjs) {
    // std::cout<<"[PublishMapObjects]"<<std::endl;
    int num = 0;
    for (auto pMO: vObjs) {

        if (!pMO)
            continue;
        if (pMO->isBad())
            continue;

        // std::cout<<"[PublishMapObjects] 有效物体数量： "<<num++<<std::endl;

        // 使用 Eigen 矩阵来表示 verts 和 faces
        auto verts = pMO->vertices;
        auto faces = pMO->faces;

        // 创建一个 Marker 消息
        visualization_msgs::Marker mesh_marker;
        mesh_marker.header.frame_id = MAP_FRAME_ID; // 设置网格的参考坐标系
        mesh_marker.header.stamp = ros::Time::now();
        mesh_marker.ns = "sdf_mesh";
        mesh_marker.id = pMO->mnId;
        mesh_marker.type = visualization_msgs::Marker::TRIANGLE_LIST; // 三角形列表类型
        mesh_marker.action = visualization_msgs::Marker::ADD;

        // 设置标记的颜色和透明度
        mesh_marker.color.a = 1.0f; // 设置透明度为 1.0（不透明）
        mesh_marker.color.r =  get<0>(mvObjectColors[pMO->mnId % 10]);
        mesh_marker.color.g =  get<1>(mvObjectColors[pMO->mnId % 10]);
        mesh_marker.color.b =  get<2>(mvObjectColors[pMO->mnId % 10]);

        // 设置标记的缩放
        mesh_marker.scale.x = 1.0;
        mesh_marker.scale.y = 1.0;
        mesh_marker.scale.z = 1.0;

        Eigen::Matrix4f Sim3Two = pMO->GetPoseSim3();
        auto Rwo = Sim3Two.topLeftCorner<3, 3>();
        Rwo *= 1.2;
        Sim3Two.topLeftCorner<3, 3>() = Rwo;

        // 遍历每个 face，并从 verts 中提取顶点
        for (int i = 0; i < faces.rows(); ++i) {
            for (int j = 0; j < 3; ++j) {
                int vertex_idx = faces(i, j); // 获取顶点索引

                const Eigen::Vector3f &vertex = verts.row(vertex_idx); // 获取顶点的坐标

                Eigen::Vector4f local_vertex(vertex(0), vertex(1), vertex(2), 1.0);  // 齐次坐标

                // 将顶点转换到 world 坐标系
                Eigen::Vector4f world_vertex = Sim3Two * local_vertex;

                geometry_msgs::Point point;
                point.x = world_vertex[0];
                point.y = world_vertex[1];
                point.z = world_vertex[2];

                mesh_marker.points.push_back(point);
            }
        }

        // 发布网格物体
        publisher_SdfObject.publish(mesh_marker);

        // 发布Cube物体
        visualization_msgs::Marker CubeMarker;
        CubeMarker.id = pMO->mnId;
        // CubeMarker.lifetime = ros::Duration(mObject_Duration);
        CubeMarker.header.frame_id= MAP_FRAME_ID;
        CubeMarker.header.stamp=ros::Time::now();
        CubeMarker.type = visualization_msgs::Marker::LINE_LIST; //LINE_STRIP;
        CubeMarker.action = visualization_msgs::Marker::ADD;
        CubeMarker.color.a = 1.0f; // 设置透明度为 1.0（不透明）
        CubeMarker.color.r =  get<0>(mvObjectColors[(pMO->mnId+5) % 10]);
        CubeMarker.color.g =  get<1>(mvObjectColors[(pMO->mnId+5) % 10]);
        CubeMarker.color.b =  get<2>(mvObjectColors[(pMO->mnId+5) % 10]);
        CubeMarker.scale.x = 0.01;
        //     8------7
        //    /|     /|
        //   / |    / |
        //  5------6  |
        //  |  4---|--3
        //  | /    | /
        //  1------2
        // lenth ：corner_2[0] - corner_1[0]
        // width ：corner_2[1] - corner_3[1]
        // height：corner_2[2] - corner_6[2]

        CubeMarker.points.push_back(corner_to_marker(pMO->corner_1));
        CubeMarker.points.push_back(corner_to_marker(pMO->corner_2));
        CubeMarker.points.push_back(corner_to_marker(pMO->corner_2));
        CubeMarker.points.push_back(corner_to_marker(pMO->corner_3));
        CubeMarker.points.push_back(corner_to_marker(pMO->corner_3));
        CubeMarker.points.push_back(corner_to_marker(pMO->corner_4));
        CubeMarker.points.push_back(corner_to_marker(pMO->corner_4));
        CubeMarker.points.push_back(corner_to_marker(pMO->corner_1));

        CubeMarker.points.push_back(corner_to_marker(pMO->corner_5));
        CubeMarker.points.push_back(corner_to_marker(pMO->corner_1));
        CubeMarker.points.push_back(corner_to_marker(pMO->corner_6));
        CubeMarker.points.push_back(corner_to_marker(pMO->corner_2));
        CubeMarker.points.push_back(corner_to_marker(pMO->corner_7));
        CubeMarker.points.push_back(corner_to_marker(pMO->corner_3));
        CubeMarker.points.push_back(corner_to_marker(pMO->corner_8));
        CubeMarker.points.push_back(corner_to_marker(pMO->corner_4));

        CubeMarker.points.push_back(corner_to_marker(pMO->corner_5));
        CubeMarker.points.push_back(corner_to_marker(pMO->corner_6));
        CubeMarker.points.push_back(corner_to_marker(pMO->corner_6));
        CubeMarker.points.push_back(corner_to_marker(pMO->corner_7));
        CubeMarker.points.push_back(corner_to_marker(pMO->corner_7));
        CubeMarker.points.push_back(corner_to_marker(pMO->corner_8));
        CubeMarker.points.push_back(corner_to_marker(pMO->corner_8));
        CubeMarker.points.push_back(corner_to_marker(pMO->corner_5));

        publisher_CubeObject.publish(CubeMarker);

        // 发布物体内的点
        PublishObjectPoints(pMO);
        PublishEllipsoid(pMO);
    }
}


void MapPublisher::PublishObjectPoints( MapObject* vObjs)
{
    mObjectPoints.points.clear();
    mObjectPoints.id=vObjs->mnId;

    mObjectPoints.color.r =  get<0>(mvObjectColors[(vObjs->mnId+5) % 10]);
    mObjectPoints.color.g =  get<1>(mvObjectColors[(vObjs->mnId+5) % 10]);
    mObjectPoints.color.b =  get<2>(mvObjectColors[(vObjs->mnId+5) % 10]);

    auto mvpMapPoints = vObjs->GetMapPointsOnObject();
    for (auto pMP : mvpMapPoints)
    {
        if (!pMP)
            continue;

        if (pMP->isBad())
            continue;
        
        if (pMP->isOutlier())   //检查为什么沙发右侧那么点没有参与物体生成，是不是被判定为outlier了
                continue;

        geometry_msgs::Point p;
        cv::Mat pos = pMP->GetWorldPos();
        p.x=pos.at<float>(0);
        p.y=pos.at<float>(1);
        p.z=pos.at<float>(2);
        mObjectPoints.points.push_back(p);
    }

    mObjectPoints.header.stamp = ros::Time::now();
    publisher_ObjectPoints.publish(mObjectPoints);
}


void MapPublisher::PublishEllipsoid(MapObject* vObjs){
    // 创建 Marker 消息
    visualization_msgs::Marker marker;
    marker.header.frame_id = MAP_FRAME_ID;
    marker.header.stamp = ros::Time::now();
    marker.ns = "ellipsoid_visualization";
    marker.id = vObjs->mnId;  // 唯一的 ID
    marker.type = visualization_msgs::Marker::SPHERE;  // 使用 SPHERE 类型表示椭球体
    marker.action = visualization_msgs::Marker::ADD;

    // 设置椭球体的位置
    double z_offset = -0.1;
    marker.pose.position.x = vObjs->SE3Two(0, 3);  // x 坐标
    marker.pose.position.y = vObjs->SE3Two(1, 3);  // y 坐标
    marker.pose.position.z = vObjs->SE3Two(2, 3)+z_offset;  // z 坐标

    // 椭球体的比例尺寸，a, b, c 分别为 x, y, z 方向的轴长度
    double mini = 0.8;
    marker.scale.x = vObjs->w*2 * mini;  // x 轴长度
    marker.scale.y = vObjs->l*2 * mini;  // y 轴长度
    marker.scale.z = vObjs->h*2 * mini;  // z 轴长度

    // 设置颜色 (RGBA)
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;  
    marker.color.b = 0.0f;
    marker.color.a = 0.25f;  // 透明度
    

    

    // // 创建四元数并设置旋转（Roll, Pitch, Yaw）
    // tf2::Quaternion quat;
    // quat.setRPY(0, 0, 0);  // 设置旋转角度 (弧度)

    // // 将四元数转换为 geometry_msgs::Quaternion 并设置到 marker 中
    // marker.pose.orientation = tf2::toMsg(quat);
    marker.pose.orientation.w=1.0;
    // 将 Marker 发布到 ROS 主题
    publisher_ellipsoid.publish(marker);
}


// void MapPublisher::PublishEllipsoidInfo(const vector<ellipsoid*> &vObjs ){

//     int num_objects = vObjs.size();
//     // 准备一个 std_msgs::Float32MultiArray 消息
//     std_msgs::Float32MultiArray multiarray_msg;

//     // 设置消息的 layout，定义每行代表一个物体，包含9个参数
//     multiarray_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
//     multiarray_msg.layout.dim[0].label = "objects";
//     multiarray_msg.layout.dim[0].size = num_objects;       // 物体数量
//     multiarray_msg.layout.dim[0].stride = num_objects * 9; // 每行9个参数
//     multiarray_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
//     multiarray_msg.layout.dim[1].label = "parameters";
//     multiarray_msg.layout.dim[1].size = 9;                 // 每个物体9个参数
//     multiarray_msg.layout.dim[1].stride = 9;

//     for(size_t i=0; i< num_objects; i++)
//     {
//         auto obj = vObjs[i];
//         auto pose = obj->pose.toXYZPRYVector();
//         auto scale = obj->scale;
//         // std::cout<<"[PublishEllipsoidInfo] pose"<<pose.transpose();
//         // std::cout<<",  scale:"<<scale.transpose()<<std::endl;

//         multiarray_msg.data.push_back(pose[0]); //x
//         multiarray_msg.data.push_back(pose[1]);
//         multiarray_msg.data.push_back(pose[2]);
//         multiarray_msg.data.push_back(pose[3]); //r
//         multiarray_msg.data.push_back(pose[4]); //p
//         multiarray_msg.data.push_back(pose[5]); //y
//         // scale
//         multiarray_msg.data.push_back(scale[0]);
//         multiarray_msg.data.push_back(scale[1]);
//         multiarray_msg.data.push_back(scale[2]);
//     }

//     publisher_ObjectInfo.publish(multiarray_msg);
// }

void MapPublisher::publishDepthAsPointCloud_debug(const cv::Mat& depth, float fx, float fy, float cx, float cy, float scale) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (int v = 0; v < depth.rows; ++v) {
        for (int u = 0; u < depth.cols; ++u) {
            ushort d = depth.at<ushort>(v, u);
            if (d == 0) continue;

            float z = d / scale;
            float x = (u - cx) * z / fx;
            float y = (v - cy) * z / fy;

            cloud->points.push_back(pcl::PointXYZ(x, y, z));
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = MAP_FRAME_ID; 

    publisher_depth_current_frame.publish(cloud_msg);
}


void MapPublisher::SetCurrentCameraPose(const cv::Mat &Tcw)    //zhangjiadong  用在map.cc中
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
    mbCameraUpdated = true;
}



cv::Mat MapPublisher::GetCurrentCameraPose()
{
    unique_lock<mutex> lock(mMutexCamera);
    return mCameraPose.clone();
}

bool MapPublisher::isCamUpdated()
{
    unique_lock<mutex> lock(mMutexCamera);
    return mbCameraUpdated;
}

void MapPublisher::ResetCamFlag()
{
    unique_lock<mutex> lock(mMutexCamera);
    mbCameraUpdated = false;
}


geometry_msgs::Point MapPublisher::corner_to_marker(Eigen::Vector3f& v){
    geometry_msgs::Point point;
    point.x = v[0];
    point.y = v[1];
    point.z = v[2];
    return point;
}
geometry_msgs::Point MapPublisher::corner_to_marker(Eigen::Vector3d& v){
    geometry_msgs::Point point;
    point.x = v[0];
    point.y = v[1];
    point.z = v[2];
    return point;
}
geometry_msgs::Point MapPublisher::corner_to_marker(const std::vector<float>& v){
    geometry_msgs::Point point;
    point.x = v[0];
    point.y = v[1];
    point.z = v[2];
    return point;
}
geometry_msgs::Point MapPublisher::corner_to_marker(const std::vector<double>& v){
    geometry_msgs::Point point;
    point.x = v[0];
    point.y = v[1];
    point.z = v[2];
    return point;
}


} //namespace ORB_SLAM
