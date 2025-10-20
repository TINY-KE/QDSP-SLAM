#include "include/ellipsoid-version/ConstrainPlane.h"
#include "include/ellipsoid-version/Plane.h"

#include <Eigen/Core>
using namespace Eigen;
namespace g2o
{

ConstrainPlane::ConstrainPlane(plane* ppl):valid(true), image_border(false), association_border(false), pPlane(ppl), state(0), type(CONSTRAINPLANE_STATE::INVALID)
{
}

VectorXd ConstrainPlane::toVector()
{
    // 一些状态
    Vector5d stateVec;
    stateVec << double(valid), double(image_border), double(association_border), double(state), double(type);

    // 还要包含内部的平面结构
    Vector4d vec_plane(-1,-1,-1,-1);
    if(pPlane!=NULL)
        vec_plane = pPlane->param;

    // 注意平面内部的其他信息不存
    VectorXd vec_cplane; vec_cplane.resize(vectorSize());
    vec_cplane << stateVec, vec_plane;

    return vec_cplane;
}

void ConstrainPlane::fromVector(VectorXd& vec){
    return;
}

// 7-8:  8 + 1: type
int ConstrainPlane::vectorSize()
{
    return 9;
}


} // g2o
