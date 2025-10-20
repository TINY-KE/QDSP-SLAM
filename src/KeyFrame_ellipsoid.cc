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

#include "KeyFrame.h"

namespace ORB_SLAM2 {

void KeyFrame::AddEllipsoldsGlobal(g2o::ellipsoid* e)
{
    unique_lock<mutex> lock(mMutexObjects);
    mpGlobalEllipsolds.push_back(e);
}

void KeyFrame::ReplaceEllipsoldsGlobal(int obj_id, g2o::ellipsoid* e_global)
{
    unique_lock<mutex> lock(mMutexObjects);
    *mpGlobalEllipsolds[obj_id] = *e_global;

}

std::vector<g2o::ellipsoid*> KeyFrame::GetEllipsoldsGlobal()
{
    unique_lock<mutex> lock(mMutexObjects);
    return mpGlobalEllipsolds;
}

}