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

#include "SLAMMap.h"
#define TEST_DATA 0xdeadbeef
#include<mutex>

namespace ORB_SLAM2
{

bool KFIdComapre::operator ()(const KeyFrame* kfleft,const KeyFrame* kfright) const
{
    return kfleft->mnId < kfright->mnId;
}

void SLAMMap::UpdateScale(const double &scale)
{
    unique_lock<mutex> lock(mMutexMapUpdate);
    for(std::set<KeyFrame*,KFIdComapre>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
    {
        KeyFrame* pKF = *sit;
        cv::Mat Tcw = pKF->GetPose();
        cv::Mat tcw = Tcw.rowRange(0,3).col(3)*scale;
        tcw.copyTo(Tcw.rowRange(0,3).col(3));
        pKF->SetPose(Tcw);
    }
    for(std::set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
    {
        MapPoint* pMP = *sit;
        pMP->SetWorldPos(pMP->GetWorldPos()*scale);
    }
    std::cout<<std::endl<<"... Map scale updated ..."<<std::endl<<std::endl;
}

//---------------------------------------

SLAMMap::SLAMMap():mnMaxKFid(0)
{
}



template<class Archive>
void SLAMMap::save(Archive & ar, const unsigned int version) const
{
    unsigned int test_data = TEST_DATA;


    // save mapPoints
    int nItems = mspMapPoints.size();
    ar & nItems;
    cout << "{INFO}mspMapPoints size = " << nItems << endl;

    std::for_each(mspMapPoints.begin(), mspMapPoints.end(), [&ar](MapPoint* pMapPoint) {
        ar & *pMapPoint;
    });


    // save keyframes
    nItems = mspKeyFrames.size();
    cout << "{INFO}mspKeyFrames size = " << nItems << endl;
    ar & nItems;
    std::for_each(mspKeyFrames.begin(), mspKeyFrames.end(), [&ar](KeyFrame* pKeyFrame) {
        ar & *pKeyFrame;
    });

    nItems = mvpKeyFrameOrigins.size();
    cout << "{INFO}mvpKeyFrameOrigins size = " << nItems << endl;
    ar & nItems;
    std::for_each(mvpKeyFrameOrigins.begin(), mvpKeyFrameOrigins.end(), [&ar](KeyFrame* pKeyFrameOrigin) {
        ar & *pKeyFrameOrigin;
    });
    // Pertaining to map drawing
    //nItems = mvpReferenceMapPoints.size();
    //cout << "$${INFO}mvpReferenceMapPoints size = %d " << nItems << endl;
    //ar & nItems;
    //std::for_each(mvpReferenceMapPoints.begin(), mvpReferenceMapPoints.end(), [&ar](MapPoint* pMapPointReference) {
    //    ar & *pMapPointReference;
    //});
    ar & const_cast<long unsigned int &> (mnMaxKFid);

    ar & test_data;
}

template<class Archive>
void SLAMMap::load(Archive & ar, const unsigned int version)
{
    unsigned int test_data;

    int nItems;
    ar & nItems;
    cout << "{INFO}mspMapPoints size = " << nItems << endl;

    for (int i = 0; i < nItems; ++i) {

        MapPoint* pMapPoint = new MapPoint();
        ar & *pMapPoint;
        mspMapPoints.insert(pMapPoint);
    }

    ar & nItems;
    cout << "{INFO}mspKeyFrames size = " << nItems << endl;

    for (int i = 0; i < nItems; ++i) {

        KeyFrame* pKeyFrame = new KeyFrame;
        ar & *pKeyFrame;
        mspKeyFrames.insert(pKeyFrame);
    }


    ar & nItems;
    cout << "{INFO}mvpKeyFrameOrigins size = " << nItems << endl;

    for (int i = 0; i < nItems; ++i) {

        KeyFrame* pKeyFrame = new KeyFrame;
        ar & *pKeyFrame;
        /* TODO : VerifyHere*/
        mvpKeyFrameOrigins.push_back(*mspKeyFrames.begin());
    }

    ar & const_cast<long unsigned int &> (mnMaxKFid);

    ar & test_data;
    if (test_data == TEST_DATA)
        cout <<">>Map Loading Validated as True" << endl;
    else
        cout <<"ERROR Map Loading Validated as False: Got -" << test_data << " :( Check Load Save sequence" << endl;

}


// Explicit template instantiation
    template void SLAMMap::save<boost::archive::binary_oarchive>(
            boost::archive::binary_oarchive &,
            const unsigned int) const;
    template void SLAMMap::save<boost::archive::binary_iarchive>(
            boost::archive::binary_iarchive &,
            const unsigned int) const;
    template void SLAMMap::load<boost::archive::binary_oarchive>(
            boost::archive::binary_oarchive &,
            const unsigned int);
    template void SLAMMap::load<boost::archive::binary_iarchive>(
            boost::archive::binary_iarchive &,
            const unsigned int);


void SLAMMap::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void SLAMMap::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void SLAMMap::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void SLAMMap::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void SLAMMap::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

vector<KeyFrame*> SLAMMap::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> SLAMMap::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int SLAMMap::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int SLAMMap::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> SLAMMap::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int SLAMMap::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void SLAMMap::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

    void SLAMMap::SetKeyframe_id_index(const std::map<long unsigned int,long unsigned int>& map){
        mmKeyframe_id_index = map;

    }
    void SLAMMap::SetMappoint_id_index(const std::map<long unsigned int,long unsigned int>& map){
        mmMappoint_id_index = map;

    }

    std::map<long unsigned int,long unsigned int> SLAMMap::GetKeyframe_id_index(){
        return mmKeyframe_id_index;

    }
    std::map<long unsigned int,long unsigned int> SLAMMap::GetMappoint_id_index(){
        return mmMappoint_id_index;

    }

} //namespace ORB_SLAM
