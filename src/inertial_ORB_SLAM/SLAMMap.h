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

#ifndef SLAMMAP_H
#define SLAMMAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include <set>

#include <boost/serialization/serialization.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/split_member.hpp>

#include <mutex>



namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;
class KeyFrameDatabase;



class KFIdComapre
{
public:
    bool operator()(const KeyFrame* kfleft,const KeyFrame* kfright) const;
};

class SLAMMap
{
public:
    // Update after an absolute scale is available
    void UpdateScale(const double &scale);

    //-----------------------------------------
public:
    SLAMMap();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    // getter and setter for reload map
    void SetKeyframe_id_index(const std::map<long unsigned int,long unsigned int>& map);
    void SetMappoint_id_index(const std::map<long unsigned int,long unsigned int>& map);

    std::map<long unsigned int,long unsigned int> GetKeyframe_id_index();
    std::map<long unsigned int,long unsigned int> GetMappoint_id_index();


    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*,KFIdComapre> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    std::map<long unsigned int,long unsigned int> mmKeyframe_id_index;
    std::map<long unsigned int,long unsigned int> mmMappoint_id_index;

    long unsigned int mnMaxKFid;

    std::mutex mMutexMap;


    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        boost::serialization::split_member(ar, *this, version);
    }

    template<class Archive>
    void save(Archive & ar, const unsigned int version) const;


    template<class Archive>
    void load(Archive & ar, const unsigned int version);
};

} //namespace ORB_SLAM

#endif // MAP_H
