#ifndef MODEL_BASED_LOCALIZATION_H
#define MODEL_BASED_LOCALIZATION_H
#include "inertial_ORB_SLAM/SLAMMap.h"
#include <Eigen/Core>

class Model_based_localization{
public:


    Model_based_localization(ORB_SLAM2::SLAMMap* pMap,
                             ORB_SLAM2::ORBVocabulary* pORBVocabulary,
                             ORB_SLAM2::KeyFrameDatabase* pKeyFrameDatabase);

    void LoadMap(const string &filename);

    void RecoverMap();
    void SetMap(ORB_SLAM2::SLAMMap* pMap);

    bool loacate(ORB_SLAM2::KeyFrame* pKF, Eigen::Matrix3d& result_R_WC, Eigen::Vector3d& result_t_WC);


private:

    ORB_SLAM2::SLAMMap* mpMap;
    ORB_SLAM2::ORBVocabulary* mpORBVocabulary;
    ORB_SLAM2::KeyFrameDatabase* mpKeyFrameDatabase;
    std::vector<std::vector<ORB_SLAM2::MapPoint* >> mvvMapPoitnsPtr_per_VW;
    // map info
    unsigned int  mi_nb_3dpoints;
    unsigned int  mi_nb_non_empty_vw; ///< number of visual words which have corresponding 3D points
    bool initVocTreeAndDescriptorAssignment();




};

#endif