#include "model_based_localization.hpp"

Model_based_localization::Model_based_localization(ORB_SLAM2::Map* pMap,
                                                   ORB_SLAM2::ORBVocabulary* pORBVocabulary,
                                                   ORB_SLAM2::KeyFrameDatabase* pKeyFrameDatabase ):
        mpMap(pMap),
        mpORBVocabulary(pORBVocabulary),
        mpKeyFrameDatabase(pKeyFrameDatabase){

    mi_nb_3dpoints = mpMap->MapPointsInMap();
    mi_nb_non_empty_vw = mpORBVocabulary->size();



}

void Model_based_localization::SetMap(ORB_SLAM2::Map* pMap){
    mpMap = pMap;
    mi_nb_3dpoints = mpMap->MapPointsInMap();
    initVocTreeAndDescriptorAssignment();

}

bool Model_based_localization::loacate(ORB_SLAM2::KeyFrame* pKF,
                                       Eigen::Matrix3d& result_R_WC,
                                       Eigen::Vector3d& result_t_WC){



    return true;
}





bool Model_based_localization::initVocTreeAndDescriptorAssignment(){






    return true;
}

