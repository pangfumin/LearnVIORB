#include "model_based_localization.hpp"

Model_based_localization::Model_based_localization(ORB_SLAM2::SLAMMap* pMap,
                                                   ORB_SLAM2::ORBVocabulary* pORBVocabulary,
                                                   ORB_SLAM2::KeyFrameDatabase* pKeyFrameDatabase ):
        mpMap(pMap),
        mpORBVocabulary(pORBVocabulary),
        mpKeyFrameDatabase(pKeyFrameDatabase){

    mi_nb_3dpoints = mpMap->MapPointsInMap();
    mi_nb_non_empty_vw = mpORBVocabulary->size();
}


void Model_based_localization::LoadMap(const string &filename)
{
    {
        std::ifstream is(filename);
        boost::archive::binary_iarchive ia(is, boost::archive::no_header);
        //ia >> mpKeyFrameDatabase;
        ia >> mpMap;

    }

    cout << endl << filename <<" : Map Loaded!" << endl;
}


void Model_based_localization::RecoverMap(){


    vector<ORB_SLAM2::KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    for (vector<ORB_SLAM2::KeyFrame*>::iterator it = vpKFs.begin(); it != vpKFs.end(); ++it) {
        (*it)->SetKeyFrameDatabase(mpKeyFrameDatabase);
        (*it)->SetORBvocabulary(mpORBVocabulary);
        (*it)->SetMap(mpMap);
        (*it)->ComputeBoW();
        mpKeyFrameDatabase->add(*it);
        (*it)->SetMapPoints(mpMap->GetAllMapPoints());
        (*it)->SetSpanningTree(vpKFs);
        (*it)->SetGridParams(vpKFs);

        // Reconstruct map points Observation
    }

    vector<ORB_SLAM2::MapPoint*> vpMPs = mpMap->GetAllMapPoints();
    for (vector<ORB_SLAM2::MapPoint*>::iterator mit = vpMPs.begin(); mit != vpMPs.end(); ++mit) {
        (*mit)->SetMap(mpMap);
        (*mit)->SetObservations(vpKFs);
    }

    for (vector<ORB_SLAM2::KeyFrame*>::iterator it = vpKFs.begin(); it != vpKFs.end(); ++it) {
        (*it)->UpdateConnections();
    }
    // do some statistics:
    std::cout<<"MapPoints in Map: "<< mpMap->MapPointsInMap()<<std::endl;
    std::cout<<"Keyframes in Map: "<< mpMap->KeyFramesInMap()<<std::endl;

    std::map<long unsigned int,long unsigned int> mKeyframe_id_index;
    std::map<long unsigned int,long unsigned int> mMappoint_id_index;

    uint cnt = 0;
    for(auto i: mpMap->GetAllKeyFrames()){
        //std::cout<<"keyframe id at "<< cnt++ <<" : "<<i->mnId<<std::endl;
        mKeyframe_id_index[i->mnId] = cnt;
    }

    cnt = 0;
    for(auto i: mpMap->GetAllMapPoints()){
        //std::cout<<"mapPoint id at "<< cnt++ <<" : "<<i->mnId<<std::endl;
        mMappoint_id_index[i->mnId] = cnt;

    }

    std::cout<<"mMappoint_id_index: "<<mMappoint_id_index.size()<<std::endl;
    std::cout<<"mKeyframe_id_index: "<<mKeyframe_id_index.size()<<std::endl;

    mpMap->SetKeyframe_id_index(mKeyframe_id_index);
    mpMap->SetMappoint_id_index(mMappoint_id_index);

}

void Model_based_localization::SetMap(ORB_SLAM2::SLAMMap* pMap){
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

