
#include "localization/model_based_localization.hpp"


int main(){

    ORB_SLAM2::ORBVocabulary* orbVocabulary = new ORB_SLAM2::ORBVocabulary();
    std::string strVocFile = "/home/pang/software/LearnVIORB/Vocabulary/ORBvoc.bin";
    bool bVocLoad = orbVocabulary->loadFromBinaryFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }


    ORB_SLAM2::KeyFrameDatabase* keyFrameDatabase = new ORB_SLAM2::KeyFrameDatabase(*orbVocabulary);

    ORB_SLAM2::SLAMMap* model = new ORB_SLAM2::SLAMMap();

    Model_based_localization locater(model,orbVocabulary,keyFrameDatabase);

    std::string map_path = "/home/pang/software/LearnVIORB/euroc_map.bin";

    locater.LoadMap(map_path);




    return 0;

}





