
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

    std::cout<<"VW size: "<< orbVocabulary->size()<<std::endl;
    std::cout<<"VW branch: "<<orbVocabulary->getBranchingFactor()<<std::endl;
    std::cout<<"VW depth: "<<orbVocabulary->getDepthLevels()<<std::endl;
    std::cout<<"VW ave-depth: "<<orbVocabulary->getEffectiveLevels()<<std::endl;
    std::cout<<"VW scoretype: "<<orbVocabulary->getScoringType()<<std::endl;
    std::cout<<"VW weighttype: "<<orbVocabulary->getWeightingType()<<std::endl;







    ORB_SLAM2::KeyFrameDatabase* keyFrameDatabase = new ORB_SLAM2::KeyFrameDatabase(*orbVocabulary);

    ORB_SLAM2::SLAMMap* model = new ORB_SLAM2::SLAMMap();

    Model_based_localization locater(model,orbVocabulary,keyFrameDatabase);

    std::string map_path = "/home/pang/software/LearnVIORB/euroc_map.bin";

    locater.LoadMap(map_path);
    locater.RecoverMap();






    return 0;

}





