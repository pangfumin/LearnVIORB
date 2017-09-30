
#include "utility/catch/catch.hpp"
#include "localization/model_based_localization.hpp"




ORB_SLAM2::ORBVocabulary* orbVocabulary = new ORB_SLAM2::ORBVocabulary();
ORB_SLAM2::KeyFrameDatabase* keyFrameDatabase = new ORB_SLAM2::KeyFrameDatabase(*orbVocabulary);

ORB_SLAM2::Map* model = new ORB_SLAM2::Map();

Model_based_localization locater(model,orbVocabulary,keyFrameDatabase);

std::string map_path = "/home/pang/software/LearnVIORB/euroc_map.bin";

TEST_CASE( "Save_Load ", "[load]"){







}

