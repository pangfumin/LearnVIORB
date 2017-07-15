#include <LocalTrack/LocalTracking.h>

namespace LocalTrack{
    LocalTracking::LocalTracking() {}
    LocalTracking::LocalTracking(uint SlidingWindowSize)
            : miSlidingWindowSize(SlidingWindowSize){

    }
    void LocalTracking::TrackVIO(cv::Mat& image,
                                 std::vector<ORB_SLAM2::IMUData> & ImuDatasSinceLastFrame ){

        

    }
}