#ifndef LOCALTRACKING_H
#define LOCALTRACKING_H

#include <LocalTrack/SlidingWindowManager.h>
#include <IMU/imudata.h>

namespace LocalTrack{
    class LocalTracking{
    public:
        LocalTracking();
        LocalTracking(uint SlidingWindowSize);
        void TrackVIO(cv::Mat& image,std::vector<ORB_SLAM2::IMUData> & ImuDatasSinceLastFrame );
    private:
        uint miSlidingWindowSize;
        SlidingWindowManager mSWManager;






    };
}
#endif