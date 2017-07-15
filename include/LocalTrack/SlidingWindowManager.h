#ifndef SLIDINGWINDOWMANAGER_H
#define SLIDINGWINDOWMANAGER_H
#include <map>
#include <deque>
#include <Frame.h>

namespace LocalTrack {
    class SlidingWindowManager {

    public:

        SlidingWindowManager();
        uint Size();
        long unsigned int GetOldestFrameId();
        long unsigned int GetNewestFrameID();
        void addFrame(ORB_SLAM2::Frame & frame);


    private:
        std::deque<ORB_SLAM2::Frame> mmSlidingWindow;








    };
}

#endif