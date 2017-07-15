#include <LocalTrack/SlidingWindowManager.h>

namespace LocalTrack {
    SlidingWindowManager::SlidingWindowManager() {}
    uint SlidingWindowManager::Size(){
        return mmSlidingWindow.size();


    }

    long unsigned int SlidingWindowManager::GetOldestFrameId(){
        return mmSlidingWindow.front().mnId;
    }
    long unsigned int SlidingWindowManager::GetNewestFrameID(){
        return mmSlidingWindow.back().mnId;
    }

    void SlidingWindowManager::addFrame(ORB_SLAM2::Frame & frame){
        mmSlidingWindow.push_back(frame);

    }
}