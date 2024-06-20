/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once

#include "MORB_SLAM/ImprovedTypes.hpp"
#include "MORB_SLAM/KeyFrame.h"
#include "MORB_SLAM/LocalMapping.h"
#include "MORB_SLAM/Atlas.h"
#include "MORB_SLAM/ORBVocabulary.h"
#include "MORB_SLAM/Tracking.h"
#include "MORB_SLAM/KeyFrameDatabase.h"

#include <boost/algorithm/string.hpp>
#include <thread>
#include <mutex>
#include <utility>
#include <map>
#include <string>
#include <set>
#include <vector>
#ifdef FactoryEngine
#include <apps/morb_g2o/g2o/types/types_seven_dof_expmap.h>
#else
#include "g2o/types/types_seven_dof_expmap.h"
#endif

namespace MORB_SLAM
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;
class Map;
typedef std::shared_ptr<Tracking> Tracking_ptr;


class LoopClosing {
 public:
    typedef std::map<std::shared_ptr<KeyFrame>,g2o::Sim3,std::less<std::shared_ptr<KeyFrame>>, Eigen::aligned_allocator<std::pair<std::shared_ptr<KeyFrame> const, g2o::Sim3>>> KeyFrameAndPose;

    bool hasMergedLocalMap;

 public:

    bool loopClosed = false;

    LoopClosing(const Atlas_ptr &pAtlas, std::shared_ptr<KeyFrameDatabase> pDB, std::shared_ptr<ORBVocabulary> pVoc,const bool bFixScale, const bool bActiveLC, bool bInertial);

    void SetTracker(Tracking_ptr pTracker);

    void SetLocalMapper(std::shared_ptr<LocalMapping> pLocalMapper);

    // Main function
    void Run();

    void InsertKeyFrame(std::shared_ptr<KeyFrame> pKF);

    void RequestReset();
    void RequestResetActiveMap(std::shared_ptr<Map> pMap);

    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(std::shared_ptr<Map> pActiveMap, unsigned long nLoopKF);

    bool isRunningGBA(){
        std::unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }

    void RequestFinish();

protected:

    bool CheckNewKeyFrames();

    //Methods to implement the new place recognition algorithm
    bool NewDetectCommonRegions();

    bool DetectAndReffineSim3FromLastKF(std::shared_ptr<KeyFrame> pCurrentKF, std::shared_ptr<KeyFrame> pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                        std::vector<std::shared_ptr<MapPoint>> &vpMPs, std::vector<std::shared_ptr<MapPoint>> &vpMatchedMPs);

    bool DetectCommonRegionsFromBoW(std::vector<std::shared_ptr<KeyFrame>> &vpBowCand, std::shared_ptr<KeyFrame> &pMatchedKF, std::shared_ptr<KeyFrame> &pLastCurrentKF, g2o::Sim3 &g2oScw,
                                    int &nNumCoincidences, std::vector<std::shared_ptr<MapPoint>> &vpMPs, std::vector<std::shared_ptr<MapPoint>> &vpMatchedMPs);

    bool DetectCommonRegionsFromLastKF(std::shared_ptr<KeyFrame> pCurrentKF, std::shared_ptr<KeyFrame> pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                        std::vector<std::shared_ptr<MapPoint>> &vpMPs, std::vector<std::shared_ptr<MapPoint>> &vpMatchedMPs);

    int FindMatchesByProjection(std::shared_ptr<KeyFrame> pCurrentKF, std::shared_ptr<KeyFrame> pMatchedKFw, g2o::Sim3 &g2oScw, std::set<std::shared_ptr<MapPoint>> &spMatchedMPinOrigin,
                                std::vector<std::shared_ptr<MapPoint>> &vpMapPoints, std::vector<std::shared_ptr<MapPoint>> &vpMatchedMapPoints);

    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, std::vector<std::shared_ptr<MapPoint>> &vpMapPoints);
    void SearchAndFuse(const std::vector<std::shared_ptr<KeyFrame>> &vConectedKFs, std::vector<std::shared_ptr<MapPoint>> &vpMapPoints);

    void CorrectLoop();

    void MergeLocal();
    void MergeLocal2();

    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetActiveMapRequested;
    std::shared_ptr<Map> mpMapToReset;
    std::mutex mMutexReset;

    bool CheckFinish();
    bool mbFinishRequested;
    std::mutex mMutexFinish;

    Atlas_ptr mpAtlas;
    Tracking_ptr mpTracker;
    std::shared_ptr<KeyFrameDatabase> mpKeyFrameDB;
    std::shared_ptr<LocalMapping> mpLocalMapper;

    std::list<std::shared_ptr<KeyFrame>> mlpLoopKeyFrameQueue;

    std::mutex mMutexLoopQueue;

    // Loop detector variables
    std::shared_ptr<KeyFrame> mpCurrentKF;
    std::shared_ptr<KeyFrame> mpLastCurrentKF;
    std::vector<std::shared_ptr<MapPoint>> mvpLoopMapPoints;
    cv::Mat mScw;

    //-------
    std::shared_ptr<Map> mpLastMap;

    bool mbLoopDetected;
    int mnLoopNumCoincidences;
    int mnLoopNumNotFound;
    std::shared_ptr<KeyFrame> mpLoopLastCurrentKF;
    g2o::Sim3 mg2oLoopSlw;
    g2o::Sim3 mg2oLoopScw;
    std::shared_ptr<KeyFrame> mpLoopMatchedKF;
    std::vector<std::shared_ptr<MapPoint>> mvpLoopMPs;
    std::vector<std::shared_ptr<MapPoint>> mvpLoopMatchedMPs;
    bool mbMergeDetected;
    int mnMergeNumCoincidences;
    int mnMergeNumNotFound;
    std::shared_ptr<KeyFrame> mpMergeLastCurrentKF;
    g2o::Sim3 mg2oMergeSlw;
    g2o::Sim3 mg2oMergeScw;
    std::shared_ptr<KeyFrame> mpMergeMatchedKF;
    std::vector<std::shared_ptr<MapPoint>> mvpMergeMPs;
    std::vector<std::shared_ptr<MapPoint>> mvpMergeMatchedMPs;

    g2o::Sim3 mSold_new;
    //-------

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::jthread mpThreadGBA;

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;

    int mnFullBAIdx;

    // To (de)activate LC
    bool mbActiveLC;

    bool mbInertial;
};

} //namespace ORB_SLAM

