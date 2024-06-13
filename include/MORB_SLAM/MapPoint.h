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

#include "MORB_SLAM/KeyFrame.h"
#include "MORB_SLAM/Frame.h"
#include "MORB_SLAM/Map.h"
#include "MORB_SLAM/Converter.h"

#include "MORB_SLAM/SerializationUtils.h"

#include <opencv2/core/core.hpp>
#include <mutex>
#include <set>
#include <map>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/map.hpp>

namespace MORB_SLAM
{

class KeyFrame;
class Map;
class Frame;

class MapPoint : public std::enable_shared_from_this<MapPoint> {

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        ar & mnId;
        ar & mnFirstKFid;
        ar & mnFirstFrame;
        ar & nObs;

        // Protected variables
        ar & boost::serialization::make_array(mWorldPos.data(), mWorldPos.size());
        ar & boost::serialization::make_array(mNormalVector.data(), mNormalVector.size());
        ar & mBackupObservationsId1;
        ar & mBackupObservationsId2;
        serializeMatrix(ar,mDescriptor,version);
        ar & mBackupRefKFId;

        ar & mbBad;
        ar & mBackupReplacedId;

        ar & mfMinDistance;
        ar & mfMaxDistance;

    }


public:
    
    MapPoint();

    MapPoint(const Eigen::Vector3f &Pos, std::shared_ptr<KeyFrame> pRefKF, std::shared_ptr<Map> pMap);
    MapPoint(const double invDepth, cv::Point2f uv_init, std::shared_ptr<KeyFrame> pRefKF, std::shared_ptr<KeyFrame> pHostKF, std::shared_ptr<Map> pMap);
    MapPoint(const Eigen::Vector3f &Pos,  std::shared_ptr<Map> pMap, Frame* pFrame, const int &idxF);
    ~MapPoint();

    void SetWorldPos(const Eigen::Vector3f &Pos);
    Eigen::Vector3f GetWorldPos();

    Eigen::Vector3f GetNormal();
    void SetNormalVector(const Eigen::Vector3f& normal);

    std::weak_ptr<KeyFrame> GetReferenceKeyFrame();

    std::map<std::weak_ptr<KeyFrame>, std::tuple<int,int>, std::owner_less<>> GetObservations();
    int Observations();

    void AddObservation(std::shared_ptr<KeyFrame> pKF,int idx);
    void EraseObservation(std::shared_ptr<KeyFrame> pKF);

    std::tuple<int,int> GetIndexInKeyFrame(std::shared_ptr<KeyFrame> pKF);
    bool IsInKeyFrame(std::shared_ptr<KeyFrame> pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(std::shared_ptr<MapPoint> pMP);
    std::shared_ptr<MapPoint> GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){ return mnFound; }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, std::shared_ptr<KeyFrame>pKF);
    int PredictScale(const float &currentDist, Frame* pF);

    std::shared_ptr<Map> GetMap();
    void UpdateMap(std::shared_ptr<Map> pMap);

    void PrintObservations();

    void PreSave(std::set<std::shared_ptr<KeyFrame>>& spKF,std::set<std::shared_ptr<MapPoint>>& spMP);
    void PostLoad(std::map<long unsigned int, std::shared_ptr<KeyFrame>>& mpKFid, std::map<long unsigned int, std::shared_ptr<MapPoint>>& mpMPid);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackDepth;
    float mTrackDepthR;
    float mTrackProjXR;
    float mTrackProjYR;
    bool mbTrackInView, mbTrackInViewR;
    int mnTrackScaleLevel, mnTrackScaleLevelR;
    float mTrackViewCos, mTrackViewCosR;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;
    Eigen::Vector3f mPosGBA;
    long unsigned int mnBAGlobalForKF;
    long unsigned int mnBALocalForMerge;

    // Variable used by merging
    Eigen::Vector3f mPosMerge;
    Eigen::Vector3f mNormalVectorMerge;


    // Fopr inverse depth optimization
    double mInvDepth;
    double mInitU;
    double mInitV;

    static std::mutex mGlobalMutex;

    unsigned int mnOriginMapId;

    static long unsigned int nMPsInMemory;

protected:

    // Position in absolute coordinates
    Eigen::Vector3f mWorldPos;

    // Keyframes observing the point and associated index in keyframe
    std::map<std::weak_ptr<KeyFrame>, std::tuple<int,int>, std::owner_less<>> mObservations;
    // For save relation without pointer, this is necessary for save/load function
    std::map<long unsigned int, int> mBackupObservationsId1;
    std::map<long unsigned int, int> mBackupObservationsId2;

    // Mean viewing direction
    Eigen::Vector3f mNormalVector;

    // Best descriptor to fast matching
    cv::Mat mDescriptor;

    // Reference KeyFrame
    std::weak_ptr<KeyFrame> mpRefKF;
    long unsigned int mBackupRefKFId;

    // Tracking counters
    int mnVisible;
    int mnFound;

    // Bad flag (we do not currently erase MapPoint from memory)
    bool mbBad;
    std::shared_ptr<MapPoint> mpReplaced;
    // For save relation without pointer, this is necessary for save/load function
    long long int mBackupReplacedId;

    // Scale invariance distances
    float mfMinDistance;
    float mfMaxDistance;

    std::shared_ptr<Map> mpMap;

    // Mutex
    std::mutex mMutexPos;
    std::mutex mMutexFeatures;
    std::mutex mMutexMap;

};

} //namespace ORB_SLAM
