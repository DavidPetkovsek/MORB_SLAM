#include "MORB_SLAM/Odometry.hpp"

#include "MORB_SLAM/Atlas.h"
#include "MORB_SLAM/Tracking.h"

namespace MORB_SLAM {


    void Odometry::SetLocalMapper(const std::shared_ptr<LocalMapping> &pLocalMapper) { mwpLocalMapper = pLocalMapper; };
    void Odometry::SetTracker(const std::shared_ptr<Tracking> &pTracker) { mwpTracker = pTracker; };
    void Odometry::SetAtlas(const std::shared_ptr<Atlas> &pAtlas) { mpAtlas = pAtlas; };

    // Local Mapping
    bool Odometry::LocalMappingResetRequested() {
        if (std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock())
            return pLocalMapper->mbResetRequested;
        else
            throw std::runtime_error("ERROR: Cannot access 'LocalMapping' object because it has been destroyed.");
    }

    void Odometry::LocalMappingSetInitializing(bool is_initializing) {
        if(std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock())
            pLocalMapper->bInitializing = is_initializing;
        else
            throw std::runtime_error("ERROR: Cannot access 'LocalMapping' object because it has been destroyed.");
    }

    void Odometry::LocalMappingProcessKeyFramesInQueue(std::vector<std::shared_ptr<KeyFrame>> &vpKF) {
        if(std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock()) {
            while (pLocalMapper->CheckNewKeyFrames()) {
                pLocalMapper->ProcessNewKeyFrame();
                vpKF.push_back(pLocalMapper->mpCurrentKeyFrame);
            }
        } else {
            throw std::runtime_error("ERROR: Cannot access 'LocalMapping' object because it has been destroyed.");
        }
    }

    void Odometry::LocalMappingSetPoseReverseAxisFlip(Sophus::SE3f pose) {
        if(std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock())
            pLocalMapper->mPoseReverseAxisFlip = pose;
        else
            throw std::runtime_error("ERROR: Cannot access 'LocalMapping' object because it has been destroyed.");
    }

    void Odometry::LocalMappingSetNewKeyFramesBad() {
        if(std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock()) {
            for (std::shared_ptr<KeyFrame> newKeyFrame : pLocalMapper->mlNewKeyFrames) {
                newKeyFrame->SetBadFlag();
            }
            pLocalMapper->mlNewKeyFrames.clear();
        } else {
            throw std::runtime_error("ERROR: Cannot access 'LocalMapping' object because it has been destroyed.");
        }
    }

    std::shared_ptr<KeyFrame> Odometry::LocalMappingGetCurrentKeyFrame() {
        if(std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock())
            return pLocalMapper->mpCurrentKeyFrame;
        else
            throw std::runtime_error("ERROR: Cannot access 'LocalMapping' object because it has been destroyed.");
    }

    void Odometry::LocalMappingCorrectMapAfterGBA(const std::shared_ptr<Map> &pMap, unsigned long GBAid) {
        // Correct keyframes starting at map first keyframe
        std::list<std::shared_ptr<KeyFrame>> lpKFtoCheck(
            pMap->mvpKeyFrameOrigins.begin(),
            pMap->mvpKeyFrameOrigins.end());

        while (!lpKFtoCheck.empty()) {
            std::shared_ptr<KeyFrame> pKF = lpKFtoCheck.front();
            const std::set<std::shared_ptr<KeyFrame>> sChilds = pKF->GetChilds();
            Sophus::SE3f Twc = pKF->GetPoseInverse();
            for (std::set<std::shared_ptr<KeyFrame>>::const_iterator sit = sChilds.begin();
                sit != sChilds.end(); sit++) {
                std::shared_ptr<KeyFrame> pChild = *sit;
                if (!pChild || pChild->isBad()) continue;

                if (pChild->mnBAGlobalForKF != GBAid) {
                    Sophus::SE3f Tchildc = pChild->GetPose() * Twc;
                    pChild->mTcwGBA = Tchildc * pKF->mTcwGBA;

                    Sophus::SO3f Rcor = pChild->mTcwGBA.so3().inverse() * pChild->GetPose().so3();
                    if (pChild->isVelocitySet()) {
                        pChild->mVwbGBA = Rcor * pChild->GetVelocity();
                    } else {
                        Verbose::Log(Verbose::INFO, "Child velocity empty!!");
                    }
                    
                    if(pChild->mpExternalKeyFrameData)
                        pChild->mpExternalKeyFrameData->UpdateChildSpanningTree();

                    pChild->mnBAGlobalForKF = GBAid;
                }
                lpKFtoCheck.push_back(pChild);
            }

            pKF->mTcwBefGBA = pKF->GetPose();
            pKF->SetPose(pKF->mTcwGBA);

            if (pKF->bOdom) {
                pKF->mVwbBefGBA = pKF->GetVelocity();
                pKF->SetVelocity(pKF->mVwbGBA);
                
                if(pKF->mpExternalKeyFrameData)
                    pKF->mpExternalKeyFrameData->UpdateParentSpanningTree();
            } else {
                Verbose::Log(Verbose::WARNING, "KF ", pKF->mnId, " not set to inertial!!");
            }

            lpKFtoCheck.pop_front();
        }

        // Correct MapPoints
        const std::vector<std::shared_ptr<MapPoint>> vpMPs = pMap->GetAllMapPoints();

        for (size_t i = 0; i < vpMPs.size(); i++) {
            std::shared_ptr<MapPoint> pMP = vpMPs[i];

            if (pMP->isBad()) continue;

            if (pMP->mnBAGlobalForKF == GBAid) {
                // If optimized by Global BA, just update
                pMP->SetWorldPos(pMP->mPosGBA);
            // Update according to the correction of its reference keyframe
            } else if(std::shared_ptr<KeyFrame> pRefKF = (pMP->GetReferenceKeyFrame()).lock()) {
                if (pRefKF->mnBAGlobalForKF != GBAid) continue;

                // Map to non-corrected camera
                Eigen::Vector3f Xc = pRefKF->mTcwBefGBA * pMP->GetWorldPos();

                // Backproject using corrected camera
                pMP->SetWorldPos(pRefKF->GetPoseInverse() * Xc);
            }
        }

        Verbose::Log(Verbose::DEBUG, "Map updated!");
    }

} //namespace MORB_SLAM