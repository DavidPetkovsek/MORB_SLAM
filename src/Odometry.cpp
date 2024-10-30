#include "MORB_SLAM/Odometry.hpp"

#include "MORB_SLAM/Atlas.h"
#include "MORB_SLAM/Tracking.h"

namespace MORB_SLAM {


    void Odometry::SetLocalMapper(std::shared_ptr<LocalMapping> pLocalMapper) { mwpLocalMapper = pLocalMapper; };
    void Odometry::SetTracker(std::shared_ptr<Tracking> pTracker) { mwpTracker = pTracker; };
    void Odometry::SetAtlas(const std::shared_ptr<Atlas> &pAtlas) { mpAtlas = pAtlas; };

    int Odometry::AtlasNumMaps() { return mpAtlas->CountMaps(); }

    int Odometry::TrackingGetMatchesInliers() { std::shared_ptr<Tracking> pTracker = mwpTracker.lock(); return pTracker->GetMatchesInliers(); }

} //namespace MORB_SLAM