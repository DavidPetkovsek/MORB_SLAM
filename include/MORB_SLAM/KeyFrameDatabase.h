/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
#include <list>
#include <mutex>
#include <set>
#include <vector>
#include <map>

#include "MORB_SLAM/Frame.h"
#include "MORB_SLAM/KeyFrame.h"
#include "MORB_SLAM/Map.h"
#include "MORB_SLAM/ORBVocabulary.h"

namespace MORB_SLAM {

class KeyFrame;
class Frame;
class Map;

class KeyFrameDatabase {
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar& mvBackupInvertedFileId;
  }

 public:
  

  KeyFrameDatabase() {}
  KeyFrameDatabase(std::shared_ptr<ORBVocabulary> voc);

  void add(std::shared_ptr<KeyFrame> pKF);

  void erase(std::shared_ptr<KeyFrame> pKF);

  void clear();
  void clearMap(std::shared_ptr<Map> pMap);

  // Loop and Merge Detection
  void DetectNBestCandidates(std::shared_ptr<KeyFrame> pKF, std::vector<std::shared_ptr<KeyFrame>>& vpLoopCand, std::vector<std::shared_ptr<KeyFrame>>& vpMergeCand, int nNumCandidates);

  // Relocalization
  std::vector<std::shared_ptr<KeyFrame>> DetectRelocalizationCandidates(Frame* F, std::shared_ptr<Map> pMap);

  void PreSave();
  void PostLoad(std::map<long unsigned int, std::shared_ptr<KeyFrame>> mpKFid);
  void SetORBVocabulary(std::shared_ptr<ORBVocabulary> pORBVoc);

  size_t GetMemoryUsage();

 protected:
  // Associated vocabulary
  std::shared_ptr<ORBVocabulary> mpVoc;

  // Inverted file
  std::vector<std::list<std::shared_ptr<KeyFrame>> > mvInvertedFile;

  // For save relation without pointer, this is necessary for save/load function
  std::vector<std::list<long unsigned int> > mvBackupInvertedFileId;

  // Mutex
  std::mutex mMutex;
};

}  // namespace MORB_SLAM

