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

#include "MORB_SLAM/KeyFrameDatabase.h"
#include "MORB_SLAM/KeyFrame.h"
#include <mutex>

#ifdef FactoryEngine
#include <apps/morb_dbow2/DBoW2/BowVector.h>
#else
#include "DBoW2/BowVector.h"
#endif


namespace MORB_SLAM {

KeyFrameDatabase::KeyFrameDatabase(std::shared_ptr<ORBVocabulary> voc) : mpVoc(std::move(voc)) {
  mvInvertedFile.resize(mpVoc->size());
}

void KeyFrameDatabase::add(std::shared_ptr<KeyFrame> pKF) {
  std::unique_lock<std::mutex> lock(mMutex);

  for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
    mvInvertedFile[vit->first].push_back(pKF);
}

void KeyFrameDatabase::erase(std::shared_ptr<KeyFrame> pKF) {
  std::unique_lock<std::mutex> lock(mMutex);

  // Erase elements in the Inverse File for the entry
  for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++) {
    // List of keyframes that share the word
    std::list<std::shared_ptr<KeyFrame>>& lKFs = mvInvertedFile[vit->first];

    for (std::list<std::shared_ptr<KeyFrame>>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++) {
      if (pKF == *lit) {
        lKFs.erase(lit);
        break;
      }
    }
  }
}

void KeyFrameDatabase::clear() {
  mvInvertedFile.clear();
  mvInvertedFile.resize(mpVoc->size());
}

void KeyFrameDatabase::clearMap(std::shared_ptr<Map> pMap) {
  std::unique_lock<std::mutex> lock(mMutex);

  // Erase elements in the Inverse File for the entry
  for (std::vector<std::list<std::shared_ptr<KeyFrame>> >::iterator vit = mvInvertedFile.begin(), vend = mvInvertedFile.end(); vit != vend; vit++) {
    // List of keyframes that share the word
    std::list<std::shared_ptr<KeyFrame>>& lKFs = *vit;

    for (std::list<std::shared_ptr<KeyFrame>>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend;) {
      std::shared_ptr<KeyFrame> pKFi = *lit;
      if (pMap == pKFi->GetMap()) {
        lit = lKFs.erase(lit);
      } else {
        ++lit;
      }
    }
  }
}

bool compFirst(const std::pair<float, std::shared_ptr<KeyFrame>>& a, const std::pair<float, std::shared_ptr<KeyFrame>>& b) {
  return a.first > b.first;
}

void KeyFrameDatabase::DetectNBestCandidates(std::shared_ptr<KeyFrame> pKF, std::vector<std::shared_ptr<KeyFrame>>& vpLoopCand, std::vector<std::shared_ptr<KeyFrame>>& vpMergeCand, int nNumCandidates) {
  std::list<std::shared_ptr<KeyFrame>> lKFsSharingWords;
  std::set<std::shared_ptr<KeyFrame>> spConnectedKF;

  // Search all keyframes that share a word with current frame
  {
    std::unique_lock<std::mutex> lock(mMutex);

    spConnectedKF = pKF->GetConnectedKeyFrames();

    for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++) {
      std::list<std::shared_ptr<KeyFrame>>& lKFs = mvInvertedFile[vit->first];

      for (std::list<std::shared_ptr<KeyFrame>>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++) {
        std::shared_ptr<KeyFrame> pKFi = *lit;

        if (pKFi->mnPlaceRecognitionQuery != pKF->mnId) {
          pKFi->mnPlaceRecognitionWords = 0;
          if (!spConnectedKF.count(pKFi)) {
            pKFi->mnPlaceRecognitionQuery = pKF->mnId;
            lKFsSharingWords.push_back(pKFi);
          }
        }
        pKFi->mnPlaceRecognitionWords++;
      }
    }
  }
  if (lKFsSharingWords.empty()) return;

  // Only compare against those keyframes that share enough words
  int maxCommonWords = 0;
  for (std::list<std::shared_ptr<KeyFrame>>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++) {
    if ((*lit)->mnPlaceRecognitionWords > maxCommonWords)
      maxCommonWords = (*lit)->mnPlaceRecognitionWords;
  }

  int minCommonWords = maxCommonWords * 0.8f;

  std::list<std::pair<float, std::shared_ptr<KeyFrame>>> lScoreAndMatch;

  int nscores = 0;

  // Compute similarity score.
  for (std::list<std::shared_ptr<KeyFrame>>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++) {
    std::shared_ptr<KeyFrame> pKFi = *lit;

    if (pKFi->mnPlaceRecognitionWords > minCommonWords) {
      nscores++;
      float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);
      pKFi->mPlaceRecognitionScore = si;
      lScoreAndMatch.push_back(std::make_pair(si, pKFi));
    }
  }

  if (lScoreAndMatch.empty()) return;

  std::list<std::pair<float, std::shared_ptr<KeyFrame>> > lAccScoreAndMatch;

  // Lets now accumulate score by covisibility
  for (std::list<std::pair<float, std::shared_ptr<KeyFrame>>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++) {
    std::shared_ptr<KeyFrame> pKFi = it->second;
    std::vector<std::shared_ptr<KeyFrame>> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

    float bestScore = it->first;
    float accScore = bestScore;
    std::shared_ptr<KeyFrame> pBestKF = pKFi;
    for (std::vector<std::shared_ptr<KeyFrame>>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++) {
      std::shared_ptr<KeyFrame> pKF2 = *vit;
      if (pKF2->mnPlaceRecognitionQuery != pKF->mnId) continue;

      accScore += pKF2->mPlaceRecognitionScore;
      if (pKF2->mPlaceRecognitionScore > bestScore) {
        pBestKF = pKF2;
        bestScore = pKF2->mPlaceRecognitionScore;
      }
    }
    lAccScoreAndMatch.push_back(std::make_pair(accScore, pBestKF));
  }

  lAccScoreAndMatch.sort(compFirst);

  vpLoopCand.reserve(nNumCandidates);
  vpMergeCand.reserve(nNumCandidates);
  std::set<std::shared_ptr<KeyFrame>> spAlreadyAddedKF;
  size_t i = 0;
  std::list<std::pair<float, std::shared_ptr<KeyFrame>>>::iterator it = lAccScoreAndMatch.begin();
  while ((i < lAccScoreAndMatch.size()) || (i < lAccScoreAndMatch.size() && (static_cast<int>(vpLoopCand.size()) < nNumCandidates || static_cast<int>(vpMergeCand.size()) < nNumCandidates))) {
    std::shared_ptr<KeyFrame> pKFi = it->second;
    if (pKFi->isBad()){
      i++;
      it++;
      continue;
    }

    if (!spAlreadyAddedKF.count(pKFi)) {
      if (pKF->GetMap() == pKFi->GetMap() && static_cast<int>(vpLoopCand.size()) < nNumCandidates) {
        vpLoopCand.push_back(pKFi);
      } else if (pKF->GetMap() != pKFi->GetMap() && static_cast<int>(vpMergeCand.size()) < nNumCandidates && !pKFi->GetMap()->IsBad()) {
        vpMergeCand.push_back(pKFi);
      }
      spAlreadyAddedKF.insert(pKFi);
    }
    i++;
    it++;
  }
}

std::vector<std::shared_ptr<KeyFrame>> KeyFrameDatabase::DetectRelocalizationCandidates(Frame* F, std::shared_ptr<Map> pMap) {
  std::list<std::shared_ptr<KeyFrame>> lKFsSharingWords;

  // Search all keyframes that share a word with current frame
  {
    std::unique_lock<std::mutex> lock(mMutex);

    for (const auto &pair : F->mBowVec) {
      std::list<std::shared_ptr<KeyFrame>>& lKFs = mvInvertedFile[pair.first];

      for (std::shared_ptr<KeyFrame> pKFi : lKFs) {
        if (pKFi->mnRelocQuery != F->mnId) {
          pKFi->mnRelocWords = 0;
          pKFi->mnRelocQuery = F->mnId;
          lKFsSharingWords.push_back(pKFi);
        }
        pKFi->mnRelocWords++;
      }
    }
  }
  if (lKFsSharingWords.empty()) return std::vector<std::shared_ptr<KeyFrame>>();

  // Only compare against those keyframes that share enough words
  int maxCommonWords = 0;
  for (std::shared_ptr<KeyFrame> pKFi : lKFsSharingWords) {
    if (pKFi->mnRelocWords > maxCommonWords)
      maxCommonWords = pKFi->mnRelocWords;
  }

  int minCommonWords = maxCommonWords * 0.8f;

  std::list<std::pair<float, std::shared_ptr<KeyFrame>>> lScoreAndMatch;

  int nscores = 0;

  // Compute similarity score.
  for (std::shared_ptr<KeyFrame> pKFi : lKFsSharingWords) {
    if (pKFi->mnRelocWords > minCommonWords) {
      nscores++;
      float si = mpVoc->score(F->mBowVec, pKFi->mBowVec);
      pKFi->mRelocScore = si;
      lScoreAndMatch.emplace_back(si, pKFi);
    }
  }

  if (lScoreAndMatch.empty()) return std::vector<std::shared_ptr<KeyFrame>>();

  std::list<std::pair<float, std::shared_ptr<KeyFrame>>> lAccScoreAndMatch;
  float bestAccScore = 0;

  // Lets now accumulate score by covisibility
  for (auto &pair : lScoreAndMatch) {
    std::shared_ptr<KeyFrame> pKFi = pair.second;
    std::vector<std::shared_ptr<KeyFrame>> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

    float bestScore = pair.first;
    float accScore = bestScore;
    std::shared_ptr<KeyFrame> pBestKF = pKFi;
    for (std::shared_ptr<KeyFrame> pKF2 : vpNeighs) {
      if (pKF2->mnRelocQuery != F->mnId) continue;

      accScore += pKF2->mRelocScore;
      if (pKF2->mRelocScore > bestScore) {
        pBestKF = pKF2;
        bestScore = pKF2->mRelocScore;
      }
    }
    lAccScoreAndMatch.emplace_back(accScore, pBestKF);
    if (accScore > bestAccScore) bestAccScore = accScore;
  }

  // Return all those keyframes with a score higher than 0.75*bestScore
  float minScoreToRetain = 0.75f * bestAccScore;
  std::set<std::shared_ptr<KeyFrame>> spAlreadyAddedKF;
  std::vector<std::shared_ptr<KeyFrame>> vpRelocCandidates;
  vpRelocCandidates.reserve(lAccScoreAndMatch.size());
  for (auto &pair : lAccScoreAndMatch) {
    const float& si = pair.first;
    if (si > minScoreToRetain) {
      std::shared_ptr<KeyFrame> pKFi = pair.second;
      if (pKFi->GetMap() != pMap) continue;
      if (!spAlreadyAddedKF.count(pKFi)) {
        vpRelocCandidates.push_back(pKFi);
        spAlreadyAddedKF.insert(pKFi);
      }
    }
  }

  return vpRelocCandidates;
}

void KeyFrameDatabase::SetORBVocabulary(std::shared_ptr<ORBVocabulary> pORBVoc) {
  mpVoc = std::move(pORBVoc);

  mvInvertedFile.clear();
  mvInvertedFile.resize(mpVoc->size());
}

size_t KeyFrameDatabase::GetMemoryUsage() {
    size_t totalMemory = 0;

    // Memory for the vector itself
    totalMemory += sizeof(mvInvertedFile);
    totalMemory += mvInvertedFile.capacity() * sizeof(std::list<std::shared_ptr<KeyFrame>>);

    for (const auto& lst : mvInvertedFile) {
        // Memory for each list itself
        totalMemory += sizeof(lst);
        totalMemory += lst.size() * (sizeof(std::shared_ptr<KeyFrame>) + 2 * sizeof(void*)); // List node overhead

        for (const auto& sharedPtr : lst) {
            if (sharedPtr) {
                totalMemory += sizeof(std::shared_ptr<KeyFrame>); // Size of the shared pointer itself
            }
        }
    }
    return totalMemory;
}

}  // namespace MORB_SLAM
