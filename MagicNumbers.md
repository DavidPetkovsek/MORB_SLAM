### List of all random hardcoded values within MORB-SLAM that have no obvious justification for their value.

# General Constants That Exist In Many Places
Various Optimizers:
- 3.841, 5.991, 7.815;
    - These are the critical values of the Chi-Square distribution
    - P(X < 3.841 | df = 1) = 95%
    - P(X < 5.991 | df = 2) = 95%
    - P(X < 7.815 | df = 3) = 95%
- its[4] = {10, 10, 10, 10}, its = 10;
    - They default to 10 iterations. I don't know why.
- ORBMatcher.nnratio
    - The multiplier that decides if the best ORB match should be considered an observation.
    - It is a valid observation if bestDist <= nnratio * bestDist2, where bestDist is the vector norm b/t the observed ORB and the ORB of a MapPoint. bestDist2 is the distance of the second-best MapPoint match.

When there's a few non-constant thresholds at the top of an ORB-Matching function (nProjMatches, nProjOptMatches, nProjMatchesRep, etc.):
    - Mystery values. Picked at random. Good luck!

# Specific Hardcoded Values
KeyFrame::UpdateConnections():
- th = 15;
    - KFs are "connected" if they share 15 MPs

Frame::ComputeStereoMatches():
- w = 5, L = 5;
    - Use a sliding window size of 5px for ORB detection
- thDist = 1.5f * 1.4f * median;
    - I have absolutely no idea. Used to delete any ORB observations further than 2.1*median_orb_distance.

FrameDrawer::DrawFrame() / FrameDrawer::DrawFrame():
- r = 5;
    - Reason: the size of the square around each ORB on the camera feed has an edge length of 5px, since that's the size of an ORB from ComputeStereoMatches().
    - This value should be equal to w,L from ComputeStereoMatches().

ImuTypes.cc:
- eps = 1e-4;
    - This value is used as the threshold for if a number is equal to 0 (i.e. if abs(x) < eps, then assume x=0)

LocalMapping::InitializeIMU():
- minTime = mbMonocular ? 2.0 : 1.0, size_t nMinKF = 10;
    - Only init the IMU if minTime has elapsed since the first KF, and there are at least nMinKF KFs

LocalMapping::MapPointCulling():
- cnThObs = mbMonocular ? 2 : 3;
    - Only cull MapPoints if n_observations <= cnThObs. The idea is to have it such that they're only deleted if they've been observed by <=2 KFs in the mono case, and <2 KFs in the stereo case (since each stereo obs counts twice).

LocalMapping::CreateNewMapPoints():
- nn = mbMonocular ? 30 : 10;
    - Presumably because they're less confident in monocular Maps, so they check more KeyFrames? I don't know.
- ratioFactor = 1.5f * mpCurrentKeyFrame->mfScaleFactor;
    - I don't know. Maybe this is supposed to be the value taken from the ORBextractor.scaleFactor setting from the .yaml?

LocalMapping::KeyFrameCulling():
- Nd = 21;
    - Don't cull to less than 9+10 KFs. Don't ask why.
- thObs = 3;
    - Should probably be the same as cnThObs in MapPointCulling()?
- (mbAbortBA && numChecked > 20) || numChecked > 100
    - Check at least 20 KFs for culling, maximum 100. The 20 could probably be way less? Probably the 100 too?

Tracking::UpdateLocalKeyFrames():
- Nd = 20;
    - Should probably always be 1 less Nd in KeyFrameCulling()? Just because this Nd represents how many KFs are considered "local", which can be adjusted. The origin KF should never be changed, so nothing should be culled until there are 20 local KFs, plus the 1 for the origin KF.

Tracking::SearchLocalPoints():
- th = [insert horrific conditional here];
    - Here be dragons.

Tracking::TrackWithMotionModel():
- th = (mSensor == CameraType::STEREO) ? 7 : 15;
    - Should this be ORBextractor.minThFAST : ORBextractor.iniThFAST from settings? Who knows.

Optimizer::PoseInertialOptimizationLastKeyFrame():
- chi2MonoOut = 18.f, chi2StereoOut = 24.f;

Optimizer::LocalInertialBA():
- maxOpt = bLarge ? 25 : 10, opt_it = bLarge ? 4 : 10;
    - Should maxOpt be the same as Nd from UpdateLocalKeyFrames() when bLarge==true? Maybe?
    - When bLarge is false, we use the default of its=10
    - optimize using more KFs if there's >100 MPs in the KF. WHY??
    - perform less iterations if we use more KFs. WHY??
- maxFixKF = 200;
    - probably just an arbitrarily high number to prevent trying to optimize a billion KFs

Optimizer::MergeInertialBA():
- Nd = 6, maxCovKF = 30; 

Optimizer::OptimizeEssentialGraph():
- minFeat = 100;