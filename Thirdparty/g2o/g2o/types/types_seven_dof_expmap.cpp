// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "g2o/types/types_seven_dof_expmap.h"

namespace g2o {

  VertexSim3Expmap::VertexSim3Expmap() : BaseVertex<7, Sim3>() {
    _marginalized=false;
    _fix_scale = false;
  }


  EdgeSim3::EdgeSim3() : BaseBinaryEdge<7, Sim3, VertexSim3Expmap, VertexSim3Expmap>(){}

  bool VertexSim3Expmap::read(std::istream& is) {
    Vector7d cam2world;
    for (int i=0; i<6; i++){
      is >> cam2world[i];
    }
    is >> cam2world[6];

    for (int i=0; i<2; i++) {
      is >> _focal_length1[i];
    }
    for (int i=0; i<2; i++) {
      is >> _principle_point1[i];
    }

    setEstimate(Sim3(cam2world).inverse());
    return true;
  }

  bool VertexSim3Expmap::write(std::ostream& os) const {
    Sim3 cam2world(estimate().inverse());
    Vector7d lv=cam2world.log();
    for (int i=0; i<7; i++) {
      os << lv[i] << " ";
    }
    for (int i=0; i<2; i++) {
      os << _focal_length1[i] << " ";
    }
    for (int i=0; i<2; i++) {
      os << _principle_point1[i] << " ";
    }
    return os.good();
  }

  bool EdgeSim3::read(std::istream& is) {
    Vector7d v7;
    for (int i=0; i<7; i++) {
      is >> v7[i];
    }

    Sim3 cam2world(v7);
    setMeasurement(cam2world.inverse());

    for (int i=0; i<7; i++) {
      for (int j=i; j<7; j++) {
        is >> information()(i,j);
        if (i!=j)
          information()(j,i)=information()(i,j);
      }
    }
    return true;
  }

  bool EdgeSim3::write(std::ostream& os) const {
    Sim3 cam2world(measurement().inverse());
    Vector7d v7 = cam2world.log();
    for (int i=0; i<7; i++) {
      os  << v7[i] << " ";
    }
    for (int i=0; i<7; i++) {
      for (int j=i; j<7; j++) {
        os << " " <<  information()(i,j);
      }
    }
    return os.good();
  }

  /**Sim3ProjectXYZ*/
  EdgeSim3ProjectXYZ::EdgeSim3ProjectXYZ() : BaseBinaryEdge<2, Eigen::Vector2d, VertexSBAPointXYZ, VertexSim3Expmap>(){}

  bool EdgeSim3ProjectXYZ::read(std::istream& is) {
    for (int i=0; i<2; i++) {
      is >> _measurement[i];
    }

    for (int i=0; i<2; i++) {
      for (int j=i; j<2; j++) {
        is >> information()(i,j);
        if (i!=j)
          information()(j,i)=information()(i,j);
      }
    }
    return true;
  }

  bool EdgeSim3ProjectXYZ::write(std::ostream& os) const {
    for (int i=0; i<2; i++) {
      os  << _measurement[i] << " ";
    }

    for (int i=0; i<2; i++) {
      for (int j=i; j<2; j++){
        os << " " <<  information()(i,j);
      }
    }
    return os.good();
  }

/**InverseSim3ProjectXYZ*/
  EdgeInverseSim3ProjectXYZ::EdgeInverseSim3ProjectXYZ() :
  BaseBinaryEdge<2, Eigen::Vector2d, VertexSBAPointXYZ, VertexSim3Expmap>(){}

  bool EdgeInverseSim3ProjectXYZ::read(std::istream& is) {
    for (int i=0; i<2; i++) {
      is >> _measurement[i];
    }

    for (int i=0; i<2; i++) {
      for (int j=i; j<2; j++) {
        is >> information()(i,j);
        if (i!=j)
          information()(j,i)=information()(i,j);
      }
    }
    return true;
  }

  bool EdgeInverseSim3ProjectXYZ::write(std::ostream& os) const {
    for (int i=0; i<2; i++) {
      os  << _measurement[i] << " ";
    }
    for (int i=0; i<2; i++) {
      for (int j=i; j<2; j++) {
        os << " " <<  information()(i,j);
      }
    }
    return os.good();
  }

} // end namespace
