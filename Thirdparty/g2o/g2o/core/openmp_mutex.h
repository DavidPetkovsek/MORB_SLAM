// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
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

#ifndef G2O_OPENMP_MUTEX
#define G2O_OPENMP_MUTEX

#include "g2o/config.h"

#ifdef G2O_OPENMP
#include <omp.h>
#else
#include <cassert>
#endif

namespace g2o {

#ifdef G2O_OPENMP

  /**
   * \brief std::mutex realized via OpenMP
   */
  class OpenMPmutex
  {
    public:
      OpenMPmutex() { omp_init_lock(&_lock); }
      ~OpenMPmutex() { omp_destroy_lock(&_lock); }
      void lock() { omp_set_lock(&_lock); }
      void unlock() { omp_unset_lock(&_lock); }
    protected:
      omp_lock_t _lock;
  };

#else

  /*
   * dummy which does nothing in case we don't have OpenMP support.
   * In debug mode, the std::mutex allows to verify the correct lock and unlock behavior
   */
  class OpenMPmutex
  {
    public:
#ifdef NDEBUG
      OpenMPmutex() {}
#else
      OpenMPmutex() : _cnt(0) {}
#endif
      ~OpenMPmutex() { assert(_cnt == 0 && "Freeing locked std::mutex");}
      void lock() { assert(++_cnt == 1 && "Locking already locked std::mutex");}
      void unlock() { assert(--_cnt == 0 && "Trying to unlock a std::mutex which is not locked");}
    protected:
#ifndef NDEBUG
      char _cnt;
#endif
  };

#endif

  /**
   * \brief lock a std::mutex within a scope
   */
  class ScopedOpenMPmutex
  {
    public:
      explicit ScopedOpenMPmutex(OpenMPmutex* mutex) : _mutex(mutex) { _mutex->lock(); }
      ~ScopedOpenMPmutex() { _mutex->unlock(); }
    private:
      OpenMPmutex* const _mutex;
      ScopedOpenMPmutex(const ScopedOpenMPmutex&);
      void operator=(const ScopedOpenMPmutex&);
  };

}

#endif
