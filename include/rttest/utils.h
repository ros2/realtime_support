// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RTTEST_UTILS_H_
#define RTTEST_UTILS_H_

#include <time.h>

#define NSEC_PER_SEC 1000000000

static inline bool timespec_gt(const struct timespec *t1,
    const struct timespec *t2)
{
  if (t1->tv_sec > t2->tv_sec)
  {
    return true;
  }
  if (t1->tv_sec < t2->tv_sec)
  {
    return false;
  }
  return t1->tv_nsec > t2->tv_nsec;
}

static inline void normalize_timespec(struct timespec *t)
{
  // TODO: maybe could use some work
  while (t->tv_nsec >= NSEC_PER_SEC) {
    t->tv_nsec -= NSEC_PER_SEC;
    t->tv_sec++;
  }
}

static inline void add_timespecs(const struct timespec *t1,
    const struct timespec *t2,
    struct timespec *dst)
{
  dst->tv_sec = t1->tv_sec + t2->tv_sec;
  dst->tv_nsec = t1->tv_nsec + t2->tv_nsec;
  normalize_timespec(dst);
}

static inline bool subtract_timespecs(const struct timespec *t1,
    const struct timespec *t2,
    struct timespec *dst)
{
  if (timespec_gt(t2, t1))
  {
    return subtract_timespecs(t2, t1, dst);
  }

  dst->tv_sec = t1->tv_sec - t2->tv_sec;
  dst->tv_nsec = t1->tv_nsec - t2->tv_nsec;

  normalize_timespec(dst);
  return true;
}

static inline unsigned long timespec_to_long(const struct timespec *t)
{
  return t->tv_sec * NSEC_PER_SEC + t->tv_nsec;
}

static inline void long_to_timespec(const unsigned long input, struct timespec *t)
{
  //return t->tv_sec * NSEC_PER_SEC + t->tv_nsec;
  unsigned long nsecs = input % 1000000000;
  unsigned long secs = (input - nsecs) / 1000000000;
  t->tv_sec = secs;
  t->tv_nsec = nsecs;
}


#endif
