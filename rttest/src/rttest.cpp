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

#include <rttest/rttest.h>
#include <rttest/utils.h>

#include <limits.h>
#include <malloc.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <string.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

extern "C"
{
class rttest_sample_buffer
{
public:
  rttest_sample_buffer()
  : latency_samples(nullptr),
    major_pagefaults(nullptr),
    minor_pagefaults(nullptr),
    buffer_size(0)
  {}
  ~rttest_sample_buffer()
  {
    resize(0);
  }

  /// Free the current buffer and allocate a new buffer if new_buffer_size is greater than zero.
  void resize(size_t new_buffer_size)
  {
    if (new_buffer_size > 0) {
      resize(0);
      this->buffer_size = new_buffer_size;
      this->latency_samples = static_cast<int *>(
        std::malloc(new_buffer_size * sizeof(int)));
      if (!this->latency_samples) {
        fprintf(stderr, "Failed to allocate latency samples buffer\n");
        exit(-1);
      }
      memset(this->latency_samples, 0,
        new_buffer_size * sizeof(int));

      this->major_pagefaults = static_cast<size_t *>(
        std::malloc(new_buffer_size * sizeof(size_t)));
      if (!this->major_pagefaults) {
        fprintf(stderr, "Failed to allocate major pagefaults buffer\n");
        exit(-1);
      }
      memset(this->major_pagefaults, 0,
        new_buffer_size * sizeof(size_t));

      this->minor_pagefaults = static_cast<size_t *>(
        std::malloc(new_buffer_size * sizeof(size_t)));
      if (!this->minor_pagefaults) {
        fprintf(stderr, "Failed to allocate minor pagefaults buffer\n");
        exit(-1);
      }
      memset(this->minor_pagefaults, 0,
        new_buffer_size * sizeof(size_t));
    } else {
      if (this->latency_samples) {
        free(this->latency_samples);
        this->latency_samples = nullptr;
      }
      if (this->major_pagefaults) {
        free(this->major_pagefaults);
        this->major_pagefaults = nullptr;
      }
      if (this->minor_pagefaults) {
        free(this->minor_pagefaults);
        this->minor_pagefaults = nullptr;
      }
      this->buffer_size = 0;
    }
  }
  rttest_sample_buffer(const rttest_sample_buffer & other)
  : rttest_sample_buffer()
  {
    *this = other;
  }
  void operator=(const rttest_sample_buffer & other)
  {
    resize(other.buffer_size);
    memcpy(this->latency_samples, other.latency_samples, this->buffer_size * sizeof(int));
    memcpy(this->major_pagefaults, other.major_pagefaults, this->buffer_size * sizeof(size_t));
    memcpy(this->minor_pagefaults, other.minor_pagefaults, this->buffer_size * sizeof(size_t));
  }

  // Stored in nanoseconds
  // A negative latency means that the event was early (unlikely)
  int * latency_samples;

  size_t * major_pagefaults;
  size_t * minor_pagefaults;

  size_t buffer_size;
};

class Rttest
{
private:
  struct rttest_params params;
  struct rttest_sample_buffer sample_buffer;
  struct rusage prev_usage;

  pthread_t thread_id;

  int record_jitter(
    const struct timespec * deadline,
    const struct timespec * result_time, const size_t iteration);

  int accumulate_statistics(size_t iteration);

public:
  int running = 0;
  struct rttest_results results;

  Rttest();
  ~Rttest();

  int read_args(int argc, char ** argv);

  int init(
    size_t iterations, struct timespec update_period,
    size_t sched_policy, int sched_priority, size_t stack_size,
    char * filename);

  int spin(void *(*user_function)(void *), void * args);

  int spin_period(
    void *(*user_function)(void *), void * args,
    const struct timespec * update_period, const size_t iterations);

  int spin_once(
    void *(*user_function)(void *), void * args,
    const struct timespec * start_time, const size_t i);

  int spin_once(
    void *(*user_function)(void *), void * args,
    const struct timespec * start_time,
    const struct timespec * update_period, const size_t i);

  int lock_memory();

  int lock_and_prefault_dynamic();

  int prefault_stack();

  int set_thread_default_priority();

  int get_next_rusage(size_t i);

  int calculate_statistics(struct rttest_results * results);

  int get_sample_at(const size_t iteration, int & sample) const;

  int write_results();

  int write_results_file(char * filename);

  std::string results_to_string(char * name);

  int finish();

  struct rttest_params * get_params();

  void set_params(struct rttest_params * params);

  void initialize_dynamic_memory();
};

// Global variables, for tracking threads
std::map<pthread_t, Rttest> rttest_instance_map;
pthread_t initial_thread_id = 0;

Rttest::Rttest()
{
  memset(&this->results, 0, sizeof(struct rttest_results));
  this->results.min_latency = INT_MAX;
  this->results.max_latency = INT_MIN;
}

Rttest::~Rttest()
{}

// Functions
void Rttest::set_params(struct rttest_params * params)
{
  this->params = *params;
}

struct rttest_params * Rttest::get_params()
{
  return &(this->params);
}

int Rttest::record_jitter(
  const struct timespec * deadline,
  const struct timespec * result_time, const size_t iteration)
{
  size_t i = iteration;
  // Check if settings authorize buffer recording
  if (this->params.iterations == 0) {
    i = 0;
  }
  struct timespec jitter;
  int parity = 1;
  if (timespec_gt(result_time, deadline)) {
    // missed a deadline
    subtract_timespecs(result_time, deadline, &jitter);
  } else {
    subtract_timespecs(deadline, result_time, &jitter);
    parity = -1;
  }
  // Record jitter
  if (i >= this->sample_buffer.buffer_size) {
    return -1;
  }
  this->sample_buffer.latency_samples[i] = parity * timespec_to_long(&jitter);
  return 0;
}


Rttest * get_rttest_thread_instance(pthread_t thread_id)
{
  if (rttest_instance_map.count(thread_id) == 0) {
    return NULL;
  }
  return &rttest_instance_map[thread_id];
}

int Rttest::read_args(int argc, char ** argv)
{
  // -i,--iterations
  size_t iterations = 1000;
  // -u,--update-period
  struct timespec update_period;
  update_period.tv_sec = 0;
  update_period.tv_nsec = 1000000;
  // -t,--thread-priority
  int sched_priority = 80;
  // -s,--sched-policy
  size_t sched_policy = SCHED_RR;
  // -m,--memory-size
  size_t stack_size = 1024 * 1024;
  // -f,--filename
  // Don't write a file unless filename specified
  char * filename = nullptr;
  int index;
  int c;

  std::string args_string = "i:u:p:t:s:m:f:r:";
  opterr = 0;
  optind = 1;

  while ((c = getopt(argc, argv, args_string.c_str())) != -1) {
    switch (c) {
      case 'i':
        {
          int arg = atoi(optarg);
          if (arg < 0) {
            iterations = 0;
          } else {
            iterations = arg;
          }
          break;
        }
      case 'u':
        {
          // parse units
          size_t nsec;
          std::string input(optarg);
          std::vector<std::string> tokens = {"ns", "us", "ms", "s"};
          for (size_t i = 0; i < 4; ++i) {
            size_t idx = input.find(tokens[i]);
            if (idx != std::string::npos) {
              nsec = stol(input.substr(0, idx)) * pow(10, i * 3);
              break;
            }
            if (i == 3) {
              // Default units are microseconds
              nsec = stol(input) * 1000;
            }
          }

          long_to_timespec(nsec, &update_period);
        }
        break;
      case 't':
        sched_priority = atoi(optarg);
        break;
      case 's':
        {
          std::string input(optarg);
          if (input == "fifo") {
            sched_policy = SCHED_FIFO;
          } else if (input == "rr") {
            sched_policy = SCHED_RR;
          } else {
            fprintf(stderr, "Invalid option entered for scheduling policy: %s\n",
              input.c_str());
            fprintf(stderr, "Valid options are: fifo, rr\n");
            exit(-1);
          }
        }
        break;
      case 'm':
        {
          // parse units
          std::string input(optarg);
          std::vector<std::string> tokens = {"b", "kb", "mb", "gb"};
          for (size_t i = 0; i < 4; ++i) {
            size_t idx = input.find(tokens[i]);
            if (idx != std::string::npos) {
              stack_size = stoi(input.substr(0, idx)) * pow(2, i * 10);
              break;
            }
            if (i == 3) {
              // Default units are megabytes
              stack_size = std::stoi(input) * pow(2, 20);
            }
          }
        }
        break;
      case 'f':
        filename = optarg;
        break;
      case '?':
        if (args_string.find(optopt) != std::string::npos) {
          fprintf(stderr, "Option -%c requires an argument.\n", optopt);
        } else if (isprint(optopt)) {
          fprintf(stderr, "Unknown option `-%c'.\n", optopt);
        } else {
          fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
        }
        break;
      default:
        exit(-1);
    }
  }

  return this->init(
    iterations, update_period, sched_policy, sched_priority, stack_size, filename);
}

int rttest_get_params(struct rttest_params * params_in)
{
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());

  if (!thread_rttest_instance) {
    return -1;
  }

  if (params_in == NULL) {
    params_in = thread_rttest_instance->get_params();
  } else {
    *params_in = *thread_rttest_instance->get_params();
  }

  return 0;
}

int rttest_init_new_thread()
{
  auto thread_id = pthread_self();
  auto thread_rttest_instance = get_rttest_thread_instance(thread_id);
  if (thread_rttest_instance == nullptr) {
    // Create the new Rttest instance for this thread
    rttest_instance_map.emplace(std::make_pair(thread_id, Rttest()));
  } else {
    fprintf(stderr, "rttest instance for %lu already exists!\n", thread_id);
    return -1;
  }
  if (initial_thread_id == 0 || rttest_instance_map.count(initial_thread_id) == 0) {
    return -1;
  }
  rttest_instance_map[thread_id].set_params(
    rttest_instance_map[initial_thread_id].get_params());
  rttest_instance_map[thread_id].initialize_dynamic_memory();
  return 0;
}

int rttest_read_args(int argc, char ** argv)
{
  auto thread_id = pthread_self();
  auto thread_rttest_instance = get_rttest_thread_instance(thread_id);
  if (!thread_rttest_instance) {
    // Create the new Rttest instance for this thread
    rttest_instance_map.emplace(std::make_pair(thread_id, Rttest()));
    if (rttest_instance_map.size() == 1 && initial_thread_id == 0) {
      initial_thread_id = thread_id;
    }
    thread_rttest_instance = &rttest_instance_map[thread_id];
  }
  return thread_rttest_instance->read_args(argc, argv);
}

int Rttest::init(
  size_t iterations, struct timespec update_period,
  size_t sched_policy, int sched_priority, size_t stack_size,
  char * filename)
{
  this->params.iterations = iterations;
  this->params.update_period = update_period;
  this->params.sched_policy = sched_policy;
  this->params.sched_priority = sched_priority;
  this->params.stack_size = stack_size;

  if (filename != nullptr) {
    size_t n = strlen(filename);
    this->params.filename = static_cast<char *>(std::malloc(n * sizeof(char) + 1));
    if (!this->params.filename) {
      fprintf(stderr, "Failed to allocate major pagefaults buffer\n");
      return -1;
    }
    this->params.filename[n] = 0;
    strncpy(this->params.filename, filename, n);
    fprintf(stderr, "Writing results to file: %s\n", this->params.filename);
  } else {
    this->params.filename = nullptr;
  }

  this->initialize_dynamic_memory();
  this->running = 1;
  return 0;
}

void Rttest::initialize_dynamic_memory()
{
  size_t iterations = this->params.iterations;
  if (iterations == 0) {
    // Allocate a sample buffer of size 1
    iterations = 1;
  }
  this->sample_buffer.resize(iterations);
}

int rttest_init(
  size_t iterations, struct timespec update_period,
  size_t sched_policy, int sched_priority, size_t stack_size,
  char * filename)
{
  auto thread_id = pthread_self();
  auto thread_rttest_instance = get_rttest_thread_instance(thread_id);
  if (thread_rttest_instance == nullptr) {
    // Create the new Rttest instance for this thread
    std::pair<pthread_t, Rttest> instance;
    instance.first = thread_id;
    rttest_instance_map.emplace(instance);
    if (rttest_instance_map.size() == 1 && initial_thread_id == 0) {
      initial_thread_id = thread_id;
    }
    thread_rttest_instance = &rttest_instance_map[thread_id];
  }
  return thread_rttest_instance->init(iterations, update_period,
           sched_policy, sched_priority, stack_size, filename);
}

int Rttest::get_next_rusage(size_t i)
{
  size_t prev_maj_pagefaults = this->prev_usage.ru_majflt;
  size_t prev_min_pagefaults = this->prev_usage.ru_minflt;
  if (getrusage(RUSAGE_THREAD, &this->prev_usage) != 0) {
    return -1;
  }
  assert(this->prev_usage.ru_majflt >= prev_maj_pagefaults);
  assert(this->prev_usage.ru_minflt >= prev_min_pagefaults);
  if (this->params.iterations == 0) {
    i = 0;
  }
  this->sample_buffer.major_pagefaults[i] =
    this->prev_usage.ru_majflt - prev_maj_pagefaults;
  this->sample_buffer.minor_pagefaults[i] =
    this->prev_usage.ru_minflt - prev_min_pagefaults;
  return 0;
}

int rttest_get_next_rusage(size_t i)
{
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  if (!thread_rttest_instance) {
    return -1;
  }
  return thread_rttest_instance->get_next_rusage(i);
}

int rttest_spin(void *(*user_function)(void *), void * args)
{
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  if (!thread_rttest_instance) {
    return -1;
  }
  return thread_rttest_instance->spin(user_function, args);
}

int rttest_spin_once_period(
  void *(*user_function)(void *), void * args,
  const struct timespec * start_time,
  const struct timespec * update_period, const size_t i)
{
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  if (!thread_rttest_instance) {
    return -1;
  }
  return thread_rttest_instance->spin_once(user_function, args, start_time, update_period, i);
}

int rttest_spin_once(
  void *(*user_function)(void *), void * args,
  const struct timespec * start_time, const size_t i)
{
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  if (!thread_rttest_instance) {
    return -1;
  }
  return thread_rttest_instance->spin_once(user_function, args, start_time, i);
}

int Rttest::spin(void *(*user_function)(void *), void * args)
{
  return rttest_spin_period(user_function, args, &this->params.update_period,
           this->params.iterations);
}

int Rttest::spin_period(
  void *(*user_function)(void *), void * args,
  const struct timespec * update_period, const size_t iterations)
{
  struct timespec start_time;
  clock_gettime(CLOCK_MONOTONIC, &start_time);

  if (iterations == 0) {
    size_t i = 0;
    while (this->running != 0) {
      if (spin_once(user_function, args, &start_time, update_period, i) != 0) {
        throw std::runtime_error("error in spin_once");
      }
      ++i;
    }
  } else {
    for (size_t i = 0; i < iterations; i++) {
      if (spin_once(user_function, args, &start_time, update_period, i) != 0) {
        throw std::runtime_error("error in spin_once");
      }
    }
  }

  return 0;
}

int Rttest::spin_once(
  void *(*user_function)(void *), void * args,
  const struct timespec * start_time, const size_t i)
{
  return this->spin_once(user_function, args, start_time, &this->params.update_period, i);
}

int Rttest::spin_once(
  void *(*user_function)(void *), void * args,
  const struct timespec * start_time,
  const struct timespec * update_period, const size_t i)
{
  if (!start_time || !update_period || (i > params.iterations && params.iterations > 0)) {
    return -1;
  }
  if (i == 0) {
    if (getrusage(RUSAGE_THREAD, &this->prev_usage) != 0) {
      return -1;
    }
    printf("Initial major pagefaults: %zu\n", this->prev_usage.ru_majflt);
    printf("Initial minor pagefaults: %zu\n", this->prev_usage.ru_minflt);
  }
  struct timespec wakeup_time, current_time;
  multiply_timespec(update_period, i, &wakeup_time);
  add_timespecs(start_time, &wakeup_time, &wakeup_time);
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  this->record_jitter(&wakeup_time, &current_time, i);

  user_function(args);
  this->get_next_rusage(i);
  this->accumulate_statistics(i);
  return 0;
}

int rttest_spin_period(
  void *(*user_function)(void *), void * args,
  const struct timespec * update_period, const size_t iterations)
{
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  if (!thread_rttest_instance) {
    return -1;
  }
  return thread_rttest_instance->spin_period(user_function, args, update_period, iterations);
}

int rttest_lock_memory()
{
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  if (!thread_rttest_instance) {
    return -1;
  }
  return thread_rttest_instance->lock_memory();
}

int Rttest::lock_memory()
{
  return mlockall(MCL_CURRENT | MCL_FUTURE);
}

int rttest_lock_and_prefault_dynamic()
{
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  if (!thread_rttest_instance) {
    return -1;
  }
  return thread_rttest_instance->lock_and_prefault_dynamic();
}

int Rttest::lock_and_prefault_dynamic()
{
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    perror("mlockall failed");
    return -1;
  }

  // Turn off malloc trimming.
  if (mallopt(M_TRIM_THRESHOLD, -1) == 0) {
    perror("mallopt for trim threshold failed");
    munlockall();
    return -1;
  }

  // Turn off mmap usage.
  if (mallopt(M_MMAP_MAX, 0) == 0) {
    perror("mallopt for mmap failed");
    mallopt(M_TRIM_THRESHOLD, 128 * 1024);
    munlockall();
    return -1;
  }

  struct rusage usage;
  size_t page_size = sysconf(_SC_PAGESIZE);
  getrusage(RUSAGE_SELF, &usage);
  std::vector<char *> prefaulter;
  size_t prev_minflts = usage.ru_minflt;
  size_t prev_majflts = usage.ru_majflt;
  size_t encountered_minflts = 1;
  size_t encountered_majflts = 1;
  // prefault until you see no more pagefaults
  while (encountered_minflts > 0 || encountered_majflts > 0) {
    char * ptr;
    try {
      ptr = new char[64 * page_size];
      memset(ptr, 0, 64 * page_size);
    } catch (std::bad_alloc & e) {
      fprintf(stderr, "Caught exception: %s\n", e.what());
      fprintf(stderr, "Unlocking memory and continuing.\n");
      for (auto & ptr : prefaulter) {
        delete[] ptr;
      }

      mallopt(M_TRIM_THRESHOLD, 128 * 1024);
      mallopt(M_MMAP_MAX, 65536);
      munlockall();
      return -1;
    }
    prefaulter.push_back(ptr);
    getrusage(RUSAGE_SELF, &usage);
    size_t current_minflt = usage.ru_minflt;
    size_t current_majflt = usage.ru_majflt;
    encountered_minflts = current_minflt - prev_minflts;
    encountered_majflts = current_majflt - prev_majflts;
    prev_minflts = current_minflt;
    prev_majflts = current_majflt;
  }

  for (auto & ptr : prefaulter) {
    delete[] ptr;
  }
  return 0;
}

int rttest_prefault_stack_size(const size_t stack_size)
{
  unsigned char stack[stack_size];
  memset(stack, 0, stack_size);
  return 0;
}

int rttest_prefault_stack()
{
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  if (!thread_rttest_instance) {
    return -1;
  }
  return rttest_prefault_stack_size(thread_rttest_instance->get_params()->stack_size);
}

int rttest_set_thread_default_priority()
{
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  if (!thread_rttest_instance) {
    return -1;
  }
  return rttest_set_sched_priority(
    thread_rttest_instance->get_params()->sched_priority,
    thread_rttest_instance->get_params()->sched_policy);
}

int rttest_set_sched_priority(size_t sched_priority, int policy)
{
  struct sched_param param;

  param.sched_priority = sched_priority;

  // note that sched_setscheduler can set the priority of an arbitrary process
  return sched_setscheduler(0, policy, &param);
}

int Rttest::accumulate_statistics(size_t iteration)
{
  size_t i = iteration;
  this->results.iteration = iteration;
  if (params.iterations == 0) {
    i = 0;
  } else if (iteration > params.iterations) {
    return -1;
  }
  int latency = sample_buffer.latency_samples[i];
  if (latency > this->results.max_latency) {
    this->results.max_latency = latency;
  }
  if (latency < this->results.min_latency) {
    this->results.min_latency = latency;
  }

  if (iteration > 0) {
    // Accumulate the mean
    this->results.mean_latency = this->results.mean_latency +
      (sample_buffer.latency_samples[i] - this->results.mean_latency) / (iteration + 1);
  } else {
    // Initialize the mean
    this->results.mean_latency = sample_buffer.latency_samples[i];
  }
  this->results.minor_pagefaults += sample_buffer.minor_pagefaults[i];
  this->results.major_pagefaults += sample_buffer.major_pagefaults[i];
  return 0;
}

int Rttest::calculate_statistics(struct rttest_results * output)
{
  if (output == NULL) {
    fprintf(stderr, "Need to allocate rttest_results struct\n");
    return -1;
  }
  if (this->sample_buffer.latency_samples == NULL) {
    fprintf(stderr, "Pointer to latency samples was NULL\n");
    return -1;
  }
  if (this->sample_buffer.minor_pagefaults == NULL) {
    fprintf(stderr, "Pointer to minor pagefaults was NULL\n");
    return -1;
  }
  if (this->sample_buffer.major_pagefaults == NULL) {
    fprintf(stderr, "Pointer to major pagefaults was NULL\n");
    return -1;
  }

  std::vector<int> latency_dataset;
  latency_dataset.assign(this->sample_buffer.latency_samples,
    this->sample_buffer.latency_samples + this->sample_buffer.buffer_size);

  output->min_latency = *std::min_element(latency_dataset.begin(),
      latency_dataset.end());
  output->max_latency = *std::max_element(latency_dataset.begin(),
      latency_dataset.end());
  output->mean_latency = std::accumulate(latency_dataset.begin(),
      latency_dataset.end(), 0.0) / latency_dataset.size();

  // Calculate standard deviation and try to avoid overflow
  std::vector<int> latency_diff(latency_dataset.size());
  std::transform(latency_dataset.begin(), latency_dataset.end(), latency_diff.begin(),
    std::bind2nd(std::minus<int>(), output->mean_latency));
  int sq_sum = std::inner_product(latency_diff.begin(), latency_diff.end(),
      latency_diff.begin(), 0);
  output->latency_stddev = std::sqrt(sq_sum / latency_dataset.size());

  std::vector<size_t> min_pagefaults;
  min_pagefaults.assign(this->sample_buffer.minor_pagefaults,
    this->sample_buffer.minor_pagefaults + this->sample_buffer.buffer_size);
  output->minor_pagefaults = std::accumulate(min_pagefaults.begin(), min_pagefaults.end(), 0);

  std::vector<size_t> maj_pagefaults;
  maj_pagefaults.assign(this->sample_buffer.major_pagefaults,
    this->sample_buffer.major_pagefaults + this->sample_buffer.buffer_size);
  output->major_pagefaults = std::accumulate(maj_pagefaults.begin(), maj_pagefaults.end(), 0);
  return 0;
}

int rttest_calculate_statistics(struct rttest_results * results)
{
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  if (!thread_rttest_instance) {
    return -1;
  }
  return thread_rttest_instance->calculate_statistics(results);
}

int rttest_get_statistics(struct rttest_results * output)
{
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  if (!thread_rttest_instance) {
    return -1;
  }
  if (output == NULL) {
    output = &thread_rttest_instance->results;
  } else {
    // if output is not null, try to copy the results struct into the memory location
    *output = thread_rttest_instance->results;
  }

  return 0;
}

int Rttest::get_sample_at(const size_t iteration, int & sample) const
{
  if (this->params.iterations == 0) {
    sample = this->sample_buffer.latency_samples[0];
    return 0;
  }
  if (iteration < this->params.iterations) {
    sample = this->sample_buffer.latency_samples[iteration];
    return 0;
  }
  return -1;
}

int rttest_get_sample_at(const size_t iteration, int * sample)
{
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  if (!thread_rttest_instance) {
    return -1;
  }
  if (sample == NULL) {
    return -1;
  }
  return thread_rttest_instance->get_sample_at(iteration, *sample);
}

std::string Rttest::results_to_string(char * name)
{
  std::stringstream sstring;

  sstring << "rttest statistics";
  if (name != NULL) {
    sstring << " for " << name << ":" << std::endl;
  } else {
    sstring << ":" << std::endl;
  }
  sstring << "  - Minor pagefaults: " << results.minor_pagefaults << std::endl;
  sstring << "  - Major pagefaults: " << results.major_pagefaults << std::endl;
  sstring << "  Latency (time after deadline was missed):" << std::endl;
  sstring << "    - Min: " << results.min_latency << " ns" << std::endl;
  sstring << "    - Max: " << results.max_latency << " ns" << std::endl;
  sstring << "    - Mean: " << results.mean_latency << " ns" << std::endl;
  sstring << "    - Standard deviation: " << results.latency_stddev << std::endl;
  sstring << std::endl;

  return sstring.str();
}

int rttest_finish()
{
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  if (!thread_rttest_instance) {
    return -1;
  }
  int status = thread_rttest_instance->finish();

  rttest_instance_map.erase(pthread_self());

  return status;
}

int Rttest::finish()
{
  this->running = 0;
  munlockall();

  // Print statistics to screen
  this->calculate_statistics(&this->results);
  printf("%s\n", this->results_to_string(this->params.filename).c_str());
  free(this->params.filename);

  return 0;
}

int rttest_write_results_file(char * filename)
{
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  if (!thread_rttest_instance) {
    return -1;
  }
  return thread_rttest_instance->write_results_file(filename);
}

int rttest_write_results()
{
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  if (!thread_rttest_instance) {
    return -1;
  }
  return thread_rttest_instance->write_results();
}

int Rttest::write_results()
{
  return this->write_results_file(this->params.filename);
}

int Rttest::write_results_file(char * filename)
{
  if (this->params.iterations == 0) {
    fprintf(stderr, "No sample buffer was saved, not writing results\n");
    return -1;
  }
  if (filename == NULL) {
    fprintf(stderr, "No results filename given, not writing results\n");
    return -1;
  }

  if (this->sample_buffer.latency_samples == NULL) {
    fprintf(stderr, "Sample buffer was NULL, not writing results\n");
    return -1;
  }
  if (this->sample_buffer.minor_pagefaults == NULL) {
    fprintf(stderr, "Sample buffer was NULL, not writing results\n");
    return -1;
  }
  if (this->sample_buffer.major_pagefaults == NULL) {
    fprintf(stderr, "Sample buffer was NULL, not writing results\n");
    return -1;
  }

  std::ofstream fstream(filename, std::ios::out);

  if (!fstream.is_open()) {
    fprintf(stderr, "Couldn't open file %s, not writing results\n", filename);
    return -1;
  }

  fstream << "iteration timestamp latency minor_pagefaults minor_pagefaults" << std::endl;
  for (size_t i = 0; i < this->sample_buffer.buffer_size; ++i) {
    fstream << i << " " << timespec_to_long(&this->params.update_period) * i <<
      " " << this->sample_buffer.latency_samples[i] << " " <<
      this->sample_buffer.minor_pagefaults[i] << " " <<
      this->sample_buffer.major_pagefaults[i] << std::endl;
  }

  fstream.close();

  return 0;
}

int rttest_running()
{
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  if (!thread_rttest_instance) {
    return 0;
  }
  return thread_rttest_instance->running;
}
}
