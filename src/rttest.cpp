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

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <numeric>
#include <fstream>
#include <malloc.h>
#include <map>
#include <sched.h>
#include <string.h>
#include <sstream>
#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>
#include <vector>

#include <utils.h>
#include <rttest.h>

extern "C"
{

  struct rttest_sample_buffer
  {
    // Stored in nanoseconds
    // A negative latency means that the event was early (unlikely/impossible)
    int *latency_samples;

    unsigned int *major_pagefaults;
    unsigned int *minor_pagefaults;

    unsigned int buffer_size;
  };

  class Rttest
  {
    private:
      struct rttest_params params;
      struct rttest_sample_buffer sample_buffer;
      struct rttest_results results;
      struct rusage prev_usage;

      pthread_t thread_id;

      int record_jitter(const struct timespec *deadline,
          const struct timespec *result_time, const unsigned int iteration);


    public:
      int read_args(int argc, char** argv);

      int init(unsigned int iterations, struct timespec update_period,
          size_t sched_policy, int sched_priority, int lock_memory, size_t stack_size,
          int plot, int write, char *filename, unsigned int repetitions);

      int spin(void *(*user_function)(void *), void *args);

      int spin_period(void *(*user_function)(void *), void *args,
          const struct timespec *update_period, const unsigned int iterations);


      int lock_memory();

      int prefault_stack();

      int set_thread_default_priority();

      int get_next_rusage(unsigned int i);

      int calculate_statistics(struct rttest_results *results);

      int write_results();

      int write_results_file(char *filename);

      int finish();
  };

  std::map<pthread_t, Rttest*> rttest_instance_map;

  int Rttest::record_jitter(const struct timespec *deadline,
      const struct timespec *result_time, const unsigned int iteration)
  {
    struct timespec jitter;
    int parity = 1;
    if (timespec_gt(result_time, deadline))
    {
      // missed a deadline
      subtract_timespecs(result_time, deadline, &jitter);
    }
    else
    {
      subtract_timespecs(deadline, result_time, &jitter);
      parity = -1;
    }
    // Record jitter
    if (iteration > this->sample_buffer.buffer_size)
    {
      return -1;
    }
    this->sample_buffer.latency_samples[iteration] = parity*timespec_to_long(&jitter);
    return 0;
  }


  Rttest *get_rttest_thread_instance(pthread_t thread_id)
  {
    if (rttest_instance_map.count(thread_id) == 0)
    {
      return NULL;
    }
    return rttest_instance_map[thread_id];
  }

  int Rttest::read_args(int argc, char** argv)
  {
    //parse arguments
    // -i,--iterations
    unsigned int iterations = 1000;
    // -u,--update-period
    struct timespec update_period;
    update_period.tv_sec = 0;
    update_period.tv_nsec = 1000000;
    // -p,--plot
    int plot = 0;
    // -t,--thread-priority
    int sched_priority = 80;
    // -s,--sched-policy
    size_t sched_policy = SCHED_RR;
    // -m,--memory-size
    // Don't lock memory unless stack size specified
    int lock_memory = 0;
    size_t stack_size = 1024*1024;
    // -f,--filename
    // Don't write a file unless filename specified
    int write = 0;
    char *filename;
    // -r, --repeat
    unsigned int repetitions = 1;
    int index;
    int c;

    std::string args_string = "i:u:p:t:s:m:f:r:";
    opterr = 0;

    while ((c = getopt(argc, argv, args_string.c_str())) != -1)
    {
      switch(c)
      {
        case 'i':
          iterations = atoi(optarg);
          break;
        case 'u':
          {
            // parse units
            unsigned int nsec;
            std::string input(optarg);
            std::vector<std::string> tokens = {"ns", "us", "ms", "s"};
            for (unsigned int i = 0; i < 4; ++i)
            {
              size_t idx = input.find(tokens[i]);
              if (idx != std::string::npos)
              {
                nsec = stol(input.substr(0, idx)) * pow(10, i*3);
                break;
              }
              if (i == 3)
              {
                // Default units are microseconds
                nsec = stol(input)*1000;
              }
            }

            long_to_timespec(nsec, &update_period);
          }
          break;
        case 'p':
          plot = 1;
          break;
        case 't':
          sched_priority = atoi(optarg);
          break;
        case 's':
          {
            // translate string to number. is there a utility for this?
            std::string input(optarg);
            if (input == "fifo")
            {
              sched_policy = SCHED_FIFO;
            }
            else if (input == "rr")
            {
              sched_policy = SCHED_RR;
            }
            else
            {
              fprintf(stderr, "Invalid option entered for scheduling policy: %s\n",
                     input.c_str());
              fprintf(stderr, "Valid options are: fifo, rr\n");
              exit(-1);
            }
          }
          break;
        case 'm':
          {
            lock_memory = 1;
            // parse units
            std::string input(optarg);
            std::vector<std::string> tokens = {"b", "kb", "mb", "gb"};
            for (unsigned int i = 0; i < 4; ++i)
            {
              size_t idx = input.find(tokens[i]);
              if (idx != std::string::npos)
              {
                stack_size = stoi(input.substr(0, idx)) * pow(2, i*10);
                break;
              }
              if (i == 3)
              {
                // Default units are megabytes
                stack_size = std::stoi(input) * pow(2, 20);
              }
            }
          }
          break;
        case 'f':
          filename = optarg;
          fprintf(stderr, "Writing results to file: %s\n", filename);
          write = 1;
          // check if file exists
          break;
        case 'r':
          repetitions = atoi(optarg);
          break;
        case '?':
          if (args_string.find(optopt) != std::string::npos)
            fprintf(stderr, "Option -%c requires an argument.\n", optopt);
          else if (isprint(optopt))
            fprintf(stderr, "Unknown option `-%c'.\n", optopt);
          else
            fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
          break;
        default:
          exit(-1);
      }
    }

    this->init(iterations, update_period, sched_policy, sched_priority,
        lock_memory, stack_size, plot, write, filename, repetitions);
  }

  int rttest_read_args(int argc, char** argv)
  {
    auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
    if (!thread_rttest_instance)
      return -1;
    return thread_rttest_instance->read_args(argc, argv);
  }

  int Rttest::init(unsigned int iterations, struct timespec update_period,
      size_t sched_policy, int sched_priority, int lock_memory, size_t stack_size,
      int plot, int write, char *filename, unsigned int repetitions)
  {
    this->params.iterations = iterations;
    this->params.update_period = update_period;
    this->params.sched_policy = sched_policy;
    this->params.sched_priority = sched_priority;
    this->params.lock_memory = lock_memory;
    this->params.stack_size = stack_size;
    this->params.plot = plot;
    this->params.write = write;

    this->params.filename = filename;
    this->params.reps = repetitions;

    this->sample_buffer.buffer_size = iterations;

     this->sample_buffer.latency_samples =
        (int *) std::malloc(iterations*sizeof(int));
    memset(this->sample_buffer.latency_samples, 0,
        iterations*sizeof(int));

    this->sample_buffer.minor_pagefaults =
        (unsigned int *) std::malloc(iterations*sizeof(unsigned int));
    memset(this->sample_buffer.minor_pagefaults, 0,
           iterations*sizeof(unsigned int));

    this->sample_buffer.major_pagefaults =
        (unsigned int *) std::malloc(iterations*sizeof(unsigned int));
    memset(this->sample_buffer.major_pagefaults, 0,
           iterations*sizeof(unsigned int));

    this->sample_buffer.buffer_size = iterations;

    return 0;
  }

  int rttest_init(unsigned int iterations, struct timespec update_period,
      size_t sched_policy, int sched_priority, int lock_memory, size_t stack_size,
      int plot, int write, char *filename, unsigned int repetitions)
  {
    auto thread_id = pthread_self();
    auto thread_rttest_instance = get_rttest_thread_instance(thread_id);
    if (thread_rttest_instance == nullptr)
    {
      // Create the new Rttest instance for this thread
      rttest_instance_map[thread_id] = new Rttest;
    }
    return thread_rttest_instance->init(iterations, update_period,
      sched_policy, sched_priority, lock_memory, stack_size,
      plot, write, filename, repetitions);
  }

  int Rttest::get_next_rusage(unsigned int i)
  {
    long prev_maj_pagefaults = this->prev_usage.ru_majflt;
    long prev_min_pagefaults = this->prev_usage.ru_minflt;
    if (getrusage(RUSAGE_THREAD, &this->prev_usage) != 0)
    {
      return -1;
    }
    assert(this->prev_usage.ru_majflt >= prev_maj_pagefaults);
    assert(this->prev_usage.ru_minflt >= prev_min_pagefaults);
    this->sample_buffer.major_pagefaults[i] =
        this->prev_usage.ru_majflt - prev_maj_pagefaults;
    this->sample_buffer.minor_pagefaults[i] =
        this->prev_usage.ru_minflt - prev_min_pagefaults;
    return 0;
  }

  int rttest_get_next_rusage(unsigned int i)
  {
    auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
    if (!thread_rttest_instance)
      return -1;
    return thread_rttest_instance->get_next_rusage(i);
  }

  int rttest_spin(void *(*user_function)(void *), void *args)
  {
    auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
    if (!thread_rttest_instance)
      return -1;
    return thread_rttest_instance->spin(user_function, args);
  }

  int Rttest::spin(void *(*user_function)(void *), void *args)
  {
    return rttest_spin_period(user_function, args, &this->params.update_period,
        this->params.iterations);
  }

  int Rttest::spin_period(void *(*user_function)(void *), void *args,
      const struct timespec *update_period, const unsigned int iterations)
  {
    struct timespec wakeup_time, current_time;
    clock_gettime(0, &current_time);
    wakeup_time = current_time;

    // until there's more sophisticated threading logic in place, just
    // getrusage for the whole process
    if (getrusage(RUSAGE_THREAD, &this->prev_usage) != 0)
    {
      return -1;
    }
    std::cout << "Initial major pagefaults: " << this->prev_usage.ru_majflt << std::endl;
    std::cout << "Initial minor pagefaults: " << this->prev_usage.ru_minflt << std::endl;

    for (unsigned int i = 0; i < iterations; i++)
    {
      // Plan the next shot
      // This seems vulnerable to clock drift
      add_timespecs(&wakeup_time, update_period, &wakeup_time);
      clock_gettime(0, &current_time);
      if (timespec_gt(&current_time, &wakeup_time))
      {
        // Missed a deadline before we could sleep! Record it
        std::cout << "Clock drift detected" << std::endl;
        this->record_jitter(&wakeup_time, &current_time, i);
      }
      else
      {
        clock_nanosleep(0, TIMER_ABSTIME, &wakeup_time, NULL);
        clock_gettime(0, &current_time);

        this->record_jitter(&wakeup_time, &current_time, i);
      }

      user_function(args);
      this->get_next_rusage(i);
    }

    return 0;
  }

  int rttest_spin_period(void *(*user_function)(void *), void *args,
      const struct timespec *update_period, const unsigned int iterations)
  {
    auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
    if (!thread_rttest_instance)
      return -1;
    return thread_rttest_instance->spin_period(user_function, args, update_period, iterations);
  }

  /*
  int rttest_schedule_wakeup(void *(*user_function)(void *), void *args,
      const struct timespec *absolute_wakeup)
  {
    struct timespec current_time;
    clock_gettime(0, &current_time);
    if (timespec_gt(&current_time, absolute_wakeup))
    {
      // Missed a deadline before we could sleep! Record it
      std::cout << "clock drift detected" << std::endl;
      rttest_record_jitter(absolute_wakeup, &current_time, -1);
    }
    else
    {
      clock_nanosleep(0, TIMER_ABSTIME, absolute_wakeup, NULL);
      clock_gettime(0, &current_time);

      rttest_record_jitter(absolute_wakeup, &current_time, -1);
    }

    user_function(args);
    rttest_get_next_rusage(-1);

    return 0;
  }
  */

  int rttest_lock_memory()
  {
    return mlockall(MCL_CURRENT | MCL_FUTURE);
  }

  int rttest_lock_and_prefault_dynamic(const size_t pool_size)
  {
    int ret;
    if ((ret = mlockall(MCL_CURRENT | MCL_FUTURE )) != 0)
    {
      return ret;
    }

    // Turn off malloc trimming.
    mallopt (M_TRIM_THRESHOLD, -1);

    // Turn off mmap usage.
    mallopt (M_MMAP_MAX, 0);

    int page_size = sysconf(_SC_PAGESIZE);
    char *buffer = (char*) malloc(pool_size);
    for (int i=0; i < pool_size; i+=page_size)
    {
      buffer[i] = 0;
    }

    free(buffer);
    return 0;
  }

  int rttest_prefault_stack_size(const size_t stack_size)
  {
    unsigned char stack[stack_size];
    memset(stack, 0, stack_size);
    return 0;
  }

  int Rttest::prefault_stack()
  {
    return rttest_prefault_stack_size(this->params.stack_size);
  }

  int rttest_prefault_stack()
  {
    auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
    if (!thread_rttest_instance)
      return -1;
    return thread_rttest_instance->prefault_stack();
  }

  int Rttest::set_thread_default_priority()
  {
    return rttest_set_sched_priority(this->params.sched_priority,
        this->params.sched_policy);
  }

  int rttest_set_thread_default_priority()
  {
    auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
    if (!thread_rttest_instance)
      return -1;
    return thread_rttest_instance->set_thread_default_priority();
  }

  int rttest_set_sched_priority(size_t sched_priority, int policy)
  {
    struct sched_param param;

    param.sched_priority = sched_priority;

    // note that sched_setscheduler can set the priority of an arbitrary process
    return sched_setscheduler(0, policy, &param);
  }

  int Rttest::calculate_statistics(struct rttest_results *results)
  {
    if (results == NULL)
    {
      fprintf(stderr, "Need to allocate rttest_results struct\n");
      return -1;
    }
    if (this->sample_buffer.latency_samples == NULL)
    {
      fprintf(stderr, "Pointer to latency samples was NULL\n");
      return -1;
    }
    if (this->sample_buffer.minor_pagefaults == NULL)
    {
      fprintf(stderr, "Pointer to minor pagefaults was NULL\n");
      return -1;
    }
    if (this->sample_buffer.major_pagefaults == NULL)
    {
      fprintf(stderr, "Pointer to major pagefaults was NULL\n");
      return -1;
    }

    std::vector<int> latency_dataset;
    latency_dataset.assign(this->sample_buffer.latency_samples,
        this->sample_buffer.latency_samples + this->sample_buffer.buffer_size);

    results->min_latency = *std::min_element(latency_dataset.begin(),
                                             latency_dataset.end());
    results->max_latency = *std::max_element(latency_dataset.begin(),
                                             latency_dataset.end());
    results->mean_latency = std::accumulate(latency_dataset.begin(),
        latency_dataset.end(), 0.0) / latency_dataset.size();
    double sq_sum = std::inner_product(latency_dataset.begin(), latency_dataset.end(),
        latency_dataset.begin(), 0.0);
    results->latency_stddev = std::sqrt(sq_sum / latency_dataset.size() -
                                    results->mean_latency * results->mean_latency);

    std::vector<unsigned int> min_pagefaults;
    min_pagefaults.assign(this->sample_buffer.minor_pagefaults,
        this->sample_buffer.minor_pagefaults + this->sample_buffer.buffer_size);
    results->minor_pagefaults = std::accumulate(min_pagefaults.begin(), min_pagefaults.end(), 0);

    std::vector<unsigned int> maj_pagefaults;
    maj_pagefaults.assign(this->sample_buffer.major_pagefaults,
        this->sample_buffer.major_pagefaults + this->sample_buffer.buffer_size);
    results->major_pagefaults = std::accumulate(maj_pagefaults.begin(), maj_pagefaults.end(), 0);
    return 0;
  }

  int rttest_calculate_statistics(struct rttest_results *results)
  {
    auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
    if (!thread_rttest_instance)
      return -1;
    return thread_rttest_instance->calculate_statistics(results);
  }

  std::string rttest_results_to_string(struct rttest_results *results)
  {
    if (!results)
    {
      return "ERROR: rttest got NULL results string!";
    }
    std::stringstream sstring;

    sstring << "rttest statistics:" << std::endl;
    sstring << "  - Minor pagefaults: " << results->minor_pagefaults << std::endl;
    sstring << "  - Major pagefaults: " << results->major_pagefaults << std::endl;
    sstring << std::endl;
    sstring << "  Latency (time after deadline was missed):" << std::endl;
    sstring << "    - Min: " << results->min_latency << std::endl;
    sstring << "    - Max: " << results->max_latency << std::endl;
    sstring << "    - Mean: " << results->mean_latency << std::endl;
    sstring << "    - Standard deviation: " << results->latency_stddev << std::endl;
    sstring << std::endl;

    return sstring.str();
  }

  int rttest_finish()
  {
    auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
    if (!thread_rttest_instance)
      return -1;
    return thread_rttest_instance->finish();
  }

  int Rttest::finish()
  {
    // Print statistics to screen
    this->calculate_statistics(&this->results);
    std::cout << rttest_results_to_string(&this->results);

    if (this->sample_buffer.latency_samples != NULL)
    {
      free(this->sample_buffer.latency_samples);
    }

    if (this->sample_buffer.minor_pagefaults != NULL)
    {
      free(this->sample_buffer.minor_pagefaults);
    }

    if (this->sample_buffer.major_pagefaults != NULL)
    {
      free(this->sample_buffer.major_pagefaults);
    }

    return 0;
  }

  int rttest_write_results_file(char *filename)
  {
    auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
    if (!thread_rttest_instance)
      return -1;
    return thread_rttest_instance->write_results_file(filename);
  }

  int rttest_write_results()
  {
    auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
    if (!thread_rttest_instance)
      return -1;
    return thread_rttest_instance->write_results();
  }

  int Rttest::write_results()
  {
    this->write_results_file(this->params.filename);
  }

  int Rttest::write_results_file(char *filename)
  {
    if (!this->params.write)
    {
      fprintf(stderr, "Write flag not set, not writing results\n");
      return -1;
    }

    if (this->sample_buffer.latency_samples == NULL)
    {
      fprintf(stderr, "Samples buffer was NULL, not writing results\n");
      return -1;
    }
    if (this->sample_buffer.minor_pagefaults == NULL)
    {
      fprintf(stderr, "Samples buffer was NULL, not writing results\n");
      return -1;
    }
    if (this->sample_buffer.major_pagefaults == NULL)
    {
      fprintf(stderr, "Samples buffer was NULL, not writing results\n");
      return -1;
    }

    std::ofstream fstream(filename, std::ios::out);

    if (!fstream.is_open())
    {
      fprintf(stderr, "Couldn't open file %s, not writing results\n", filename);
      return -1;
    }

    fstream << "iteration timestamp latency minor_pagefaults minor_pagefaults" << std::endl;
    for (unsigned int i = 0; i < this->sample_buffer.buffer_size; ++i)
    {
      fstream << i << " " << timespec_to_long(&this->params.update_period) * i
              << " " << this->sample_buffer.latency_samples[i] << " "
              << this->sample_buffer.minor_pagefaults[i] << " "
              << this->sample_buffer.major_pagefaults[i] << std::endl;
    }

    fstream.close();

    return 0;
  }
}
