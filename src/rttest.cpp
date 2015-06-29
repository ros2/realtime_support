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
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <numeric>
#include <fstream>
#include <sched.h>
#include <string.h>
#include <sstream>
#include <sys/mman.h>
#include <sys/rusage.h>
#include <unistd.h>
#include <vector>

#include <utils.h>
#include <rttest.h>

#define MAX_FILENAME_SIZE 1024

extern "C"
{

  struct rttest_sample_buffer
  {
    // Stored in nanoseconds
    long *latency_samples;
    bool *missed_deadlines;

    unsigned int buffer_size;
  };

  // Global variables
  struct rttest_params _rttest_params;
  struct rttest_sample_buffer _rttest_sample_buffer;
  struct rttest_results _rttest_results;

  int rttest_record_missed_deadline(const struct timespec *deadline,
      const struct timespec *result_time, const int iteration)
  {
    struct timespec jitter;
    subtract_timespecs(result_time, deadline, &jitter);
    // Record jitter
    if (iteration > _rttest_sample_buffer.buffer_size)
    {
      return -1;
    }
    _rttest_sample_buffer.missed_deadlines[iteration] = true;
    _rttest_sample_buffer.latency_samples[iteration] = -timespec_to_long(&jitter);
    return 0;
  }

  int rttest_record_jitter(const struct timespec *deadline,
      const struct timespec *result_time, const int iteration)
  {
    if (timespec_gt(result_time, deadline))
    {
      // missed a deadline
      return rttest_record_missed_deadline(deadline, result_time, iteration);
    }
    struct timespec jitter;
    subtract_timespecs(deadline, result_time, &jitter);
    // Record jitter
    if (iteration > _rttest_sample_buffer.buffer_size)
    {
      return -1;
    }
    _rttest_sample_buffer.missed_deadlines[iteration] = false;
    _rttest_sample_buffer.latency_samples[iteration] = timespec_to_long(&jitter);
    return 0;
  }

  int rttest_read_args(int argc, char** argv)
  {
    // TODO

    //parse arguments
    // -i,--iterations
    unsigned long iterations = 1000;
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
          iterations = atol(optarg);
          break;
        case 'u':
          // parse units
          break;
        case 'p':
          plot = 1;
          break;
        case 't':
          sched_priority = atoi(optarg);
          break;
        case 's':
          // translate string to number. is there a utility for this?
          break;
        case 'm':
          lock_memory = 1;
          // parse units
          break;
        case 'f':
          filename = optarg;
          printf("got filename %s\n", filename);
          write = 1;
          // check if file exists
          break;
        case 'r':
          repetitions = atoi(optarg);
          break;
        case '?':
          if (args_string.find(optopt) != std::string::npos)
            fprintf (stderr, "Option -%c requires an argument.\n", optopt);
          else if (isprint(optopt))
            fprintf(stderr, "Unknown option `-%c'.\n", optopt);
          else
            fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
          break;
        default:
          exit(-1);
      }
    }

    rttest_init(iterations, update_period, sched_policy, sched_priority,
        lock_memory, stack_size, plot, write, filename, repetitions);
  }

  int rttest_init(unsigned long iterations, struct timespec update_period,
      size_t sched_policy, int sched_priority, int lock_memory, size_t stack_size,
      int plot, int write, char *filename, unsigned int repetitions)
  {
    _rttest_params.iterations = iterations;
    _rttest_params.update_period = update_period;
    _rttest_params.sched_policy = sched_policy;
    _rttest_params.sched_priority = sched_priority;
    _rttest_params.lock_memory = lock_memory;
    _rttest_params.stack_size = stack_size;
    _rttest_params.plot = plot;
    _rttest_params.write = write;
    //printf("Copy filename %s\n", filename);
    //memcpy(_rttest_params.filename, filename, MAX_FILENAME_SIZE);
    _rttest_params.filename = filename;
    _rttest_params.reps = repetitions;

    _rttest_sample_buffer.latency_samples =
        (long *) std::malloc(iterations*sizeof(unsigned long));
    memset(_rttest_sample_buffer.latency_samples, 0,
        iterations*sizeof(long));
    _rttest_sample_buffer.missed_deadlines =
        (bool *) std::malloc(iterations*sizeof(bool));
    memset(_rttest_sample_buffer.missed_deadlines, 0, iterations*sizeof(bool));
    _rttest_sample_buffer.buffer_size = iterations;

    return 0;
  }

  int rttest_spin(void *(*user_function)(void *), void *args)
  {
    return rttest_spin_period(user_function, args, &_rttest_params.update_period,
        _rttest_params.iterations);
  }

  int rttest_spin_period(void *(*user_function)(void *), void *args,
      const struct timespec *update_period, const unsigned long iterations)
  {
    struct timespec wakeup_time, current_time;
    clock_gettime(0, &current_time);
    wakeup_time = current_time;

    for (int i = 0; i < iterations; i++)
    {
      // Plan the next shot
      add_timespecs(&wakeup_time, update_period, &wakeup_time);
      clock_gettime(0, &current_time);
      if (timespec_gt(&current_time, &wakeup_time))
      {
        // Missed a deadline before we could sleep! Record it
        rttest_record_missed_deadline(&wakeup_time, &current_time, i);
      }
      else
      {
        clock_nanosleep(0, TIMER_ABSTIME, &wakeup_time, NULL);
        clock_gettime(0, &current_time);

        rttest_record_jitter(&wakeup_time, &current_time, i);
      }

      user_function(args);
    }

    return 0;
  }

  int rttest_schedule_wakeup(void *(*user_function)(void *), void *args,
      const struct timespec *absolute_wakeup)
  {
    struct timespec current_time;
    clock_gettime(0, &current_time);
    if (timespec_gt(&current_time, absolute_wakeup))
    {
      // Missed a deadline before we could sleep! Record it
      rttest_record_missed_deadline(absolute_wakeup, &current_time, -1);
    }
    else
    {
      clock_nanosleep(0, TIMER_ABSTIME, absolute_wakeup, NULL);
      clock_gettime(0, &current_time);

      rttest_record_jitter(absolute_wakeup, &current_time, -1);
    }

    user_function(args);

    return 0;
  }

  int rttest_lock_memory()
  {
    return mlockall(MCL_CURRENT | MCL_FUTURE);
  }

  int rttest_prefault_stack_size(const size_t stack_size)
  {
    unsigned char stack[stack_size];
    memset(stack, 0, stack_size);
    // TODO: catch errors, maybe verify memset return value points to stack?
    return 0;
  }

  int rttest_prefault_stack()
  {
    return rttest_prefault_stack_size(_rttest_params.stack_size);
  }


  int rttest_set_thread_default_priority()
  {
    return rttest_set_sched_priority(_rttest_params.sched_priority,
        _rttest_params.sched_policy);
  }

  int rttest_set_sched_priority(size_t sched_priority, int policy)
  {
    struct sched_param param;

    param.sched_priority = sched_priority;

    // note that sched_setscheduler can set the priority of an arbitrary process
    return sched_setscheduler(0, policy, &param);
  }

  int rttest_calculate_statistics(struct rttest_results *results)
  {
    if (results == NULL)
    {
      return -1;
    }

    std::vector<long> jitter_dataset;
    jitter_dataset.assign(_rttest_sample_buffer.latency_samples,
        _rttest_sample_buffer.latency_samples + _rttest_sample_buffer.buffer_size);

    std::vector<long> latency_dataset(jitter_dataset.size());
    std::copy_if(jitter_dataset.begin(), jitter_dataset.end(),
        latency_dataset.begin(),
        [](long sample){ return sample < 0; } );

    std::vector<bool> missed_deadlines_data;
    missed_deadlines_data.assign(_rttest_sample_buffer.missed_deadlines,
        _rttest_sample_buffer.missed_deadlines + _rttest_sample_buffer.buffer_size);

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

    results->min_jitter = *std::min_element(jitter_dataset.begin(),
                                            jitter_dataset.end());
    results->max_jitter = *std::max_element(jitter_dataset.begin(),
                                            jitter_dataset.end());
    results->mean_jitter = std::accumulate(jitter_dataset.begin(),
        jitter_dataset.end(), 0.0) / jitter_dataset.size();
    sq_sum = std::inner_product(jitter_dataset.begin(), jitter_dataset.end(),
        jitter_dataset.begin(), 0.0);
    results->jitter_stddev = std::sqrt(sq_sum / jitter_dataset.size() -
                                     results->mean_jitter * results->mean_jitter);

    results->missed_deadlines = std::count(missed_deadlines_data.begin(),
        missed_deadlines_data.end(), true);
    results->early_deadlines =
        missed_deadlines_data.size() - results->missed_deadlines;
    return 0;
  }

  std::string rttest_results_to_string(struct rttest_results *results)
  {
    if (!results)
    {
      return "ERROR: rttest got NULL results string!";
    }
    std::stringstream sstring;

    sstring << "rttest statistics:" << std::endl;
    sstring << "  - Missed deadlines: " << results->missed_deadlines << std::endl;
    sstring << "  - Early deadlines: " << results->early_deadlines << std::endl;
    sstring << std::endl;
    sstring << "  Latency (time after deadline was missed):" << std::endl;
    sstring << "    - Min: " << results->min_latency << std::endl;
    sstring << "    - Max: " << results->max_latency << std::endl;
    sstring << "    - Mean: " << results->mean_latency << std::endl;
    sstring << "    - Standard deviation: " << results->latency_stddev << std::endl;
    sstring << std::endl;
    sstring << "  Jitter (scheduling variation for early or missed deadlines):" << std::endl;
    sstring << "    - Min: " << results->min_jitter << std::endl;
    sstring << "    - Max: " << results->max_jitter << std::endl;
    sstring << "    - Mean: " << results->mean_jitter << std::endl;
    sstring << "    - Standard deviation: " << results->jitter_stddev << std::endl;

    return sstring.str();
  }

  int rttest_finish()
  {
    // Print statistics to screen
    rttest_calculate_statistics(&_rttest_results);
    std::cout << rttest_results_to_string(&_rttest_results);

    if (_rttest_sample_buffer.latency_samples != NULL)
    {
      free(_rttest_sample_buffer.latency_samples);
    }

    if (_rttest_sample_buffer.missed_deadlines != NULL)
    {
      free(_rttest_sample_buffer.missed_deadlines);
    }
    return 0;
  }

  int rttest_write_results()
  {
    if (!_rttest_params.write)
    {
      printf("Write bit not set, not writing results\n");
      return -1;
    }

    if (_rttest_sample_buffer.latency_samples == NULL)
    {
      printf("Samples buffer was NULL, not writing results\n");
      return -1;
    }

    if (_rttest_sample_buffer.missed_deadlines == NULL)
    {
      printf("Deadlines buffer was NULL, not writing results\n");
      return -1;
    }

    std::ofstream fstream(_rttest_params.filename, std::ios::out);

    if (!fstream.is_open())
    {
      printf("Couldn't open file %s, not writing results\n", _rttest_params.filename);
      return -1;
    }

    // Format:
    // iteration  timestamp (ns)  latency  missed_deadline? (1/0)
    fstream << "iteration timestamp latency missed_deadline" << std::endl;
    for (unsigned long i = 0; i < _rttest_sample_buffer.buffer_size; ++i)
    {
      fstream << i << " " << timespec_to_long(&_rttest_params.update_period) * i
              << " " << _rttest_sample_buffer.latency_samples[i] << " "
              << _rttest_sample_buffer.missed_deadlines[i] << std::endl;
    }

    fstream.close();

    return 0;
  }

  int rttest_plot()
  {
    if (!_rttest_params.plot)
    {
      return -1;
    }
    return 0;
  }

}
