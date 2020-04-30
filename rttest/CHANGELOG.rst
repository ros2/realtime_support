^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rttest
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.0 (2020-04-30)
------------------
* Use std::bind instead of deprecated std::bind2nd (`#93 <https://github.com/ros2/realtime_support/issues/93>`_)
* Use a uint64_t to store the prefault_dynamic_size. (`#90 <https://github.com/ros2/realtime_support/issues/90>`_)
* Make dynamic prefault memory size configurable (`#89 <https://github.com/ros2/realtime_support/issues/89>`_)
* code style only: wrap after open parenthesis if not in one line (`#88 <https://github.com/ros2/realtime_support/issues/88>`_)
* Contributors: Chris Lalancette, Dirk Thomas, Michel Hidalgo, Roman Sokolkov

0.8.2 (2019-11-13)
------------------

0.8.1 (2019-10-23)
------------------
* Fix typo (`#83 <https://github.com/ros2/realtime_support/issues/83>`_)
* Contributors: Servando

0.8.0 (2019-09-26)
------------------
* Switch the latency storage to int64_t (`#82 <https://github.com/ros2/realtime_support/issues/82>`_)
* Only return statistics if they have been calculated (`#81 <https://github.com/ros2/realtime_support/issues/81>`_)
* Remove non-package from ament_target_dependencies() (`#79 <https://github.com/ros2/realtime_support/issues/79>`_)
* Fix armhf build warnings (`#78 <https://github.com/ros2/realtime_support/issues/78>`_)
* Contributors: Chris Lalancette, Prajakta Gokhale, Shane Loretz

0.7.1 (2019-05-08)
------------------

0.7.0 (2019-04-14)
------------------
* Add section about DCO to CONTRIBUTING.md
* Install headers into the same location when ament_cmake is not used (`#72 <https://github.com/ros2/realtime_support/issues/72>`_)
* Contributors: Dirk Thomas

0.6.0 (2018-11-20)
------------------
* Drop obsolete RTLinux (`#68 <https://github.com/ros2/realtime_support/issues/68>`_)
  RTLinux is obsolete, and both commercial and GPL editions were no longer
  maintained. Through the use of the real-time Linux kernel patch
  PREEMPT_RT, support for full preemption of critical sections, interrupt
  handlers, and "interrupt disable" code sequences can be supported.
* Contributors: Jim Huang

0.5.0 (2018-06-27)
------------------
* Update the maintainer (`#65 <https://github.com/ros2/realtime_support/issues/65>`_)
* Contributors: Chris Lalancette

0.4.0 (2017-12-08)
------------------
* Update style to match latest uncrustify (`#57 <https://github.com/ros2/realtime_support/issues/57>`_)
* 0.0.3
* Update style to satisfy new flake8 plugins (`#56 <https://github.com/ros2/realtime_support/issues/56>`_)
* Use CMAKE_X_STANDARD and check compiler rather than platform
* Comply with flake8 + flake8-import-order linters (`#52 <https://github.com/ros2/realtime_support/issues/52>`_)
* Require CMake 3.5
* C++14 (`#50 <https://github.com/ros2/realtime_support/issues/50>`_)
* Remove usage of internal variables and noops (`#43 <https://github.com/ros2/realtime_support/issues/43>`_)
* Add schema to manifest files
* Disable on Android (`#41 <https://github.com/ros2/realtime_support/issues/41>`_)
* Use CTest BUILD_TESTING (`#38 <https://github.com/ros2/realtime_support/issues/38>`_)
* uint32 -> uint64 in a few places for time calculation `#33 <https://github.com/ros2/realtime_support/issues/33>`_)
* Store filename on heap, add more checks for null pointers (`#29 <https://github.com/ros2/realtime_support/issues/29>`_)
* Fix memory management of rttest_sample_buffer (`#22 <https://github.com/ros2/realtime_support/issues/22>`_)
* Fix boundary check (`#20 <https://github.com/ros2/realtime_support/issues/20>`_)
* Reorganize realtime_support repository (`#16 <https://github.com/ros2/realtime_support/issues/16>`_)
    * Add tlsf_cpp repo
    * Move rttest into subfolder
* Contributors: Dirk Thomas, Esteve Fernandez, Jackie Kay, Mikael Arguedas, Morgan Quigley, dhood
