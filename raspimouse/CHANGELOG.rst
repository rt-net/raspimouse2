^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package raspimouse
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2020-05-21)
------------------
* Change pulse counter for calculating odometry accurately (`#16 <https://github.com/rt-net/raspimouse2/issues/16>`_)
* Add lint check (`#14 <https://github.com/rt-net/raspimouse2/issues/14>`_)

  * Add ament_lint_auto
  * Fix package.xml for xmllint
  * Fix CMakeLists for cmake_lint
  * Fix raspimouse_component.hpp for cpp_lint
  * Fix raspimouse_component.hpp for uncrustify
  * Fix raspimouse_component.cpp for cpp_lint
  * fix raspimouse_component.cpp for uncrustify
  * fix raspimouse.cpp for uncrustify
  * Update README.md
* Update tf timestamp on use_pulse_counters\_ == true (`#6 <https://github.com/rt-net/raspimouse2/issues/6>`_)
* invert switch status (`#11 <https://github.com/rt-net/raspimouse2/issues/11>`_)

  * invert switch status
  * Refactoring
* change to base_footprint (`#5 <https://github.com/rt-net/raspimouse2/issues/5>`_)
* Fix odom (`#4 <https://github.com/rt-net/raspimouse2/issues/4>`_)

  * comment in tf
  * initialize all instance variables
  * update readme
  * fix odom\_
* Re-enable tf (`#3 <https://github.com/rt-net/raspimouse2/issues/3>`_)

  * comment in tf
  * initialize all instance variables
  * update readme
* Add Dashing support (`#2 <https://github.com/rt-net/raspimouse2/issues/2>`_)
* Add Crystal support (`#1 <https://github.com/rt-net/raspimouse2/issues/1>`_)

  * Add Crystal support
  * Use type alias for CallbackReturn
* Add readme
* Adding sensors
* Contributors: Daisuke Sato, Geoffrey Biggs, Shota Aoki, Shota Hirama, Yutaka Kondo

0.1.0 (2018-09-07)
------------------
* Initial commit
* Contributors: Geoffrey Biggs