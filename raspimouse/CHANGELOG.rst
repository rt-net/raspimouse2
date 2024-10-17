^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package raspimouse
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2024-10-17)
------------------
* Update for Jazzy
* Update documents for migration to Jazzy
* Change the message type of `/cmd/vel` from `Twist` to `TwistStamped`
* Contributors: Kazushi Kurasawa

1.1.2 (2023-08-24)
------------------
* Calculate speed from robot parameters (`#50 <https://github.com/rt-net/raspimouse2/issues/50>`_)
* Contributors: Tatsuhiro Ikebe

1.1.1 (2023-01-26)
------------------
* Humble対応 (`#48 <https://github.com/rt-net/raspimouse2/issues/48>`_)
* Contributors: Shuhei Kozasa, Daisuke Sato

1.1.0 (2022-07-28)
------------------
* Add authors to package.xml
* トピックpublish周波数とフレームIDを変更するパラメータを追加 (`#42 <https://github.com/rt-net/raspimouse2/issues/42>`_)
* ノードの実行を簡単にするためlaunchファイルとconfigファイルを追加 `#41 <https://github.com/rt-net/raspimouse2/issues/41>`_
* Add initial_motor_power param (`#39 <https://github.com/rt-net/raspimouse2/issues/39>`_)
* パラメータの追加（`#35 <https://github.com/rt-net/raspimouse2/issues/35>`_） (`#38 <https://github.com/rt-net/raspimouse2/issues/38>`_)
* パラメータの追加 （`#34 <https://github.com/rt-net/raspimouse2/issues/34>`_） (`#36 <https://github.com/rt-net/raspimouse2/issues/36>`_)
* Contributors: Shota Aoki, Shuhei Kozasa

1.0.2 (2020-12-08)
------------------
* Update for foxy (`#29 <https://github.com/rt-net/raspimouse2/issues/29>`_)
* Contributors: Shota Aoki

1.0.1 (2020-06-10)
------------------
* Fix documents of release 1.0.0 (`#23 <https://github.com/rt-net/raspimouse2/issues/23>`_)
* Contributors: Shota Aoki

1.0.0 (2020-06-08)
------------------
* Update CMakeLists.txt for exporting include dir (`#21 <https://github.com/rt-net/raspimouse2/issues/21>`_)
* Fix package.xml (`#20 <https://github.com/rt-net/raspimouse2/issues/20>`_)
* Add on_shutdown callback (`#17 <https://github.com/rt-net/raspimouse2/issues/17>`_)
* Contributors: Shota Aoki

0.2.0 (2020-05-22)
------------------
* Release 0.2.0 (`#18 <https://github.com/rt-net/raspimouse2/issues/18>`_)
* Change pulse counter for calculating odometry accurately (`#16 <https://github.com/rt-net/raspimouse2/issues/16>`_)
* Add lint check (`#14 <https://github.com/rt-net/raspimouse2/issues/14>`_)
* Update tf timestamp on use_pulse_counters\_ == true (`#6 <https://github.com/rt-net/raspimouse2/issues/6>`_)
* invert switch status (`#11 <https://github.com/rt-net/raspimouse2/issues/11>`_)
* change to base_footprint (`#5 <https://github.com/rt-net/raspimouse2/issues/5>`_)
* Fix odom (`#4 <https://github.com/rt-net/raspimouse2/issues/4>`_)
* Re-enable tf (`#3 <https://github.com/rt-net/raspimouse2/issues/3>`_)
* Add Dashing support (`#2 <https://github.com/rt-net/raspimouse2/issues/2>`_)
* Add Crystal support (`#1 <https://github.com/rt-net/raspimouse2/issues/1>`_)
* Add readme
* Adding sensors
* Contributors: Daisuke Sato, Geoffrey Biggs, Shota Aoki, Shota Hirama, Yutaka Kondo

0.1.0 (2018-09-07)
------------------
* Initial commit
* Contributors: Geoffrey Biggs
