^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sparse_bundle_adjustment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.4 (2020-03-28)
------------------
* fix unitialized variable causing crashes (`#10 <https://github.com/ros-perception/sparse_bundle_adjustment/issues/10>`_)
* Contributors: Michael Ferguson

0.4.3 (2020-01-31)
------------------
* remove bouncing email from maintainers
* Merge pull request `#9 <https://github.com/ros-perception/sparse_bundle_adjustment/issues/9>`_ from seanyen/modern_cpp
  [windows] use modern cpp & more portable fixes
* modernize cpp code & CMake files.
* Changing maintainer email
* Contributors: Luc Bettaieb, Michael Ferguson, seanyen

0.4.2 (2018-08-23)
------------------
* rework how we set the C++ standard
* Merge pull request `#6 <https://github.com/ros-perception/sparse_bundle_adjustment/issues/6>`_ from moriarty/set-cpp-11
  Set C++ 11
* [Maintainers] Add myself as a maintainer
  Mostly so that I can see build failures.
* [CMake][C++11] compile with -std=c++11
* Contributors: Alexander Moriarty, Michael Ferguson

0.4.1 (2018-08-21)
------------------
* Merge pull request `#4 <https://github.com/ros-perception/sparse_bundle_adjustment/issues/4>`_ from moriarty/eigen-and-pkg-fmt-2
  Fixes Eigen3 warnings and bumps to package.xml format 2
* Update email address so I see build failures
* [REP-140] Package.xml format 2
* fix deprecated eigen3 cmake warning
* Contributors: Alexnader Moriarty, Michael Ferguson, Steffen Fuchs

0.4.0 (2018-08-18)
------------------
* Merge pull request `#3 <https://github.com/ros-perception/sparse_bundle_adjustment/issues/3>`_ from moriarty/melodic-devel
  [melodic-devel][18.04] fix compile errors
* [melodic-devel][18.04] fix compile errors
  fix compile errors for newer gcc
* Merge pull request `#2 <https://github.com/ros-perception/sparse_bundle_adjustment/issues/2>`_ from ros-perception/maintainer-add
  Adding myself as a maintainer for sparse_bundle_adjustment
* Adding myself as a maintainer for sparse_bundle_adjustment
* Contributors: Alexander Moriarty, Luc Bettaieb, Michael Ferguson

0.3.2 (2014-06-17)
------------------
* major build/install fixes for the farm
* Contributors: Michael Ferguson

0.3.1 (2014-06-17)
------------------
* add depend on eigen
* Contributors: Michael Ferguson

0.3.0 (2014-06-16)
------------------
* import cleaned up sba (this is the version from wg)
* Contributors: Michael Ferguson
