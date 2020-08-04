# \file     CMakeLists.txt
# \author   Thibaut Mattio <thibaut.mattio@gmail.com>
# \date     08/05/2015
# \copyright    2015 Club SONIA AUV, ETS. All rights reserved.
# Use of this source code is governed by the MIT license that can be
# found in the LICENSE file.

catkin_add_gtest(black_box_test black_box_test.cc ${proc_image_processing_FILES})
target_link_libraries(black_box_test
    pthread ${catkin_LIBRARIES} ${sonia_common_LIBRARIES} ${proc_image_processing_LIBRARIES}
    ${OpenCV_LIBRARIES} yaml-cpp )

add_dependencies(black_box_test ${PROJECT_NAME}_node)