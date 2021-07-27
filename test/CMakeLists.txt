catkin_add_gtest(filter_test filter_test.cc ${proc_image_processing_SRC_FILES})
add_dependencies(filter_test ${PROJECT_NAME}_node)

#catkin_add_gtest(black_box_test black_box_test.cc ${proc_image_processing_SRC_FILES})
#target_link_libraries(black_box_test
#    pthread ${catkin_LIBRARIES} ${sonia_common_LIBRARIES} ${proc_image_processing_LIBRARIES}
#    ${OpenCV_LIBRARIES} yaml-cpp )
#
#add_dependencies(black_box_test ${PROJECT_NAME}_node)