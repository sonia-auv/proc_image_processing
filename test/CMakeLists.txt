# Cannot be used on CI since a ROSCORE needs to be running
catkin_add_gtest(filter_factory_test filter_factory_test.cc ${proc_image_processing_SRC_FILES})
target_link_libraries(
        filter_factory_test
        ${catkin_LIBRARIES}
        ${sonia_common_LIBRARIES}
        ${proc_image_processing_LIBRARIES}
        ${OpenCV_LIBRARIES}
        yaml-cpp
)

catkin_add_gtest(filter_test filter_test.cc ${proc_image_processing_SRC_FILES})
target_link_libraries(
        filter_test
        ${catkin_LIBRARIES}
        ${sonia_common_LIBRARIES}
        ${proc_image_processing_LIBRARIES}
        ${OpenCV_LIBRARIES}
        yaml-cpp
)

catkin_add_gtest(global_parameter_handler_test global_parameter_handler_test.cc ${proc_image_processing_SRC_FILES})
target_link_libraries(
        global_parameter_handler_test
        ${catkin_LIBRARIES}
        ${sonia_common_LIBRARIES}
        ${proc_image_processing_LIBRARIES}
        ${OpenCV_LIBRARIES}
        yaml-cpp
)

catkin_add_gtest(filter_chain_test filter_chain_test.cc ${proc_image_processing_SRC_FILES})
target_link_libraries(
        filter_chain_test
        ${catkin_LIBRARIES}
        ${sonia_common_LIBRARIES}
        ${proc_image_processing_LIBRARIES}
        ${OpenCV_LIBRARIES}
        yaml-cpp
)


#catkin_add_gtest(black_box_test black_box_test.cc ${proc_image_processing_SRC_FILES})
#target_link_libraries(black_box_test
#    pthread ${catkin_LIBRARIES} ${sonia_common_LIBRARIES} ${proc_image_processing_LIBRARIES}
#    ${OpenCV_LIBRARIES} yaml-cpp )
#
#add_dependencies(black_box_test ${PROJECT_NAME}_node)