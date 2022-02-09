execute_process(COMMAND "/home/jerry/realtobe2/catkin_ws/build/tobe2_real/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/jerry/realtobe2/catkin_ws/build/tobe2_real/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
