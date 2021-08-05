add_executable(or_hitl src/demos/OR_HITL.cpp)
target_link_libraries(or_hitl
        ${catkin_LIBRARIES}
        ${OMPL_LIBRARIES}
        ${LIB_MUJOCO}
        ${GLFW}
        libGL.so
        libglew.so
        ${CMAKE_THREAD_LIBS_INIT}  # To have -pthread
        GL
        GLU
        glut
        -fopenmp
        -lpthread
        -lboost_thread
        -lboost_system)

