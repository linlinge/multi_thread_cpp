cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(mproc)

set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE "Release")

# PCL
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenMP
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# OpenCV 
FIND_PACKAGE(OpenCV REQUIRED)

# Eigen
include_directories("/usr/include/eigen3")

# Ceres
# find_package(Ceres REQUIRED)
# include_directories(${CERES_INCLUDE_DIRS})


############################################# executable #############################################
include_directories(glib .)
ADD_SUBDIRECTORY(glib)
aux_source_directory(. MAIN_SRC)
add_executable (mproc ${MAIN_SRC})


###########################################  link library ###############################################
target_link_libraries(mproc GLIB)

# PCL
target_link_libraries (mproc ${PCL_LIBRARIES})

# OpenCV
target_link_libraries(mproc ${OpenCV_LIBS})

# Ceres
# target_link_libraries(pcl_demo ${CERES_LIBRARIES})
