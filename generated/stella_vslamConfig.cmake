include(CMakeFindDependencyMacro)

find_dependency(Threads)
find_dependency(OpenMP)
find_dependency(OpenCV)
find_dependency(Eigen3)
find_dependency(g2o)
find_dependency(yaml-cpp)
if(1)
    find_dependency(spdlog)
endif()
if()
    find_dependency(Tracy)
endif()

if(FBoW MATCHES "DBoW2")
    find_dependency(DBoW2)
elseif(FBoW MATCHES "FBoW")
    find_dependency(fbow)
endif()

include("${CMAKE_CURRENT_LIST_DIR}/stella_vslamTargets.cmake")

include_directories("/usr/local/include")

set(stella_vslam_INCLUDE_DIRS "/usr/local/include")

link_directories("/usr/local/lib")
set(stella_vslam_LIB_DIR "/usr/local/lib")

set(stella_vslam_LIBS  stella_vslam)

set(stella_vslam_FOUND TRUE)
set(stella_vslam_VERSION )
set(stella_vslam_VERSION_MAJOR )
set(stella_vslam_VERSION_MINOR )
set(stella_vslam_VERSION_PATCH )
