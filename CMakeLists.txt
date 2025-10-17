# CMake 최소 요구 버전 설정
cmake_minimum_required(VERSION 3.10)

# 프로젝트 이름 설정
project(PlaneExtractor)

if(APPLE)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "12.0" CACHE STRING "Minimum OS X deployment version")
endif()

# C++ 표준 설정 (C++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# PCL (Point Cloud Library) 찾기
find_package(PCL 1.8 REQUIRED)

# PCL 관련 헤더 디렉토리 및 라이브러리 포함
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 실행 파일 생성
# 모든 .cpp 파일을 포함하도록 수정
add_executable(plane_extractor_app
    main.cpp
    plane_extractor.cpp
    file_io.cpp
)

# 실행 파일에 PCL 라이브러리 링크
target_link_libraries(plane_extractor_app ${PCL_LIBRARIES})

