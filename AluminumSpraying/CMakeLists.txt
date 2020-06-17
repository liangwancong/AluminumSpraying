cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(PROJECTNAME AluminumSpraying)     
 
add_definitions(-DDEBUGC)   
aux_source_directory(src/. PRO_SOURCES)  
set(PRO_INCLUDES src/AluminumSpraying.h src/paint_model.h src/json_model_parse.h src/cv_search_points.h src/cv_match_points.h src/pcl_filter_cloud.h src/parameters.h src/global.h src/Registration.h)

#git version
#执行git命令,获取hash值
execute_process(
	COMMAND	git log -1 --pretty=format:%h 
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} 
	OUTPUT_VARIABLE GIT_HASH
)   
#执行git命令,获取提交次数
execute_process(
	COMMAND	git rev-list HEAD --count
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} 
	OUTPUT_VARIABLE GIT_VERSION 
)
string(REPLACE "\n" ";" GIT_VERSION ${GIT_VERSION})

#动态配置生成.rc文件
set(VERSION_RC "${CMAKE_SOURCE_DIR}/version.rc")
file(STRINGS ${VERSION_RC} PARSED_STRINGS)  
string(REPLACE "*****" "\n" PARSED_STRINGS ${PARSED_STRINGS})
string(REPLACE "*FileDescription" ${GIT_HASH} PARSED_STRINGS ${PARSED_STRINGS})  
string(REPLACE "*InternalName" ${PROJECTNAME}.dll PARSED_STRINGS ${PARSED_STRINGS})
string(REPLACE "*OriginalFilename" ${PROJECTNAME}.dll PARSED_STRINGS ${PARSED_STRINGS})
string(REPLACE "*ProductName" ${PROJECTNAME}.dll PARSED_STRINGS ${PARSED_STRINGS})
string(REPLACE "*version" "1.0.0.${GIT_VERSION}" PARSED_STRINGS ${PARSED_STRINGS})
string(REPLACE "$version" "1,0,0,${GIT_VERSION}" PARSED_STRINGS ${PARSED_STRINGS})
file(WRITE ${CMAKE_BINARY_DIR}/${PROJECTNAME}.rc ${PARSED_STRINGS}) 
set(VERSIONINFO_RC ${CMAKE_BINARY_DIR}/${PROJECTNAME}.rc)

#cJSON
#set(ENABLE_CUSTOM_COMPILER_FLAGS OFF)
#set(ENABLE_CJSON_TEST OFF)
#set(CJSON_BUILD_SHARED_LIBS OFF)
#set(CJSON_OVERRIDE_BUILD_SHARED_LIBS ON)

#simdjson

project(${PROJECTNAME})

add_subdirectory(module\\cv_geomerty)
add_subdirectory(module\\pcl_geomerty)
add_subdirectory(module\\file_util)
add_subdirectory(module\\string_util)
#add_subdirectory(module\\cJSON)
#add_subdirectory(module\\simdjson)
add_subdirectory(module\\cv_skeleton)
add_subdirectory(module\\pcl_filter_helper)
add_subdirectory(module\\logger)
add_subdirectory(module\\pcl_tree)
add_subdirectory(module\\pcl_viewer)
add_subdirectory(module\\CRMath)
add_subdirectory(module\\Mvbb)
include_directories(module\\Mvbb)

include_directories(${CMAKE_SOURCE_DIR}\\module\\ApproxMVBB\\include)
LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}\\module\\ApproxMVBB\\lib)

find_package(PCL 1.8.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} module\\file_util module\\cv_geomerty module\\pcl_geomerty module\\cv_skeleton module\\pcl_filter_helper module\\string_util module\\logger module\\pcl_tree module\\pcl_viewer module\\CRMath)

add_library(${PROJECTNAME} SHARED ${PRO_SOURCES} ${VERSIONINFO_RC})
add_executable (${PROJECTNAME}_exe ${PRO_SOURCES} ${PRO_INCLUDES})
target_link_libraries (${PROJECTNAME}_exe ${PCL_LIBRARIES} ${OpenCV_LIBS} file_util_static cv_geomerty_static pcl_geomerty_static cv_skeleton_static pcl_filter_helper_static string_util_static logger_static pcl_tree_static pcl_viewer_static CRMath_static Mvbb_static ApproxMVBB)
target_link_libraries (${PROJECTNAME} ${PCL_LIBRARIES} ${OpenCV_LIBS} file_util_static cv_geomerty_static pcl_geomerty_static cv_skeleton_static pcl_filter_helper_static string_util_static logger_static pcl_tree_static pcl_viewer_static CRMath_static Mvbb_static ApproxMVBB)