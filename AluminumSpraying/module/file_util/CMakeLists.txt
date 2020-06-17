cmake_minimum_required(VERSION 2.8 FATAL_ERROR)  

set(PROJECTNAME file_util)

set(PRO_SOURCES file_util.h file_util.cpp)

project(${PROJECTNAME})

#执行git命令,获取hash值
execute_process(
COMMAND git log -1 --pretty=format:%h
WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
OUTPUT_VARIABLE GIT_HASH
)

#执行git命令,获取提交次数
execute_process(
COMMAND git rev-list HEAD --count
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

find_package(PCL 1.8.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECTNAME} SHARED  ${PRO_SOURCES} ${VERSIONINFO_RC})
add_library(${PROJECTNAME}_static STATIC  ${PRO_SOURCES})
set_target_properties(${PROJECTNAME}_static PROPERTIES OUTPUT_NAME ${PROJECTNAME})
set_target_properties(${PROJECTNAME} PROPERTIES CLEAN_DIRECT_OUTPUT 1)
set_target_properties(${PROJECTNAME}_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)

target_include_directories(${PROJECTNAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>)