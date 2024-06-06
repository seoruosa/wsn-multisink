cmake_minimum_required(VERSION 3.22.1)
project(wsn-multisink)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# ########## CONFIGURE CPLEX ##########
set(CMAKE_MODULE_PATH               "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_CXX_FLAGS                 "${CMAKE_CXX_FLAGS} -DIL_STD -m64")
set(CMAKE_CXX_FLAGS_DEBUG           "${CMAKE_CXX_FLAGS_DEBUG} -Og -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE         "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG -O3 -flto")
set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX   "-isystem ")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "${CMAKE_CURRENT_SOURCE_DIR}/build")
set(LINKER_OPTIONS                  -flto -Wl,--no-as-needed)

if(CMAKE_SYSTEM_NAME STREQUAL Darwin)
    set(CMAKE_CXX_FLAGS_DEBUG       "${CMAKE_CXX_FLAGS_DEBUG} -save-temps=obj")
endif()

find_package(Cplex)
include_directories(SYSTEM ${CPLEX_INCLUDE_DIRS})

include_directories(include)

add_executable(play src/playground.cpp)
add_executable(all_models src/all_models.cpp)

target_link_libraries(play 
                        PRIVATE ${CPLEX_LIBRARIES}
                          )

target_link_libraries(all_models 
                        # PRIVATE utillib
                        PRIVATE ${CPLEX_LIBRARIES}
                          )

target_include_directories(all_models PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")

enable_testing()

# add_subdirectory(src)
add_subdirectory(tests)
