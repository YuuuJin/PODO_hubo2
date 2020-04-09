set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5Core REQUIRED)
add_executable(FreeWalking main.cpp
		joint.cpp
		WalkingFSM.cpp
		FootClass.cpp
		BasicTrajectory.cpp
		BasicFSM.cpp
		orient_fnc.cpp
		ManualCAN.cpp)
find_library(COPPERPLATE copperplate /usr/xenomai/lib)
find_library(COBALT cobalt /usr/xenomai/lib)
find_library(ALCHEMY alchemy /usr/xenomai/lib)
find_library(IK_MATH4 NAMES libik_math4.a libik_math4 PATHS ${podo_SOURCE_DIR}/share/Libs)
find_library(KINE_DRC_HUBO4 NAMES libKINE_DRC_HUBO4.a KINE_DRC_HUBO4 PATHS ${podo_SOURCE_DIR}/share/Libs)
target_include_directories(FreeWalking PRIVATE /usr/xenomai/include /usr/xenomai/include/cobalt)
qt5_use_modules(FreeWalking Core)
target_link_libraries(FreeWalking rt ${COBALT} ${ALCHEMY} ${COPPERPLATE} ${IK_MATH4} ${KINE_DRC_HUBO4})
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /usr/xenomai/lib/xenomai/bootstrap.o -Wl,--wrap=main -Wl,--dynamic-list=/usr/xenomai/lib/dynlist.ld")
add_custom_command(TARGET FreeWalking POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/share/Gain $<TARGET_FILE_DIR:FreeWalking>/../../../share/Gain)