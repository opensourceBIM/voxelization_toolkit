cmake_minimum_required (VERSION 2.8.11)
project (voxel)

OPTION(IFC_SUPPORT "Build Volization Toolkit with IFC support (requires IfcOpenShell/master)." ON)

include(CTest)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

set(OCC_LIBRARY_NAMES TKernel TKMath TKBRep TKGeomBase TKGeomAlgo TKG3d TKG2d TKShHealing TKTopAlgo TKMesh TKOffset
                      TKPrim TKBool TKBO TKFillet TKSTEP TKSTEPBase TKSTEPAttr TKXSBase TKSTEP209 TKIGES TKHLR TKFeat)
                      
if(IFC_SUPPORT)
add_definitions(-DWITH_IFC)
endif()


if(UNIX)
    if(IFC_SUPPORT)
        set(OCC_LIBRARY_DIR  ${IFCOPENSHELL_INSTALL}/occt-7.3.0/lib)
        set(OCC_INCLUDE_DIR  ${IFCOPENSHELL_INSTALL}/occt-7.3.0/include/opencascade)
    endif()
    
    string(REGEX REPLACE "([^;]+)" "${OCC_LIBRARY_DIR}/lib\\1.a" OCC_LIBRARIES "${OCC_LIBRARY_NAMES}")

    if(NOT APPLE)
        set(LIB_RT rt)
    endif()

    find_package(Threads)
    find_library(dl NAMES dl)

    set(OCC_LIBRARIES -Wl,--start-group ${OCC_LIBRARIES} -Wl,--end-group)
    
    add_definitions(-Wno-deprecated-declarations)
else()
    if(IFC_SUPPORT)
        set(MSVC_YEAR 2017)
        if ("${MSVC_VERSION}" STREQUAL "1900")
            set(MSVC_YEAR 2015)
        endif()
        
        file(TO_CMAKE_PATH "${IFCOPENSHELL_ROOT}" IFCOPENSHELL_ROOT)
        if (EXISTS ${IFCOPENSHELL_ROOT}/deps-vs${MSVC_YEAR}-x86-installed/oce/Win32/lib)
            set(OCC_LIBRARY_DIR  ${IFCOPENSHELL_ROOT}/deps-vs${MSVC_YEAR}-x86-installed/oce/Win32/lib)    
        else()
            set(OCC_LIBRARY_DIR  ${IFCOPENSHELL_ROOT}/deps-vs${MSVC_YEAR}-x86-installed/opencascade-7.3.0/win32/lib)
        endif()
                             
        set(OCC_INCLUDE_DIR  ${IFCOPENSHELL_ROOT}/deps-vs${MSVC_YEAR}-x86-installed/oce/include/oce
                             ${IFCOPENSHELL_ROOT}/deps-vs${MSVC_YEAR}-x86-installed/opencascade-7.3.0/inc)
    else()
        file(TO_CMAKE_PATH "${OCC_LIBRARY_DIR}" OCC_LIBRARY_DIR)
    endif()
    
	add_definitions(-D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -D_UNICODE)
    
    string(REGEX REPLACE "([^;]+)" "${OCC_LIBRARY_DIR}/\\1.lib" OCC_LIBRARIES "${OCC_LIBRARY_NAMES}")
    
	set(Boost_USE_STATIC_LIBS   ON)
	set(Boost_USE_MULTITHREADED ON)

	if(IFC_SUPPORT)
		# todo detect
		# With IFC_SUPPORT we link against IfcOpenShell 0.5.0, otherwise
		# this is probably a dependency for IfcOpenShell 0.7.0. The first
		# uses a static runtime on win.
		set(Boost_USE_STATIC_RUNTIME ON)
    
		# Copied from IfcOpenShell to match compilation flags, todo detect whether necessary
		FOREACH(flag CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE CMAKE_CXX_FLAGS_MINSIZEREL
				CMAKE_CXX_FLAGS_RELWITHDEBINFO CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
				CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO)
			IF(${flag} MATCHES "/MD")
				STRING(REGEX REPLACE "/MD" "/MT" ${flag} "${${flag}}")
			ENDIF()
			IF(${flag} MATCHES "/MDd")
				STRING(REGEX REPLACE "/MDd" "/MTd" ${flag} "${${flag}}")
			ENDIF()
		ENDFOREACH()
    endif()

    set(WS2_LIBRARIES ws2_32.lib)
endif()

set(BOOST_COMPONENTS regex program_options iostreams system)

if(IFC_SUPPORT)
set(BOOST_COMPONENTS ${BOOST_COMPONENTS} filesystem)
if(UNIX)
    set(ICU_LIBRARY_NAMES icuuc icudata)
    
    set(IFCOPENSHELL_INSTALL ${IFCOPENSHELL_ROOT}/build/Linux/x86_64/install)
    
    set(BOOST_ROOT       ${IFCOPENSHELL_INSTALL}/boost-1.59.0)
    
    set(ICU_LIBRARY_DIR  ${IFCOPENSHELL_INSTALL}/icu-56.1/lib)
    set(ICU_INCLUDE_DIR  ${IFCOPENSHELL_INSTALL}/icu-56.1/include)
    
    set(IFC_INCLUDE_DIR  ${IFCOPENSHELL_ROOT}/src)
    set(IFC_LIBRARY_DIR  ${IFCOPENSHELL_INSTALL}/ifcopenshell/lib)
    set(IFC_LIBRARIES    ${IFC_LIBRARY_DIR}/libIfcGeom.a ${IFC_LIBRARY_DIR}/libIfcParse.a)
    
    string(REGEX REPLACE "([^;]+)" "${ICU_LIBRARY_DIR}/lib\\1.a" ICU_LIBRARIES "${ICU_LIBRARY_NAMES}")
else()
    set(ICU_LIBRARY_NAMES icuuc icudt)
    
    set(BOOST_ROOT       ${IFCOPENSHELL_ROOT}/deps/boost_1_67_0)
    set(BOOST_LIBRARYDIR ${IFCOPENSHELL_ROOT}/deps/boost_1_67_0/stage/vs${MSVC_YEAR}-Win32/lib)
	
    set(ICU_LIBRARY_DIR  ${IFCOPENSHELL_ROOT}/deps-vs${MSVC_YEAR}-x86-installed/icu/lib)
    set(ICU_INCLUDE_DIR  ${IFCOPENSHELL_ROOT}/deps-vs${MSVC_YEAR}-x86-installed/icu/include)
    
    set(IFC_INCLUDE_DIR  ${IFCOPENSHELL_ROOT}/src)
    set(IFC_LIBRARY_DIR  ${IFCOPENSHELL_ROOT}/build-vs${MSVC_YEAR}-x86/${CMAKE_BUILD_TYPE})
    set(IFC_LIBRARIES    ${IFC_LIBRARY_DIR}/IfcParse.lib ${IFC_LIBRARY_DIR}/IfcGeom.lib)
    
    string(REGEX REPLACE "([^;]+)" "${ICU_LIBRARY_DIR}/\\1.lib" ICU_LIBRARIES "${ICU_LIBRARY_NAMES}")
endif()
endif()

find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})

message(STATUS "Boost include files found in ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost libraries found in ${Boost_LIBRARY_DIRS}")

include_directories(${IFC_INCLUDE_DIR} ${OCC_INCLUDE_DIR} ${ICU_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include_directories(${INCLUDE_DIRECTORIES} ${OCC_INCLUDE_DIR} ${ICU_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${IFC_INCLUDE_DIR})
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    add_definitions(-std=c++14)
else()
    message(FATAL_ERROR "requires a compiler with C++11 support")
endif()

# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

if(not IFC_SUPPORT)
	# Prevent overriding the parent project's compiler/linker
	# settings on Windows
	set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
endif()

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

set(LIBVOXELIZER_HEADER_FILES progress.h dim3.h sweep.h shift.h collapse.h edge_detect.h fill_gaps.h offset.h polyfill.h resample.h storage.h writer.h factory.h processor.h volume.h voxelizer.h traversal.h util.h)
set(LIBVOXELIZER_SOURCE_FILES tribox3.cpp polyfill.cpp progress.cpp set_voxel_iterator.cpp factory.cpp)
add_library(libvoxel STATIC ${LIBVOXELIZER_HEADER_FILES} ${LIBVOXELIZER_SOURCE_FILES})
target_link_libraries(libvoxel ${IFC_LIBRARIES} ${ICU_LIBRARIES} ${Boost_LIBRARIES} ${OCC_LIBRARIES} ${LIB_RT} ${dl} ${CMAKE_THREAD_LIBS_INIT} ${WS2_LIBRARIES})

set(LIBVOXEC_HEADER_FILES voxelfile.h voxec.h)
set(LIBVOXEC_SOURCE_FILES voxec.cpp)
add_library(libvoxec STATIC ${LIBVOXEC_HEADER_FILES} ${LIBVOXEC_SOURCE_FILES})
target_link_libraries(libvoxec libvoxel)

file(GLOB test_filenames tests/*.cpp)
foreach(filename ${test_filenames})
    get_filename_component(test_name ${filename} NAME_WE)
    add_executable("${test_name}" ${filename})
    SET(libvoxec_IF_USED "")
    if ("${test_name}" MATCHES "parser")
    SET(libvoxec_IF_USED libvoxec)
    endif()
    target_link_libraries("${test_name}" gtest_main libvoxel ${libvoxec_IF_USED} ${IFC_LIBRARIES} ${ICU_LIBRARIES} ${Boost_LIBRARIES} ${OCC_LIBRARIES} ${LIB_RT} ${dl} ${CMAKE_THREAD_LIBS_INIT})
    add_test(NAME "${test_name}" COMMAND "${test_name}")
endforeach()

add_executable(voxec voxec_main.cpp)
target_link_libraries(voxec libvoxec)

install(TARGETS voxec libvoxel libvoxec
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install(FILES ${LIBVOXELIZER_HEADER_FILES} ${LIBVOXEC_HEADER_FILES} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/voxel)
