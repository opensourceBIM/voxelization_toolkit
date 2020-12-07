cmake_minimum_required (VERSION 2.8.11)
project (voxel)

OPTION(IFC_SUPPORT "Build Volization Toolkit with IFC support (requires IfcOpenShell/master)." ON)
OPTION(USE_STATIC_MSVC_RUNTIME "Link to the static runtime on MSVC." ON)
OPTION(ENABLE_TESTS "Enable tests." ON)

include(CTest)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(WIN_ARCH x64)
elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(WIN_ARCH Win32)
endif()

set(OCC_LIBRARY_NAMES TKernel TKMath TKBRep TKGeomBase TKGeomAlgo TKG3d TKG2d TKShHealing TKTopAlgo TKMesh TKOffset
                      TKPrim TKBool TKBO TKFillet TKSTEP TKSTEPBase TKSTEPAttr TKXSBase TKSTEP209 TKIGES TKHLR TKFeat)
                      
if(IFC_SUPPORT)
add_definitions(-DWITH_IFC)
endif()


if(UNIX)
    set(IFCOPENSHELL_INSTALL ${IFCOPENSHELL_ROOT}/build/Linux/x86_64/install)
    
    if(IFC_SUPPORT)
        foreach(occt_dir occt-7.3.0 occt-7.3.0p3)
            if(EXISTS "${IFCOPENSHELL_INSTALL}/${occt_dir}/")
                set(OCCT_PATH "${IFCOPENSHELL_INSTALL}/${occt_dir}/")
            endif()
        endforeach()

        set(OCC_LIBRARY_DIR  ${OCCT_PATH}/lib)
        set(OCC_INCLUDE_DIR  ${OCCT_PATH}/include/opencascade)
    endif()
    
    foreach(libname ${OCC_LIBRARY_NAMES})
        unset(lib CACHE)
    	find_library(lib NAMES "${libname}" PATHS "${OCC_LIBRARY_DIR}" NO_DEFAULT_PATH)
    	list(APPEND OCC_LIBRARIES "${lib}")
    endforeach()

    if(NOT APPLE)
        set(LIB_RT rt)
    endif()

    find_package(Threads)
    find_library(dl NAMES dl)

    if(NOT APPLE)
        set(OCC_LIBRARIES -Wl,--start-group ${OCC_LIBRARIES} -Wl,--end-group)
    endif()
    
    add_definitions(-Wno-deprecated-declarations)
else()
    if(IFC_SUPPORT)
        set(MSVC_YEAR 2017)
        if ("${MSVC_VERSION}" STREQUAL "1900")
            set(MSVC_YEAR 2015)
        endif()
        
        file(TO_CMAKE_PATH "${IFCOPENSHELL_ROOT}" IFCOPENSHELL_ROOT)
        if (EXISTS ${IFCOPENSHELL_ROOT}/deps-vs${MSVC_YEAR}-x86-installed/oce/${WIN_ARCH}/lib)
            set(OCC_LIBRARY_DIR  ${IFCOPENSHELL_ROOT}/deps-vs${MSVC_YEAR}-x86-installed/oce/${WIN_ARCH}/lib)    
        else()
            set(OCC_LIBRARY_DIR  ${IFCOPENSHELL_ROOT}/deps-vs${MSVC_YEAR}-x86-installed/opencascade-7.3.0p3/win32/lib)
        endif()
                             
        set(OCC_INCLUDE_DIR  ${IFCOPENSHELL_ROOT}/deps-vs${MSVC_YEAR}-x86-installed/oce/include/oce
                             ${IFCOPENSHELL_ROOT}/deps-vs${MSVC_YEAR}-x86-installed/opencascade-7.3.0p3/inc)
    else()
        file(TO_CMAKE_PATH "${OCC_LIBRARY_DIR}" OCC_LIBRARY_DIR)
    endif()
    
	add_definitions(-D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -D_UNICODE)
    
    string(REGEX REPLACE "([^;]+)" "${OCC_LIBRARY_DIR}/\\1.lib" OCC_LIBRARIES "${OCC_LIBRARY_NAMES}")
    
	set(Boost_USE_STATIC_LIBS   ON)
	set(Boost_USE_MULTITHREADED ON)

	if(USE_STATIC_MSVC_RUNTIME)
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

if (IFC_SUPPORT)
# Find IfcOpenShell version, v0.6.0 does not have the IfcSchema namespace anymore
file(READ "${IFCOPENSHELL_ROOT}/src/ifcparse/IfcParse.h" header)
string(REGEX MATCH "#define IFCOPENSHELL_VERSION \"[0-9a-zA-Z\\.\\-]+\"" ifcopenshell_version "${header}")
string(REGEX MATCH "[0-9]\\.[0-9]" ifcopenshell_major_version "${ifcopenshell_version}")
message(STATUS "IfcOpenShell version ${ifcopenshell_major_version}")

if (${ifcopenshell_major_version} STREQUAL "0.5")
set(IFC_LIBRARY_NAMES IfcGeom IfcParse)
add_definitions("-DIFCOPENSHELL_05")
else()
set(IFC_LIBRARY_NAMES IfcGeom IfcGeom_ifc2x3 IfcGeom_ifc4 IfcGeom_ifc4x1
                      IfcGeom_ifc4x2 IfcGeom_ifc4x3_rc1 IfcParse)
endif()
endif()

if (UNIX)
set(IFC_LIBRARY_NAMES ${IFC_LIBRARY_NAMES} ${IFC_LIBRARY_NAMES})
endif()

if(IFC_SUPPORT)
set(BOOST_COMPONENTS ${BOOST_COMPONENTS} filesystem)
if(UNIX)
    set(BOOST_ROOT       ${IFCOPENSHELL_INSTALL}/boost-1.59.0)
    
    set(IFC_INCLUDE_DIR  ${IFCOPENSHELL_ROOT}/src)
    set(IFC_LIBRARY_DIR  ${IFCOPENSHELL_INSTALL}/ifcopenshell/lib)
    
    string(REGEX REPLACE "([^;]+)" "${IFC_LIBRARY_DIR}/lib\\1.a" IFC_LIBRARIES "${IFC_LIBRARY_NAMES}")

    if (${ifcopenshell_major_version} STREQUAL "0.5")
        set(ICU_LIBRARY_NAMES icuuc icudata)
        set(ICU_LIBRARY_DIR  ${IFCOPENSHELL_INSTALL}/icu-56.1/lib)
        set(ICU_INCLUDE_DIR  ${IFCOPENSHELL_INSTALL}/icu-56.1/include)
        string(REGEX REPLACE "([^;]+)" "${ICU_LIBRARY_DIR}/lib\\1.a" ICU_LIBRARIES "${ICU_LIBRARY_NAMES}")
    endif()
else()
    set(BOOST_ROOT       ${IFCOPENSHELL_ROOT}/deps/boost_1_67_0)
    set(BOOST_LIBRARYDIR ${IFCOPENSHELL_ROOT}/deps/boost_1_67_0/stage/vs${MSVC_YEAR}-${WIN_ARCH}/lib)
	
    set(IFC_INCLUDE_DIR  ${IFCOPENSHELL_ROOT}/src)
    set(IFC_LIBRARY_DIR  ${IFCOPENSHELL_ROOT}/build-vs${MSVC_YEAR}-x86/${CMAKE_BUILD_TYPE})
    
    string(REGEX REPLACE "([^;]+)" "${IFC_LIBRARY_DIR}/\\1.lib" IFC_LIBRARIES "${IFC_LIBRARY_NAMES}")

    if (${ifcopenshell_major_version} STREQUAL "0.5")
        set(ICU_LIBRARY_NAMES icuuc icudt)
        set(ICU_LIBRARY_DIR  ${IFCOPENSHELL_ROOT}/deps-vs${MSVC_YEAR}-x86-installed/icu/lib)
        set(ICU_INCLUDE_DIR  ${IFCOPENSHELL_ROOT}/deps-vs${MSVC_YEAR}-x86-installed/icu/include)
        string(REGEX REPLACE "([^;]+)" "${ICU_LIBRARY_DIR}/\\1.lib" ICU_LIBRARIES "${ICU_LIBRARY_NAMES}")
    endif()
endif()
endif()

find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})

message(STATUS "Boost include files found in ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost libraries found in ${Boost_LIBRARY_DIRS}")

include_directories(${IFC_INCLUDE_DIR} ${OCC_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include_directories(${INCLUDE_DIRECTORIES} ${OCC_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${IFC_INCLUDE_DIR})
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    add_definitions(-std=c++14)
else()
    message(FATAL_ERROR "requires a compiler with C++11 support")
endif()

if (ENABLE_TESTS)

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

if(NOT USE_STATIC_MSVC_RUNTIME)
	# Prevent overriding the parent project's compiler/linker
	# settings on Windows
	set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
endif()

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)
endif()

set(LIBVOXELIZER_HEADER_FILES progress.h dim3.h sweep.h shift.h collapse.h edge_detect.h fill_gaps.h offset.h polyfill.h resample.h storage.h writer.h factory.h processor.h volume.h voxelizer.h traversal.h util.h)
set(LIBVOXELIZER_SOURCE_FILES tribox3.cpp polyfill.cpp progress.cpp storage.cpp factory.cpp)
add_library(libvoxel STATIC ${LIBVOXELIZER_HEADER_FILES} ${LIBVOXELIZER_SOURCE_FILES})
target_link_libraries(libvoxel ${IFC_LIBRARIES} ${ICU_LIBRARIES} ${Boost_LIBRARIES} ${OCC_LIBRARIES} ${LIB_RT} ${dl} ${CMAKE_THREAD_LIBS_INIT} ${WS2_LIBRARIES})

set(LIBVOXEC_HEADER_FILES voxelfile.h voxec.h)
set(LIBVOXEC_SOURCE_FILES voxec.cpp)
add_library(libvoxec STATIC ${LIBVOXEC_HEADER_FILES} ${LIBVOXEC_SOURCE_FILES})
target_link_libraries(libvoxec libvoxel)

if (ENABLE_TESTS)
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
endif()

add_executable(voxec voxec_main.cpp)
target_link_libraries(voxec libvoxec)

install(TARGETS voxec libvoxel libvoxec
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install(FILES ${LIBVOXELIZER_HEADER_FILES} ${LIBVOXEC_HEADER_FILES} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/voxel)
