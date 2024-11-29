cmake_minimum_required (VERSION 3.21)
project (voxel)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

OPTION(IFC_SUPPORT "Build Volization Toolkit with IFC support (requires IfcOpenShell/master)." ON)
OPTION(USE_STATIC_MSVC_RUNTIME "Link to the static runtime on MSVC." OFF)
OPTION(ENABLE_TESTS "Enable tests." ON)
OPTION(USE_BUILD_SCRIPT_OUTPUT "Use the ifcopenshell nix/build-all.py or win/build-all.cmd build script output to find dependencies." ON)
OPTION(ENABLE_PYTHON_WRAPPER "Enable Python wrapper." OFF)

include(CTest)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(WIN_ARCH x64)
elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(WIN_ARCH Win32)
endif()

if(UNIX)
    add_definitions(-fPIC)
endif()

set(OCC_LIBRARY_NAMES TKernel TKMath TKBRep TKGeomBase TKGeomAlgo TKG3d TKG2d TKShHealing TKTopAlgo TKMesh TKOffset
                      TKPrim TKBool TKBO TKFillet TKSTEP TKSTEPBase TKSTEPAttr TKXSBase TKSTEP209 TKIGES TKHLR TKFeat)
                      
if(IFC_SUPPORT)
add_definitions(-DWITH_IFC)
set(SWIG_DEFINES ${SWIG_DEFINES} -DWITH_IFC)
endif()

if (USE_BUILD_SCRIPT_OUTPUT)
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
				set(OCC_LIBRARY_DIR  ${IFCOPENSHELL_ROOT}/deps-vs${MSVC_YEAR}-x86-installed/opencascade-7.3.0/win32/lib)
			endif()
                             
			set(OCC_INCLUDE_DIR  ${IFCOPENSHELL_ROOT}/deps-vs${MSVC_YEAR}-x86-installed/oce/include/oce
								 ${IFCOPENSHELL_ROOT}/deps-vs${MSVC_YEAR}-x86-installed/opencascade-7.3.0/inc)
		else()
			file(TO_CMAKE_PATH "${OCC_LIBRARY_DIR}" OCC_LIBRARY_DIR)
		endif()
    
		# set(Boost_USE_STATIC_LIBS   ON)
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
	endif()
else()
	# user to supply:
	#  * IFCOPENSHELL_ROOT or IFC_INCLUDE_DIR
	#  * OCC_INCLUDE_DIR
	#  * OCC_LIBRARY_DIR
	#  * BOOST_ROOT
	#  * IFC_LIBRARY_DIR
	#  * (BOOST_LIBRARYDIR)
	#  * (USE_STATIC_MSVC_RUNTIME=Off)
	#  * (Boost_USE_STATIC_LIBS=On)

	if (NOT DEFINED IFC_INCLUDE_DIR)
		set (IFC_INCLUDE_DIR "${IFCOPENSHELL_ROOT}/src")
	endif()
endif()

foreach(libname ${OCC_LIBRARY_NAMES})
	unset(lib CACHE)
    find_library(lib NAMES "${libname}" PATHS "${OCC_LIBRARY_DIR}" NO_DEFAULT_PATH)
        if (lib)
                list(APPEND OCC_LIBRARIES "${lib}")
        else()
                message(WARNING "OCC library ${libname} not found in ${OCC_LIBRARY_DIR}")
		message(STATUS "${OCC_LIBRARY_DIR}")
        endif()
endforeach()

if (UNIX)
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
	add_definitions(-D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -D_UNICODE)    
	set(WS2_LIBRARIES ws2_32.lib)
endif()

set(BOOST_COMPONENTS regex program_options iostreams system filesystem)
# add_definitions(-DBOOST_ALL_NO_LIB)

if(IFC_SUPPORT AND USE_BUILD_SCRIPT_OUTPUT)
	if(UNIX)
		set(BOOST_ROOT       ${IFCOPENSHELL_INSTALL}/boost-1.71.0)
    
		set(IFC_INCLUDE_DIR  ${IFCOPENSHELL_ROOT}/src)
		set(IFC_LIBRARY_DIR  ${IFCOPENSHELL_INSTALL}/ifcopenshell/lib)
    
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
    
		if (${ifcopenshell_major_version} STREQUAL "0.5")
			set(ICU_LIBRARY_NAMES icuuc icudt)
			set(ICU_LIBRARY_DIR  ${IFCOPENSHELL_ROOT}/deps-vs${MSVC_YEAR}-x86-installed/icu/lib)
			set(ICU_INCLUDE_DIR  ${IFCOPENSHELL_ROOT}/deps-vs${MSVC_YEAR}-x86-installed/icu/include)
			string(REGEX REPLACE "([^;]+)" "${ICU_LIBRARY_DIR}/\\1.lib" ICU_LIBRARIES "${ICU_LIBRARY_NAMES}")
		endif()
	endif()
endif()

if (IFC_SUPPORT)
	# Find IfcOpenShell version, v0.6.0 does not have the IfcSchema namespace anymore
	file(READ "${IFC_INCLUDE_DIR}/ifcparse/IfcParse.h" header)
	string(REGEX MATCH "#define IFCOPENSHELL_VERSION \"[0-9a-zA-Z\\.\\-]+\"" ifcopenshell_version "${header}")
	string(REGEX MATCH "[0-9]\\.[0-9]" ifcopenshell_major_version "${ifcopenshell_version}")
	message(STATUS "IfcOpenShell version ${ifcopenshell_major_version}")

	if (${ifcopenshell_major_version} STREQUAL "0.5")
		set(IFC_LIBRARY_NAMES IfcGeom IfcParse)
		add_definitions("-DIFCOPENSHELL_05")
		set(SWIG_DEFINES ${SWIG_DEFINES} -DIFCOPENSHELL_05)
	else()
		if(${ifcopenshell_major_version} STREQUAL "0.7" OR
			${ifcopenshell_major_version} STREQUAL "0.8")

			if(${ifcopenshell_major_version} STREQUAL "0.8")
				add_definitions("-DIFCOPENSHELL_08")
			else()
				add_definitions("-DIFCOPENSHELL_07")
			endif()
			
			set(SWIG_DEFINES ${SWIG_DEFINES} -DIFCOPENSHELL_07)

			string(REPLACE ";" ")(" schema_version_seq "(${SCHEMA_VERSIONS})")
			ADD_DEFINITIONS(-DSCHEMA_SEQ=${schema_version_seq})
			# set(SWIG_DEFINES ${SWIG_DEFINES} -DSCHEMA_SEQ=${schema_version_seq})

			IF(MSVC)
				# warning due to virtual inheritance
				ADD_DEFINITIONS(-wd4250)
				# warning due to select definitions in the schema being redundant
				ADD_DEFINITIONS(-wd4584)
			endif()
		endif()

		if (${ifcopenshell_major_version} STREQUAL "0.8")
			file(GLOB geomlibfiles ${IFC_LIBRARY_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}geometry_mapping_*)
			foreach(file ${geomlibfiles})
				get_filename_component(file_we ${file} NAME_WE)
				string(REPLACE "${CMAKE_SHARED_LIBRARY_PREFIX}geometry_mapping_ifc" "" schema_name ${file_we})
				list(APPEND SCHEMA_VERSIONS ${schema_name})
			endforeach()
			file(GLOB geomlibfiles ${IFC_LIBRARY_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}geometry_kernel_*)
			foreach(file ${geomlibfiles})
				get_filename_component(file_we ${file} NAME_WE)
				string(REPLACE "${CMAKE_SHARED_LIBRARY_PREFIX}" "" file_we_wp ${file_we})
				if ("${file_we_wp}" STREQUAL "geometry_kernel_cgal")
					find_library(libGMP NAMES gmp mpir PATHS ${GMP_LIBRARY_DIR} NO_DEFAULT_PATH)
					find_library(libMPFR NAMES mpfr PATHS ${MPFR_LIBRARY_DIR} NO_DEFAULT_PATH)
					if(NOT libGMP)
						message(FATAL_ERROR "Unable to find GMP library files, aborting")
					endif()
					if(NOT libMPFR)
						message(FATAL_ERROR "Unable to find MPFR library files, aborting")
					endif()

					list(APPEND CGAL_LIBRARIES "${libMPFR}")
					list(APPEND CGAL_LIBRARIES "${libGMP}")
				endif()
				list(APPEND IFC_LIBRARY_NAMES "${file_we_wp}")
			endforeach()
		else()
			file(GLOB geomlibfiles ${IFC_LIBRARY_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}IfcGeom_*)
			foreach(file ${geomlibfiles})
				get_filename_component(file_we ${file} NAME_WE)
				string(REPLACE "${CMAKE_SHARED_LIBRARY_PREFIX}IfcGeom_ifc" "" schema_name ${file_we})
				list(APPEND SCHEMA_VERSIONS ${schema_name})
			endforeach()
		endif()

		list(APPEND IFC_LIBRARY_NAMES IfcGeom)
		foreach(schema_name ${SCHEMA_VERSIONS})
			message(STATUS "Detected schema ifc${schema_name}")
			if (${ifcopenshell_major_version} STREQUAL "0.8")
				list(APPEND IFC_LIBRARY_NAMES geometry_mapping_ifc${schema_name})
			else()
				list(APPEND IFC_LIBRARY_NAMES IfcGeom_ifc${schema_name})
			endif()
		endforeach()
		list(APPEND IFC_LIBRARY_NAMES IfcParse)
	endif()
endif()

if (UNIX)
	set(IFC_LIBRARY_NAMES ${IFC_LIBRARY_NAMES} ${IFC_LIBRARY_NAMES} ${IFC_LIBRARY_NAMES} ${IFC_LIBRARY_NAMES})
endif()

foreach(libname ${IFC_LIBRARY_NAMES})
	unset(lib CACHE)
	message("Looking for ${libname} in ${IFC_LIBRARY_DIR}")
    find_library(lib NAMES "${libname}" PATHS "${IFC_LIBRARY_DIR}" NO_DEFAULT_PATH)
	if (lib)
		list(APPEND IFC_LIBRARIES "${lib}")
	else()
		message(WARNING "IfcOpenShell library ${libname} not found")
	endif()
endforeach()

find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})

message(STATUS "Boost include files found in ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost libraries found in ${Boost_LIBRARY_DIRS}")

include_directories(${IFC_INCLUDE_DIR} ${OCC_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/eigen)
link_directories(${Boost_LIBRARY_DIRS})

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

if(MSVC)
    SET(BCRYPT_LIBRARIES "bcrypt.lib")
ENDIF()

set(LIBVOXELIZER_HEADER_FILES progress_writer.h dim3.h sweep.h shift.h collapse.h edge_detect.h fill_gaps.h offset.h polyfill.h resample.h storage.h writer.h factory.h processor.h volume.h voxelizer.h traversal.h memoized_traversal.h lru_cache.h util.h json_logger.h)
set(LIBVOXELIZER_SOURCE_FILES tribox3.cpp polyfill.cpp progress_writer.cpp storage.cpp factory.cpp json_logger.cpp)
add_library(libvoxel STATIC ${LIBVOXELIZER_HEADER_FILES} ${LIBVOXELIZER_SOURCE_FILES})
target_link_libraries(libvoxel ${IFC_LIBRARIES} ${ICU_LIBRARIES} ${Boost_LIBRARIES} ${BCRYPT_LIBRARIES} ${OCC_LIBRARIES} ${CGAL_LIBRARIES} ${LIB_RT} ${dl} ${CMAKE_THREAD_LIBS_INIT} ${WS2_LIBRARIES})

set(LIBVOXEC_HEADER_FILES voxelfile.h voxec.h)
set(LIBVOXEC_SOURCE_FILES voxec.cpp)
add_library(libvoxec STATIC ${LIBVOXEC_HEADER_FILES} ${LIBVOXEC_SOURCE_FILES})
target_link_libraries(libvoxec libvoxel)

if (ENABLE_PYTHON_WRAPPER)
	FIND_PACKAGE(SWIG)
	IF(NOT SWIG_FOUND)
		MESSAGE(FATAL_ERROR "ENABLE_PYTHON_WRAPPER enabled, but unable to find SWIG. Disable ENABLE_PYTHON_WRAPPER or fix SWIG paths to proceed.")
	ENDIF()
	INCLUDE(${SWIG_USE_FILE})
	find_package(Python COMPONENTS Interpreter Development)
	IF(NOT Python_FOUND)
		MESSAGE(FATAL_ERROR "ENABLE_PYTHON_WRAPPER enabled, but unable to find Python lib or header. Disable ENABLE_PYTHON_WRAPPER or fix Python paths to proceed.")
	ENDIF()
	
	INCLUDE_DIRECTORIES(${Python_INCLUDE_DIRS})
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
	link_directories(${Python_LIBRARY_DIRS})
	
	SET(CMAKE_SWIG_FLAGS ${SWIG_DEFINES})
	SET_SOURCE_FILES_PROPERTIES(wrap/wrapper.i PROPERTIES CPLUSPLUS ON)
	swig_add_library(voxec LANGUAGE python SOURCES wrap/wrapper.i)
	if("$ENV{LDFLAGS}" MATCHES ".undefined.suppress")
		# On osx there is some state in the python dylib. With `-Wl,undefined,suppress` we can ignore the missing symbols at compile time.
		SWIG_LINK_LIBRARIES(voxec libvoxec)
	else()
		SWIG_LINK_LIBRARIES(voxec libvoxec ${Python_LIBRARIES})
	endif()

    INSTALL(FILES "${CMAKE_BINARY_DIR}/voxec.py" "${CMAKE_CURRENT_SOURCE_DIR}/module/__init__.py"
            DESTINATION "${Python_SITEARCH}/voxec/")
    INSTALL(TARGETS ${SWIG_MODULE_voxec_REAL_NAME}
            DESTINATION "${Python_SITEARCH}/voxec/")
endif()

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

add_executable(voxec_exe voxec_main.cpp progress.h)
if(MSVC)
	target_link_options(voxec_exe PRIVATE "/NOIMPLIB")
endif()
target_link_libraries(voxec_exe libvoxec)
set_property(TARGET voxec_exe PROPERTY OUTPUT_NAME voxec)

install(TARGETS voxec_exe libvoxel libvoxec
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

install(FILES ${LIBVOXELIZER_HEADER_FILES} ${LIBVOXEC_HEADER_FILES} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/voxel)
