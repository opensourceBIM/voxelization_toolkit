mkdir build && cd build

cmake -G "Ninja" ^
 -D CMAKE_BUILD_TYPE:STRING=Release ^
 -D CMAKE_INSTALL_PREFIX:FILEPATH="%LIBRARY_PREFIX%" ^
 -D CMAKE_PREFIX_PATH:FILEPATH="%LIBRARY_PREFIX%" ^
 -D CMAKE_SYSTEM_PREFIX_PATH:FILEPATH="%LIBRARY_PREFIX%" ^
 ^
 -DUSE_BUILD_SCRIPT_OUTPUT=Off ^
 -DUSE_STATIC_MSVC_RUNTIME=Off ^
 -DBoost_USE_STATIC_LIBS=Off ^
 -DIFC_INCLUDE_DIR=%PREFIX%\include ^
 -DIFC_LIBRARY_DIR=%PREFIX%\lib ^
 -DOCC_INCLUDE_DIR=%PREFIX%\include/opencascade ^
 -DOCC_LIBRARY_DIR=%PREFIX%\lib ^
 %SRC_DIR%
 
if errorlevel 1 exit 1

:: Build and install
cmake --build . -- install

if errorlevel 1 exit 1
