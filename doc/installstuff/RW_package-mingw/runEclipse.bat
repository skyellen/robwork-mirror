echo Setting up RobWork devel environment
echo -- PATH

set PATH=ROBWORKPACKAGE\boost_1_39_0;ROBWORKPACKAGE\MinGW\bin;ROBWORKPACKAGE\doxygen\bin;ROBWORKPACKAGE\xerces-c_2_8_0-mingw-msys\lib;%QT_ROOT_DIR%\bin;%PATH%

set MINGW_ROOT=ROBWORKPACKAGE\MinGW
set XERCES_ROOT=ROBWORKPACKAGE\xerces-c_2_8_0-mingw-msys
set BOOST_ROOT=ROBWORKPACKAGE\boost_1_39_0

start ROBWORKPACKAGE\eclipse\eclipse.exe -data ROBWORKPACKAGE\workspace