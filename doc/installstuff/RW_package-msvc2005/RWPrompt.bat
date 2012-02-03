echo Setting up RobWork devel environment
echo -- PATH

set PATH=ROBWORKPACKAGE\boost_1_39_0;ROBWORKPACKAGE\MinGW\bin;ROBWORKPACKAGE\doxygen\bin;ROBWORKPACKAGE\xerces-c_2_8_0-mingw-msys\lib;%PATH%

set XERCES_ROOT=ROBWORKPACKAGE\xerces-c_2_8_0-mingw-msys
set BOOST_ROOT=ROBWORKPACKAGE\boost_1_39_0

cmd.exe ROBWORKPACKAGE\workspace
cd ROBWORKPACKAGE\workspace