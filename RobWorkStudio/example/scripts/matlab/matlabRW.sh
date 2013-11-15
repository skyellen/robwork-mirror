#!/bin/bash
export RW_ROOT=/home/thomas/Eclipse/RobWork
export RW_BUILD=debug

export RW_LIBS=$RW_ROOT/RobWork/libs/$RW_BUILD
export RWStudio_LIBS=$RW_ROOT/RobWorkStudio/libs/$RW_BUILD
export RWSim_LIBS=$RW_ROOT/RobWorkSim/libs/$RW_BUILD

export MATLAB_JAVA=$JAVA_HOME/jre
export LD_PRELOAD=" /usr/lib/x86_64-linux-gnu/libstdc++.so.6"
export LD_PRELOAD="$LD_PRELOAD /usr/lib/x86_64-linux-gnu/libgfortran.so.3"

/opt/MATLAB/bin/matlab
