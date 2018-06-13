#!/bin/sh

LOADER="matlab.Loader"
MATLAB_PATH="/data/tools/matlab/R2018a"

cd `dirname $0`

PWD=`pwd`
CP=`find $PWD/library/ -name '*.jar' ! -name '*-sources.jar' | awk -F '\n' -v ORS=':' '{print}'`
CP=$CP$MATLAB_PATH"/extern/engines/java/jar/engine.jar:"

java -Djava.library.path=${MATLAB_PATH}/bin/glnxa64 -classpath "${CP}./build" adf.Main ${LOADER} -t $1,$2,$3,$4,$5,$6 -h $7 -pre true
