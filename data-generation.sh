#! /bin/bash

OUTPUT_DIR='data'

PWD=`pwd`
CP=`find $PWD/library/ -name '*.jar' | awk -F '\n' -v ORS=':' '{print}'`
CP=$CP:`find $PWD/library/rescue/ -name '*.jar' | awk -F '\n' -v ORS=':' '{print}'`

java -classpath "${CP}./build" matlab/generator/simple/LaunchSimpleAgents -1 -1 -1 -1 -1 -1 localhost 7000 ${OUTPUT_DIR}

for file in ${OUTPUT_DIR}/ambulance*.csv
do
  if [ ! -f ${OUTPUT_DIR}/ambulance.csv ]
  then
  head -1 $file >> ${OUTPUT_DIR}/ambulance.csv
  fi
  tail +2 $file >> ${OUTPUT_DIR}/ambulance.csv
  rm -f $file
done