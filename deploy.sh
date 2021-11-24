#!/bin/bash

######################################################
# CanAirIO deploy release utility
#
# Author: @hpsaturn
# 2021
######################################################

SRC_VER=`cat library.properties | grep version | sed -n -e 's/^.*version=//p'`
SRC_REV=`cat platformio.ini | grep SRC_REV | sed -n -e 's/^.*SRC_REV=//p'`
DATE=`date +%Y%m%d`
RELDIR="releases"
RELNAME="CanAirIOAirQualitySensorsLibrary-${SRC_VER}.tar.gz"
OUTPUT="${RELDIR}/${RELNAME}" 

showHelp () {
  echo ""
  echo "************************************************"
  echo "** Build and deploy tag and release           **"
  echo "************************************************"
  echo ""
  echo "Usage alternatives:"
  echo ""
  echo "./deploy_release test"
  echo "./deploy_release clean"
  echo "./deploy_release build"
  echo "./deploy_release github"
  echo "./deploy_release pio"
  echo ""
}

clean () {
  rm -f $OUTPUT
}

runtest () {
  pio run --target clean && pio run 
}

build () {

  echo ""
  echo "***********************************************"
  echo "** Building rev$SRC_REV ($SRC_VER)"
  echo "***********************************************"
  echo ""
  pio package pack -o $RELDIR/
  echo ""
  tar ztf $OUTPUT
  echo ""
  echo "***********************************************"
  echo "************** Build done *********************" 
  echo "***********************************************"
  echo ""
  md5sum $OUTPUT
  echo ""
}

publish_release () {
  echo ""
  echo "***********************************************"
  echo "********** Publishing release *****************" 
  echo "***********************************************"
  echo ""
  COMMIT_LOG=`git log -1 --format='%ci %H %s'`
  github-release upload --owner kike-canaries --repo canairio_sensorlib --tag "rev${SRC_REV}" --release-name "v${SRC_VER} rev${SRC_REV}" --body "${COMMIT_LOG}" $OUTPUT
  echo ""
  echo "***********************************************"
  echo "*************     done    *********************" 
  echo "***********************************************"
  echo ""
}

publish_pio () {
  pio package publish
}

current_branch=`git rev-parse --abbrev-ref HEAD` 

if [ ${current_branch} != "master" ]; then
  echo ""
  echo "Error: you are in ${current_branch} branch please change to master branch."
  echo ""
  exit 1
fi 

if [ "$1" = "" ]; then
  showHelp
else
  case "$1" in
    clean)
      clean
      ;;

    test)
      runtest 
      ;;

    help)
      showHelp
      ;;

    --help)
      showHelp
      ;;

    -help)
      showHelp
      ;;

    -h)
      showHelp
      ;;

    print)
      printOutput
      ;;

    pio)
      publish_pio
      ;;

    publish)
      publish_release
      ;;

    github)
      publish_release
      ;;

    *)
      build $1
      ;;
  esac
fi

exit 0

