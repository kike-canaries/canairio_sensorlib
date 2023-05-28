#!/bin/bash

######################################################
# CanAirIO deploy release utility
#
# Author: @hpsaturn
# 2021
######################################################

SRC_VER=`cat library.properties | grep version | sed -n -e 's/^.*version=//p'`
HDR_VER=`cat src/Sensors.hpp | grep CSL_VERSION | awk '{ print $3 }'`
SRC_REV=`cat src/Sensors.hpp | grep CSL_REVISION | awk '{ print $3 }'`
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

validate_version() {
  if [ "\"${SRC_VER}\"" != "${HDR_VER}" ]; then
      echo ""
      echo "Error: Version mistmatch with header version!"
      echo ""
      echo "revision library: $SRC_REV"
      echo "version library : \"$SRC_VER\""
      echo "version header  : $HDR_VER"
      exit 1
  fi
}

validate_branch () {
  current_branch=`git rev-parse --abbrev-ref HEAD` 

  if [ ${current_branch} != "master" ]; then
    echo ""
    echo "Error: you are in ${current_branch} branch please change to master branch."
    echo ""
    exit 1
  fi 
}

clean () {
  rm -rf .pio
  rm -rf examples/advanced_sensirion/.pio
  rm -rf examples/ttgo_tdisplay_s3/.pio
  rm -f $OUTPUT
}

runtest () {  
  current_dir=`pwd`
  echo ""
  echo "***********************************************"
  echo "** TESTING: $2"
  echo "***********************************************"
  echo ""
  cd $1
  pio run -s 
  echo ""
  echo "***********************************************"
  echo "** TEST DONE ON: $2"
  echo "***********************************************"
  echo ""
  cd $current_dir
} 

runtests () {
   runtest "examples/advanced_sensirion" "Advanced Sensirion"
   runtest "examples/ttgo_tdisplay_s3" "TTGO T-Display S3"
   runtest "./" "All architectures"
}

build () {
  
  clean
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
  du -hs $OUTPUT
  echo ""
}

publish_release () {
  echo ""
  echo "***********************************************"
  echo "********** Publishing release *****************" 
  echo "***********************************************"
  echo ""
  echo "Publishing release: v${SRC_VER} rev${SRC_REV}" 
  echo "uploading: ${OUTPUT}"
  COMMIT_LOG=`git log -1 --format='%ci %H %s'`
  git tag -a "v${SRC_VER}" -m "release v${SRC_VER} rev${SRC_REV}"
  git push origin "v${SRC_VER}"
  git log -n 10 --pretty=format:"%h %s" | gh release create "v${SRC_VER}" -F - -t "v${SRC_VER} rev${SRC_REV}" -p ${OUTPUT} 
  echo ""
  echo "***********************************************"
  echo "*************     done    *********************" 
  echo "***********************************************"
  echo ""
}

publish_pio () {
  pio package publish
}

if [ "$1" = "" ]; then
  showHelp
else
  validate_version
  validate_branch
  case "$1" in
    clean)
      clean
      ;;

    test)
      runtests
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

