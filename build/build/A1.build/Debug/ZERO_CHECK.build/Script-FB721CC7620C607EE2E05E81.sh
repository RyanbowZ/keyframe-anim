#!/bin/sh
set -e
if test "$CONFIGURATION" = "Debug"; then :
  cd /Users/ryanbowz/Documents/xcode/CSCE689/A1/634005710/build
  make -f /Users/ryanbowz/Documents/xcode/CSCE689/A1/634005710/build/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "Release"; then :
  cd /Users/ryanbowz/Documents/xcode/CSCE689/A1/634005710/build
  make -f /Users/ryanbowz/Documents/xcode/CSCE689/A1/634005710/build/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "MinSizeRel"; then :
  cd /Users/ryanbowz/Documents/xcode/CSCE689/A1/634005710/build
  make -f /Users/ryanbowz/Documents/xcode/CSCE689/A1/634005710/build/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "RelWithDebInfo"; then :
  cd /Users/ryanbowz/Documents/xcode/CSCE689/A1/634005710/build
  make -f /Users/ryanbowz/Documents/xcode/CSCE689/A1/634005710/build/CMakeScripts/ReRunCMake.make
fi

