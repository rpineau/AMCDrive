#!/bin/bash

mkdir -p ROOT/tmp/AMCDrive_X2/
cp "../AMCDrive.ui" ROOT/tmp/AMCDrive_X2/
cp "../domelist AMCDrive.txt" ROOT/tmp/AMCDrive_X2/
cp "../build/Release/libAMCDrive.dylib" ROOT/tmp/AMCDrive_X2/

if [ ! -z "$installer_signature" ]; then
# signed package using env variable installer_signature
pkgbuild --root ROOT --identifier org.rti-zone.AMCDrive_X2 --sign "$installer_signature" --scripts Scripts --version 1.0 AMCDrive_X2.pkg
pkgutil --check-signature ./AMCDrive_X2.pkg
else
pkgbuild --root ROOT --identifier org.rti-zone.AMCDrive_X2 --scripts Scripts --version 1.0 AMCDrive_X2.pkg
fi

rm -rf ROOT
