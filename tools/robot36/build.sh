#!/bin/sh
# Build robot36's decoder headless: build.sh REF_DIR BUILD_DIR -> BUILD_DIR/robot36-ref.jar
# Needs a JDK (javac, jar). SPDX-License-Identifier: ISC
set -eu
here=$(cd "$(dirname "$0")" && pwd)
ref=${1:-$here/../../ref/robot36}
out=${2:-$here/../../build/robot36}
src=$ref/app/src/main/java/xdsopl/robot36
mkdir -p "$out/classes"
files="Decoder Demodulator Mode BaseMode PixelBuffer RawDecoder HFFax RGBDecoder RGBModes
       Robot_36_Color Robot_72_Color PaulDon ColorConverter Complex ComplexConvolution Delay
       ExponentialMovingAverage FastFourierTransform Filter FrequencyModulation Hann Kaiser
       Phasor SchmittTrigger SimpleMovingAverage SimpleMovingSum"
list=""
for f in $files; do list="$list $src/$f.java"; done
javac -nowarn -encoding UTF-8 -d "$out/classes" $here/stub/android/graphics/*.java \
    $here/src/xdsopl/robot36/Robot36Ref.java $list
printf 'Main-Class: xdsopl.robot36.Robot36Ref\n' > "$out/manifest.txt"
jar cfm "$out/robot36-ref.jar" "$out/manifest.txt" -C "$out/classes" .
echo "built $out/robot36-ref.jar"
