#!/bin/zsh
set -euo pipefail
trap 'print -u2 "FATAL: native dependency build failed"' ZERR
[[ "$(uname -s)" == Darwin ]] || { print -u2 'FATAL: this native build script currently targets macOS'; exit 1; }
root="${0:A:h}/../../../.."
native="${root:A}/.tools/omega-native"
mkdir -p "$native"
fetch() {
  local filename="$1" url="$2" checksum="$3"
  if [[ ! -f "$native/$filename" ]]; then
    curl -fL --retry 2 "$url" -o "$native/$filename.partial"
    mv "$native/$filename.partial" "$native/$filename"
  fi
  print -r -- "$checksum  $native/$filename" | shasum -a 256 -c -
}
fetch hidapi-0.15.0.tar.gz https://github.com/libusb/hidapi/archive/refs/tags/hidapi-0.15.0.tar.gz 5d84dec684c27b97b921d2f3b73218cb773cf4ea915caee317ac8fc73cef8136
fetch libusb-1.0.29.tar.bz2 https://github.com/libusb/libusb/releases/download/v1.0.29/libusb-1.0.29.tar.bz2 5977fc950f8d1395ccea9bd48c06b3f808fd3c2c961b44b0c2e6e29fc3a70a85
tar -xzf "$native/hidapi-0.15.0.tar.gz" -C "$native"
clang -O2 -dynamiclib -framework IOKit -framework CoreFoundation -framework AppKit \
  -I"$native/hidapi-hidapi-0.15.0/hidapi" -I"$native/hidapi-hidapi-0.15.0/mac" \
  "$native/hidapi-hidapi-0.15.0/mac/hid.c" -o "$native/libhidapi.dylib" \
  -Wl,-install_name,@rpath/libhidapi.dylib
tar -xjf "$native/libusb-1.0.29.tar.bz2" -C "$native"
(
  cd "$native/libusb-1.0.29"
  ./configure --disable-static --enable-shared
  # Build the upstream Darwin source set directly: bundled libtool misparses
  # this Mac's toolchain and drops internal symbols during its object relink.
  clang -O2 -dynamiclib -DHAVE_CONFIG_H -I. -Ilibusb \
    libusb/{core,descriptor,hotplug,io,strerror,sync}.c \
    libusb/os/{events_posix,threads_posix,darwin_usb}.c \
    -framework IOKit -framework CoreFoundation -framework Security -lobjc \
    -Wl,-install_name,@rpath/libusb-1.0.dylib -o "$native/libusb-1.0.dylib"
)
codesign --force --sign - "$native/libhidapi.dylib" "$native/libusb-1.0.dylib"
print 'Built HIDAPI 0.15.0 and libusb 1.0.29 for the current Mac architecture.'
