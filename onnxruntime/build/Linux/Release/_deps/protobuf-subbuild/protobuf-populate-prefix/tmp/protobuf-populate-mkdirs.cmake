# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/tmp/onnxruntime/build/Linux/Release/_deps/protobuf-src")
  file(MAKE_DIRECTORY "/tmp/onnxruntime/build/Linux/Release/_deps/protobuf-src")
endif()
file(MAKE_DIRECTORY
  "/tmp/onnxruntime/build/Linux/Release/_deps/protobuf-build"
  "/tmp/onnxruntime/build/Linux/Release/_deps/protobuf-subbuild/protobuf-populate-prefix"
  "/tmp/onnxruntime/build/Linux/Release/_deps/protobuf-subbuild/protobuf-populate-prefix/tmp"
  "/tmp/onnxruntime/build/Linux/Release/_deps/protobuf-subbuild/protobuf-populate-prefix/src/protobuf-populate-stamp"
  "/tmp/onnxruntime/build/Linux/Release/_deps/protobuf-subbuild/protobuf-populate-prefix/src"
  "/tmp/onnxruntime/build/Linux/Release/_deps/protobuf-subbuild/protobuf-populate-prefix/src/protobuf-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/tmp/onnxruntime/build/Linux/Release/_deps/protobuf-subbuild/protobuf-populate-prefix/src/protobuf-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/tmp/onnxruntime/build/Linux/Release/_deps/protobuf-subbuild/protobuf-populate-prefix/src/protobuf-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
