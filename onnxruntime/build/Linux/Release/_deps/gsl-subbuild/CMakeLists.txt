# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 4.4.3)

# Reject any attempt to use a toolchain file. We must not use one because
# we could be downloading it here. If the CMAKE_TOOLCHAIN_FILE environment
# variable is set, the cache variable will have been initialized from it.
unset(CMAKE_TOOLCHAIN_FILE CACHE)
unset(ENV{CMAKE_TOOLCHAIN_FILE})

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(gsl-populate NONE)



include(ExternalProject)
ExternalProject_Add(gsl-populate
                     "UPDATE_DISCONNECTED" "False" "DOWNLOAD_EXTRACT_TIMESTAMP" "FALSE" "EXTERNALPROJECT_INTERNAL_ARGUMENT_SEPARATOR" "URL" "https://github.com/microsoft/GSL/archive/refs/tags/v4.0.0.zip" "URL_HASH" "SHA1=cf368104cd22a87b4dd0c80228919bb2df3e2a14" "PATCH_COMMAND" "/usr/bin/patch" "--binary" "--ignore-whitespace" "-p1" "<" "/tmp/onnxruntime/cmake/patches/gsl/1064.patch"
                    SOURCE_DIR          "/tmp/onnxruntime/build/Linux/Release/_deps/gsl-src"
                    BINARY_DIR          "/tmp/onnxruntime/build/Linux/Release/_deps/gsl-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
                    USES_TERMINAL_PATCH     YES
)


