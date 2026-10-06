/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ONNX_ONNX_PB_H
#define ONNX_ONNX_PB_H

// Defines ONNX_EXPORT and ONNX_IMPORT. On Windows, this corresponds to
// different declarations (dllexport and dllimport). On Linux/Mac, it just
// resolves to the same "default visibility" setting.
#if defined(_MSC_VER)
#if defined(ONNX_BUILD_SHARED_LIBS) || defined(ONNX_BUILD_MAIN_LIB)
#define ONNX_EXPORT __declspec(dllexport)
#define ONNX_IMPORT __declspec(dllimport)
#else
#define ONNX_EXPORT
#define ONNX_IMPORT
#endif
#else
#if defined(__GNUC__)
#define ONNX_EXPORT __attribute__((__visibility__("default")))
#else
#define ONNX_EXPORT
#endif
#define ONNX_IMPORT ONNX_EXPORT
#endif

// ONNX_API is a macro that, depends on whether you are building the
// main ONNX library or not, resolves to either ONNX_EXPORT or
// ONNX_IMPORT.
//
// This is used in e.g. ONNX's protobuf files: when building the main library,
// it is defined as ONNX_EXPORT to fix a Windows global-variable-in-dll
// issue, and for anyone dependent on ONNX it will be defined as
// ONNX_IMPORT. ONNX_BUILD_MAIN_LIB can also be set when being built
// statically if ONNX is being linked into a shared library that wants
// to export the ONNX APIs and classes.
//
// More details on Windows dllimport / dllexport can be found at
// https://msdn.microsoft.com/en-us/library/3y1sfaz2.aspx
//
// This solution is similar to
// https://github.com/pytorch/pytorch/blob/master/caffe2/core/common.h
#if defined(ONNX_BUILD_SHARED_LIBS) || defined(ONNX_BUILD_MAIN_LIB)
#define ONNX_API ONNX_EXPORT
#else
#define ONNX_API ONNX_IMPORT
#endif

#if defined(__GNUC__)
#pragma GCC diagnostic push

// In file included from onnx/onnx-ml.pb.h:30:
// In file included from google/protobuf/extension_set.h:53:
// google/protobuf/parse_context.h:328:47: error: implicit conversion loses integer precision: 'long' to 'int' [-Werror,-Wshorten-64-to-32]
#if defined(__has_warning)
#if __has_warning("-Wshorten-64-to-32")
#pragma GCC diagnostic ignored "-Wshorten-64-to-32"
#endif
#endif  // defined(__has_warning)

#endif  // defined(__GNUC__)

#ifdef ONNX_ML
#include "onnx/onnx-ml.pb.h"
#else
#include "onnx/onnx.pb.h"
#endif

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#endif // ! ONNX_ONNX_PB_H
