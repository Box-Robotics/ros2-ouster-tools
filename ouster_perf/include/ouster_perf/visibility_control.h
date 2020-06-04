// -*- c++ -*-
// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OUSTER_TOOLS__VISIBILITY_CONTROL_H_
#define OUSTER_TOOLS__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define OUSTER_TOOLS_EXPORT __attribute__ ((dllexport))
    #define OUSTER_TOOLS_IMPORT __attribute__ ((dllimport))
  #else
    #define OUSTER_TOOLS_EXPORT __declspec(dllexport)
    #define OUSTER_TOOLS_IMPORT __declspec(dllimport)
  #endif
  #ifdef OUSTER_TOOLS_BUILDING_DLL
    #define OUSTER_TOOLS_PUBLIC OUSTER_TOOLS_EXPORT
  #else
    #define OUSTER_TOOLS_PUBLIC OUSTER_TOOLS_IMPORT
  #endif
  #define OUSTER_TOOLS_PUBLIC_TYPE OUSTER_TOOLS_PUBLIC
  #define OUSTER_TOOLS_LOCAL
#else
  #define OUSTER_TOOLS_EXPORT __attribute__ ((visibility("default")))
  #define OUSTER_TOOLS_IMPORT
  #if __GNUC__ >= 4
    #define OUSTER_TOOLS_PUBLIC __attribute__ ((visibility("default")))
    #define OUSTER_TOOLS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define OUSTER_TOOLS_PUBLIC
    #define OUSTER_TOOLS_LOCAL
  #endif
  #define OUSTER_TOOLS_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // OUSTER_TOOLS__VISIBILITY_CONTROL_H_
