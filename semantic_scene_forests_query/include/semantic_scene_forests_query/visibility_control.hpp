// Copyright (c) 2026 FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  ifdef __GNUC__
#    define SEMANTIC_SCENE_FORESTS_QUERY_EXPORT __attribute__((dllexport))
#    define SEMANTIC_SCENE_FORESTS_QUERY_IMPORT __attribute__((dllimport))
#  else
#    define SEMANTIC_SCENE_FORESTS_QUERY_EXPORT __declspec(dllexport)
#    define SEMANTIC_SCENE_FORESTS_QUERY_IMPORT __declspec(dllimport)
#  endif
#  ifdef SEMANTIC_SCENE_FORESTS_QUERY_BUILDING_DLL
#    define SEMANTIC_SCENE_FORESTS_QUERY_PUBLIC SEMANTIC_SCENE_FORESTS_QUERY_EXPORT
#  else
#    define SEMANTIC_SCENE_FORESTS_QUERY_PUBLIC SEMANTIC_SCENE_FORESTS_QUERY_IMPORT
#  endif
#  define SEMANTIC_SCENE_FORESTS_QUERY_PUBLIC_TYPE SEMANTIC_SCENE_FORESTS_QUERY_PUBLIC
#  define SEMANTIC_SCENE_FORESTS_QUERY_LOCAL
#else
#  define SEMANTIC_SCENE_FORESTS_QUERY_EXPORT __attribute__((visibility("default")))
#  define SEMANTIC_SCENE_FORESTS_QUERY_IMPORT
#  if __GNUC__ >= 4
#    define SEMANTIC_SCENE_FORESTS_QUERY_PUBLIC __attribute__((visibility("default")))
#    define SEMANTIC_SCENE_FORESTS_QUERY_LOCAL __attribute__((visibility("hidden")))
#  else
#    define SEMANTIC_SCENE_FORESTS_QUERY_PUBLIC
#    define SEMANTIC_SCENE_FORESTS_QUERY_LOCAL
#  endif
#  define SEMANTIC_SCENE_FORESTS_QUERY_PUBLIC_TYPE
#endif
