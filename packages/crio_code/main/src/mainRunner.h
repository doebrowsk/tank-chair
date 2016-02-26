/*
 * Copyright (c) 2009,
 * National Instruments Corporation.
 * All rights reserved.
 */

#ifndef __NiFpga_h__
#define __NiFpga_h__

/*
 * Determine platform details.
 */
#if defined(_M_IX86) \
 || defined(_M_X64) \
 || defined(i386) \
 || defined(__i386__) \
 || defined(__amd64__) \
 || defined(__amd64) \
 || defined(__x86_64__) \
 || defined(__x86_64) \
 || defined(__i386) \
 || defined(_X86_) \
 || defined(__THW_INTEL__) \
 || defined(__I86__) \
 || defined(__INTEL__) \
 || defined(__X86__) \
 || defined(__386__) \
 || defined(__I86__) \
 || defined(M_I386) \
 || defined(M_I86) \
 || defined(_M_I386) \
 || defined(_M_I86)
   #define NiFpga_Intel 1
   #if defined(PHARLAPOS)
      #define NiFpga_Ets 1
   #elif defined(_WIN32) \
      || defined(_WIN64) \
      || defined(__WIN32__) \
      || defined(__TOS_WIN__) \
      || defined(__WINDOWS__) \
      || defined(_WINDOWS) \
      || defined(__WINDOWS_386__)
      #define NiFpga_Windows 1
   #else
      #error Unsupported OS.
   #endif
#elif defined(__powerpc) \
   || defined(__powerpc__) \
   || defined(__POWERPC__) \
   || defined(__ppc__) \
   || defined(__PPC) \
   || defined(_M_PPC) \
   || defined(_ARCH_PPC) \
   || defined(__PPC__) \
   || defined(__ppc)
   #define NiFpga_Ppc 1
   #if defined(__vxworks)
      #define NiFpga_VxWorks 1
   #else
      #error Unsupported OS.
   #endif
#else
   #error Unsupported architecture.
#endif

/*
 * Determine compiler.
 */
#if defined(_CVI_)
   #define NiFpga_Cvi 1
#elif defined(_MSC_VER)
   #define NiFpga_Msvc 1
#elif defined(__GNUC__)
   #define NiFpga_Gcc 1
#else
   /* Unknown compiler. */
#endif

/*
 * Determine compliance with different C/C++ language standards.
 */
#if defined(__cplusplus)
   #define NiFpga_Cpp 1
   #if __cplusplus >= 199707L
      #define NiFpga_Cpp98 1
   #endif
#endif
#if defined(__STDC__)
   #define NiFpga_C89 1
   #if defined(__STDC_VERSION__)
      #define NiFpga_C90 1
      #if __STDC_VERSION__ >= 199409L
         #define NiFpga_C94 1
         #if __STDC_VERSION__ >= 199901L
            #define NiFpga_C99 1
         #endif
      #endif
   #endif
#endif
#if !NiFpga_Cpp
   /* const was added in C89, but MSVC doesn't define the standard macro. */
   #if !NiFpga_C89 && !NiFpga_Msvc
      #define const
   #endif
   /* inline was added in C99, but GCC has had it for years. */
   #if !NiFpga_C99 && !NiFpga_Gcc
      #define inline
   #endif
#endif

/*
 * Define standard integer types.
 */
#if NiFpga_C99 \
 || NiFpga_VxWorks && NiFpga_Gcc
   #include <stdint.h>
#elif NiFpga_Cvi
   typedef   signed    char int8_t;
   typedef unsigned    char uint8_t;
   typedef   signed   short int16_t;
   typedef unsigned   short uint16_t;
   typedef   signed     int int32_t;
   typedef unsigned     int uint32_t;
   typedef   signed __int64 int64_t;
   typedef unsigned __int64 uint64_t;
#elif NiFpga_Msvc
   typedef   signed __int8  int8_t;
   typedef unsigned __int8  uint8_t;
   typedef   signed __int16 int16_t;
   typedef unsigned __int16 uint16_t;
   typedef   signed __int32 int32_t;
   typedef unsigned __int32 uint32_t;
   typedef   signed __int64 int64_t;
   typedef unsigned __int64 uint64_t;
#else
   /* Integer types must be defined by the client. */
#endif



/*
 * Platform specific includes.
 */
#if NiFpga_Windows || NiFpga_Ets
   #include <windows.h>
#elif NiFpga_VxWorks
   #include <vxWorks.h>
   #include <symLib.h>
   #include <loadLib.h>
   #include <sysSymTbl.h>
   MODULE_ID VxLoadLibraryFromPath(const char* path, int flags);
   STATUS VxFreeLibrary(MODULE_ID library, int flags);
#else
#endif
#endif
