/*
 * strsafe.h
 *
 *  Created on: 16-02-2010
 *      Author: jimali
 */
/* Written by Krzysztof Kowalczyk (http://blog.kowalczyk.info)
   The author disclaims copyright to this source code. */
#ifndef __STR_STRSAFE_H
#define __STR_STRSAFE_H

/* When using MSVC, use <strsafe.h>, emulate it on other compiler (e.g. mingw) */

#define DISABLE_STRSAFE
#ifndef DISABLE_STRSAFE
  #include <strsafe.h>
#else
  #include <stdio.h>
  #include <string.h>
  #include <windows.h>
  #define   STRSAFE_E_INSUFFICIENT_BUFFER   -1
  #define   _vsnprintf_s(p,s,z,f,a)     vsnprintf(p,s,f,a)

  /* WARNING: the return values of these two pairs of functions aren't
  compatible. The strsafe functions return an error code and the standard C
  functions return a character count */
  #define   StringCchVPrintfA           vsnprintf
  #define   StringCchVPrintfW           vsnprintf
  #define   StringCchVPrintf            vsnprintf

  #define StringCchPrintfA(str, n, format, ...) snprintf ((char*)str, n, (char const*)format, __VA_ARGS__)
  #define StringCchPrintfW(str, n, format, ...) snprintf ((char*)str, n, (char const*)format, __VA_ARGS__)
  #define StringCchPrintf(str, n, format, ...) snprintf ((char*)str, n, (char const*)format, __VA_ARGS__)

  #define StringCchCopy(dest, n, src) strncpy ((char*)dest, (char const*)src, n)
  #define StringCchCopyN(dest, n1, src, n2) strncpy ((char*)dest, (char const*)src, min (n1, n2))
  #define StringCchCat(dest, n, src) strncat ((char*)dest, (char const*)src, n)

  #define STRSAFE_MAX_CCH 0x7FFFFFFF

  #define   _stricmp                    strcasecmp
  #define   _strnicmp                   strncasecmp
#endif

#endif
