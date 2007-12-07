#ifndef RW_OS_HPP_
#define RW_OS_HPP_

#if defined(__CYGWIN__)
	#define RW_CYGWIN
#elif defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
    #define RW_WIN32
#elif defined(macintosh) || defined(__APPLE__) || defined(__APPLE_CC__)
    #define RW_MACOS
#elif defined(linux) || defined(__linux) || defined(__linux__) 
    #define RW_LINUX
#endif

#endif /*RW_OS_HPP_*/
