#ifndef __comm_h__
#define __comm_h__

#include <math.h>
enum LERR_CODE {
  LERRCODE_OK = 0,
  LERRCODE_SOCK_CREATE_FAILED,
  LERRCODE_IP_BINGING_FAILED,
  LERRCODE_MAX,
};
enum LDEBUG_LEVEL {
  LDEBUG_DEBUG = 0,
  LDEBUG_INFO,     // normal
  LDEBUG_NOTICE,   // notice
  LDEBUG_WARNING,  // warning
  LDEBUG_ERR,      // error
  LDEBUG_CRI,      // critical
  LDEBUG_ALERT,    // need do something immediately
  LDEBUG_EMERG,    // system would crash
};

#define __LDEBUG
#define __LDEBUG_LEVEL LDEBUG_WARNING

#ifdef __LDEBUG
#define LDEBUG(level, format, ...) \
  if (level >= __LDEBUG_LEVEL) printf(format, ##__VA_ARGS__)
//#define LDEBUG(format,...) printf("FILE: "__FILE__", LINE: %d: "format"/n", __LINE__, ##__VA_ARGS__)
#else
#define LDEBUG(level, format, ...)
#endif

#define go(i, a, b) for (int i = a; i < b; ++i)

const double PI = 3.1415926;  // acos(-1.0);

#endif