#ifdef ENABLE_SYSLOG
  #include <Syslog.h>
  #include <WiFiUdp.h>
#endif

#ifndef SYSLOG_LOG
#define SYSLOG_LOG
#if defined(ENABLE_SYSLOG) && defined(ENABLE_WIFI)
extern Syslog syslog;
    #define syslog_log(a, b) syslog.log(a, b)
#else
  #define LOG_EMERG 0
  #define LOG_ALERT 1
  #define LOG_CRIT  2
  #define LOG_ERR   3
  #define LOG_WARNING 4
  #define LOG_NOTICE  5
  #define LOG_INFO  6
  #define LOG_DEBUG 7

  #define syslog_log(a, b) do{}while(0)
#endif

#endif

