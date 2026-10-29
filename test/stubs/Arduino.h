// Host stub of the tiny Arduino surface wmbus_sx1262.cpp uses.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define IRAM_ATTR
class __FlashStringHelper;
#define F(x) (reinterpret_cast<const __FlashStringHelper *>(x))

struct HostSerial {
  void print(const char *s) { fputs(s, stdout); }
  void print(const __FlashStringHelper *s) { print((const char *)s); }
  void println(const char *s = "") { puts(s); }
  void println(const __FlashStringHelper *s) { println((const char *)s); }
  int printf(const char *fmt, ...) __attribute__((format(printf, 2, 3))) {
    va_list ap;
    va_start(ap, fmt);
    int n = vprintf(fmt, ap);
    va_end(ap);
    return n;
  }
};
inline HostSerial Serial;

inline unsigned long millis() { return (unsigned long)(clock() * 1000 / CLOCKS_PER_SEC); }
inline void yield() {}
inline void delay(unsigned long) {}
