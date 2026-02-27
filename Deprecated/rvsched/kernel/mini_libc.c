#include <stddef.h>
#include <stdint.h>

void* memset(void* dst, int c, size_t n) {
  uint8_t* p = (uint8_t*)dst;
  while (n--) *p++ = (uint8_t)c;
  return dst;
}

void* memcpy(void* dst, const void* src, size_t n) {
  uint8_t* d = (uint8_t*)dst;
  const uint8_t* s = (const uint8_t*)src;
  while (n--) *d++ = *s++;
  return dst;
}
