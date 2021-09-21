#pragma once

#if defined(__USE_BDX__)
#if defined(__cplusplus)
extern "C" {
#endif /* C++ */
int __bdx_cond__(int flag) __attribute__((noinline));
void __bdx_stage_begin__(volatile long flag) __attribute__((noinline));
void __bdx_stage_end__(void) __attribute__((noinline));

__attribute__ ((noinline)) int __bdx_cond__(int flag) {__asm__(""); return 1;}
__attribute__ ((noinline)) void __bdx_stage_begin__(volatile long flag) {__asm__("");}
__attribute__ ((noinline)) void __bdx_stage_end__(void) {__asm__("");}
#if defined(__cplusplus)
}
#endif /* C++ */
#endif /* __USE_BDX__ */

#if defined(__USE_TLS__)
#include <immintrin.h>
#if defined(__cplusplus)
extern "C" {
#endif /* C++ */
void _xabort2(char n __attribute__((unused))) {
  _xabort(0xff);
}
#if defined(__cplusplus)
}
#endif /* C++ */
#endif /* __USE_TLS__ */
