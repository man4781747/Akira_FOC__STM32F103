@ GCC 的單精度軟體浮點 (__aeabi_fadd 等) 改用 qfplib-m3：連結時加上 -Wl,--wrap=__aeabi_fadd 等 (CMakeLists.txt)，
@ 程式裡所有對 __aeabi_fXXX 的呼叫都會連到這裡的 __wrap___aeabi_fXXX，再跳到 qfplib 的對應函式。
@ qfplib 的四則運算是正確捨入的，但不支援 NaN 與次正規數 (次正規數當成 0)。
@ 型別轉換、比較不換：qfplib 的 float -> int 是往負無限大取整，和 C 的規定不同。

  .syntax unified
  .cpu cortex-m3
  .thumb

  .macro WRAP name, target
  .section .text.__wrap_\name, "ax", %progbits
  .global __wrap_\name
  .type __wrap_\name, %function
  .thumb_func
__wrap_\name:
  b.w \target
  .size __wrap_\name, . - __wrap_\name
  .endm

  WRAP __aeabi_fadd, qfp_fadd
  WRAP __aeabi_fsub, qfp_fsub
  WRAP __aeabi_fmul, qfp_fmul
  WRAP __aeabi_fdiv, qfp_fdiv
