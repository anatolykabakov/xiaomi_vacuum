/* Minimal RPC typedefs for parsing Player headers when host sysroot has no tirpc. */
#pragma once

typedef unsigned char u_char;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short u_short;
// NOLINTNEXTLINE(readability-identifier-naming) - Sun RPC / playerxdr expect bool_t
typedef int bool_t;
