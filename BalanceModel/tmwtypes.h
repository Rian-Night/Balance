/*
 * Simplified tmwtypes.h for MATLAB C code generation
 * Defines basic MATLAB types for embedded use
 */

#ifndef TMWTYPES_H
#define TMWTYPES_H

#include <stdint.h>
#include <stdbool.h>

/* Basic MATLAB types */
typedef double real_T;
typedef float real32_T;
typedef bool boolean_T;

/* Integer types */
typedef int8_t int8_T;
typedef uint8_t uint8_T;
typedef int16_t int16_T;
typedef uint16_t uint16_T;
typedef int32_t int32_T;
typedef uint32_t uint32_T;

/* Character types */
typedef char char_T;
typedef unsigned char uchar_T;

#endif /* TMWTYPES_H */