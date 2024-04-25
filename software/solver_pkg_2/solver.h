/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * solver.h
 *
 * Code generation for function 'solver'
 *
 */

#ifndef SOLVER_H
#define SOLVER_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void solver(double particlePosX, double particlePosY,
                   double particlePosZ, double outputArg1[1442]);

extern void solver_initialize(void);

extern void solver_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (solver.h) */
