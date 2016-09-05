#ifndef __c3_RobotPoseIntegration_h__
#define __c3_RobotPoseIntegration_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc3_RobotPoseIntegrationInstanceStruct
#define typedef_SFc3_RobotPoseIntegrationInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c3_sfEvent;
  boolean_T c3_isStable;
  boolean_T c3_doneDoubleBufferReInit;
  uint8_T c3_is_active_c3_RobotPoseIntegration;
  real_T c3_EI[3];
  boolean_T c3_EI_not_empty;
  real_T *c3_theta;
  real_T *c3_x;
  real_T (*c3_IC)[3];
  real_T *c3_y;
  real_T *c3_theta_dot;
  real_T *c3_v;
} SFc3_RobotPoseIntegrationInstanceStruct;

#endif                                 /*typedef_SFc3_RobotPoseIntegrationInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c3_RobotPoseIntegration_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c3_RobotPoseIntegration_get_check_sum(mxArray *plhs[]);
extern void c3_RobotPoseIntegration_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
