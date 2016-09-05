#ifndef __c2_RobotPoseIntegration_h__
#define __c2_RobotPoseIntegration_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc2_RobotPoseIntegrationInstanceStruct
#define typedef_SFc2_RobotPoseIntegrationInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_isStable;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_RobotPoseIntegration;
  real_T c2_x_w;
  boolean_T c2_x_w_not_empty;
  real_T c2_y_w;
  boolean_T c2_y_w_not_empty;
  real_T c2_suma_theta;
  boolean_T c2_suma_theta_not_empty;
  real_T *c2_theta;
  real_T *c2_x;
  real_T (*c2_IC)[3];
  real_T *c2_y;
  real_T *c2_theta_dot;
  real_T *c2_v;
} SFc2_RobotPoseIntegrationInstanceStruct;

#endif                                 /*typedef_SFc2_RobotPoseIntegrationInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c2_RobotPoseIntegration_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c2_RobotPoseIntegration_get_check_sum(mxArray *plhs[]);
extern void c2_RobotPoseIntegration_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
