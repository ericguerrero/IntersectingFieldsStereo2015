#ifndef __c1_RobotPoseIntegration_h__
#define __c1_RobotPoseIntegration_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc1_RobotPoseIntegrationInstanceStruct
#define typedef_SFc1_RobotPoseIntegrationInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  boolean_T c1_isStable;
  boolean_T c1_doneDoubleBufferReInit;
  uint8_T c1_is_active_c1_RobotPoseIntegration;
  real_T *c1_theta_dot;
  real_T *c1_left_angular_speed;
  real_T *c1_v;
  real_T *c1_right_angular_speed;
} SFc1_RobotPoseIntegrationInstanceStruct;

#endif                                 /*typedef_SFc1_RobotPoseIntegrationInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c1_RobotPoseIntegration_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c1_RobotPoseIntegration_get_check_sum(mxArray *plhs[]);
extern void c1_RobotPoseIntegration_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
