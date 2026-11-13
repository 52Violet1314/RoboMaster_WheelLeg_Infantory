#ifndef WHEELLEG_GROUND_DETECTION_H
#define WHEELLEG_GROUND_DETECTION_H

#ifdef __cplusplus
extern "C" {
#endif

enum {
  GROUND_SIDE_LEFT = 0,
  GROUND_SIDE_RIGHT = 1,
  GROUND_SIDE_COUNT = 2
};

typedef struct {
  double wheel_mass_kg;
  double gravity_mps2;
  double filter_alpha;
  double off_threshold_n;
  double on_ratio;
  unsigned int confirm_off_ticks;
  unsigned int confirm_on_ticks;
} GroundDetectionConfig;

typedef struct {
  double f0_without_spring_n;
  double tp_vmc_nm;
  double leg_length_m;
  double leg_velocity_mps;
  double leg_accel_mps2;
  double body_accel_down_mps2;
  double leg_angle_rad;
  double leg_rate_radps;
  double leg_accel_radps2;
} GroundDetectionInput;

typedef struct {
  double fn_raw_n;
  double fn_filtered_n;
  unsigned int confirm_count;
  int off_ground;
  int changed;
} GroundDetectionOutput;

typedef struct {
  GroundDetectionConfig config;
  GroundDetectionOutput side[GROUND_SIDE_COUNT];
  int initialized[GROUND_SIDE_COUNT];
} GroundDetector;

/* These defaults intentionally match the unvalidated STM32 implementation. */
GroundDetectionConfig ground_detection_default_config(void);
void ground_detection_init(GroundDetector* detector,
                           const GroundDetectionConfig* config);
double ground_detection_observe(const GroundDetectionConfig* config,
                                const GroundDetectionInput* input);
GroundDetectionOutput ground_detection_update(GroundDetector* detector,
                                               int side,
                                               const GroundDetectionInput* input);
int ground_detection_airborne(const GroundDetector* detector);

#ifdef __cplusplus
}
#endif

#endif
