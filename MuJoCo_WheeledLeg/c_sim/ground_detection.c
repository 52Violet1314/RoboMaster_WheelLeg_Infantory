#include "ground_detection.h"

#include <math.h>
#include <string.h>

GroundDetectionConfig ground_detection_default_config(void) {
  GroundDetectionConfig config;
  config.wheel_mass_kg = 0.50;
  config.gravity_mps2 = 9.81;
  config.filter_alpha = 0.15;
  config.off_threshold_n = 15.0;
  config.on_ratio = 1.5;
  config.confirm_off_ticks = 10U;
  config.confirm_on_ticks = 15U;
  return config;
}

void ground_detection_init(GroundDetector* detector,
                           const GroundDetectionConfig* config) {
  if (!detector) return;
  memset(detector, 0, sizeof(*detector));
  detector->config = config ? *config : ground_detection_default_config();
}

double ground_detection_observe(const GroundDetectionConfig* config,
                                const GroundDetectionInput* input) {
  if (!config || !input || input->leg_length_m <= 0.0) return 0.0;

  const double length = input->leg_length_m;
  const double angle = input->leg_angle_rad;
  const double c = cos(angle);
  const double s = sin(angle);

  /* Positive F0 pushes the wheel down.  Positive Tp uses the firmware VMC
     convention, so its downward projection is +Tp*sin(phi)/L. */
  const double force_down = input->f0_without_spring_n * c
                          + input->tp_vmc_nm * s / length;

  /* Downward-coordinate wheel acceleration for
       z_w = z_body + L*cos(phi). */
  const double wheel_accel_down = input->body_accel_down_mps2
                                + input->leg_accel_mps2 * c
                                - 2.0 * input->leg_velocity_mps
                                      * input->leg_rate_radps * s
                                - length * input->leg_accel_radps2 * s
                                - length * input->leg_rate_radps
                                      * input->leg_rate_radps * c;

  /* m*a_down = m*g + force_down - Fn. */
  return force_down
       + config->wheel_mass_kg * (config->gravity_mps2 - wheel_accel_down);
}

GroundDetectionOutput ground_detection_update(GroundDetector* detector,
                                               int side,
                                               const GroundDetectionInput* input) {
  GroundDetectionOutput empty;
  memset(&empty, 0, sizeof(empty));
  if (!detector || !input || side < 0 || side >= GROUND_SIDE_COUNT) return empty;

  GroundDetectionOutput* output = &detector->side[side];
  output->changed = 0;
  output->fn_raw_n = ground_detection_observe(&detector->config, input);

  if (!detector->initialized[side]) {
    output->fn_filtered_n = output->fn_raw_n;
    output->confirm_count = 0U;
    output->off_ground = 0;
    detector->initialized[side] = 1;
    return *output;
  }

  const double alpha = detector->config.filter_alpha;
  output->fn_filtered_n = alpha * output->fn_raw_n
                        + (1.0 - alpha) * output->fn_filtered_n;

  if (!output->off_ground) {
    if (output->fn_filtered_n < detector->config.off_threshold_n) {
      if (output->confirm_count < detector->config.confirm_off_ticks)
        output->confirm_count++;
      if (output->confirm_count >= detector->config.confirm_off_ticks) {
        output->off_ground = 1;
        output->changed = 1;
        output->confirm_count = 0U;
      }
    } else {
      output->confirm_count = 0U;
    }
  } else {
    const double on_threshold = detector->config.off_threshold_n
                              * detector->config.on_ratio;
    if (output->fn_filtered_n > on_threshold) {
      if (output->confirm_count < detector->config.confirm_on_ticks)
        output->confirm_count++;
      if (output->confirm_count >= detector->config.confirm_on_ticks) {
        output->off_ground = 0;
        output->changed = 1;
        output->confirm_count = 0U;
      }
    } else {
      output->confirm_count = 0U;
    }
  }

  return *output;
}

int ground_detection_airborne(const GroundDetector* detector) {
  if (!detector) return 0;
  return detector->side[GROUND_SIDE_LEFT].off_ground
      && detector->side[GROUND_SIDE_RIGHT].off_ground;
}
