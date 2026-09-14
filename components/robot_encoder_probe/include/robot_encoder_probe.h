#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Starts a read-only four-channel PCNT diagnostic. No PWM or motor GPIO is
 * configured by this component. */
esp_err_t robot_encoder_probe_start(void);

#ifdef __cplusplus
}
#endif
