#ifndef DATA_FILTER_H_
#define DATA_FILTER_H_

#include <stdint.h>

#define Pi 3.1415926f
#define HP_CUT_FRQ 5
#define SAMPLE_RATE  200.0f
#define WIN_NUM 5

void high_pass_filter(float in, float *out);
void LowPassFilter_RC(float Vi, float *Vo);
int16_t window_filter(int16_t data, int16_t *buf, uint8_t len);

#endif /* DATA_FILTER_H_ */
