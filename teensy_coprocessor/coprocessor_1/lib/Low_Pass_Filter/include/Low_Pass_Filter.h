#ifndef LOW_PASS_FILTER_H_
#define LOW_PASS_FILTER_H_

/**
 * 
 * FIR filter designed with
 *  http://t-filter.engineerjs.com/
 * 
 * sampling frequency: 100 Hz
 * 
 * fixed point precision: 16 bits
 * 
 *  0 Hz - 30 Hz
 *   gain = 1
 *   desired ripple = 5 dB
 *   actual ripple = n/a
 * 
 *  35 Hz - 50 Hz
 *   gain = 0
 *   desired attenuation = -40 dB
 *   actual attenuation = n/a
 * 
 */

/**
 * @brief Number of filter taps
 */
#define LOW_PASS_FILTER_TAP_NUM 24


/**
 * @brief Low Pass Filter structure
 * 
 * @param history       history buffer
 * @param last_index    index of the last inserted sample
 */
typedef struct {
    int history[LOW_PASS_FILTER_TAP_NUM];
    unsigned int last_index;
} Low_Pass_Filter;


/**
 * @brief 
 * 
 * @param f 
 */
void Low_Pass_Filter_init(Low_Pass_Filter* f);

/**
 * @brief 
 * 
 * @param f 
 * @param input
 */
void Low_Pass_Filter_put(Low_Pass_Filter* f, int input);

/**
 * @brief 
 */
int Low_Pass_Filter_get(Low_Pass_Filter* f);

#endif /* LOW_PASS_FILTER_H_ */
