/*
 * @file Project_Config.h
 * @brief
 */

#ifndef __INC_PROJECT_CONFIG_H__
#define __INC_PROJECT_CONFIG_H__


/**
 * @brief enables printf() output recognized by the accompanying Processing
 *      sketch
 */
#define PROCESSING4_OUTPUT 1


/**
 * @brief printf() output for my comprehension
 */
//#define DEBUG_OUTPUT 1


// XOR gate to see if both active when only one needs to be!
#if defined(PROCESSING4_OUTPUT) && defined(DEBUG_OUTPUT)
    #error "both `PROCESSING4_OUTPUT` and `DEBUG_OUTPUT` cannot be active at \
once. Only one can be used."
#endif

#endif /* __INC_PROJECT_CONFIG_H__ */
