/**
 * @file unity_config.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-05-15
 *  
 */
#ifndef UNITY_CONFIG_H
#define UNITY_CONFIG_H

#ifndef NULL
  #ifndef __cplusplus
    #define NULL (void*)0
  #else
    #define NULL 0
  #endif
#endif

#ifdef __cplusplus
extern "C"
{
#endif


// ----------------------------------------------------------------------------
//
//  Test framework configuration
//
// ----------------------------------------------------------------------------

void unityOutputStart();
void unityOutputChar(char);
void unityOutputFlush();
void unityOutputComplete();


// ----------------------------------------------------------------------------
//
//  Test framework configuration
//
// ----------------------------------------------------------------------------

#define UNITY_OUTPUT_START()    unityOutputStart()
#define UNITY_OUTPUT_CHAR(c)    unityOutputChar(c)
#define UNITY_OUTPUT_FLUSH()    unityOutputFlush()
#define UNITY_OUTPUT_COMPLETE() unityOutputComplete()


#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* UNITY_CONFIG_H */