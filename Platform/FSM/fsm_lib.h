/***************************************************************************
 * Copyright  2024 
 * All right reserved. See COPYRIGHT for detailed Information.
 *
 * @file       fsm_lib.h
 *
 * @author     
 * @brief      Provides support for state machines control
 *
 * @Email      
 *
 * @date       2024-07-05
 * @version    2.6.0
 ***************************************************************************/
#ifndef __FSM_LIB_DEFINE_H__
#define __FSM_LIB_DEFINE_H__

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define __weak __attribute__((weak))

//-----------------------   state machine define -----------------------------//

// define state machine ctrl code
/*
typedef enum {
    FSMCtrl_INIT = 0,
    FSMCtrl_RUN,
    FSMCtrl_IDLE,
    FSMCtrl_GETSTATE
} fsm_lib_ctrl;
*/
/**
 * @brief   state machine return vaule
 */
typedef enum {
    fsm_rt_idle = 0,  // idle
    fsm_rt_cpl,       // complete
    fsm_rt_running,   // running
    fsm_rt_initdone,  // init done
    fsm_rt_error,     // error
} fsm_lib_return;

/**
 * @brief   state machine base state define
 * @note    every state machine must  KEEP the default status number
 */
typedef enum {
    // idle state
    fsm_lib_state_idle = 0,


    fsm_lib_state_start = 1,


    fsm_lib_state_end   = 2,
    fsm_lib_state_error = 3,
} fsm_lib_defualt_state;



/**
 * @brief   state machine control handle
 */
typedef struct fsm_lib_ctrl_handle fsm_lib_ctrl_handle;
struct fsm_lib_ctrl_handle {
    int   state;
    int   last_state;
    int   forced_state;
    void *lock;        // lock info
    // lock method;  para:Additional parameters provided to the method, or null if not required
    //               lock_state :0=unlock,1=lock
    int (*lock_method)(fsm_lib_ctrl_handle *handle, void *para, uint8_t lock_state);
    fsm_lib_return return_state;
    uint16_t       name;
    uint8_t        emergency;          // 1==jump tp error state
    uint8_t        forced_state_flag;  // 0==do nothing,1==forced switch
};


#define fsm_lib_main_body(__handle_state__) \
  switch(__handle_state__)
/**
 * @brief   fsm state creat
 * @note    this is a macro ,Must be used in the specified structure
 *          Used in state machine main body
 */
#define fsm_lib_state(__state) \
    case (__state):

/**
 * @brief   fsm default state
 * @note    this is a macro ,Must be used in the specified structure
 *          Used in state machine main body
 */
#define fsm_lib_state_default() \
    default:                    \
        break

/**
 * @brief   transfer state with set return value
 * @note    this is a macro ,Must be used in the specified structure
 *          Used in state machine main body
 *          switch state and releasing cpu  and set return value
 */
#define fsm_lib_transfer_to(__fsm_handle, __state, __return)              \
    do {                                                                  \
        if (__fsm_handle->forced_state_flag > 0) {            \
          __fsm_handle->forced_state_flag = 0;                \
          __fsm_handle->state = __fsm_handle->forced_state;   \
                    return fsm_rt_running;                          \
        }else{                                                \
					    __fsm_handle->state=__state;                    \
        return __return;}                                     \
    } while (0)

/**
 * @brief   update state
 * @note    this is a macro ,Must be used in the specified structure
 *          Used in state machine main body
 */
#define fsm_lib_update_to(__fsm_handle, __state)                          \
    do {                                                                  \
        if (__fsm_handle->forced_state_flag > 0) {            \
          __fsm_handle->forced_state_flag = 0;                \
          __fsm_handle->state = __fsm_handle->forced_state;   \
          return fsm_rt_running;                              \
        }else{                                                \
					    __fsm_handle->state=__state;                    \
        return fsm_rt_running;}                                     \
    } while (0)


/**
 * @brief   releasing cpu
 * @note    this is a macro ,Must be used in the specified structure
 *          Used in state machine main body
 */
#define fsm_lib_release(__fsm_handle, __rt_val)                           \
    do {                                                                  \
        if (__fsm_handle->forced_state_flag > 0) {            \
          __fsm_handle->forced_state_flag = 0;                \
          __fsm_handle->state = __fsm_handle->forced_state;   \
          return fsm_rt_running;                              \
        }else{   			\
            return __rt_val;      \
				}					\
    } while (0)

/**
 * @brief   cheak  emergency
 * @param
 * @note    this is a macro ,Must be used in the specified structure,
 *          Used in state machine main body
 * @retval
 */
#define fsm_lib_check_emergency(__handle)               \




/**
 * @brief   state machine switch state force
 * @param
 * @note    Not recommended for use
 *          It is strongly recommended to use flag bits to notify
 *          the state machine instead of forcibly modifying the state
 *          !Requires the method itself to be thread safe
 * @retval
 */
static inline void fsm_lib_switch_state_force(fsm_lib_ctrl_handle *handle, int force_state) {
    handle->forced_state      = force_state;
    handle->forced_state_flag = 1;
}

/**
 * @brief   state machine get state
 * @param
 * @note
 * @retval
 */
static inline int fsm_lib_get_state(fsm_lib_ctrl_handle *handle) {
    return handle->state;
}
/**
 * @brief  fsm  check is in IDLE
 * @param
 * @note
 * @retval  0x00=no force flag and NOT in IDLE
 *          0x01=no force flag  and in IDLE
 *          0x02=have force flag  and NOT in IDLE
 *          0x03=have force flag  and in IDLE
 */
static inline int fsm_lib_is_idle(fsm_lib_ctrl_handle *handle) {
    int ret = 0;

    if (handle->forced_state_flag) {
        ret |= 0x10;
    }

    if (handle->state == fsm_lib_state_idle) {
        ret |= 0x01;
    }

    return ret;
}
/**
 * @brief   state machine start
 * @param
 * @note    control state machine go to start state
 *          !Requires the method itself to be thread safe
 * @retval  0=ok 1,failed
 */
static inline int fsm_lib_start(fsm_lib_ctrl_handle *handle) {
    // cheak state machine if idle
    if (fsm_lib_is_idle(handle) != 1) {
        return 1;
    }

    // switch to start state
    fsm_lib_switch_state_force(handle, fsm_lib_state_start);
    return 0;
}

/**
 * @brief   state machine stop
 * @param
 * @note    control state machine go to end state
 *          !Requires the method itself to be thread safe
 */
static inline void fsm_lib_stop(fsm_lib_ctrl_handle *handle) {
    if (handle->state == fsm_lib_state_end) {
        return;
    }

    // force switch to end(2) state
    fsm_lib_switch_state_force(handle, fsm_lib_state_end);
}



/**
 * @brief  fsm reset ,set fsm state to idle;
 * @param
 * @note   When executed, it is mutually exclusive with the fsm body.
 *          !Need to maintain thread safety with the fsm being operated on
 *          !Requires the method itself to be thread safe
 * @retval
 */
static inline void fsm_lib_reset(fsm_lib_ctrl_handle *handle) {
    handle->emergency         = 0;
    handle->forced_state_flag = 0;
    handle->forced_state      = 0;
    handle->state             = fsm_lib_state_idle;
}

#endif


