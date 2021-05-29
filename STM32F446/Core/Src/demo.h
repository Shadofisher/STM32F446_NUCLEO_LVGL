/**
 * @file demo.h
 *
 */

#ifndef DEMO_H
#define DEMO_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "lvgl.h"

lv_obj_t * gauge1;
lv_obj_t * gauge2;
lv_obj_t * speed1;
lv_obj_t * speed2;
uint32_t prev_value;


#define LV_CONF_INCLUDE_SIMPLE 1

#ifdef LV_CONF_INCLUDE_SIMPLE
#include "lvgl.h"
//#include "lv_ex_conf.h"
#else
#include "lvgl.h"
//#include "../../../lv_ex_conf.h"
#endif

#if LV_USE_DEMO

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Create a demo application
 */
void demo_create(void);

/**********************
 *      MACROS
 **********************/

#endif /*LV_USE_DEMO*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*DEMO_H*/
