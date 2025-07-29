/*
  rgb.c - RGB Status Light Plugin for CNC machines

  Copyright (c) 2021 JAC
  Version 1.0 - November 7, 2021

  For use with grblHAL: (Official GitHub) https://github.com/grblHAL
  Wiki (via deprecated GitHub location): https://github.com/terjeio/grblHAL/wiki/Compiling-GrblHAL

  Written by JAC for use with the Expatria grblHAL2000 PrintNC controller boards:
  https://github.com/Expatria-Technologies/grblhal_2000_PrintNC

  PrintNC - High Performance, Open Source, Steel Frame, CNC - https://wiki.printnc.info

  Code heavily modified for use with Sienci SuperLongBoard and NEOPIXELS.
  Copyright (c) 2023 Sienci Labs

  This file is part of the SuperLongBoard family of products.
  
   This source describes Open Hardware and is licensed under the "CERN-OHL-S v2"

   You may redistribute and modify this source and make products using
   it under the terms of the CERN-OHL-S v2 (https://ohwr.org/cern_ohl_s_v2.t). 
   This source is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY,
   INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A 
   PARTICULAR PURPOSE. Please see the CERN-OHL-S v2 for applicable conditions.
   
   As per CERN-OHL-S v2 section 4, should You produce hardware based on this 
   source, You must maintain the Source Location clearly visible on the external
   case of the CNC Controller or other product you make using this source.
  
   You should have received a copy of the CERN-OHL-S v2 license with this source.
   If not, see <https://ohwr.org/project/cernohl/wikis/Documents/CERN-OHL-version-2>.
   
   Contact for information regarding this program and its license
   can be sent through gSender@sienci.com or mailed to the main office
   of Sienci Labs Inc. in Waterloo, Ontario, Canada.

  M356 -  On = 1, Off = 2, RGB white LED inspection light in RGB Plugin
*/

#include "driver.h"

#if STATUS_LIGHT_ENABLE == 2 // 2 is reserved for this implementation

#include <string.h>
#include <math.h>

#include "grbl/protocol.h"
#include "grbl/hal.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"
#include "grbl/alarms.h"
#include "grbl/nuts_bolts.h"
#include "grbl/task.h" 
#include "grbl/modbus.h"

// Declarations

// Available RGB colors
#define RGB_OFF     (rgb_color_t){ .R = 0, .G = 0, .B = 0 }
#define RGB_RED     (rgb_color_t){ .R = 255, .G = 0, .B = 0 } // Red
#define RGB_GREEN   (rgb_color_t){ .R = 0, .G = 255, .B = 0 } // Green
#define RGB_BLUE    (rgb_color_t){ .R = 0, .G = 0, .B = 255 } // Blue
#define RGB_YELLOW  (rgb_color_t){ .R = 255, .G = 255, .B = 0 } // Red + Green
#define RGB_MAGENTA (rgb_color_t){ .R = 255, .G = 0, .B = 255 } // Red + Bue
#define RGB_CYAN    (rgb_color_t){ .R = 0, .G = 255, .B = 255 } // Green + Blue
#define RGB_WHITE   (rgb_color_t){ .R = 255, .G = 255, .B = 255 } // Red + Green + Blue
#define RGB_GREY    (rgb_color_t){ .R = 127, .G = 127, .B = 127 } // 50% Red + Green + Blue

typedef enum {
    LEDStateDriven = 0,         //!< 0 - state drive
    LEDAllWhite = 1,         //!< 1 - all white
    LEDOff = 2,  //!< 2 - all off
    LEDGreen=3,
} LED_flags_t;

static LED_flags_t strip0_override, strip1_override;

// Set preferred STATLE_IDLE light color, will be moving to a $ setting in future

static on_state_change_ptr on_state_change;
static on_report_options_ptr on_report_options;
static on_program_completed_ptr on_program_completed;
static user_mcode_ptrs_t user_mcode;

// Functions

static user_mcode_type_t mcode_check (user_mcode_t mcode)
{
    return mcode == RGB_Inspection_Light
                     ? UserMCode_Normal
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
}

static status_code_t mcode_validate (parser_block_t *gc_block)
{
    status_code_t state = Status_OK;

    if(gc_block->user_mcode == RGB_Inspection_Light) {

        if(gc_block->words.p) {
            if(!(isintf(gc_block->values.p) && gc_block->values.p >= 0.0f && gc_block->values.p <= 1.0f))
                state = Status_GcodeValueOutOfRange;
            gc_block->words.p = Off;
        }
        if(gc_block->words.q) {
            if(!(isintf(gc_block->values.q) && gc_block->values.q >= 0.0f && gc_block->values.q <= 3.0f))
                state = Status_GcodeValueOutOfRange;
            gc_block->words.q = Off;
        }

        gc_block->user_mcode_sync = On;

    } else
        state = Status_Unhandled;

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

// Physically sets the requested RGB light combination.
// Always sets all three LEDs to avoid unintended light combinations
static void rgb_set_led (rgb_color_t currColor) {

    static rgb_color_t strip0_color = RGB_OFF, strip1_color = RGB_OFF;

    uint16_t device;
    rgb_color_t neocolor;

    if(hal.rgb0.num_devices){

        switch (strip0_override){
            case LEDAllWhite:
            neocolor = RGB_WHITE;
            break;
            case LEDOff:
            neocolor = RGB_OFF;
            break;
            case LEDGreen:
            neocolor = RGB_GREEN;
            break;
            default:
            neocolor = currColor;
        }

        if(neocolor.value != strip0_color.value) {

            strip0_color = neocolor;

            for(device = 0; device < hal.rgb0.num_devices; device++) {
                hal.rgb0.out(device, strip0_color);
            }

            if(hal.rgb0.num_devices > 1 && hal.rgb0.write)
                hal.rgb0.write();
        }
    }

    if(hal.rgb1.num_devices){

        switch (strip1_override){
            case LEDAllWhite:
            neocolor = RGB_WHITE;
            break;
            case LEDOff:
            neocolor = RGB_OFF;
            break;
            case LEDGreen:
            neocolor = RGB_GREEN;
            break;
            default:
            neocolor = currColor;
        }

        if(neocolor.value != strip1_color.value) {

            strip1_color = neocolor;

            for(device = 0; device < hal.rgb1.num_devices; device++) {
                hal.rgb1.out(device, strip1_color);
            }

            if(hal.rgb1.num_devices > 1 && hal.rgb1.write)
                hal.rgb1.write();
        }
    }
}

static void set_hold (void *data)
{
    if(state_get() == STATE_HOLD) {
        if(state_get_substate() != 0 || modbus_isbusy())
            task_add_delayed(set_hold, data, 110);
        else
            rgb_set_led(*(rgb_color_t *)data);
    }
}

static void set_color (void *data)
{
    if(modbus_isbusy())
        task_add_delayed(set_color, data, 110);
    else
        rgb_set_led(*(rgb_color_t *)data);
}

static void RGBUpdateState (sys_state_t state) {

    static rgb_color_t state_color = RGB_OFF;

    switch (state) { // States with solid lights  *** These should use lookups

        // Chilling when idle, cool blue
        case STATE_IDLE:
            state_color = RGB_WHITE;
            break;

        // Running GCode
        case STATE_CYCLE:
            state_color = RGB_GREEN;
            break;

        // Investigate strange soft limits error in joggging
        case STATE_JOG:
            state_color = RGB_GREEN;
            break;

        // Would be nice to having homing be two colours as before, fast and seek - should be possible via real time thread
        case STATE_HOMING:
            state_color = RGB_BLUE;
            break;

        case STATE_HOLD:
        case STATE_SAFETY_DOOR:
            state_color = RGB_YELLOW;
            break;

        case STATE_CHECK_MODE:
            state_color = RGB_BLUE;
            break;

        case STATE_ESTOP:
        case STATE_ALARM:
            state_color = RGB_RED;
            break;

        case STATE_TOOL_CHANGE:
            state_color = RGB_MAGENTA;
            break;

        case STATE_SLEEP:
            state_color = RGB_GREY;
            break;
    }

    task_add_delayed(state == STATE_HOLD ? set_hold : set_color, &state_color, state == STATE_HOLD ? 200 : 10);
}

static void mcode_execute (uint_fast16_t state, parser_block_t *gc_block)
{
    if(gc_block->user_mcode == RGB_Inspection_Light) {

        switch((LED_flags_t)gc_block->values.q) {

            case LEDStateDriven:
                if(gc_block->values.p == 0.0f){
                    strip0_override = LEDStateDriven;
                    report_message("Rail lights automatic", Message_Info);
                }else{
                    strip1_override = LEDStateDriven;
                    report_message("Ring lights automatic", Message_Info);
                }
                break;

            case LEDAllWhite:
                if(gc_block->values.p == 0.0f){
                    strip0_override = LEDAllWhite;
                    report_message("Rail lights all white", Message_Info);
                }else{
                    strip1_override = LEDAllWhite;
                    report_message("Ring lights all white", Message_Info);
                }
                break;

            case LEDOff:
                if(gc_block->values.p == 0.0f){
                    strip0_override = LEDOff;
                    report_message("Rail lights off", Message_Info);
                }else{
                    strip1_override = LEDOff;
                    report_message("Ring lights off", Message_Info);
                }
                break;

            case LEDGreen:
                if(gc_block->values.p == 0.0f){
                    strip0_override = LEDGreen;
                    report_message("Rail lights all green", Message_Info);
                }else{
                    strip1_override = LEDGreen;
                    report_message("Ring lights all green", Message_Info);
                }
            break;
        }

        RGBUpdateState(state_get());
    }

    if(gc_block->user_mcode != RGB_Inspection_Light && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void RGBonStateChanged (sys_state_t state)
{
    RGBUpdateState(state);

    if (on_state_change)         // Call previous function in the chain.
        on_state_change(state);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);  // Call previous function in the chain.

    if(!newopt)                 // Add info about us to the $I report.
        report_plugin("SIENCI Indicator Light", "2.0");
}

// Job finished, wave the chequered flag.
static void job_completed (void *data)
{
    rgb_set_led((*(uint8_t *)data & 1) ? RGB_WHITE : RGB_OFF);
   
    if(--(*(uint8_t *)data))
        task_add_delayed(job_completed, data, 150);
    else
        RGBUpdateState(state_get());
}

// ON (Gcode) PROGRAM COMPLETION
static void onProgramCompleted (program_flow_t program_flow, bool check_mode)
{
    static uint8_t cf_cycle;

    cf_cycle = 10;
    task_add_immediate(job_completed, &cf_cycle);

    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
}

static void on_startup (void *data)
{
    RGBUpdateState(state_get());
} 

void status_light_init (void)
{
    if(rgb_is_neopixels(&hal.rgb0)) {

        on_report_options = grbl.on_report_options;         // Subscribe to report options event
        grbl.on_report_options = onReportOptions;           // Nothing here yet

        on_state_change = grbl.on_state_change;             // Subscribe to the state changed event by saving away the original
        grbl.on_state_change = RGBonStateChanged;           // function pointer and adding ours to the chain.

        on_program_completed = grbl.on_program_completed;   // Subscribe to on program completed events (lightshow on complete?)
        grbl.on_program_completed = onProgramCompleted;     // Checkered Flag for successful end of program lives here

        memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));
        grbl.user_mcode.check = mcode_check;
        grbl.user_mcode.validate = mcode_validate;
        grbl.user_mcode.execute = mcode_execute;

        task_run_on_startup(on_startup, NULL);

#ifdef DEBUG
        hal.rgb0.set_intensity(10);
        if(hal.rgb1.set_intensity)
            hal.rgb1.set_intensity(10);
#endif

    } else
        task_run_on_startup(report_warning, "Status Light plugin failed to initialize!");
}

#endif
