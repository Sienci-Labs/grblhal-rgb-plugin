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

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This RGB control plugin is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with GrblHAL.  If not, see <http://www.gnu.org/licenses/>.

  Copyright reserved by the author.

  M356 -  On = 1, Off = 2, RGB white LED inspection light in RGB Plugin
*/

#include <string.h>
#include <math.h>
#include "driver.h"

#if STATUS_LIGHT_ENABLE // Declared in my_machine.h - you must add in the section with the included plugins

#include "grbl/protocol.h"
#include "grbl/hal.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"
#include "grbl/alarms.h"
#include "grbl/nuts_bolts.h"         // For delay_sec non-blocking timer function

#include "rgb.h"
#include "WS2812.h"

// Declarations

#define NUM_RING_PIXELS 45
#define NUM_RAIL_PIXELS 11

WS2812 ring_led;
WS2812 rail_led;

typedef enum {
    LEDStateDriven = 0,         //!< 0 - state drive
    LEDAllWhite = 1,         //!< 1 - all white
    LEDOff = 2,  //!< 2 - all off
} LED_flags_t;

int ring_buf[NUM_RING_PIXELS];
int rail_buf[NUM_RAIL_PIXELS];

// Set preferred STATLE_IDLE light color, will be moving to a $ setting in future
static uint8_t rail_port;                   // Aux out connected to RAIL
static uint8_t ring_port;                   // Aux out connected to RING led strip
static sys_state_t current_state;           // For storing the current state from sys.state via state_get()

static user_mcode_ptrs_t user_mcode;
static LED_flags_t rail_led_override, ring_led_override;

static on_state_change_ptr on_state_change;                  
static on_report_options_ptr on_report_options;            
static on_program_completed_ptr on_program_completed;
static driver_reset_ptr driver_reset;

typedef struct { // Structure to store the RGB bits
    uint8_t R;
    uint8_t G;
    uint8_t B;
} COLOR_LIST;

#if STATUS_LIGHT_ENABLE == 2 //dim lights for development.
static COLOR_LIST neo_colors[] = {
        { 0, 0, 0 },  // Off [0]
        { 12, 0, 0 },  // Red [1]
        { 0, 12, 0 },  // Green [2]
        { 0, 0, 12 },  // Blue [3]
        { 12, 12, 0 },  // Yellow [4]
        { 12, 0, 12 },  // Magenta [5]
        { 0, 12, 12 },  // Cyan [6]
        { 12, 12, 12 },  // White [7]
        { 12, 5, 0 },  // Orange [7]
        { 5, 5, 5 },  // Grey [7]
};
#else
static COLOR_LIST neo_colors[] = {
        { 0, 0, 0 },  // Off [0]
        { 255, 0, 0 },  // Red [1]
        { 0, 255, 0 },  // Green [2]
        { 0, 0, 255 },  // Blue [3]
        { 255, 255, 0 },  // Yellow [4]
        { 255, 0, 255 },  // Magenta [5]
        { 0, 255, 255 },  // Cyan [6]
        { 255, 255, 255 },  // White [7]
        { 255, 127, 0 },  // Orange [7]
        { 50, 50, 50 },  // Grey [7]
};
#endif

// Functions

static user_mcode_t mcode_check (user_mcode_t mcode)
{
    return mcode == (user_mcode_t)356
                     ? mcode
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Ignore);
}

static status_code_t mcode_validate (parser_block_t *gc_block, parameter_words_t *deprecated)
{
    status_code_t state = Status_OK;

    if(gc_block->user_mcode == (user_mcode_t)356) {

        if(gc_block->words.p) {
            if(isnanf(gc_block->values.p))
                state = Status_GcodeValueWordMissing;
            else if(!(isintf(gc_block->values.p) && gc_block->values.p >= 0.0f && gc_block->values.p <= 1.0f))
                state = Status_GcodeValueOutOfRange;
        } else if(gc_block->words.q) {
            if(isnanf(gc_block->values.q))
                state = Status_GcodeValueWordMissing;
            else if(!(isintf(gc_block->values.q) && gc_block->values.q >= 0.0f && gc_block->values.q <= 2.0f))
                state = Status_GcodeValueOutOfRange;
        } else
            state = Status_GcodeValueWordMissing;

        if(state == Status_OK) {
            gc_block->words.p = gc_block->words.q = Off;
            gc_block->user_mcode_sync = On;
        }

    } else
        state = Status_Unhandled;

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block, deprecated) : state;
}

// Physically sets the requested RGB light combination.
// Always sets all three LEDs to avoid unintended light combinations
static void rgb_set_led (uint8_t reqColor) { 
    static uint8_t currColor = 99;
    int neocolor;
    if ( currColor != reqColor) {
        currColor = reqColor;
        

        switch (rail_led_override){
            case LEDAllWhite:
            neocolor = (neo_colors[RGB_WHITE].G)<<16 | (neo_colors[RGB_WHITE].R)<<8 | neo_colors[RGB_WHITE].B;
            break;
            case LEDOff:
            neocolor = (neo_colors[RGB_OFF].G)<<16 | (neo_colors[RGB_OFF].R)<<8 | neo_colors[RGB_OFF].B;
            break;
            default:
            neocolor = (neo_colors[currColor].G)<<16 | (neo_colors[currColor].R)<<8 | neo_colors[currColor].B;                       
        }
        WS2812_write_simple(&rail_led, neocolor);
        if(ring_led.size){
            switch (ring_led_override){
                case LEDAllWhite:
                neocolor = (neo_colors[RGB_WHITE].G)<<16 | (neo_colors[RGB_WHITE].R)<<8 | neo_colors[RGB_WHITE].B;
                break;
                case LEDOff:
                neocolor = (neo_colors[RGB_OFF].G)<<16 | (neo_colors[RGB_OFF].R)<<8 | neo_colors[RGB_OFF].B;
                break;
                default:
                neocolor = (neo_colors[currColor].G)<<16 | (neo_colors[currColor].R)<<8 | neo_colors[currColor].B;            
            }
            WS2812_write_simple(&ring_led, neocolor);
        }   
    }
}

static void warning_msg (uint_fast16_t state)
{
    report_message("RGB plugin failed to initialize!", Message_Warning);
}
 
static void RGBUpdateState (sys_state_t state){
    
    switch (state) { // States with solid lights  *** These should use lookups

        // Chilling when idle, cool blue
        case STATE_IDLE:
            rgb_set_led(RGB_WHITE);
            break; 

        // Running GCode (only seen if no M3 command for spindle on)
        case STATE_CYCLE:
            rgb_set_led(RGB_GREEN);
            break; 
        
        // Investigate strange soft limits error in joggging
        case STATE_JOG:
            rgb_set_led(RGB_GREEN);
            break; 

        // Would be nice to having homing be two colours as before, fast and seek - should be possible via real time thread
        case STATE_HOMING:
            rgb_set_led(RGB_BLUE);
            break;

        case STATE_HOLD:
            rgb_set_led(RGB_YELLOW);
            break;

        case STATE_SAFETY_DOOR:
            rgb_set_led(RGB_YELLOW);
            break;   

        case STATE_CHECK_MODE:
            rgb_set_led(RGB_BLUE);
            break;  

        case STATE_ESTOP:
        case STATE_ALARM:
            rgb_set_led(RGB_RED);
            break;          

        case STATE_TOOL_CHANGE:
            rgb_set_led(RGB_MAGENTA);
            break;

        case STATE_SLEEP:
            rgb_set_led(RGB_GREY);
            break;                                                                         
        }
}

static void mcode_execute (uint_fast16_t state, parser_block_t *gc_block)
{
    bool handled = false;
    
    if(gc_block->user_mcode == (user_mcode_t)356) {

        if(gc_block->values.q == 0.0f){//state drive
            if(gc_block->values.p == 0.0f){
                rail_led_override = LEDStateDriven;
                report_message("Rail lights automatic", Message_Info);
            }else{
                ring_led_override = LEDStateDriven;
                report_message("Ring lights automatic", Message_Info);
            }    
            handled = true;
            
        } else if (gc_block->values.q == 1.0f){//white override
            if(gc_block->values.p == 0.0f){
                rail_led_override = LEDAllWhite;
                report_message("Rail lights all white", Message_Info);
            }else{
                ring_led_override = LEDAllWhite;
                report_message("Ring lights all white", Message_Info);
            }            
            handled = true;               
        } else{//off override
            if(gc_block->values.p == 0.0f){
                rail_led_override = LEDOff;
                report_message("Rail lights off", Message_Info);
            }else{
                ring_led_override = LEDOff;
                report_message("Ring lights off", Message_Info);
            }         
            handled = true;
        }
    } else
        handled = false;

    rgb_set_led(RGB_OFF); 
    current_state = state_get();
    RGBUpdateState(current_state);   

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void RGBonStateChanged (sys_state_t state)
{
    RGBUpdateState(state);
    
    if (on_state_change)         // Call previous function in the chain.
        on_state_change(state);
}


static void onReportOptions (bool newopt) // Stock template
{
    on_report_options(newopt);  // Call previous function in the chain.

    if(!newopt)                 // Add info about us to the $I report.
        hal.stream.write("[PLUGIN:SIENCI Indicator Lights v1.0]" ASCII_EOL);
}

// ON (Gcode) PROGRAM COMPLETION
static void onProgramCompleted (program_flow_t program_flow, bool check_mode)
{
    int cf_cycle = 0;

    // Job finished, wave the chequered flag.  Currently blocking, but as job is finished, is this an issue?
    while (cf_cycle <= 5) {
        rgb_set_led(RGB_WHITE);    
        hal.delay_ms(150, NULL);  // Changed from just delay() to make code more portable pre Terje IO
        rgb_set_led(RGB_OFF);    
        hal.delay_ms(150, NULL);
       
        cf_cycle++;
    }
    current_state = state_get();
    RGBUpdateState(current_state);   

    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
    
    cf_cycle = 0;
}

// DRIVER RESET - Release ports
static void driverReset (void)
{
    driver_reset();
}

// INIT FUNCTION - CALLED FROM plugins_init.h()
void status_light_init() {

    // CLAIM AUX OUTPUTS FOR RGB LIGHT STRIPS
    if(hal.port.num_digital_out >= 2) {

        if(hal.port.set_pin_description) {  // Viewable from $PINS command in MDI / Console

        ring_port = RING_LED_AUXOUT;
        rail_port = RAIL_LED_AUXOUT;

        ioport_claim(Port_Digital, Port_Output, &ring_port, "RING NEOPIXEL PORT");
        ioport_claim(Port_Digital, Port_Output, &rail_port, "RAIL NEOPIXEL PORT");

        rail_led.gpo = rail_port;
        rail_led.size = NUM_RAIL_PIXELS;
        WS2812_setDelays(&rail_led, 0, 5, 10, 5);
        ring_led.gpo = ring_port;
        ring_led.size = NUM_RING_PIXELS;
        WS2812_setDelays(&ring_led, 0, 5, 10, 5); 
      
        driver_reset = hal.driver_reset;                    // Subscribe to driver reset event
        hal.driver_reset = driverReset;

        on_report_options = grbl.on_report_options;         // Subscribe to report options event
        grbl.on_report_options = onReportOptions;           // Nothing here yet

        on_state_change = grbl.on_state_change;             // Subscribe to the state changed event by saving away the original
        grbl.on_state_change = RGBonStateChanged;              // function pointer and adding ours to the chain.

        on_program_completed = grbl.on_program_completed;   // Subscribe to on program completed events (lightshow on complete?)
        grbl.on_program_completed = onProgramCompleted;     // Checkered Flag for successful end of program lives here


        memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));
        hal.user_mcode.check = mcode_check;
        hal.user_mcode.validate = mcode_validate;
        hal.user_mcode.execute = mcode_execute;         

        protocol_enqueue_rt_command(RGBUpdateState);

    }
} else
        protocol_enqueue_rt_command(warning_msg);
}
# endif
