/*
  rgbv2.c - Advanced RGB Status Light Plugin for CNC machines

  Copyright (c) 2026 Sienci Labs

  This file is part of the SuperLongBoard family of products.

   This source describes Open Hardware and is licensed under the "CERN-OHL-S v2"

   You may redistribute and modify this source and make products using
   it under the terms of the CERN-OHL-S v2 (https://ohwr.org/cern_ohl_s_v2.t).
   This source is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY,
   INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A
   PARTICULAR PURPOSE. Please see the CERN-OHL-S v2 for applicable conditions.
*/

#include "driver.h"

#if STATUS_LIGHT_ENABLE == 3

#include <string.h>

#include "grbl/protocol.h"
#include "grbl/hal.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"
#include "grbl/alarms.h"
#include "grbl/nuts_bolts.h"
#include "grbl/task.h"
#include "grbl/modbus.h"

#define RGB_OFF     (rgb_color_t){ .R = 0, .G = 0, .B = 0 }
#define RGB_RED     (rgb_color_t){ .R = 255, .G = 0, .B = 0 }
#define RGB_GREEN   (rgb_color_t){ .R = 0, .G = 255, .B = 0 }
#define RGB_BLUE    (rgb_color_t){ .R = 0, .G = 0, .B = 255 }
#define RGB_YELLOW  (rgb_color_t){ .R = 255, .G = 255, .B = 0 }
#define RGB_MAGENTA (rgb_color_t){ .R = 255, .G = 0, .B = 255 }
#define RGB_CYAN    (rgb_color_t){ .R = 0, .G = 255, .B = 255 }
#define RGB_WHITE   (rgb_color_t){ .R = 255, .G = 255, .B = 255 }
#define RGB_GREY    (rgb_color_t){ .R = 127, .G = 127, .B = 127 }
#define RGBV2_RING_LEDS 12U

typedef enum {
    LEDStateDriven = 0,
    LEDAllWhite = 1,
    LEDOff = 2,
    LEDGreen = 3,
} LED_flags_t;

typedef enum {
    AnimationNone = 0,
    AnimationAlarm,
    AnimationJog,
    AnimationHoming,
    AnimationToolChange,
    AnimationHold,
    AnimationSafetyDoor,
    AnimationCheckMode,
    AnimationEstop,
} rgbv2_animation_t;

static on_state_change_ptr on_state_change;
static on_report_options_ptr on_report_options;
static on_program_completed_ptr on_program_completed;
static user_mcode_ptrs_t user_mcode;
static on_tool_selected_ptr on_tool_selected;
static on_tool_changed_ptr on_tool_changed;

static LED_flags_t ring_override, offboard_override;
static rgb_color_t ring_color[RGBV2_RING_LEDS];
static rgb_color_t offboard_color = RGB_OFF;
static uint8_t strip_intensity = 255;
static bool animation_active = false;
static uint8_t animation_frame = 0;
static sys_state_t active_state = STATE_IDLE;

static void RGBUpdateState (sys_state_t state);
static void set_color (void *data);
static void delayed_state_update (void *data);
static void animation_step (void *data);

static rgb_color_t dim_color (rgb_color_t color, uint8_t scale)
{
    return (rgb_color_t){
        .R = (uint8_t)((uint16_t)color.R * scale / 255U),
        .G = (uint8_t)((uint16_t)color.G * scale / 255U),
        .B = (uint8_t)((uint16_t)color.B * scale / 255U)
    };
}

static rgb_color_t state_color (sys_state_t state)
{
    switch(state) {
        case STATE_IDLE:
            return RGB_WHITE;
        case STATE_CYCLE:
            return RGB_GREEN;
        case STATE_HOMING:
            return RGB_BLUE;
        case STATE_HOLD:
        case STATE_SAFETY_DOOR:
            return RGB_YELLOW;
        case STATE_CHECK_MODE:
            return RGB_CYAN;
        case STATE_JOG:
            return RGB_GREEN;
        case STATE_ESTOP:
        case STATE_ALARM:
            return RGB_RED;
        case STATE_TOOL_CHANGE:
            return RGB_MAGENTA;
        case STATE_SLEEP:
            return RGB_GREY;
        default:
            return RGB_WHITE;
    }
}

static bool animation_allowed (void)
{
    return !modbus_isbusy() && hal.rgb0.num_devices >= RGBV2_RING_LEDS && ring_override == LEDStateDriven;
}

static rgbv2_animation_t animation_for_state (sys_state_t state)
{
    switch(state) {
        case STATE_IDLE:
        case STATE_CYCLE:
        case STATE_SLEEP:
            return AnimationNone;
        case STATE_ALARM:
            return AnimationAlarm;
        case STATE_JOG:
            return AnimationJog;
        case STATE_HOMING:
            return AnimationHoming;
        case STATE_TOOL_CHANGE:
            return AnimationToolChange;
        case STATE_HOLD:
            return AnimationHold;
        case STATE_SAFETY_DOOR:
            return AnimationSafetyDoor;
        case STATE_CHECK_MODE:
            return AnimationCheckMode;
        case STATE_ESTOP:
            return AnimationEstop;
        default:
            return AnimationNone;
    }
}

static uint16_t animation_delay (rgbv2_animation_t animation)
{
    switch(animation) {
        case AnimationAlarm:
            return 60;
        case AnimationJog:
            return 75;
        case AnimationHoming:
            return 100;
        case AnimationToolChange:
            return 90;
        case AnimationHold:
            return 140;
        case AnimationSafetyDoor:
            return 180;
        case AnimationCheckMode:
            return 110;
        case AnimationEstop:
            return 220;
        default:
            return 0;
    }
}

static void fill_ring (rgb_color_t color)
{
    uint8_t i;

    for(i = 0; i < RGBV2_RING_LEDS; i++)
        ring_color[i] = color;
}

static void write_segments (void)
{
    uint16_t device;
    rgb_color_t offboard_effective;

    if(!hal.rgb0.num_devices)
        return;

    switch(offboard_override) {
        case LEDAllWhite:
            offboard_effective = RGB_WHITE;
            break;
        case LEDOff:
            offboard_effective = RGB_OFF;
            break;
        case LEDGreen:
            offboard_effective = RGB_GREEN;
            break;
        default:
            offboard_effective = offboard_color;
            break;
    }

    for(device = 0; device < hal.rgb0.num_devices; device++) {
        if(device < RGBV2_RING_LEDS) {
            rgb_color_t ring_effective;

            switch(ring_override) {
                case LEDAllWhite:
                    ring_effective = RGB_WHITE;
                    break;
                case LEDOff:
                    ring_effective = RGB_OFF;
                    break;
                case LEDGreen:
                    ring_effective = RGB_GREEN;
                    break;
                default:
                    ring_effective = ring_color[device];
                    break;
            }

            hal.rgb0.out(device, ring_effective);
        } else
            hal.rgb0.out(device, offboard_effective);
    }

    if(hal.rgb0.write)
        hal.rgb0.write();
}

static void set_static_state (sys_state_t state)
{
    rgb_color_t color = state_color(state);

    fill_ring(color);
    offboard_color = color;
    write_segments();
}

static void render_alarm_frame (void)
{
    static const uint8_t tail_scales[] = {255U, 160U, 96U, 40U};
    uint8_t i;
    uint8_t head = animation_frame % RGBV2_RING_LEDS;

    fill_ring(RGB_OFF);

    for(i = 0; i < sizeof(tail_scales); i++) {
        uint8_t led = (uint8_t)((head + RGBV2_RING_LEDS - i) % RGBV2_RING_LEDS);
        ring_color[led] = dim_color(RGB_RED, tail_scales[i]);
    }
}

static void render_jog_frame (void)
{
    uint8_t i;
    uint8_t head = animation_frame % RGBV2_RING_LEDS;

    fill_ring(dim_color(RGB_GREEN, 24U));

    for(i = 0; i < 3U; i++) {
        uint8_t led = (uint8_t)((head + i) % RGBV2_RING_LEDS);
        ring_color[led] = dim_color(RGB_GREEN, (uint8_t)(255U - (i * 64U)));
    }
}

static void render_homing_frame (void)
{
    uint8_t i;
    uint8_t radius = (animation_frame % 4U);

    fill_ring(RGB_OFF);

    for(i = 0; i < RGBV2_RING_LEDS / 2U; i++) {
        uint8_t led0 = (uint8_t)((radius + i) % RGBV2_RING_LEDS);
        uint8_t led1 = (uint8_t)((RGBV2_RING_LEDS - 1U - radius + RGBV2_RING_LEDS - i) % RGBV2_RING_LEDS);

        if(i < 2U) {
            ring_color[led0] = dim_color(RGB_BLUE, (uint8_t)(255U - i * 96U));
            ring_color[led1] = dim_color(RGB_BLUE, (uint8_t)(255U - i * 96U));
        }
    }
}

static void render_toolchange_frame (void)
{
    uint8_t i;
    uint8_t phase = animation_frame % 3U;

    fill_ring(RGB_OFF);

    for(i = phase; i < RGBV2_RING_LEDS; i += 3U)
        ring_color[i] = RGB_MAGENTA;
}

static void render_hold_frame (void)
{
    rgb_color_t pulse = (animation_frame & 1U) ? dim_color(RGB_YELLOW, 255U) : dim_color(RGB_YELLOW, 48U);
    fill_ring(pulse);
}

static void render_safety_door_frame (void)
{
    uint8_t i;
    uint8_t half = RGBV2_RING_LEDS / 2U;

    fill_ring(RGB_OFF);

    for(i = 0; i < half; i++) {
        uint8_t led = (uint8_t)((animation_frame & 1U) ? i : (i + half));
        ring_color[led] = RGB_YELLOW;
    }
}

static void render_check_mode_frame (void)
{
    uint8_t i;

    fill_ring(RGB_OFF);

    for(i = 0; i < RGBV2_RING_LEDS; i++) {
        if(((i + animation_frame) & 1U) == 0U)
            ring_color[i] = dim_color(RGB_CYAN, 180U);
    }
}

static void render_estop_frame (void)
{
    rgb_color_t pulse = (animation_frame & 1U) ? RGB_RED : RGB_OFF;
    fill_ring(pulse);
}

static void render_animation_frame (rgbv2_animation_t animation)
{
    offboard_color = state_color(active_state);

    switch(animation) {
        case AnimationAlarm:
            render_alarm_frame();
            break;
        case AnimationJog:
            render_jog_frame();
            break;
        case AnimationHoming:
            render_homing_frame();
            break;
        case AnimationToolChange:
            render_toolchange_frame();
            break;
        case AnimationHold:
            render_hold_frame();
            break;
        case AnimationSafetyDoor:
            render_safety_door_frame();
            break;
        case AnimationCheckMode:
            render_check_mode_frame();
            break;
        case AnimationEstop:
            render_estop_frame();
            break;
        default:
            fill_ring(offboard_color);
            break;
    }

    write_segments();
}

static void animation_stop (void)
{
    animation_active = false;
}

static void animation_step (void *data)
{
    rgbv2_animation_t animation = animation_for_state(active_state);

    (void)data;

    if(!animation_active)
        return;

    if(state_get() != active_state) {
        animation_stop();
        RGBUpdateState(state_get());
        return;
    }

    if(!animation_allowed()) {
        task_add_delayed(animation_step, NULL, 80);
        return;
    }

    if(animation == AnimationNone) {
        animation_stop();
        set_static_state(active_state);
        return;
    }

    render_animation_frame(animation);
    animation_frame++;
    task_add_delayed(animation_step, NULL, animation_delay(animation));
}

static void animation_start (sys_state_t state)
{
    animation_active = true;
    animation_frame = 0;
    active_state = state;
    task_add_immediate(animation_step, NULL);
}

static void RGBonToolSelected (tool_data_t *tool)
{
    static rgb_color_t toolchange_color = RGB_MAGENTA;

    task_add_delayed(set_color, &toolchange_color, 100);

    if(on_tool_selected)
        on_tool_selected(tool);
}

static void delayed_state_update (void *data)
{
    RGBUpdateState((sys_state_t)(uintptr_t)data);
}

static void RGBonToolChanged (tool_data_t *tool)
{
    if(on_tool_changed)
        on_tool_changed(tool);

    task_add_delayed(delayed_state_update, (void *)(uintptr_t)state_get(), 100);
}

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

        if(!gc_block->words.q)
            gc_block->values.q = -1.0f;

        if(!gc_block->words.s)
            gc_block->values.s = -1.0f;

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
        if(gc_block->words.s) {
            if(!(isintf(gc_block->values.s) && gc_block->values.s >= 0.0f && gc_block->values.s <= 255.0f))
                state = Status_GcodeValueOutOfRange;
            gc_block->words.s = Off;
        }

        gc_block->user_mcode_sync = On;

    } else
        state = Status_Unhandled;

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

static void set_color (void *data)
{
    rgb_color_t color = *(rgb_color_t *)data;

    if(modbus_isbusy()) {
        task_add_delayed(set_color, data, 110);
        return;
    }

    fill_ring(color);
    offboard_color = color;
    write_segments();
}

static void RGBUpdateState (sys_state_t state)
{
    rgbv2_animation_t animation = animation_for_state(state);
    sys_state_t previous_state = active_state;

    active_state = state;

    if(animation == AnimationNone) {
        animation_stop();
        set_static_state(state);
        return;
    }

    if(!animation_allowed()) {
        if(modbus_isbusy()) {
            // animation_step() retries while Modbus is busy, like v1 set_color().
            animation_start(state);
            return;
        }

        animation_stop();
        set_static_state(state);
        return;
    }

    if(!animation_active || previous_state != state)
        animation_start(state);
}

static void mcode_execute (uint_fast16_t state, parser_block_t *gc_block)
{
    (void)state;

    if(gc_block->user_mcode == RGB_Inspection_Light) {
        int mode = (int)gc_block->values.q;
        bool onboard = gc_block->values.p == 0.0f;

        if(hal.rgb0.set_intensity && gc_block->values.s >= 0.0f && gc_block->values.s <= 255.0f) {
            strip_intensity = (uint8_t)gc_block->values.s;
            hal.rgb0.set_intensity(strip_intensity);
            report_message("LED brightness updated", Message_Info);
        }

        if(onboard && mode >= 0) {
            ring_override = (LED_flags_t)mode;

            switch(mode) {
                case LEDStateDriven:
                    report_message("Onboard LEDs automatic", Message_Info);
                    break;
                case LEDAllWhite:
                    report_message("Onboard LEDs all white", Message_Info);
                    break;
                case LEDOff:
                    report_message("Onboard LEDs off", Message_Info);
                    break;
                case LEDGreen:
                    report_message("Onboard LEDs all green", Message_Info);
                    break;
            }
        } else if(mode >= 0) {
            offboard_override = (LED_flags_t)mode;

            switch(mode) {
                case LEDStateDriven:
                    report_message("Offboard LEDs automatic", Message_Info);
                    break;
                case LEDAllWhite:
                    report_message("Offboard LEDs all white", Message_Info);
                    break;
                case LEDOff:
                    report_message("Offboard LEDs off", Message_Info);
                    break;
                case LEDGreen:
                    report_message("Offboard LEDs all green", Message_Info);
                    break;
            }
        }

        RGBUpdateState(state_get());
    }

    if(gc_block->user_mcode != RGB_Inspection_Light && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void RGBonStateChanged (sys_state_t state)
{
    RGBUpdateState(state);

    if(on_state_change)
        on_state_change(state);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("SIENCI Indicator Light", "3.0");
}

static void job_completed (void *data)
{
    rgb_color_t color = (*(uint8_t *)data & 1U) ? RGB_WHITE : RGB_OFF;

    fill_ring(color);
    offboard_color = color;
    write_segments();

    if(--(*(uint8_t *)data))
        task_add_delayed(job_completed, data, 150);
    else
        RGBUpdateState(state_get());
}

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
    (void)data;
    RGBUpdateState(state_get());
}

void status_light_init (void)
{
    if(rgb_is_neopixels(&hal.rgb0)) {

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        on_state_change = grbl.on_state_change;
        grbl.on_state_change = RGBonStateChanged;

        on_program_completed = grbl.on_program_completed;
        grbl.on_program_completed = onProgramCompleted;

        memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));
        grbl.user_mcode.check = mcode_check;
        grbl.user_mcode.validate = mcode_validate;
        grbl.user_mcode.execute = mcode_execute;

        on_tool_selected = grbl.on_tool_selected;
        grbl.on_tool_selected = RGBonToolSelected;

        on_tool_changed = grbl.on_tool_changed;
        grbl.on_tool_changed = RGBonToolChanged;

        fill_ring(RGB_OFF);
        task_run_on_startup(on_startup, NULL);

#ifdef DEBUG
        strip_intensity = 10;
        hal.rgb0.set_intensity(strip_intensity);
#endif

    } else
        task_run_on_startup(report_warning, "Status Light v2 plugin failed to initialize!");
}

#endif
