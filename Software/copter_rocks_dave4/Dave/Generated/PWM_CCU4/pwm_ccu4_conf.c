/*********************************************************************************************************************
* DAVE APP Name : PWM_CCU4       APP Version: 4.1.18
*
* NOTE:
* This file is generated by DAVE. Any manual modification done to this file will be lost when the code is regenerated.
*********************************************************************************************************************/

/**
 * @cond
 ***********************************************************************************************************************
 *
 * Copyright (c) 2016, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * To improve the quality of the software, users are encouraged to share modifications, enhancements or bug fixes
 * with Infineon Technologies AG (dave@infineon.com).
 ***********************************************************************************************************************
 *
 * Change History
 * --------------
 *
 * 2015-02-14:
 *     - Initial version for DAVEv4
 *
 * 2015-08-13:
 *     - Added support for initializing  "immediate_write" member in  "PWM_CCU4_CONFIG_t"
 *     - Added support for initializing  "automatic_shadow_transfer" member in  "PWM_CCU4_CONFIG_t"
 *     - Added support for initializing  "shadow_transfer_mode" member in "PWM_CCU4_CONFIG_t"
 * @endcond
 *
 */

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "pwm_ccu4.h"

/**********************************************************************************************************************
 * DATA STRUCTURES
 **********************************************************************************************************************/

/********************************************************************************************************************/

    const XMC_CCU4_SLICE_COMPARE_CONFIG_t  PWM_OUTPUT_1_timer_handle =
    {
      .timer_mode            = (uint32_t)XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
      .monoshot              = (uint32_t)XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT,
      .shadow_xfer_clear     = 0U,
      .dither_timer_period   = 0U,
      .dither_duty_cycle     = 0U,

      .prescaler_mode        = (uint32_t)XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,

      .mcm_enable            = 0U,
      .prescaler_initval     = 3U,
      .dither_limit          = 0U,
      .timer_concatenation   = 0U,
      .passive_level         = (uint32_t)XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW, 
    };


    const XMC_CCU4_SLICE_EVENT_CONFIG_t PWM_OUTPUT_1_event0_config = 
    {
      .mapped_input        = XMC_CCU4_SLICE_INPUT_A,
      .edge                = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
      .level               = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
      .duration            = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
    };

    const XMC_CCU4_SLICE_EVENT_CONFIG_t PWM_OUTPUT_1_event1_config = 
    {
      .mapped_input        = XMC_CCU4_SLICE_INPUT_A,
      .edge                = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
      .level               = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
      .duration            = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
    };

    const XMC_CCU4_SLICE_EVENT_CONFIG_t PWM_OUTPUT_1_event2_config = 
    {
      .mapped_input        = XMC_CCU4_SLICE_INPUT_A,
      .edge                = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
      .level               = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
      .duration            = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
    };



#if UC_FAMILY == XMC1
    const XMC_GPIO_CONFIG_t  PWM_OUTPUT_1_ch1_out0_config    =
    {
      .mode                = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT3,
      .input_hysteresis    = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
      .output_level        = XMC_GPIO_OUTPUT_LEVEL_LOW
    };
#endif

#if UC_FAMILY == XMC4
    const XMC_GPIO_CONFIG_t  PWM_OUTPUT_1_ch1_out0_config    =
    {
      .mode                = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT3,
      .output_level        = XMC_GPIO_OUTPUT_LEVEL_LOW,
      .output_strength     = XMC_GPIO_OUTPUT_STRENGTH_MEDIUM
    };
#endif


    const PWM_CCU4_CONFIG_t  PWM_OUTPUT_1_config_handle =
    {
      .start_control                       = false,      
      .period_value                        = 33332U,
      .compare_value                       = 31666U,

      .int_per_match                       = false,
      .int_cmp_match_up                    = false,
      .int_cmp_match_down                  = false,

      .int_one_match_down                  = false,
      .int_e0                              = false,
      .int_e1                              = false,
      .int_e2                              = false,

      .sr_per_match                        = XMC_CCU4_SLICE_SR_ID_0,
      .sr_cmp_match_up                     = XMC_CCU4_SLICE_SR_ID_0,
      .sr_cmp_match_down                   = XMC_CCU4_SLICE_SR_ID_0,

      .sr_one_match_down                   = XMC_CCU4_SLICE_SR_ID_0,
      .sr_e0                               = XMC_CCU4_SLICE_SR_ID_0,
      .sr_e1                               = XMC_CCU4_SLICE_SR_ID_0,
      .sr_e2                               = XMC_CCU4_SLICE_SR_ID_0,

      .event0_config_ptr                   = &PWM_OUTPUT_1_event0_config,
      .event1_config_ptr                   = &PWM_OUTPUT_1_event1_config,
      .event2_config_ptr                   = &PWM_OUTPUT_1_event2_config,

      .ext_start_event                     = XMC_CCU4_SLICE_EVENT_NONE,
      .ext_start_mode                      = XMC_CCU4_SLICE_START_MODE_TIMER_START,

      .ext_stop_event                      = XMC_CCU4_SLICE_EVENT_NONE,
      .ext_stop_mode                       = XMC_CCU4_SLICE_END_MODE_TIMER_STOP,

      .ext_count_dir_event                 = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_gate_event                      = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_count_event                     = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_load_event                      = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_mod_event                       = XMC_CCU4_SLICE_EVENT_NONE,
      .ext_mod_mode                        = XMC_CCU4_SLICE_MODULATION_MODE_CLEAR_ST_OUT,
      .ext_mod_sync                        = false,

      .ext_override_edge_event             = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_override_level_event            = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_trap_enable                     = false,
      .ext_trap_event                      = XMC_CCU4_SLICE_EVENT_NONE,
      .ext_trap_sync                       = true,
      .ext_trap_exit                       = XMC_CCU4_SLICE_TRAP_EXIT_MODE_AUTOMATIC,

      .mcm_shadow_txfr_mode                = XMC_CCU4_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE3,
      
#if (UC_SERIES == XMC14)
      .shadow_transfer_mode                = XMC_CCU4_SLICE_SHADOW_TRANSFER_MODE_ONLY_IN_PERIOD_MATCH,

      .immediate_write                     = 0U,

      .automatic_shadow_transfer           = 0U,

      .cascaded_shadow_txfr_enable         = false,
#endif

      .ccu4_cc4_slice_timer_ptr            = &PWM_OUTPUT_1_timer_handle,

      .gpio_ch_out_enable                  = true,
      .gpio_ch_out_ptr                     = (XMC_GPIO_PORT_t *) PORT1_BASE,
      .gpio_ch_out_pin                     = 0U,
      .gpio_ch_out_config_ptr              = &PWM_OUTPUT_1_ch1_out0_config,    

      .global_ccu4_handle                   = (GLOBAL_CCU4_t*) &GLOBAL_CCU4_2,
    };

    PWM_CCU4_t PWM_OUTPUT_1 =
    {
      .config_ptr                          = &PWM_OUTPUT_1_config_handle,
      .ccu4_module_ptr                     = (XMC_CCU4_MODULE_t*) CCU40_BASE,
      .ccu4_slice_ptr                      = (XMC_CCU4_SLICE_t*) CCU40_CC43,
      .slice_number                        = 3U,
      .kernel_number                       = 0U,
      .shadow_txfr_msk                     = (uint32_t)XMC_CCU4_SHADOW_TRANSFER_SLICE_3,
      .dither_shadow_txfr_msk              = (uint32_t)XMC_CCU4_SHADOW_TRANSFER_DITHER_SLICE_3,
      .prescaler_shadow_txfr_msk           = (uint32_t)XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_3,

      .state                               = PWM_CCU4_STATE_UNINITIALIZED,
      .sym_duty                            = 500U,

    };

/********************************************************************************************************/


/********************************************************************************************************************/

    const XMC_CCU4_SLICE_COMPARE_CONFIG_t  PWM_OUTPUT_2_timer_handle =
    {
      .timer_mode            = (uint32_t)XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
      .monoshot              = (uint32_t)XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT,
      .shadow_xfer_clear     = 0U,
      .dither_timer_period   = 0U,
      .dither_duty_cycle     = 0U,

      .prescaler_mode        = (uint32_t)XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,

      .mcm_enable            = 0U,
      .prescaler_initval     = 3U,
      .dither_limit          = 0U,
      .timer_concatenation   = 0U,
      .passive_level         = (uint32_t)XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW, 
    };


    const XMC_CCU4_SLICE_EVENT_CONFIG_t PWM_OUTPUT_2_event0_config = 
    {
      .mapped_input        = XMC_CCU4_SLICE_INPUT_A,
      .edge                = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
      .level               = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
      .duration            = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
    };

    const XMC_CCU4_SLICE_EVENT_CONFIG_t PWM_OUTPUT_2_event1_config = 
    {
      .mapped_input        = XMC_CCU4_SLICE_INPUT_A,
      .edge                = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
      .level               = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
      .duration            = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
    };

    const XMC_CCU4_SLICE_EVENT_CONFIG_t PWM_OUTPUT_2_event2_config = 
    {
      .mapped_input        = XMC_CCU4_SLICE_INPUT_A,
      .edge                = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
      .level               = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
      .duration            = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
    };



#if UC_FAMILY == XMC1
    const XMC_GPIO_CONFIG_t  PWM_OUTPUT_2_ch1_out0_config    =
    {
      .mode                = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT3,
      .input_hysteresis    = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
      .output_level        = XMC_GPIO_OUTPUT_LEVEL_LOW
    };
#endif

#if UC_FAMILY == XMC4
    const XMC_GPIO_CONFIG_t  PWM_OUTPUT_2_ch1_out0_config    =
    {
      .mode                = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT3,
      .output_level        = XMC_GPIO_OUTPUT_LEVEL_LOW,
      .output_strength     = XMC_GPIO_OUTPUT_STRENGTH_MEDIUM
    };
#endif


    const PWM_CCU4_CONFIG_t  PWM_OUTPUT_2_config_handle =
    {
      .start_control                       = false,      
      .period_value                        = 33332U,
      .compare_value                       = 31666U,

      .int_per_match                       = false,
      .int_cmp_match_up                    = false,
      .int_cmp_match_down                  = false,

      .int_one_match_down                  = false,
      .int_e0                              = false,
      .int_e1                              = false,
      .int_e2                              = false,

      .sr_per_match                        = XMC_CCU4_SLICE_SR_ID_0,
      .sr_cmp_match_up                     = XMC_CCU4_SLICE_SR_ID_0,
      .sr_cmp_match_down                   = XMC_CCU4_SLICE_SR_ID_0,

      .sr_one_match_down                   = XMC_CCU4_SLICE_SR_ID_0,
      .sr_e0                               = XMC_CCU4_SLICE_SR_ID_0,
      .sr_e1                               = XMC_CCU4_SLICE_SR_ID_0,
      .sr_e2                               = XMC_CCU4_SLICE_SR_ID_0,

      .event0_config_ptr                   = &PWM_OUTPUT_2_event0_config,
      .event1_config_ptr                   = &PWM_OUTPUT_2_event1_config,
      .event2_config_ptr                   = &PWM_OUTPUT_2_event2_config,

      .ext_start_event                     = XMC_CCU4_SLICE_EVENT_NONE,
      .ext_start_mode                      = XMC_CCU4_SLICE_START_MODE_TIMER_START,

      .ext_stop_event                      = XMC_CCU4_SLICE_EVENT_NONE,
      .ext_stop_mode                       = XMC_CCU4_SLICE_END_MODE_TIMER_STOP,

      .ext_count_dir_event                 = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_gate_event                      = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_count_event                     = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_load_event                      = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_mod_event                       = XMC_CCU4_SLICE_EVENT_NONE,
      .ext_mod_mode                        = XMC_CCU4_SLICE_MODULATION_MODE_CLEAR_ST_OUT,
      .ext_mod_sync                        = false,

      .ext_override_edge_event             = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_override_level_event            = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_trap_enable                     = false,
      .ext_trap_event                      = XMC_CCU4_SLICE_EVENT_NONE,
      .ext_trap_sync                       = true,
      .ext_trap_exit                       = XMC_CCU4_SLICE_TRAP_EXIT_MODE_AUTOMATIC,

      .mcm_shadow_txfr_mode                = XMC_CCU4_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE2,
      
#if (UC_SERIES == XMC14)
      .shadow_transfer_mode                = XMC_CCU4_SLICE_SHADOW_TRANSFER_MODE_ONLY_IN_PERIOD_MATCH,

      .immediate_write                     = (0U),

      .automatic_shadow_transfer           = (0U),

      .cascaded_shadow_txfr_enable         = false,
#endif

      .ccu4_cc4_slice_timer_ptr            = &PWM_OUTPUT_2_timer_handle,

      .gpio_ch_out_enable                  = true,
      .gpio_ch_out_ptr                     = (XMC_GPIO_PORT_t *) PORT1_BASE,
      .gpio_ch_out_pin                     = 1U,
      .gpio_ch_out_config_ptr              = &PWM_OUTPUT_2_ch1_out0_config,    

      .global_ccu4_handle                   = (GLOBAL_CCU4_t*) &GLOBAL_CCU4_2,
    };

    PWM_CCU4_t PWM_OUTPUT_2 =
    {
      .config_ptr                          = &PWM_OUTPUT_2_config_handle,
      .ccu4_module_ptr                     = (XMC_CCU4_MODULE_t*) CCU40_BASE,
      .ccu4_slice_ptr                      = (XMC_CCU4_SLICE_t*) CCU40_CC42,
      .slice_number                        = 2U,
      .kernel_number                       = 0U,
      .shadow_txfr_msk                     = (uint32_t)XMC_CCU4_SHADOW_TRANSFER_SLICE_2,
      .dither_shadow_txfr_msk              = (uint32_t)XMC_CCU4_SHADOW_TRANSFER_DITHER_SLICE_2,
      .prescaler_shadow_txfr_msk           = (uint32_t)XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_2,

      .state                               = PWM_CCU4_STATE_UNINITIALIZED,
      .sym_duty                            = 500U,

    };

/********************************************************************************************************/


/********************************************************************************************************************/

    const XMC_CCU4_SLICE_COMPARE_CONFIG_t  PWM_OUTPUT_3_timer_handle =
    {
      .timer_mode            = (uint32_t)XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
      .monoshot              = (uint32_t)XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT,
      .shadow_xfer_clear     = 0U,
      .dither_timer_period   = 0U,
      .dither_duty_cycle     = 0U,

      .prescaler_mode        = (uint32_t)XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,

      .mcm_enable            = 0U,
      .prescaler_initval     = 3U,
      .dither_limit          = 0U,
      .timer_concatenation   = 0U,
      .passive_level         = (uint32_t)XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW, 
    };


    const XMC_CCU4_SLICE_EVENT_CONFIG_t PWM_OUTPUT_3_event0_config = 
    {
      .mapped_input        = XMC_CCU4_SLICE_INPUT_A,
      .edge                = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
      .level               = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
      .duration            = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
    };

    const XMC_CCU4_SLICE_EVENT_CONFIG_t PWM_OUTPUT_3_event1_config = 
    {
      .mapped_input        = XMC_CCU4_SLICE_INPUT_A,
      .edge                = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
      .level               = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
      .duration            = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
    };

    const XMC_CCU4_SLICE_EVENT_CONFIG_t PWM_OUTPUT_3_event2_config = 
    {
      .mapped_input        = XMC_CCU4_SLICE_INPUT_A,
      .edge                = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
      .level               = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
      .duration            = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
    };



#if UC_FAMILY == XMC1
    const XMC_GPIO_CONFIG_t  PWM_OUTPUT_3_ch1_out0_config    =
    {
      .mode                = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT3,
      .input_hysteresis    = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
      .output_level        = XMC_GPIO_OUTPUT_LEVEL_LOW
    };
#endif

#if UC_FAMILY == XMC4
    const XMC_GPIO_CONFIG_t  PWM_OUTPUT_3_ch1_out0_config    =
    {
      .mode                = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT3,
      .output_level        = XMC_GPIO_OUTPUT_LEVEL_LOW,
      .output_strength     = XMC_GPIO_OUTPUT_STRENGTH_MEDIUM
    };
#endif


    const PWM_CCU4_CONFIG_t  PWM_OUTPUT_3_config_handle =
    {
      .start_control                       = false,      
      .period_value                        = 33332U,
      .compare_value                       = 31666U,

      .int_per_match                       = false,
      .int_cmp_match_up                    = false,
      .int_cmp_match_down                  = false,

      .int_one_match_down                  = false,
      .int_e0                              = false,
      .int_e1                              = false,
      .int_e2                              = false,

      .sr_per_match                        = XMC_CCU4_SLICE_SR_ID_0,
      .sr_cmp_match_up                     = XMC_CCU4_SLICE_SR_ID_0,
      .sr_cmp_match_down                   = XMC_CCU4_SLICE_SR_ID_0,

      .sr_one_match_down                   = XMC_CCU4_SLICE_SR_ID_0,
      .sr_e0                               = XMC_CCU4_SLICE_SR_ID_0,
      .sr_e1                               = XMC_CCU4_SLICE_SR_ID_0,
      .sr_e2                               = XMC_CCU4_SLICE_SR_ID_0,

      .event0_config_ptr                   = &PWM_OUTPUT_3_event0_config,
      .event1_config_ptr                   = &PWM_OUTPUT_3_event1_config,
      .event2_config_ptr                   = &PWM_OUTPUT_3_event2_config,

      .ext_start_event                     = XMC_CCU4_SLICE_EVENT_NONE,
      .ext_start_mode                      = XMC_CCU4_SLICE_START_MODE_TIMER_START,

      .ext_stop_event                      = XMC_CCU4_SLICE_EVENT_NONE,
      .ext_stop_mode                       = XMC_CCU4_SLICE_END_MODE_TIMER_STOP,

      .ext_count_dir_event                 = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_gate_event                      = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_count_event                     = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_load_event                      = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_mod_event                       = XMC_CCU4_SLICE_EVENT_NONE,
      .ext_mod_mode                        = XMC_CCU4_SLICE_MODULATION_MODE_CLEAR_ST_OUT,
      .ext_mod_sync                        = false,

      .ext_override_edge_event             = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_override_level_event            = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_trap_enable                     = false,
      .ext_trap_event                      = XMC_CCU4_SLICE_EVENT_NONE,
      .ext_trap_sync                       = true,
      .ext_trap_exit                       = XMC_CCU4_SLICE_TRAP_EXIT_MODE_AUTOMATIC,

      .mcm_shadow_txfr_mode                = XMC_CCU4_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE1,
      
#if (UC_SERIES == XMC14)
      .shadow_transfer_mode                = XMC_CCU4_SLICE_SHADOW_TRANSFER_MODE_ONLY_IN_PERIOD_MATCH,

      .immediate_write                     = ((0U)),

      .automatic_shadow_transfer           = ((0U)),

      .cascaded_shadow_txfr_enable         = false,
#endif

      .ccu4_cc4_slice_timer_ptr            = &PWM_OUTPUT_3_timer_handle,

      .gpio_ch_out_enable                  = true,
      .gpio_ch_out_ptr                     = (XMC_GPIO_PORT_t *) PORT1_BASE,
      .gpio_ch_out_pin                     = 2U,
      .gpio_ch_out_config_ptr              = &PWM_OUTPUT_3_ch1_out0_config,    

      .global_ccu4_handle                   = (GLOBAL_CCU4_t*) &GLOBAL_CCU4_2,
    };

    PWM_CCU4_t PWM_OUTPUT_3 =
    {
      .config_ptr                          = &PWM_OUTPUT_3_config_handle,
      .ccu4_module_ptr                     = (XMC_CCU4_MODULE_t*) CCU40_BASE,
      .ccu4_slice_ptr                      = (XMC_CCU4_SLICE_t*) CCU40_CC41,
      .slice_number                        = 1U,
      .kernel_number                       = 0U,
      .shadow_txfr_msk                     = (uint32_t)XMC_CCU4_SHADOW_TRANSFER_SLICE_1,
      .dither_shadow_txfr_msk              = (uint32_t)XMC_CCU4_SHADOW_TRANSFER_DITHER_SLICE_1,
      .prescaler_shadow_txfr_msk           = (uint32_t)XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_1,

      .state                               = PWM_CCU4_STATE_UNINITIALIZED,
      .sym_duty                            = 500U,

    };

/********************************************************************************************************/


/********************************************************************************************************************/

    const XMC_CCU4_SLICE_COMPARE_CONFIG_t  PWM_OUTPUT_4_timer_handle =
    {
      .timer_mode            = (uint32_t)XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
      .monoshot              = (uint32_t)XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT,
      .shadow_xfer_clear     = 0U,
      .dither_timer_period   = 0U,
      .dither_duty_cycle     = 0U,

      .prescaler_mode        = (uint32_t)XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,

      .mcm_enable            = 0U,
      .prescaler_initval     = 3U,
      .dither_limit          = 0U,
      .timer_concatenation   = 0U,
      .passive_level         = (uint32_t)XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW, 
    };


    const XMC_CCU4_SLICE_EVENT_CONFIG_t PWM_OUTPUT_4_event0_config = 
    {
      .mapped_input        = XMC_CCU4_SLICE_INPUT_A,
      .edge                = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
      .level               = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
      .duration            = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
    };

    const XMC_CCU4_SLICE_EVENT_CONFIG_t PWM_OUTPUT_4_event1_config = 
    {
      .mapped_input        = XMC_CCU4_SLICE_INPUT_A,
      .edge                = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
      .level               = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
      .duration            = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
    };

    const XMC_CCU4_SLICE_EVENT_CONFIG_t PWM_OUTPUT_4_event2_config = 
    {
      .mapped_input        = XMC_CCU4_SLICE_INPUT_A,
      .edge                = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
      .level               = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
      .duration            = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
    };



#if UC_FAMILY == XMC1
    const XMC_GPIO_CONFIG_t  PWM_OUTPUT_4_ch1_out0_config    =
    {
      .mode                = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT3,
      .input_hysteresis    = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
      .output_level        = XMC_GPIO_OUTPUT_LEVEL_LOW
    };
#endif

#if UC_FAMILY == XMC4
    const XMC_GPIO_CONFIG_t  PWM_OUTPUT_4_ch1_out0_config    =
    {
      .mode                = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT3,
      .output_level        = XMC_GPIO_OUTPUT_LEVEL_LOW,
      .output_strength     = XMC_GPIO_OUTPUT_STRENGTH_MEDIUM
    };
#endif


    const PWM_CCU4_CONFIG_t  PWM_OUTPUT_4_config_handle =
    {
      .start_control                       = false,      
      .period_value                        = 33332U,
      .compare_value                       = 31666U,

      .int_per_match                       = false,
      .int_cmp_match_up                    = false,
      .int_cmp_match_down                  = false,

      .int_one_match_down                  = false,
      .int_e0                              = false,
      .int_e1                              = false,
      .int_e2                              = false,

      .sr_per_match                        = XMC_CCU4_SLICE_SR_ID_0,
      .sr_cmp_match_up                     = XMC_CCU4_SLICE_SR_ID_0,
      .sr_cmp_match_down                   = XMC_CCU4_SLICE_SR_ID_0,

      .sr_one_match_down                   = XMC_CCU4_SLICE_SR_ID_0,
      .sr_e0                               = XMC_CCU4_SLICE_SR_ID_0,
      .sr_e1                               = XMC_CCU4_SLICE_SR_ID_0,
      .sr_e2                               = XMC_CCU4_SLICE_SR_ID_0,

      .event0_config_ptr                   = &PWM_OUTPUT_4_event0_config,
      .event1_config_ptr                   = &PWM_OUTPUT_4_event1_config,
      .event2_config_ptr                   = &PWM_OUTPUT_4_event2_config,

      .ext_start_event                     = XMC_CCU4_SLICE_EVENT_NONE,
      .ext_start_mode                      = XMC_CCU4_SLICE_START_MODE_TIMER_START,

      .ext_stop_event                      = XMC_CCU4_SLICE_EVENT_NONE,
      .ext_stop_mode                       = XMC_CCU4_SLICE_END_MODE_TIMER_STOP,

      .ext_count_dir_event                 = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_gate_event                      = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_count_event                     = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_load_event                      = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_mod_event                       = XMC_CCU4_SLICE_EVENT_NONE,
      .ext_mod_mode                        = XMC_CCU4_SLICE_MODULATION_MODE_CLEAR_ST_OUT,
      .ext_mod_sync                        = false,

      .ext_override_edge_event             = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_override_level_event            = XMC_CCU4_SLICE_EVENT_NONE,

      .ext_trap_enable                     = false,
      .ext_trap_event                      = XMC_CCU4_SLICE_EVENT_NONE,
      .ext_trap_sync                       = true,
      .ext_trap_exit                       = XMC_CCU4_SLICE_TRAP_EXIT_MODE_AUTOMATIC,

      .mcm_shadow_txfr_mode                = XMC_CCU4_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE0,
      
#if (UC_SERIES == XMC14)
      .shadow_transfer_mode                = XMC_CCU4_SLICE_SHADOW_TRANSFER_MODE_ONLY_IN_PERIOD_MATCH,

      .immediate_write                     = (((0U))),

      .automatic_shadow_transfer           = (((0U))),

      .cascaded_shadow_txfr_enable         = false,
#endif

      .ccu4_cc4_slice_timer_ptr            = &PWM_OUTPUT_4_timer_handle,

      .gpio_ch_out_enable                  = true,
      .gpio_ch_out_ptr                     = (XMC_GPIO_PORT_t *) PORT1_BASE,
      .gpio_ch_out_pin                     = 3U,
      .gpio_ch_out_config_ptr              = &PWM_OUTPUT_4_ch1_out0_config,    

      .global_ccu4_handle                   = (GLOBAL_CCU4_t*) &GLOBAL_CCU4_2,
    };

    PWM_CCU4_t PWM_OUTPUT_4 =
    {
      .config_ptr                          = &PWM_OUTPUT_4_config_handle,
      .ccu4_module_ptr                     = (XMC_CCU4_MODULE_t*) CCU40_BASE,
      .ccu4_slice_ptr                      = (XMC_CCU4_SLICE_t*) CCU40_CC40,
      .slice_number                        = 0U,
      .kernel_number                       = 0U,
      .shadow_txfr_msk                     = (uint32_t)XMC_CCU4_SHADOW_TRANSFER_SLICE_0,
      .dither_shadow_txfr_msk              = (uint32_t)XMC_CCU4_SHADOW_TRANSFER_DITHER_SLICE_0,
      .prescaler_shadow_txfr_msk           = (uint32_t)XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_0,

      .state                               = PWM_CCU4_STATE_UNINITIALIZED,
      .sym_duty                            = 500U,

    };

/********************************************************************************************************/


