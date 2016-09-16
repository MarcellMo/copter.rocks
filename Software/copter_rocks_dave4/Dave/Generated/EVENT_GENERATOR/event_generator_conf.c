/*********************************************************************************************************************
* DAVE APP Name : EVENT_GENERATOR       APP Version: 4.1.14
*
* NOTE:
* This file is generated by DAVE. Any manual modification done to this file will be lost when the code is regenerated.
*********************************************************************************************************************/

/**
 * @cond
 ***********************************************************************************************************************
 *
 * Copyright (c) 2015, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
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
 * 2015-02-16:
 *     - Initial version<br>
 *
 * 2015-05-08:
 *     - Comments updated<br>
 *
 * @endcond
 *
 */

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "event_generator.h"

/***********************************************************************************************************************
* DATA STRUCTURES
***********************************************************************************************************************/

/**
 * @brief Contents entered via GUI
 */
const XMC_ERU_OGU_CONFIG_t DPS310_ERU_Out_OGU_Config =
{
  .peripheral_trigger         = 0U, /* OGU input peripheral trigger */
  .enable_pattern_detection   = false, /* Enables generation of pattern match event */
  .service_request            = XMC_ERU_OGU_SERVICE_REQUEST_ON_TRIGGER, /* Interrupt gating signal */
  .pattern_detection_input    = 0U
};


EVENT_GENERATOR_t DPS310_ERU_Out = 
{
  .eru         = XMC_ERU1, /* ERU module assigned */    
  .channel     = 3U,    /* ERU channel assigned(0-3) */
  .config      = &DPS310_ERU_Out_OGU_Config, /* reference to hardware configuration */
  .nmi_eru_msk = 0U, /**< Mask to enable the NMI feature */
  .init_status = false /* Initialized status */
};

		
/**
 * @brief Contents entered via GUI
 */
const XMC_ERU_OGU_CONFIG_t MPU9X50_ERU_Out_OGU_Config =
{
  .peripheral_trigger         = 0U, /* OGU input peripheral trigger */
  .enable_pattern_detection   = false, /* Enables generation of pattern match event */
  .service_request            = XMC_ERU_OGU_SERVICE_REQUEST_ON_TRIGGER, /* Interrupt gating signal */
  .pattern_detection_input    = 0U
};


EVENT_GENERATOR_t MPU9X50_ERU_Out = 
{
  .eru         = XMC_ERU1, /* ERU module assigned */    
  .channel     = 0U,    /* ERU channel assigned(0-3) */
  .config      = &MPU9X50_ERU_Out_OGU_Config, /* reference to hardware configuration */
  .nmi_eru_msk = 0U, /**< Mask to enable the NMI feature */
  .init_status = false /* Initialized status */
};

		
