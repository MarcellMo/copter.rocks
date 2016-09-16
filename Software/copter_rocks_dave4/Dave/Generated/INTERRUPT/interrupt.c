/**
 * @file interrupt.c
 * @date 2015-09-18
 *
 * NOTE:
 * This file is generated by DAVE. Any manual modification done to this file will be lost when the code is regenerated.
 */
/**
 * @cond
 ***********************************************************************************************************************
 * INTERRUPT v4.0.8 Helps the user to overwrite the provided ISR in system file
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
 * 2015-07-30:
 *     - Added support for XMC1400 devices
 * 2015-09-18:
 *     - Removed config structure access
 * @endcond
 *
 */


/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/

#include "interrupt.h"

/***********************************************************************************************************************
	* MACROS
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * LOCAL DATA
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * LOCAL ROUTINES
 **********************************************************************************************************************/

/**********************************************************************************************************************
* API IMPLEMENTATION
**********************************************************************************************************************/
/*
 * API to retrieve the version of the INTERRUPT APP
 */
DAVE_APP_VERSION_t INTERRUPT_GetAppVersion(void)
{
  DAVE_APP_VERSION_t version;

  version.major = INTERRUPT_MAJOR_VERSION;
  version.minor = INTERRUPT_MINOR_VERSION;
  version.patch = INTERRUPT_PATCH_VERSION;

  return (version);
}

/*
 * API to initialize the INTERRUPT APP
 */
INTERRUPT_STATUS_t INTERRUPT_Init(const INTERRUPT_t *const handler)
{
  XMC_ASSERT("INTERRUPT_Init:HandlePtr NULL", (handler != NULL));
  
#if(UC_FAMILY == XMC4)

  NVIC_SetPriority(handler->node,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(),
                                       handler->priority,
                                       handler->subpriority));
  if (handler->enable_at_init == true)
  {
    INTERRUPT_Enable(handler);
  }
#endif

#if(UC_FAMILY == XMC1)
  NVIC_SetPriority(handler->node, handler->priority);
  
#if (UC_SERIES == XMC14)
  XMC_SCU_SetInterruptControl((uint8_t)handler->node, (XMC_SCU_IRQCTRL_t)((handler->node << 8) | handler->irqctrl));
#endif

  /* Enable the interrupt if enable_at_init is enabled */
  if (handler->enable_at_init == true)
  {
    INTERRUPT_Enable(handler);
  }
#endif

  return (INTERRUPT_STATUS_SUCCESS);
}
