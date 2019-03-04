/*
 * FreeModbus Libary: ESP32 Port Demo Application
 * Copyright (C) 2010 Christian Walter <cwalter@embedded-solutions.at>
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * IF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: portother.c,v 1.1 2010/06/06 13:07:20 wolti Exp $
 */
/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
#include "soc/timer_group_struct.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "mbconfig.h"
#include "driver/timer.h"
#include "sdkconfig.h"

#ifdef CONFIG_MB_TIMER_PORT_ENABLED

#define MB_US50_FREQ            (20000) // 20kHz 1/20000 = 50mks
#define MB_DISCR_TIME_US        (50)    // 50uS = one discreet for timer                                    (only used in MB_TIMER_DIVIDER which is not used anyhow)

#define MB_TIMER_PRESCALER      ((TIMER_BASE_CLK / MB_US50_FREQ) - 1);
#define MB_TIMER_SCALE          (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds       (not used)
#define MB_TIMER_DIVIDER        ((TIMER_BASE_CLK / 1000000UL) * MB_DISCR_TIME_US - 1) // divider for 50uS   (not used)
#define MB_TIMER_WITH_RELOAD    (1)

//#define USE_TIMER_SETUP_FROM_MASTER_PORT     // in case of trouble, use different timer set from erfengwe's master port. However I assume both settings should result in the same behavior.
#ifdef USE_TIMER_SETUP_FROM_MASTER_PORT
#define MB_MASTER_TIMER_INTR_SEL      TIMER_INTR_LEVEL                             // Timer level interrupt
#define MB_MASTER_TIMER_GROUP         TIMER_GROUP_0                                // Test on timer group 0
#define MB_MASTER_TIMER_DIVIDER       2                                            // Hardware timer clock divider
#define MB_MASTER_TIMER_INTERVAL0_SEC (0.00005)                                    // [50us interval] test interval for timer 0 [1000usec/1.0msec(0.001),100usec/0.1msec(0.0001),8.5usec/0.0085msec(0.00001)]
#define MB_MASTER_TIMER_CPU_CLOCK     160000000L                                   // CPU Clock 160Mhz
#define MB_MASTER_TIMER_SCALE         (TIMER_BASE_CLK / MB_MASTER_TIMER_DIVIDER)   // used to calculate counter value
#endif

// TODO these statics need to be instance-local, to enable handling multiple buses simultaneously by separate instances
static const USHORT usTimerIndex = CONFIG_MB_TIMER_INDEX; // Modbus Timer index used by stack
static const USHORT usTimerGroupIndex = CONFIG_MB_TIMER_GROUP; // Modbus Timer group index used by stack

static timg_dev_t *MB_TG[2] = {&TIMERG0, &TIMERG1};
static BOOL  MB_master_mode = FALSE;
static const char *TAG    = "MB_TIMERS";


/* ----------------------- Static functions ----------------------------------*/
#ifdef USE_TIMER_SETUP_FROM_MASTER_PORT
static void prvvMasterTIMERExpiredISR(void *param)
{
    TIMERG0.hw_timer[0].update = 1;            // PORTING: not nice.. hardcoded for timer 0 only (value is irrelevant anyhow). Different to slave code: setting the update bit.
    TIMERG0.int_clr_timers.t0 = 1;             // PORTING: not nice.. hardcoded value and for timer 0 only. Same as slave code but hard coded.
    (void)pxMBMasterPortCBTimerExpired();      // PORTING: calling a different callback compared to slave code; however intend to merge these with the original slave callbacks.
    TIMERG0.hw_timer[0].config.alarm_en = 1;   // PORTING: not nice.. hardcoded value and for timer 0 only. Same as slave code but hard coded.
}
#else /*USE_TIMER_SETUP_FROM_MASTER_PORT */
static void IRAM_ATTR vTimerGroupIsr(void *param)
{
    // Retrieve the interrupt status and the counter value
    // from the timer that reported the interrupt
    uint32_t intr_status = MB_TG[usTimerGroupIndex]->int_st_timers.val;
    if (intr_status & BIT(usTimerIndex)) {
        // NOTE: In master code, they triggered a timer update before clearing the interrupt. See if we need this at all.
        if (MB_master_mode == TRUE) {
            MB_TG[usTimerGroupIndex]->hw_timer[usTimerIndex].update = 1; // any value triggers the value update
        }
        MB_TG[usTimerGroupIndex]->int_clr_timers.val |= BIT(usTimerIndex);
        // NOTE: In master code, they use a different callback pointer (pxMBMasterPortCBTimerExpired), but goal is to merge these anyhow so slave/master role are not different here anymore.
        (void)pxMBPortCBTimerExpired(); // Timer callback function
        MB_TG[usTimerGroupIndex]->hw_timer[usTimerIndex].config.alarm_en = TIMER_ALARM_EN;
    }
}
#endif /*USE_TIMER_SETUP_FROM_MASTER_PORT */

static void timer_value_update(USHORT val)
{
#ifdef USE_TIMER_SETUP_FROM_MASTER_PORT
    int timer_group = MB_MASTER_TIMER_GROUP;
    int timer_idx = TIMER_0;

    timer_pause(timer_group, timer_idx);
    timer_set_alarm_value(timer_group, timer_idx, val * MB_MASTER_TIMER_INTERVAL0_SEC * MB_MASTER_TIMER_SCALE);
#else
    timer_pause(usTimerGroupIndex, usTimerIndex);
    timer_set_alarm_value(usTimerGroupIndex, usTimerIndex, (uint32_t)(val));
#endif
}

#endif /* CONFIG_MB_TIMER_PORT_ENABLED */

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortTimersInit(USHORT usTim1Timerout50us, BOOL is_master)
{
#ifdef CONFIG_MB_TIMER_PORT_ENABLED

#ifdef USE_TIMER_SETUP_FROM_MASTER_PORT
    int timer_group = MB_MASTER_TIMER_GROUP;
    int timer_idx = TIMER_0;
#endif
    ESP_LOGI(TAG, "entered xMBPortTimersInit, timeout %d, is_master = %d", usTim1Timerout50us, is_master);

    MB_master_mode = is_master;

    MB_PORT_CHECK((usTim1Timerout50us > 0), FALSE,
            "Modbus timeout discreet is incorrect.");
    esp_err_t xErr;
    timer_config_t config;
    config.alarm_en = TIMER_ALARM_EN;
    config.auto_reload = MB_TIMER_WITH_RELOAD;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
#ifdef USE_TIMER_SETUP_FROM_MASTER_PORT
    config.divider = MB_MASTER_TIMER_DIVIDER;
    config.intr_type = MB_MASTER_TIMER_INTR_SEL;
    timer_init(timer_group, timer_idx, &config);
    timer_pause(timer_group, timer_idx);
    timer_set_alarm_value(timer_group, timer_idx, N50us * MB_MASTER_TIMER_INTERVAL0_SEC * MB_MASTER_TIMER_SCALE);

    timer_enable_intr(timer_group, timer_idx);
    timer_isr_register(timer_group, timer_idx,
                       prvvMasterTIMERExpiredISR, (void *)timer_idx, ESP_INTR_FLAG_LOWMED, NULL);
#else
    config.divider = MB_TIMER_PRESCALER;
    config.intr_type = TIMER_INTR_LEVEL;
    // Configure timer
    xErr = timer_init(usTimerGroupIndex, usTimerIndex, &config);
    MB_PORT_CHECK((xErr == ESP_OK), FALSE,
            "timer init failure, timer_init() returned (0x%x).", (uint32_t)xErr);
    // Stop timer counter
    xErr = timer_pause(usTimerGroupIndex, usTimerIndex);
    MB_PORT_CHECK((xErr == ESP_OK), FALSE,
                    "stop timer failure, timer_pause() returned (0x%x).", (uint32_t)xErr);
    // Reset counter value
    xErr = timer_set_counter_value(usTimerGroupIndex, usTimerIndex, 0x00000000ULL);
    MB_PORT_CHECK((xErr == ESP_OK), FALSE,
                    "timer set value failure, timer_set_counter_value() returned (0x%x).",
                    (uint32_t)xErr);
    // wait3T5_us = 35 * 11 * 100000 / baud; // the 3.5T symbol time for baudrate
    // Set alarm value for usTim1Timerout50us * 50uS
    xErr = timer_set_alarm_value(usTimerGroupIndex, usTimerIndex, (uint32_t)(usTim1Timerout50us));
    MB_PORT_CHECK((xErr == ESP_OK), FALSE,
                    "failure to set alarm failure, timer_set_alarm_value() returned (0x%x).",
                    (uint32_t)xErr);
    // Register ISR for timer
    xErr = timer_isr_register(usTimerGroupIndex, usTimerIndex, vTimerGroupIsr, NULL, ESP_INTR_FLAG_IRAM, NULL);
    MB_PORT_CHECK((xErr == ESP_OK), FALSE,
                    "timer set value failure, timer_isr_register() returned (0x%x).",
                    (uint32_t)xErr);
#endif /* USE_TIMER_SETUP_FROM_MASTER_PORT */

#endif /* CONFIG_MB_TIMER_PORT_ENABLED */
    return TRUE;
}

void vMBPortTimersEnable()
{
#ifdef CONFIG_MB_TIMER_PORT_ENABLED
    //if (!bMBPortIsWithinException()) { ESP_LOGI(TAG, "entered vMBPortTimersEnable"); }
#ifdef USE_TIMER_SETUP_FROM_MASTER_PORT
    /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
    timer_start(MB_MASTER_TIMER_GROUP, TIMER_0);
#else  /* USE_TIMER_SETUP_FROM_MASTER_PORT */
    ESP_ERROR_CHECK(timer_pause(usTimerGroupIndex, usTimerIndex));
    ESP_ERROR_CHECK(timer_set_counter_value(usTimerGroupIndex, usTimerIndex, 0ULL));
    ESP_ERROR_CHECK(timer_enable_intr(usTimerGroupIndex, usTimerIndex));
    ESP_ERROR_CHECK(timer_start(usTimerGroupIndex, usTimerIndex));
#endif /* USE_TIMER_SETUP_FROM_MASTER_PORT */
#endif /* CONFIG_MB_TIMER_PORT_ENABLED */
}

void vMBPortTimersDisable()
{
#ifdef CONFIG_MB_TIMER_PORT_ENABLED
    // log message only if not in interrupt context
    //if (!bMBPortIsWithinException()) { ESP_LOGI(TAG, "entered vMBPortTimersDisable"); }

#ifdef USE_TIMER_SETUP_FROM_MASTER_PORT
    /* Disable any pending timers. */
    timer_pause(MB_MASTER_TIMER_GROUP, TIMER_0);
#else  /* USE_TIMER_SETUP_FROM_MASTER_PORT */
    ESP_ERROR_CHECK(timer_pause(usTimerGroupIndex, usTimerIndex));
    ESP_ERROR_CHECK(timer_set_counter_value(usTimerGroupIndex, usTimerIndex, 0ULL));
    // Disable timer interrupt
    ESP_ERROR_CHECK(timer_disable_intr(usTimerGroupIndex, usTimerIndex));
#endif /* USE_TIMER_SETUP_FROM_MASTER_PORT */
#endif /* CONFIG_MB_TIMER_PORT_ENABLED */
}

void vMBPortTimerClose()
{
#ifdef CONFIG_MB_TIMER_PORT_ENABLED
    ESP_ERROR_CHECK(timer_pause(usTimerGroupIndex, usTimerIndex));
    ESP_ERROR_CHECK(timer_disable_intr(usTimerGroupIndex, usTimerIndex));
#endif /* CONFIG_MB_TIMER_PORT_ENABLED */
}

void vMBMasterPortTimersT35Enable()
{
#ifdef CONFIG_MB_TIMER_PORT_ENABLED
    //ESP_LOGI(TAG, "entered vMBMasterPortTimersT35Enable");
    USHORT timer_tick = 2500;//1000;
    /* Set current timer mode, don't change it.*/
    vMBMasterSetCurTimerMode(MB_TMODE_T35);
    timer_value_update(timer_tick);
    vMBPortTimersEnable();
#endif /* CONFIG_MB_TIMER_PORT_ENABLED */
}

void vMBMasterPortTimersConvertDelayEnable()
{
#ifdef CONFIG_MB_TIMER_PORT_ENABLED
    //ESP_LOGI(TAG, "entered vMBMasterPortTimersConvertDelayEnable");
    USHORT timer_tick = MB_MASTER_DELAY_MS_CONVERT * 20;
    /* Set current timer mode, don't change it.*/
    vMBMasterSetCurTimerMode(MB_TMODE_CONVERT_DELAY);
    timer_value_update(timer_tick);
    vMBPortTimersEnable();
#endif /* CONFIG_MB_TIMER_PORT_ENABLED */
}

void vMBMasterPortTimersRespondTimeoutEnable()
{
#ifdef CONFIG_MB_TIMER_PORT_ENABLED
    //ESP_LOGI(TAG, "entered vMBMasterPortTimersRespondTimeoutEnable");
    USHORT timer_tick = MB_MASTER_TIMEOUT_MS_RESPOND * 20;
    /* Set current timer mode, don't change it.*/
    vMBMasterSetCurTimerMode(MB_TMODE_RESPOND_TIMEOUT);
    timer_value_update(timer_tick);
    vMBPortTimersEnable();
#endif /* CONFIG_MB_TIMER_PORT_ENABLED */
}

