/*******************************************************************************
 * File Name:   main.c
 *
 * Description:
 *  This example demonstrates how to transition PSoC 4 MCU among the following
 *  low power modes: Sleep and Deep Sleep.
 *
 * Related Document: See readme.md
 *
 *******************************************************************************
 * (c) 2020, Cypress Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 * This software, including source code, documentation and related materials
 * ("Software"), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries ("Cypress") and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software ("EULA").
 *
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress's integrated circuit products.
 * Any reproduction, modification, translation, compilation, or representation
 * of this Software except as specified above is prohibited without the express
 * written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 ******************************************************************************/

/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg_pins.h"


/******************************************************************************
 * Macros
 *****************************************************************************/
#define LED_ON                  (1U)
#define LED_OFF                 (0U)
#define SWITCH_INTR_PRIORITY    (3U)

/* Constants to define SHORT and LONG presses on User Button (x10 = ms) */
#define SHORT_PRESS_COUNT       (20U)    /* 200ms < button pressed time < 2 sec */
#define LONG_PRESS_COUNT        (200U)   /* button pressed time > 2 sec */

/* Delays */
#define DELAY                   (10U)
#define BLINK_TIME_MS           (200U)

/* Glitch delays */
#define SHORT_GLITCH_DELAY_MS   (20U)     /* in ms */
#define LONG_GLITCH_DELAY_MS    (250U)    /* in ms */


/******************************************************************************
 * Enumerated data types
 *****************************************************************************/
typedef enum
{
    SWITCH_NO_EVENT     = 0U,
    SWITCH_SHORT_PRESS  = 1U,
    SWITCH_LONG_PRESS   = 2U,
} en_switch_event_t;


/*******************************************************************************
 * UART context structure
 ******************************************************************************/
cy_stc_scb_uart_context_t CYBSP_UART_context;


/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
void switch_isr();
void led_blink(uint32_t blink_time, uint32_t num_toggles);
en_switch_event_t get_switch_event(void);

/* Sleep Callback function */
cy_en_syspm_status_t sleep_callback(cy_stc_syspm_callback_params_t  *callbackParams,
                                    cy_en_syspm_callback_mode_t mode);
/* Deep Sleep Callback function */
cy_en_syspm_status_t deep_sleep_callback(cy_stc_syspm_callback_params_t *callbackParams,
                                         cy_en_syspm_callback_mode_t mode);


/*******************************************************************************
 * Switch interrupt configuration structure
*******************************************************************************/
const cy_stc_sysint_t switch_intr_config = {
        .intrSrc = CYBSP_USER_BTN_IRQ,          /* Source of interrupt signal */
        .intrPriority = SWITCH_INTR_PRIORITY    /* Interrupt priority */
};


/*******************************************************************************
 * Function Name: main
 *******************************************************************************
 *
 * Summary:
 *  System entrance point. This function configures and initializes the GPIO
 *  interrupt, UART Component and Register callback functions.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 ******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Configure and enable the UART peripheral */
    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &CYBSP_UART_context);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);

    /* Send a string over serial terminal */
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\x1b[2J\x1b[;H");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n**************CE231725-PSoC 4:Power Modes**************\r\n");

    /* Initialize and enable GPIO interrupt */
    result = Cy_SysInt_Init(&switch_intr_config, switch_isr);
    if(result != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Clear Pending Interrupt */
    NVIC_ClearPendingIRQ(switch_intr_config.intrSrc);

    /* Enables interrupt in the NVIC interrupt controller */
    NVIC_EnableIRQ(switch_intr_config.intrSrc);

    /* SysPm callback params */
    cy_stc_syspm_callback_params_t callbackParams = {
            /*.base       =*/ NULL,
            /*.context    =*/ NULL
    };

    /* Callback declaration for Sleep mode */
    cy_stc_syspm_callback_t sleep_cb    =  {sleep_callback,              /* Callback function */
                                           CY_SYSPM_SLEEP,               /* Callback type */
                                           0,                            /* Skip mode */
                                           &callbackParams,              /* Callback params */
                                           NULL, NULL};                  /* For internal usage */

    /* Callback declaration for Deep Sleep mode */
    cy_stc_syspm_callback_t deep_sleep_cb = {deep_sleep_callback,        /* Callback function */
                                           CY_SYSPM_DEEPSLEEP,           /* Callback type */
                                           0,                            /* Skip mode */
                                           &callbackParams,              /* Callback params */
                                           NULL, NULL};                  /* For internal usage */

    /* Register Sleep callback */
    Cy_SysPm_RegisterCallback(&sleep_cb);

    /* Register Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&deep_sleep_cb);

    for (;;)
    {
        /* Turn ON LED */
        Cy_GPIO_Write(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_NUM, LED_ON);

        switch (get_switch_event())
        {
            case SWITCH_SHORT_PRESS:

                /* Send a string over serial terminal */
                Cy_SCB_UART_PutString(CYBSP_UART_HW, "Enter Sleep mode\r\n");
                Cy_SysLib_Delay(SHORT_GLITCH_DELAY_MS);

                /* Go to Sleep */
                Cy_SysPm_CpuEnterSleep();
                /* Wait a bit to avoid glitches in the button press */
                Cy_SysLib_Delay(LONG_GLITCH_DELAY_MS);
                break;

            case SWITCH_LONG_PRESS:

                /* Send a string over serial terminal */
                Cy_SCB_UART_PutString(CYBSP_UART_HW, "Enter Deep Sleep mode\r\n");
                Cy_SysLib_Delay(SHORT_GLITCH_DELAY_MS);

                /* Go to Deep Sleep */
                Cy_SysPm_CpuEnterDeepSleep();
                /* Wait a bit to avoid glitches in the button press */
                Cy_SysLib_Delay(LONG_GLITCH_DELAY_MS);
                break;

            default:
                break;
        }
    }
}


/*******************************************************************************
 * Function Name: switch_isr
 *******************************************************************************
 *
 * Summary:
 *  This function is executed when GPIO interrupt is triggered.
 *  It Clears the triggered pin interrupt and Clears Pending Interrupt.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void switch_isr()
{
    /* Clears the triggered pin interrupt */
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_NUM);
    /* Clear Pending Interrupt */
    NVIC_ClearPendingIRQ(switch_intr_config.intrSrc);
}


/*******************************************************************************
 * Function Name: get_switch_event
 *******************************************************************************
 * Summary:
 *  Returns how the User button was pressed:
 *  - SWITCH_NO_EVENT: No press
 *  - SWITCH_SHORT_PRESS: Short press was detected
 *  - SWITCH_LONG_PRESS: Long press was detected
 *
 * Parameters:
 *  void
 *
 * Return:
 *  Switch event that occurred, if any.
 *
 ******************************************************************************/
en_switch_event_t get_switch_event(void)
{
    en_switch_event_t event = SWITCH_NO_EVENT;
    uint32_t press_count = 0;

    /* Check if User button is pressed */
    while (Cy_GPIO_Read(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_NUM) == CYBSP_BTN_PRESSED)
    {
        /* Wait for 10 ms */
        Cy_SysLib_Delay(DELAY);

        /* Increment counter. Each count represents 10 ms */
        press_count++;
    }

    /* Check for how long the button was pressed */
    if (press_count > LONG_PRESS_COUNT)
    {
        event = SWITCH_LONG_PRESS;
    }
    else if (press_count > SHORT_PRESS_COUNT)
    {
        event = SWITCH_SHORT_PRESS;
    }

    /* Debounce the USER button */
    Cy_SysLib_Delay(SHORT_GLITCH_DELAY_MS);

    return event;
}


/*******************************************************************************
 * Function Name: sleep_callback
 *******************************************************************************
 *
 * Sleep callback implementation. It turns the LED off before going to
 * sleep power mode. After waking up, it sets the LED to blink for two times.
 *
 * Parameters:
 *  callbackParams: The pointer to the callback parameters structure
 *  cy_stc_syspm_callback_params_t.
 *  mode: Callback mode, see cy_en_syspm_callback_mode_t
 *
 * Return:
 *  Entered status, see cy_en_syspm_status_t.
 *
 ******************************************************************************/
cy_en_syspm_status_t sleep_callback(
        cy_stc_syspm_callback_params_t *callbackParams, cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t ret_val = CY_SYSPM_FAIL;

    switch (mode)
    {
        case CY_SYSPM_CHECK_READY:

            while(Cy_SCB_UART_IsTxComplete(CYBSP_UART_HW) == 0u)
                {
                    /* Wait until the TX FIFO
                     * and Shifter are empty and there is no more data to send. */
                }

            /* Disable the UART */
            Cy_SCB_UART_Disable(CYBSP_UART_HW, &CYBSP_UART_context);

            ret_val = CY_SYSPM_SUCCESS;
            break;

        case CY_SYSPM_CHECK_FAIL:

            /* Enable the UART */
            Cy_SCB_UART_Enable(CYBSP_UART_HW);
            /* Send a string over serial terminal */
            Cy_SCB_UART_PutString(CYBSP_UART_HW, "Device failed to enter Sleep mode\r\n");

            ret_val = CY_SYSPM_SUCCESS;
            break;

        case CY_SYSPM_BEFORE_TRANSITION:

            /* Blink the LED for two times before entering into Sleep mode */
            led_blink(BLINK_TIME_MS, 2);

            ret_val = CY_SYSPM_SUCCESS;
            break;

        case CY_SYSPM_AFTER_TRANSITION:

            /* Enable the UART */
            Cy_SCB_UART_Enable(CYBSP_UART_HW);
            /* Send a string over serial terminal */
            Cy_SCB_UART_PutString(CYBSP_UART_HW, "Wake-up from Sleep mode and enters Active mode\r\n");

            ret_val = CY_SYSPM_SUCCESS;
            break;

        default:
            /* Don't do anything in the other modes */
            ret_val = CY_SYSPM_SUCCESS;
            break;
    }
        return ret_val;
}


/*******************************************************************************
 * Function Name: deep_sleep_callback
 *******************************************************************************
 *
 * Summary:
 *  Deep Sleep callback implementation. It turns the LED off before going to deep
 *  sleep power mode. After waking up, it sets the LED to blink for three times.
 *
 * Parameters:
 *  callbackParams: The pointer to the callback parameters structure cy_stc_syspm_callback_params_t.
 *  mode: Callback mode, see cy_en_syspm_callback_mode_t
 *
 * Return:
 *  Entered status, see cy_en_syspm_status_t.
 *
 ******************************************************************************/
cy_en_syspm_status_t deep_sleep_callback(
        cy_stc_syspm_callback_params_t *callbackParams, cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t ret_val = CY_SYSPM_FAIL;

    switch (mode)
    {
        case CY_SYSPM_CHECK_READY:

            while(Cy_SCB_UART_IsTxComplete(CYBSP_UART_HW) == 0u)
                {
                    /* Wait until the TX FIFO
                     * and Shifter are empty and there is no more data to send. */
                }

            /* Disable the UART */
            Cy_SCB_UART_Disable(CYBSP_UART_HW, &CYBSP_UART_context);

            ret_val = CY_SYSPM_SUCCESS;
            break;

        case CY_SYSPM_CHECK_FAIL:

            /* Enable the UART */
            Cy_SCB_UART_Enable(CYBSP_UART_HW);
            /* Send a string over serial terminal */
            Cy_SCB_UART_PutString(CYBSP_UART_HW, "Device failed to enter Deep Sleep mode\r\n");

            ret_val = CY_SYSPM_SUCCESS;
            break;

        case CY_SYSPM_BEFORE_TRANSITION:

            /* Blink the LED for three times before entering into Deep Sleep mode */
            led_blink(BLINK_TIME_MS, 3);

            ret_val = CY_SYSPM_SUCCESS;
            break;

        case CY_SYSPM_AFTER_TRANSITION:

            /* Enable the UART */
            Cy_SCB_UART_Enable(CYBSP_UART_HW);
            /* Send a string over serial terminal */
            Cy_SCB_UART_PutString(CYBSP_UART_HW, "Wake-up from Deep Sleep mode and enters Active mode\r\n");

            ret_val = CY_SYSPM_SUCCESS;
            break;

        default:
            /* Don't do anything in the other modes */
            ret_val = CY_SYSPM_SUCCESS;
            break;
    }
    return ret_val;
}


/*******************************************************************************
 * Function Name: led_blink
 *******************************************************************************
 * Summary:
 * Blinks the LED for num_toggles times, with a period of blink_time.
 *
 * Parameters:
 * blink_time:  Time in ms for on and off times.
 * num_toggles: Describes how many times to toggle the LED on and off.
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void led_blink(uint32_t blink_time, uint32_t num_toggles)
{
    /* variable used to set LED blink time */
    uint8_t count = 0U;

    /* Turn OFF LED */
    Cy_GPIO_Write(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_NUM, LED_OFF);
    Cy_SysLib_Delay(blink_time);

    /* Toggle the LED the desired number of times in this loop */
    for (count = 0; count < num_toggles; count++)
    {
        Cy_GPIO_Write(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_NUM, LED_ON);
        Cy_SysLib_Delay(blink_time);
        Cy_GPIO_Write(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_NUM, LED_OFF);
        Cy_SysLib_Delay(blink_time);
    }
}

/* [] END OF FILE */
