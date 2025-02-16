#include "drive_main.h"

#include "adc.h"
#include "tim.h"

#include "interface_uart.h"
#include "drv8323.h"
#include "foc.h"
#include "motor.h"
#include "encoder.h"
#include "open_loop_controller.h"

volatile bool counting_down_ = false;

uint32_t adc_voltage_temp[2];

OpenLoopController_t open_loop_controller = {.max_current_ramp_   = 1.0f,
                                             .max_voltage_ramp_   = 2.0f,
                                             .max_phase_vel_ramp_ = 200.0f,
                                             .target_vel_         = 62.8f,
                                             .target_current_     = 0.0f,
                                             .target_voltage_     = 0.5f,
                                             .phase_              = 0.0f,
                                             .total_distance_     = 0.0f};

void drive_init(void)
{
    usart_intrface_init();
    encoder_setup();
    /* ADC校准 */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    /* */
    HAL_ADC_Start_DMA(&hadc1, &adc_voltage_temp[0], 1);
    HAL_ADC_Start_DMA(&hadc2, &adc_voltage_temp[1], 1);
    /* drv8323自校准 */
    drv8323_init();
    /* 开启注入组触发中断 */
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    /* PWM 初始化 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    /* 开启定时器 */
    HAL_TIM_Base_Start_IT(&htim1);
}

bool fetch_and_reset_adcs(Iph_ABC_t *current)
{
    bool all_adcs_done = (ADC1->ISR & ADC_ISR_JEOC) == ADC_ISR_JEOC;
    if (!all_adcs_done) { return false; }

    vbus_sense_adc_cb(adc_voltage_temp[0]);

    current->phA = phase_current_from_adcval(ADC1->JDR1);
    current->phB = phase_current_from_adcval(ADC1->JDR2);
    current->phC = phase_current_from_adcval(ADC1->JDR3);

    ADC1->ISR = ~(ADC_ISR_JEOC);
    return true;
}

void TIM1_UP_TIM16_IRQHandler(void)
{
    /* clear tim1 update event isr flag */
    /* 清除定时器1更新事件中断标志 */
    TIM1->SR = ~TIM_IT_UPDATE;
    /* Timer 1 counts the direction, if counting_down = 0, count up mode;
    When PWM_MODE_2 mode is set, PWM1 = 0 and PWM1N = 1 */
    /* 定时器1计数方向,如果 counting_down = 0，向上计数模式；
    当设定PWM_MODE_2模式时，PWM1 = 0，PWM1N = 1 */
    bool counting_down = TIM1->CR1 & TIM_CR1_DIR;
    /* Check whether the current count direction is different
     from the previously saved count direction */
    /* 判断当前计数方向与之前保存的计数方向是否不同 */
    bool timer_update_missed = (counting_down_ == counting_down);
    if (timer_update_missed) {
        set_error(ERROR_TIMER_UPDATE_MISSED);
        TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM_1_PERIOD_CLOCKS / 2;
        return;
    }
    /* Update the saved count direction */
    /* 更新保存的计数方向 */
    counting_down_ = counting_down;

    Iph_ABC_t current_;

    if (!counting_down) {
        /* PWM1N = 1, 测量相电流 */
        encoder_sample_now();

        if (!fetch_and_reset_adcs(&current_)) {
            //
            disarm_with_error(ERROR_BAD_TIMING);
        }

        if (!(TIM1->BDTR & TIM_BDTR_MOE_Msk)) { current_.phA = current_.phB = current_.phC = TIM_1_PERIOD_CLOCKS / 2; }

        current_meas_cb(&current_);

        encoder_update();

        open_loop_controller_update(&open_loop_controller);

    } else {
        /* PWM1N = 0, 校准电流 */

        for (char i = 0; i < 3; i++) { ; }

        if (!fetch_and_reset_adcs(&current_)) {
            //
            disarm_with_error(ERROR_BAD_TIMING);
        }

        dc_calib_cb(&current_);

        FOC_voltage(open_loop_controller.Vdq_setpoint_[0], 0.0f,
                    open_loop_controller.phase_ + 1.5f * current_meas_period);
    }
}
