#include "usermain.h"

/**
 * @brief   串口重定向函数
 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}

uint8_t system_sample_flag = 0;
uint8_t system_enable      = 0;
float system_sample_time   = 0.00005f;

float adc_uout_offset = 0;
float u_out           = 0;

PID_t u_pid; // 电压环 PID

float u_out_ref; // 输出电压指令值

static void init()
{
    system_enable = 0;
    // Controller init
    PID_init(&u_pid, 0.07, 0.30, 0, 0.8, 0);
    // DC-DC Start
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    TIM1->CCR2 = 0;
    // Sample
    system_sample_flag = 0;
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
}

void usermain()
{
    init();
    while (1) {
        printf("U:%f\r\n", u_out);
    }
}

static void control(void)
{
    // Voltage PI Controller
    u_pid.fdb = u_out;
    u_pid.ref = u_out_ref;
    PID_Calc(&u_pid, system_enable, system_sample_time);
    TIM1->CCR2 = (u_pid.output) * TIM1->ARR;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    static int adc_cnt                  = 0;
    static uint32_t adc_uout_offset_sum = 0;
    static uint32_t adc_uout            = 0;
    UNUSED(hadc);
    /**********************************
     * @brief   sample
     */
    if (hadc == &hadc1) {
        // Read Offset
        if (system_sample_flag == 0) {
            adc_uout_offset_sum += hadc1.Instance->JDR1;
            adc_cnt++;
            if (adc_cnt == 40) {
                adc_cnt             = 0;
                adc_uout_offset     = adc_uout_offset_sum / 40.0f;
                system_sample_flag  = 1;
                adc_uout_offset_sum = 0;
            }
        }
        // Read Sample
        else {
            adc_uout = hadc1.Instance->JDR1;
            u_out    = ((adc_uout - adc_uout_offset) / 40.96f) + 0.07f;
        }
    }
    /**********************************
     * @brief   control
     */
    control();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
    }
}
