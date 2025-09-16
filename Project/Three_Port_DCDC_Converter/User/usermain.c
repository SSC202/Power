#include "usermain.h"

/**
 * @brief   串口重定向函数
 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}

/**********************************************
 * @brief   用户自定义变量
 */
uint8_t system_sample_flag = 0;         // 采样标志
float system_sample_time   = 0.000025f; // 采样时间
uint8_t system_print       = 0;         // 用户打印标志

// MPPT Parameters

uint8_t system_mppt_enable; // MPPT 主电路使能
float R_s  = 9.6f;          // 电源内阻
float u_in = 0;             // 输入电压
float i_in = 0;             // 输入电流
LPF_t i_in_filter;          // 输入电流滤波器
LPF_t u_in_filter;          // 输入电压滤波器
PID_t mppt_u_controller;    // MPPT 电压控制器
PID_t mppt_i_controller;    // MPPT 电流控制器
float mppt_duty;            // MPPT-Boost 输出占空比

// Charge Parameters

uint8_t system_charge_enable; // 充电电路使能
float u_bref = 30;            // 中心点电压预设值
float u_b;                    // 中心点电压
LPF_t u_b_filter;             // 中心点电压滤波器
float i_b;                    // 中心点电流
PID_t charge_u_controller;    // 充放电电路电压控制器
float charge_duty;            // 充放电电压输出占空比

/**********************************************
 * @brief   用户自定义临时变量
 */

/**
 * @brief System init
 */
static void init()
{
    // System init
    system_charge_enable = 0;
    system_mppt_enable   = 0;
    // System parameter init
    PID_init(&charge_u_controller, 0.05, 1, 0, 0.9, 0.1);
    PID_init(&mppt_u_controller, 0.01, 10, 0, INFINITY, -INFINITY);
    PID_init(&mppt_i_controller, 0.01, 0.1, 0, 0.9, 0.1);
    // Sample Start
    LPF_Init(&i_in_filter, 50, system_sample_time);
    LPF_Init(&u_in_filter, 1, system_sample_time);
    LPF_Init(&u_b_filter, 1, system_sample_time);
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    while (system_sample_flag == 0) {
        ;
    }
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void usermain()
{
    init();
    while (1) {
        if (system_print == 0) {
            printf("U:%f\r\n", u_b);
        } else if (system_print == 1) {
            printf("U:%f\r\n", u_in);
        } else if (system_print == 2) {
            printf("U:%f\r\n", i_in);
        }
    }
}

/************************************ MPPT电路控制器 ***************************************** */

/**
 * @brief MPPT 控制函数
 */
static void mppt_control(void)
{
    // MPPT 计算
    // 电压环计算
    mppt_u_controller.ref = u_in;
    mppt_u_controller.fdb = i_in * R_s;
    PID_Calc(&mppt_u_controller, system_mppt_enable, system_sample_time);
    if (system_mppt_enable == 0) {
        ;
    }

    // 电流环计算
    mppt_i_controller.ref = mppt_u_controller.output;
    mppt_i_controller.fdb = i_in;
    PID_Calc(&mppt_i_controller, system_mppt_enable, system_sample_time);
    if (system_mppt_enable == 0) {
        mppt_i_controller.output = 0.5;
    }
    mppt_duty = mppt_i_controller.output;

    // 输出控制
    if (system_mppt_enable == 0) {
        mppt_duty = 0.5;
    } else {
        ;
    }
    TIM1->CCR3 = TIM1->ARR * mppt_duty;
}

/************************************ 充放电电路控制器 ***************************************** */
/**
 * @brief 充放电电路控制函数
 */
static void charge_control(void)
{
    // PID 计算
    charge_u_controller.ref = u_b;
    charge_u_controller.fdb = u_bref;
    PID_Calc(&charge_u_controller, system_charge_enable, system_sample_time);
    if (system_charge_enable == 0) {
        charge_u_controller.output = 0.5;
    }
    charge_duty = charge_u_controller.output;

    // 输出控制
    if (system_charge_enable == 0) {
        charge_duty = 0.5;
    } else {
        ;
    }
    TIM1->CCR2 = TIM1->ARR * charge_duty;
}

/*************************************** 中断主程序 ******************************************* */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    static uint32_t adc_cnt = 0;

    static uint32_t adc_uin_offset_sum = 0;
    static uint32_t adc_uin            = 0;
    static float adc_uin_offset        = 0;
    static float u_in_origin           = 0;

    static uint32_t adc_iin_offset_sum = 0;
    static uint32_t adc_iin            = 0;
    static float adc_iin_offset        = 0;
    static float i_in_origin           = 0;

    static uint32_t adc_ub_offset_sum = 0;
    static uint32_t adc_ub            = 0;
    static float adc_ub_offset        = 0;
    static float u_b_origin           = 0;

    UNUSED(hadc);
    /**********************************
     * @brief   sample
     */
    if (hadc == &hadc1) {
        // Read Offset
        if (system_sample_flag == 0) {
            adc_cnt++;
            if (adc_cnt >= 90001) {
                adc_uin_offset_sum += hadc1.Instance->JDR1;
                adc_iin_offset_sum += hadc1.Instance->JDR2;
                adc_ub_offset_sum += hadc1.Instance->JDR3;
            }
            if (adc_cnt == 100000) {
                adc_cnt            = 0;
                adc_uin_offset     = adc_uin_offset_sum / 10000.0f;
                adc_iin_offset     = adc_iin_offset_sum / 10000.0f;
                adc_ub_offset      = adc_ub_offset_sum / 10000.0f;
                system_sample_flag = 1;
                adc_uin_offset_sum = 0;
                adc_iin_offset_sum = 0;
                adc_ub_offset_sum  = 0;
            }
        }
        // Read Sample
        else {
            adc_uin     = hadc1.Instance->JDR1;
            adc_iin     = hadc1.Instance->JDR2;
            adc_ub      = hadc1.Instance->JDR3;
            u_in_origin = ((((float)adc_uin - adc_uin_offset) / 4096.f) * 3.3f) * 209.f - 0.07f;
            i_in_origin = ((((float)adc_iin - adc_iin_offset) / 4096.f) * 3.3f) / 0.2f;
            u_b_origin  = ((((float)adc_ub - adc_ub_offset) / 4096.f) * 3.3f) * 211.f - 0.02f;

            i_in_filter.input = i_in_origin;
            LPF_Calc(&i_in_filter);
            i_in = i_in_filter.output;

            u_in_filter.input = u_in_origin;
            LPF_Calc(&u_in_filter);
            u_in = u_in_filter.output;

            u_b_filter.input = u_b_origin;
            LPF_Calc(&u_b_filter);
            u_b = u_b_filter.output;
        }
    }
    /**********************************
     * @brief   controller
     */
    mppt_control();
    charge_control();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
    }
}
