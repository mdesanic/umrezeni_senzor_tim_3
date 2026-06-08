/*******************************************************************************
* File Name: main.c
*
* Description:
*   DPS310 pressure/temperature + BMI160 motion sensing.
*   Debug output ide preko UART-a bez DMA-a.
*
* PuTTY setup:
*   Connection type: Serial
*   Serial line: COMx
*   Speed: 115200
*
*   Connection -> Serial:
*   Data bits: 8
*   Stop bits: 1
*   Parity: None
*   Flow control: None
*
* Napomena:
*   COMx se provjeri u Windows Device Manageru:
*   Device Manager -> Ports (COM & LPT)
*******************************************************************************/

#include "cy_pdl.h"
#include "cy_result.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cyhal_system.h"

#include "xensiv_dps3xx_mtb.h"
#include "xensiv_dps3xx.h"
#include "mtb_bmi160.h"
#include "cy8ckit_028_sense_pins.h"

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>

#define OVERSAMPLING            7
#define I2C_MASTER_FREQUENCY    400000

#define R                       287.05f
#define G                       9.80665f
#define REFERENCE_SAMPLING      100

/* Motion sensing */
#define ACC_SCALE               (1.0f / 16384.0f * G)
#define CALIB_SAMPLES           64
#define ZERO_THRESHOLD          0.08f
#define DT_S                    1.0f

#define IMU_SPI_FREQUENCY       1000000u

#define STILL_COUNT_MAX         3
#define VEL_DAMPING             0.95f

/*******************************************************************************
* Global Variables
*******************************************************************************/

xensiv_dps3xx_t dps310_sensor;
cyhal_i2c_t I2Cm_HW;
cyhal_spi_t spi_imu;
mtb_bmi160_t bmi160_sensor;

cyhal_i2c_cfg_t i2c_cfg_master = {
    CYHAL_I2C_MODE_MASTER,
    0,
    I2C_MASTER_FREQUENCY
};

static float s_bias_x = 0.0f;
static float s_bias_y = 0.0f;
static float s_bias_z = 0.0f;

static float s_vel_x = 0.0f;
static float s_vel_y = 0.0f;
static float s_vel_z = 0.0f;

static int s_still_count = 0;

/*******************************************************************************
* UART helpers
*******************************************************************************/

/*
 * UART output ide preko cy_retarget_io_init().
 * Zbog toga koristimo printf/putchar umjesto dodatnog cyhal_uart_init().
 */
static void uart_send_text(const char *text)
{
    printf("%s", text);
}

static void uart_send_data(const uint8_t *data, size_t length)
{
    for (size_t i = 0; i < length; i++)
    {
        putchar((int)data[i]);
    }
}

/*******************************************************************************
* Motion sensing
*******************************************************************************/

static int motion_sensing_init(void)
{
    cy_rslt_t result = cyhal_spi_init(&spi_imu,
                                      CY8CKIT_028_SENSE_PIN_SPI_MOSI,
                                      CY8CKIT_028_SENSE_PIN_SPI_MISO,
                                      CY8CKIT_028_SENSE_PIN_SPI_SCK,
                                      NC,
                                      NULL,
                                      8,
                                      CYHAL_SPI_MODE_11_MSB,
                                      false);

    if (result != CY_RSLT_SUCCESS)
    {
        printf("spi_init failed -> 0x%08lX\r\n", (unsigned long)result);
        return -1;
    }

    cyhal_spi_set_frequency(&spi_imu, IMU_SPI_FREQUENCY);
    cyhal_system_delay_ms(10);

    cyhal_gpio_init(CY8CKIT_028_SENSE_PIN_SPI_CS,
                    CYHAL_GPIO_DIR_OUTPUT,
                    CYHAL_GPIO_DRIVE_STRONG,
                    1);

    result = mtb_bmi160_init_spi(&bmi160_sensor,
                                 &spi_imu,
                                 CY8CKIT_028_SENSE_PIN_SPI_CS);

    if (result != CY_RSLT_SUCCESS)
    {
        printf("bmi160_init_spi failed -> 0x%08lX\r\n", (unsigned long)result);
        return -1;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;

    mtb_bmi160_data_t sample;

    for (int i = 0; i < CALIB_SAMPLES; i++)
    {
        if (mtb_bmi160_read(&bmi160_sensor, &sample) != CY_RSLT_SUCCESS)
        {
            return -1;
        }

        sum_x += (double)sample.accel.x * ACC_SCALE;
        sum_y += (double)sample.accel.y * ACC_SCALE;
        sum_z += (double)sample.accel.z * ACC_SCALE;

        cyhal_system_delay_ms(5);
    }

    s_bias_x = (float)(sum_x / CALIB_SAMPLES);
    s_bias_y = (float)(sum_y / CALIB_SAMPLES);
    s_bias_z = (float)(sum_z / CALIB_SAMPLES);

    s_vel_x = 0.0f;
    s_vel_y = 0.0f;
    s_vel_z = 0.0f;
    s_still_count = 0;

    return 0;
}

static inline void motion_sensing_reset_velocity(void)
{
    s_vel_x = 0.0f;
    s_vel_y = 0.0f;
    s_vel_z = 0.0f;
    s_still_count = 0;
}

static int motion_sensing_get_displacement(float dt_s, float *position)
{
    if (position == NULL)
    {
        return -3;
    }

    mtb_bmi160_data_t sample;

    if (mtb_bmi160_read(&bmi160_sensor, &sample) != CY_RSLT_SUCCESS)
    {
        return -2;
    }

    float ax = (float)sample.accel.x * ACC_SCALE - s_bias_x;
    float ay = (float)sample.accel.y * ACC_SCALE - s_bias_y;
    float az = (float)sample.accel.z * ACC_SCALE - s_bias_z;

    printf("raw a: %.3f %.3f %.3f\r\n", ax, ay, az);

    if (ax > -ZERO_THRESHOLD && ax < ZERO_THRESHOLD)
    {
        ax = 0.0f;
    }

    if (ay > -ZERO_THRESHOLD && ay < ZERO_THRESHOLD)
    {
        ay = 0.0f;
    }

    if (az > -ZERO_THRESHOLD && az < ZERO_THRESHOLD)
    {
        az = 0.0f;
    }

    if (ax == 0.0f && ay == 0.0f && az == 0.0f)
    {
        s_still_count++;

        if (s_still_count >= STILL_COUNT_MAX)
        {
            s_vel_x = 0.0f;
            s_vel_y = 0.0f;
            s_vel_z = 0.0f;
        }
    }
    else
    {
        s_still_count = 0;
    }

    s_vel_x = (s_vel_x + ax * dt_s) * VEL_DAMPING;
    s_vel_y = (s_vel_y + ay * dt_s) * VEL_DAMPING;
    s_vel_z = (s_vel_z + az * dt_s) * VEL_DAMPING;

    position[0] = s_vel_x * dt_s + 0.5f * ax * dt_s * dt_s;
    position[1] = s_vel_y * dt_s + 0.5f * ay * dt_s * dt_s;
    position[2] = s_vel_z * dt_s + 0.5f * az * dt_s * dt_s;

    return 0;
}

/*******************************************************************************
* Main
*******************************************************************************/

int main(void)
{
    cy_rslt_t result;
    uint32_t revisionID = 0;

    float pressure = 0.0f;
    float temperature = 0.0f;
    float position[3] = {0.0f, 0.0f, 0.0f};

    result = cybsp_init();

    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX,
                                 CYBSP_DEBUG_UART_RX,
                                 CY_RETARGET_IO_BAUDRATE);

    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    printf("\x1b[2J\x1b[;H\r");
    printf("=========================================================\r\n");
    printf("  PSoC 6: DPS310 + BMI160 + UART output without DMA\r\n");
    printf("=========================================================\r\n\r\n");

    uart_send_text("UART started without DMA.\r\n");

    uint8_t test_data[] = {
        'D', 'A', 'T', 'A', ':', ' ',
        '1', '2', '3', '4',
        '\r', '\n'
    };

    uart_send_data(test_data, sizeof(test_data));

    result = cyhal_i2c_init(&I2Cm_HW, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);

    if (result != CY_RSLT_SUCCESS)
    {
        printf("I2C initialization failed\r\n");
        CY_ASSERT(0);
    }

    result = cyhal_i2c_configure(&I2Cm_HW, &i2c_cfg_master);

    if (result != CY_RSLT_SUCCESS)
    {
        printf("Failed to configure I2C\r\n");
        CY_ASSERT(0);
    }

    result = xensiv_dps3xx_mtb_init_i2c(&dps310_sensor,
                                        &I2Cm_HW,
                                        XENSIV_DPS3XX_I2C_ADDR_DEFAULT);

    if (result != CY_RSLT_SUCCESS)
    {
        printf("Failed to initialize DPS310 I2C\r\n");
        CY_ASSERT(0);
    }

    if (xensiv_dps3xx_get_revision_id(&dps310_sensor, (uint8_t *)&revisionID) == CY_RSLT_SUCCESS)
    {
        printf("DPS310 Revision ID = %d\r\n\r\n", (uint8_t)revisionID);
    }
    else
    {
        printf("Failed to get DPS310 Revision ID\r\n");
        CY_ASSERT(0);
    }


    printf("Calibrating BMI160, keep sensor stationary...\r\n");

    if (motion_sensing_init() != 0)
    {
        printf("BMI160 init failed\r\n");
        CY_ASSERT(0);
    }

    printf("BMI160 calibration done.\r\n\r\n");

    for (int i = 0; i < 10; i++)
    {
        float p_tmp;
        float t_tmp;

        xensiv_dps3xx_read(&dps310_sensor, &p_tmp, &t_tmp);
        cyhal_system_delay_ms(100);
    }

    float p0_sum = 0.0f;
    float t0_sum = 0.0f;
    int valid = 0;

    while (valid < REFERENCE_SAMPLING)
    {
        float p_tmp;
        float t_tmp;

        if (xensiv_dps3xx_read(&dps310_sensor, &p_tmp, &t_tmp) == CY_RSLT_SUCCESS)
        {
            p0_sum += p_tmp;
            t0_sum += t_tmp;
            valid++;
        }

        cyhal_system_delay_ms(50);
    }

    float p0 = p0_sum / REFERENCE_SAMPLING;
    float t0 = (t0_sum / REFERENCE_SAMPLING) + 273.15f;

    printf("Reference pressure: %.2f mBar\r\n", p0);
    printf("Reference temperature: %.2f K\r\n\r\n", t0);

    for (;;)
    {
        float height = 0.0f;

        if (xensiv_dps3xx_read(&dps310_sensor, &pressure, &temperature) == CY_RSLT_SUCCESS)
        {
            printf("Pressure: %.2f mBar\tTemperature: %.2f C\r\n",
                   pressure,
                   temperature);

            float temperature_k = temperature + 273.15f;
            height = (R / G) * ((temperature_k + t0) / 2.0f) * logf(p0 / pressure);

            printf("Height: %.2f m\r\n", height);
        }
        else
        {
            printf("Failed to read temperature and pressure data.\r\n");
            cyhal_system_delay_ms(500);
            continue;
        }

        int err = motion_sensing_get_displacement(DT_S, position);

        if (err == 0)
        {
            printf("dx: %.4f m  dy: %.4f m  dz: %.4f m\r\n",
                   position[0],
                   position[1],
                   position[2]);

            printf("%.4f,%.4f,%.4f\n",
                   position[0],
                   position[1],
                   height);
        }

        else
        {
            printf("BMI160 read failed, err=%d\r\n", err);
            motion_sensing_reset_velocity();
        }

        printf("---------------------------------------------------------\r\n");

        cyhal_system_delay_ms((uint32_t)(DT_S * 1000.0f));
    }
}

/* [] END OF FILE */