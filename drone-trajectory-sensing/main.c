#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "xensiv_dps3xx_mtb.h"
#include "xensiv_dps3xx.h"
#include "mtb_bmi160.h"
#include "math.h"

#define OVERSAMPLING            7
#define I2C_MASTER_FREQUENCY    400000

#define R 287.05f
#define G 9.80665f
#define REFERENCE_SAMPELING 20

/* Motion sensing */
#define ACC_SCALE      (1.0f / 16384.0f * G)
#define CALIB_SAMPLES   64
#define ZERO_THRESHOLD  0.05f
#define DT_S            0.1f

/*******************************************************************************
* Global Variables
********************************************************************************/
/* Context for dps310 */
xensiv_dps3xx_t dps310_sensor;

/* Declaration for i2c handler */
cyhal_i2c_t I2Cm_HW;

/* Define the I2C master configuration structure */
cyhal_i2c_cfg_t i2c_cfg_master = {
        CYHAL_I2C_MODE_MASTER,
        0,                          /* address is not used for master mode */
        I2C_MASTER_FREQUENCY
};

// -----------------------------------------------------------------------------------------------------------
// MOTION

mtb_bmi160_t    bmi160_sensor;

static float s_bias_x = 0.0f;
static float s_bias_y = 0.0f;
static float s_bias_z = 0.0f;

static float s_vel_x  = 0.0f;
static float s_vel_y  = 0.0f;
static float s_vel_z  = 0.0f;

static int motion_sensing_init(cyhal_i2c_t *i2c_ptr)
{
    if (i2c_ptr == NULL)
        return -3;
 
    cy_rslt_t result = mtb_bmi160_init_i2c(&bmi160_sensor, i2c_ptr, MTB_BMI160_DEFAULT_ADDRESS);
    if (result != CY_RSLT_SUCCESS)
        return -1;
 
    /* Calibration — keep sensor stationary during this loop (~0.4 s) */
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    mtb_bmi160_data_t sample;
 
    for (int i = 0; i < CALIB_SAMPLES; i++)
    {
        if (mtb_bmi160_read(&bmi160_sensor, &sample) != CY_RSLT_SUCCESS)
            return -1;
 
        sum_x += (double)sample.accel.x * ACC_SCALE;
        sum_y += (double)sample.accel.y * ACC_SCALE;
        sum_z += (double)sample.accel.z * ACC_SCALE;
 
        cyhal_system_delay_ms(5);
    }
 
    s_bias_x = (float)(sum_x / CALIB_SAMPLES);
    s_bias_y = (float)(sum_y / CALIB_SAMPLES);
    s_bias_z = (float)(sum_z / CALIB_SAMPLES) - G;
 
    s_vel_x = s_vel_y = s_vel_z = 0.0f;
    return 0;
}


static inline void motion_sensing_reset_velocity(void)
{
    s_vel_x = s_vel_y = s_vel_z = 0.0f;
}

static int motion_sensing_get_displacement(float dt_s, float *position)
{
    if (position == NULL)
        return -3;
 
    mtb_bmi160_data_t sample;
    if (mtb_bmi160_read(&bmi160_sensor, &sample) != CY_RSLT_SUCCESS)
        return -2;
 
    float ax = (float)sample.accel.x * ACC_SCALE - s_bias_x;
    float ay = (float)sample.accel.y * ACC_SCALE - s_bias_y;
    float az = (float)sample.accel.z * ACC_SCALE - s_bias_z;
 
    if (ax > -ZERO_THRESHOLD && ax < ZERO_THRESHOLD) ax = 0.0f;
    if (ay > -ZERO_THRESHOLD && ay < ZERO_THRESHOLD) ay = 0.0f;
    if (az > -ZERO_THRESHOLD && az < ZERO_THRESHOLD) az = 0.0f;
 
    position[0] = s_vel_x * dt_s + 0.5f * ax * dt_s * dt_s;
    position[1] = s_vel_y * dt_s + 0.5f * ay * dt_s * dt_s;
    position[2] = s_vel_z * dt_s + 0.5f * az * dt_s * dt_s;
 
    s_vel_x += ax * dt_s;
    s_vel_y += ay * dt_s;
    s_vel_z += az * dt_s;
 
    return 0;
}


// -----------------------------------------------------------------------------------------------------------

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 * This is the main function for CM4 CPU. It perfroms the following opeartions:
 *    1. Initializes the BSP
 *    2. Initializes retarget IO for UART debug printing
 *    3. Initializes I2C using HAL driver
 *    4. Initializes the DPS310 pressure sensor
 *    5. Measures the temperature and the pressure values and prints it on the
*        serial terminal every 1 second.
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    uint32_t revisionID = 0;
    float pressure, temperature;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen. */
    printf("\x1b[2J\x1b[;H \r");
    printf("=========================================================\n\r");
    printf("  PSoC 6 MCU:  Interfacing DPS310 Pressure Sensor \r\n");
    printf("=========================================================\n\n\r");

    /* Initialize i2c for pressure sensor */
    result = cyhal_i2c_init(&I2Cm_HW, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("\r\nI2C initialization failed\r\n");
        CY_ASSERT(0);
    }

    /* Configure i2c with master configurations */
    result = cyhal_i2c_configure(&I2Cm_HW, &i2c_cfg_master);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("\r\nFailed to configure I2C\r\n");
        CY_ASSERT(0);
    }

    /* Initialize pressure sensor */
    result = xensiv_dps3xx_mtb_init_i2c(&dps310_sensor, &I2Cm_HW,
                                        XENSIV_DPS3XX_I2C_ADDR_DEFAULT);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("\r\nFailed to initialize DPS310 I2C\r\n");
        CY_ASSERT(0);
    }

    /* Retrieve the DPS310 Revision ID and display the same */
    if (xensiv_dps3xx_get_revision_id(&dps310_sensor,(uint8_t*)&revisionID) == CY_RSLT_SUCCESS)
    {
        printf("DPS310 Revision ID = %d\r\n\n",(uint8_t)revisionID);
    }
    else
    {
        printf("Failed to get Revision ID\r\n");
        CY_ASSERT(0);
    }

	/* BMI160 init + calibration */
	float position[3];
    printf("Calibrating BMI160 — keep sensor stationary...\r\n");
    if (motion_sensing_init(&I2Cm_HW) != 0)
    {
        printf("BMI160 init failed\r\n");
        CY_ASSERT(0);
    }
    printf("Calibration done.\r\n\n");


	float p0 = 0.0f;
	float t0 = 0.0f;

	float p0_sum = 0.0f;
	float t0_sum = 0.0f;

	for (int i = 0; i < REFERENCE_SAMPELING; i++) {
		float p01;
		float t01;
		xensiv_dps3xx_read(&dps310_sensor, &p01, &t01);
		p0_sum = p0_sum + p01;
		t0_sum = t0_sum + t01;
	}
	p0 = p0_sum / REFERENCE_SAMPELING;
	t0 = t0_sum / REFERENCE_SAMPELING;
	t0 = t0 + 273.15f;
	

    for (;;)
    {
        /* Read the pressure and temperature data */
	
        if (xensiv_dps3xx_read(&dps310_sensor, &pressure, &temperature) == CY_RSLT_SUCCESS)
        {
            /* Display the pressure and temperature data in console*/
            printf("Pressure : %0.2f mBar", pressure);
            /* 0xF8 - ASCII Degree Symbol */
            printf("\t Temperature: %0.2f %cC \r\n", temperature, 0xF8);
			temperature = temperature + 273.15f;
			float z = 0.0f;
			z = (R / G) * ((temperature + t0) / 2) * logf(p0/pressure);
			printf ("Height : %0.2f m", z);
        }
        else
        {
            printf("\n Failed to read temperature and pressure data.\r\n");
            CY_ASSERT(0);
        }
        
		
		int err = motion_sensing_get_displacement(DT_S, position);
        if (err == 0)
        {
            printf("dx: %.4f m  dy: %.4f m  dz: %.4f m\r\n",
                   position[0], position[1], position[2]);
        }
        else
        {
            printf("BMI160 read failed (err=%d)\r\n", err);
            motion_sensing_reset_velocity();
        }



        /* Generate a delay of 1 second before next read */
        cyhal_system_delay_ms((uint32_t)(DT_S * 1000.0f));
    }
}

/* [] END OF FILE */