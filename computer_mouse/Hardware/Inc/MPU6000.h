#ifndef MPU6000_H
#define MPU6000_H

#include <stdint.h>
#include <main.h>
#define M_PI                        (3.1415)

// MPU6000 CHIP SELECT ON APM 2.0
#define MPU6000_CHIP_SELECT_PIN     (53)

// For quaternion (4*4) + Footer(2)
#define FIFO_PACKET_SIZE            (18)
// Rate of the gyro bias from gravity correction (200Hz/8) => 25Hz
#define GYRO_BIAS_FROM_GRAVITY_RATE (8)
// (2G range) We can remove this in future...
#define ACCEL_SCALE_G               (8192)

// Initial value to detect that compass correction is not initialized
#define COMPASS_NULL                (-9999)

// MPU6000 registers
#define MPUREG_XG_OFFS_TC           (0x00)
#define MPUREG_YG_OFFS_TC           (0x01)
#define MPUREG_ZG_OFFS_TC           (0x02)
#define MPUREG_X_FINE_GAIN          (0x03)
#define MPUREG_Y_FINE_GAIN          (0x04)
#define MPUREG_Z_FINE_GAIN          (0x05)
#define MPUREG_XA_OFFS_H            (0x06)
#define MPUREG_XA_OFFS_L            (0x07)
#define MPUREG_YA_OFFS_H            (0x08)
#define MPUREG_YA_OFFS_L            (0x09)
#define MPUREG_ZA_OFFS_H            (0x0A)
#define MPUREG_ZA_OFFS_L            (0x0B)
#define MPUREG_PRODUCT_ID           (0x0C)
#define MPUREG_XG_OFFS_USRH         (0x13)
#define MPUREG_XG_OFFS_USRL         (0x14)
#define MPUREG_YG_OFFS_USRH         (0x15)
#define MPUREG_YG_OFFS_USRL         (0x16)
#define MPUREG_ZG_OFFS_USRH         (0x17)
#define MPUREG_ZG_OFFS_USRL         (0x18)
#define MPUREG_SMPLRT_DIV           (0x19)
#define MPUREG_CONFIG               (0x1A)
#define MPUREG_GYRO_CONFIG          (0x1B)
#define MPUREG_ACCEL_CONFIG         (0x1C)
#define MPUREG_ACCEL_CONFIG_2       (0x1D)
#define MPUREG_INT_PIN_CFG          (0x37)
#define MPUREG_INT_ENABLE           (0x38)
#define MPUREG_ACCEL_XOUT_H         (0x3B)
#define MPUREG_ACCEL_XOUT_L         (0x3C)
#define MPUREG_ACCEL_YOUT_H         (0x3D)
#define MPUREG_ACCEL_YOUT_L         (0x3E)
#define MPUREG_ACCEL_ZOUT_H         (0x3F)
#define MPUREG_ACCEL_ZOUT_L         (0x40)
#define MPUREG_TEMP_OUT_H           (0x41)
#define MPUREG_TEMP_OUT_L           (0x42)
#define MPUREG_GYRO_XOUT_H          (0x43)
#define MPUREG_GYRO_XOUT_L          (0x44)
#define MPUREG_GYRO_YOUT_H          (0x45)
#define MPUREG_GYRO_YOUT_L          (0x46)
#define MPUREG_GYRO_ZOUT_H          (0x47)
#define MPUREG_GYRO_ZOUT_L          (0x48)
#define MPUREG_USER_CTRL            (0x6A)
#define MPUREG_PWR_MGMT_1           (0x6B)
#define MPUREG_PWR_MGMT_2           (0x6C)
#define MPUREG_BANK_SEL             (0x6D)
#define MPUREG_MEM_START_ADDR       (0x6E)
#define MPUREG_MEM_R_W              (0x6F)
#define MPUREG_DMP_CFG_1            (0x70)
#define MPUREG_DMP_CFG_2            (0x71)
#define MPUREG_FIFO_COUNTH          (0x72)
#define MPUREG_FIFO_COUNTL          (0x73)
#define MPUREG_FIFO_R_W             (0x74)
#define MPUREG_WHOAMI               (0x75)

// Configuration bits MPU6000
#define BIT_SLEEP                   (0x40)
#define BIT_H_RESET                 (0x80)
#define BITS_CLKSEL                 (0x07)
#define MPU_CLK_SEL_PLLGYROX        (0x01)
#define MPU_CLK_SEL_PLLGYROZ        (0x03)
#define MPU_EXT_SYNC_GYROX          (0x02)
#define BITS_FS_250DPS              (0x00)
#define BITS_FS_500DPS              (0x08)
#define BITS_FS_1000DPS             (0x10)
#define BITS_FS_2000DPS             (0x18)
#define BITS_FS_2G                  (0x00)
#define BITS_FS_4G                  (0x08)
#define BITS_FS_8G                  (0x10)
#define BITS_FS_16G                 (0x18)
#define BITS_FS_MASK                (0x18)
#define BITS_DLPF_CFG_256HZ_NOLPF2  (0x00)
#define BITS_DLPF_CFG_188HZ         (0x01)
#define BITS_DLPF_CFG_98HZ          (0x02)
#define BITS_DLPF_CFG_42HZ          (0x03)
#define BITS_DLPF_CFG_20HZ          (0x04)
#define BITS_DLPF_CFG_10HZ          (0x05)
#define BITS_DLPF_CFG_5HZ           (0x06)
#define BITS_DLPF_CFG_2100HZ_NOLPF  (0x07)
#define BITS_DLPF_CFG_MASK          (0x07)
#define BIT_INT_ANYRD_2CLEAR        (0x10)
#define BIT_RAW_RDY_EN              (0x01)
#define BIT_I2C_IF_DIS              (0x10)

// DMP output rate constants
// default value = 0
#define MPU6000_200HZ               (0)
#define MPU6000_100HZ               (1)
#define MPU6000_66HZ                (2)
#define MPU6000_50HZ                (3)

#define MPU6000_DEG_PER_LSB_250  (float)((2 * 250.0) / 65536.0)
#define MPU6000_DEG_PER_LSB_500  (float)((2 * 500.0) / 65536.0)
#define MPU6000_DEG_PER_LSB_1000 (float)((2 * 1000.0) / 65536.0)
#define MPU6000_DEG_PER_LSB_2000 (float)((2 * 2000.0) / 65536.0)

#define MPU6000_G_PER_LSB_2      (float)((2 * 2) / 65536.0)
#define MPU6000_G_PER_LSB_4      (float)((2 * 4) / 65536.0)
#define MPU6000_G_PER_LSB_8      (float)((2 * 8) / 65536.0)
#define MPU6000_G_PER_LSB_16     (float)((2 * 16) / 65536.0)

#define MPU6000_GYRO_FS_250         0x00
#define MPU6000_GYRO_FS_500         0x01
#define MPU6000_GYRO_FS_1000        0x02
#define MPU6000_GYRO_FS_2000        0x03

#define MPU6000_ACCEL_FS_2          0x00
#define MPU6000_ACCEL_FS_4          0x01
#define MPU6000_ACCEL_FS_8          0x02
#define MPU6000_ACCEL_FS_16         0x03

#define SENSORS_DEG_PER_LSB_CFG   MPU6000_DEG_PER_LSB_2000
#define SENSORS_G_PER_LSB_CFG     MPU6000_G_PER_LSB_16

#define SENSORS_GYRO_FS_CFG       MPU6000_GYRO_FS_2000
#define SENSORS_ACCEL_FS_CFG      MPU6000_ACCEL_FS_16	

#define SENSORS_NBR_OF_BIAS_SAMPLES		1024	/* 计算方差的采样样本个数 */
#define GYRO_VARIANCE_BASE				4000	/* 陀螺仪零偏方差阈值 */
#define SENSORS_ACC_SCALE_SAMPLES  		200		/* 加速计采样个数 */

#define MPUREG_PWR1_DEVICE_RESET_BIT   7
#define MPUREG_PWR1_SLEEP_BIT          6
#define MPUREG_PWR1_TEMP_DIS_BIT       3
#define MPUREG_PWR1_CLKSEL_BIT         2
#define MPUREG_PWR1_CLKSEL_LENGTH      3

#define MPUREG_GCONFIG_FS_SEL_BIT      4
#define MPUREG_GCONFIG_FS_SEL_LENGTH   2

#define MPUREG_ACONFIG_AFS_SEL_BIT         4
#define MPUREG_ACONFIG_AFS_SEL_LENGTH      2

#define MPUREG_ACONFIG2_DLPF_BIT           0
#define MPUREG_ACONFIG2_DLPF_LENGTH        2

#define MPUREG_CFG_DLPF_CFG_BIT    2
#define MPUREG_CFG_DLPF_CFG_LENGTH 3

#define MPU6000_ACCEL_DLPF_BW_460   0x00
#define MPU6000_ACCEL_DLPF_BW_184   0x01
#define MPU6000_ACCEL_DLPF_BW_92    0x02
#define MPU6000_ACCEL_DLPF_BW_41    0x03
#define MPU6000_ACCEL_DLPF_BW_20    0x04
#define MPU6000_ACCEL_DLPF_BW_10    0x05
#define MPU6000_ACCEL_DLPF_BW_5     0x06

#define MPU6000_DLPF_BW_256         0x00
#define MPU6000_DLPF_BW_188         0x01
#define MPU6000_DLPF_BW_98          0x02
#define MPU6000_DLPF_BW_42          0x03
#define MPU6000_DLPF_BW_20          0x04
#define MPU6000_DLPF_BW_10          0x05
#define MPU6000_DLPF_BW_5           0x06

#define MPUREG_CLOCK_PLL_XGYRO         0x01

#define RATE_5_HZ		5
#define RATE_10_HZ		10
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000

#define MPU6000_INTCFG_INT_LEVEL_BIT        7
#define MPU6000_INTCFG_INT_OPEN_BIT         6
#define MPU6000_INTCFG_LATCH_INT_EN_BIT     5
#define MPU6000_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU6000_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU6000_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU6000_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU6000_INTCFG_CLKOUT_EN_BIT        0

#define MPU6000_INTERRUPT_DATA_RDY_BIT      0

#define MPU6000_USERCTRL_I2C_IF_DIS_BIT         4

#define ATTITUDE_ESTIMAT_RATE	RATE_250_HZ	//姿态解算速率
#define ATTITUDE_ESTIMAT_DT		(1.0/RATE_250_HZ)

#pragma anon_unions

typedef union 
{
	struct 
	{
		float x;
		float y;
		float z;
	};
	float axis[3];
} Axis3f;

typedef union 
{
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
	};
	int16_t axis[3];
} Axis3i16;
typedef struct 
{
	float a1;
	float a2;
	float b0;
	float b1;
	float b2;
	float delay_element_1;
	float delay_element_2;
} lpf2pData;

typedef struct  
{
	__IO uint32_t timestamp;	/*时间戳*/

	__IO float roll;
	__IO float pitch;
	__IO float yaw;
} attitude_t;

#define DEG2RAD		0.017453293f	/* 度转弧度 π/180 */
#define RAD2DEG		57.29578f		/* 弧度转度 180/π */

extern attitude_t attitudeData;
extern uint8_t ucMPUTBuf[32],ucMPURBuf[32];
//dmpMem from dmpDefaultMantis.c
//extern unsigned char dmpMem[8][16][16] PROGMEM;
// unsigned char dmpMem[8][16][16];

// new data flag variable
//extern volatile uint8_t _MPU6000_newdata;
void MPUInit(void);
void MPUUpdate(void);
uint8_t MPUReadBufferDMA(uint8_t RegAddr,uint8_t len);
// ----- MPU6000 CLASS ----------
//class MPU6000_Class
//{
//  public:
//
//        MPU6000_Class(void);       // Constructor

extern void MPU6000_init(void);           // MPU6000 initialization
extern void MPU6000_dmp_init(void);       // MPU6000 DMP initialization
extern void set_dmp_rate(uint8_t rate);  // set DMP output rate (see constants)
extern void MPU6000_read(void);           // read raw data
extern int MPU6000_newdata(void);     // new attitude data?
extern void MPU6000_calculate(void);      // Read quaternion data and calculate DCM and euler angles

// Calibration methods
extern void MPU6000_gyro_offset_calibration(void);  // Performs a gyro offset calibration and store offset values
extern void MPU6000_accel_offset_calibration(void);  // Performs an accel offset calibration and store offset values
extern void MPU6000_accel_set_offset(int accel_x_offset, int accel_y_offset, int accel_z_offset);  // Set accelerometer offsets
extern void MPU6000_accel_get_offset(int *accel_offset);

// Gyro bias correction methods
extern void MPU6000_gyro_bias_correction_from_gravity(void);  // Function to correct the gyroX and gyroY bias (roll and pitch) using the gravity vector from accelerometers

// Method to compensate for centrifugal force
extern void MPU6000_accel_centrifugal_force_correction(float speed);  // We need to provide and external speed estimation in (m/s)

// Methods to support external magnetometer fusion
extern float MPU6000_compass_angle(float mag_x, float mag_y, float mag_z, float declination);  // Calculate tilt compensated compass heading (from external magnetometer)
extern void MPU6000_update_yaw_compass_correction(float compass_heading);  // Update the yaw correction using compass info (compass sensor fusion)
extern void MPU6000_get_yaw_compass_corrected(void);  // Get the corrected yaw with the compass fusion

// Functions to retrieve attitude data from FIFO (DMP)
extern void MPU6000_FIFO_reset(void);
extern void MPU6000_FIFO_getPacket(void);
extern int MPU6000_FIFO_ready(void);

// Functions to set gains
extern void MPU6000_set_accel_fusion_gain(uint8_t gain);
extern void MPU6000_set_gyro_bias_correction_from_gravity_gain(float gain);
extern void MPU6000_set_compass_correction_gain(float gain);

//Sensor variables
extern int accelX;
extern int accelY;
extern int accelZ;
extern int MPU6000_ACCEL_OFFSET[3];
extern unsigned int mpu_temp;
extern int gyroX;
extern int gyroY;
extern int gyroZ;

// Euler angles
extern float MPU_roll;
extern float MPU_pitch;
extern float MPU_yaw;
extern float MPU_yaw_without_compass;

// quaternion
extern float MPU_q[4];

// DCM Rotation matrix (in APM2.0 axis definition)
extern float DCM[3][3];

extern __IO bool gbMpuUpdated;
#endif /* MPU6000_H */


