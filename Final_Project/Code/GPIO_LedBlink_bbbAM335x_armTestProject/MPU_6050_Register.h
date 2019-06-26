/*
 * MPU_6000_Register.h
 *
 *  Created on: 17 de mai de 2019
 *      Author: joao
 */

#ifndef MPU_6050_REGISTER_H_
#define MPU_6050_REGISTER_H_
/***** TESTES *****/
#define SELF_TEST_X     (0x0D)
#define SELF_TEST_Y     (0x0E)
#define SELF_TEST_Z     (0x0F)
#define SELF_TEST_A     (0x10)

/***** NÃO SEI*****/
#define SMPLRT_DIV      (0x19)

/***** CONFIGURAÇÕES DOS SENSORES *****/
#define CONFIG          (0x1A)
#define GYRO_CONFIG     (0x1B)
#define ACCEL_CONFIG    (0x1C)

/***** CONFIGURAÇÃO DO FIFO *****/
#define FIFO_EN         (0x23)

/***** CONFIGURAÇÃO DOS MÓDULOS I2C *****/
#define I2C_MST_CTRL    (0x24)

/***** I2C0 *****/
#define I2C_SLV0_ADDR   (0x25)
#define I2C_SLV0_REG    (0x26)
#define I2C_SLV0_CTRL   (0x27)

/***** I2C1 *****/
#define I2C_SLV1_ADDR   (0x28)
#define I2C_SLV1_REG    (0x29)
#define I2C_SLV1_CTRL   (0x2A)

/***** I2C2 *****/
#define I2C_SLV2_ADDR   (0x2B)
#define I2C_SLV2_REG    (0x2C)
#define I2C_SLV2_CTRL   (0x2D)

/***** I2C3 *****/
#define I2C_SLV3_ADDR   (0x2E)
#define I2C_SLV3_REG    (0x2F)
#define I2C_SLV3_CTRL   (0x30)

/***** I2C4 *****/
#define I2C_SLV4_ADDR   (0x31)
#define I2C_SLV4_REG    (0x32)
#define I2C_SLV4_DO     (0x33)
#define I2C_SLV4_CTRL   (0x34)
#define I2C_SLV4_DI     (0x35)

/**** CONFIGURAÇÃO DO MÓDULO I2C *****/
#define I2C_MST_STATUS  (0x36)

/***** DEFINES INTERRUPÇÃO *****/
#define INT_PIN_CFG     (0x37)
#define INT_ENABLE      (0x38)
#define INT_STATUS      (0x3A)

/**** ACELELOMETRO *****/
#define ACCEL_XOUT_H    (0x3B)
#define ACCEL_XOUT_L    (0x3C)

#define ACCEL_YOUT_H    (0x3D)
#define ACCEL_YOUT_L    (0x3E)

#define ACCEL_ZOUT_H    (0x3F)
#define ACCEL_ZOUT_L    (0x40)

/**** TEMPERATURA ******/

#define TEMP_OUT_H      (0x41)
#define TEMP_OUT_L      (0x42)

/***** GIROSCOPIO *****/
#define GYRO_XOUT_H     (0x43)
#define GYRO_XOUT_L     (0x44)

#define GYRO_YOUT_H     (0x45)
#define GYRO_YOUT_L     (0x46)

#define GYRO_ZOUT_H     (0x47)
#define GYRO_ZOUT_L     (0x48)

/*****  *****/
#define EXT_SENS_DATA_00    (0x49)
#define EXT_SENS_DATA_01    (0x4A)
#define EXT_SENS_DATA_02    (0x4B)
#define EXT_SENS_DATA_03    (0x4C)
#define EXT_SENS_DATA_04    (0x4D)
#define EXT_SENS_DATA_05    (0x4E)
#define EXT_SENS_DATA_06    (0x4F)
#define EXT_SENS_DATA_07    (0x50)
#define EXT_SENS_DATA_08    (0x51)
#define EXT_SENS_DATA_09    (0x52)
#define EXT_SENS_DATA_10    (0x53)
#define EXT_SENS_DATA_11    (0x54)
#define EXT_SENS_DATA_12    (0x55)
#define EXT_SENS_DATA_13    (0x56)
#define EXT_SENS_DATA_14    (0x57)
#define EXT_SENS_DATA_15    (0x58)
#define EXT_SENS_DATA_16    (0x59)
#define EXT_SENS_DATA_17    (0x5A)
#define EXT_SENS_DATA_18    (0x5B)
#define EXT_SENS_DATA_19    (0x5C)
#define EXT_SENS_DATA_20    (0x5D)
#define EXT_SENS_DATA_21    (0x5E)
#define EXT_SENS_DATA_22    (0x5F)
#define EXT_SENS_DATA_23    (0x60)

/*****  *****/
#define I2C_SLV0_DO     (0x63)
#define I2C_SLV1_DO     (0x64)
#define I2C_SLV2_DO     (0x65)
#define I2C_SLV3_DO     (0x66)

/*****  *****/
#define I2C_MST_DELAY_CTRL      (0x67)
#define SIGNAL_PATH_RESET       (0x68)
#define USER_CTRL               (0x6A)
#define PWR_MGMT_1              (0x6B)
#define PWR_MGMT_2              (0x6C)
#define FIFO_COUNTH             (0x72)
#define FIFO_COUNTL             (0x73)
#define FIFO_R_W                (0x74)
#define WHO_AM_I                (0x75)

/*****  *****/
#define LSB_SENSITIVITY_ACCEL_2G       (16384) //LSB/g
#define LSB_SENSITIVITY_ACCEL_4G       (8192) //LSB/g
#define LSB_SENSITIVITY_ACCEL_8G       (4096) //LSB/g
#define LSB_SENSITIVITY_ACCEL_16G      (2048) //LSB/g
/*****  *****/
#define LSB_SENSITIVITY_GYRO_250       (131) //LSBº/s
#define LSB_SENSITIVITY_GYRO_500       (65.5) //LSBº/s
#define LSB_SENSITIVITY_GYRO_1000      (38.8) //LSBº/s
#define LSB_SENSITIVITY_GYRO_2000      (16.4) //LSBº/s

#endif /* MPU_6050_REGISTER_H_ */
