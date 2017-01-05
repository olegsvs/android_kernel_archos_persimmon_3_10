#ifndef OIS_OTP_DEF_H
#define OIS_OTP_DEF_H

#define NVR0_OFFSET 0x00000000
#define NVR0_ADDR 0x00000000
#define NVR0_SIZE 0x00000040
#define NVR11_OFFSET NVR0_SIZE
#define NVR11_ADDR 0x00000100
#define NVR11_SIZE 0x00000082
#define NVR12_OFFSET (NVR11_OFFSET + NVR11_SIZE)
#define NVR12_ADDR 0x000001B0
#define NVR12_SIZE 0x00000020
#define NVR_TOTAL_SZ (NVR0_SIZE + NVR11_SIZE + NVR12_SIZE)
#define NVR_BLOCK_SIZE 256
#define NVR1_COORDINATE_ETC_DATA_ADDR_OFFSET 0x1B0


/**********************************************************************
 *
 * 			Map of nvr flash & ois otp
 * --------------------------------------------------------------------
 *		nvr flash	|		OTP
 *----------------------------------------------------------------------
 *  NVR0	0x0000 - 0x0040	|	0x0a04 - 0x0a44		page 3-5
 *------------------------------|---------------------------------------
 *  NVR1 Hall Calibration data + Gyro Gain(1)
 *  NVR1	0x0100 - 0x0140	|	0x0a04 - 0x0a44		page 6-9
 *				|---------------------------------------
 *  NVR1 Hall Calibration data + Gyro Gain(2)
 *  NVR1	0x0140 - 0x0180	|	0x0a04 - 0x0a44		page 10-11
 *				|---------------------------------------
 *  NVR1 Primax AFMidPos(2 bytes)
 *  NVR1	0x0180 - 0x0182	|	0x0a04 - 0x0a06		page 12-14
 *------------------------------|
 *  NVR1 Coordinate ... etc data
 *  NVR1	0x01B0 - 0x01D0	|	0x0a14 - 0x0a34		page 12-14
 *------------------------------|---------------------------------------
 *
************************************************************************/



#define NVR1_HALL_CAL_GYRO_GAIN_1_OFFSET (NVR0_OFFSET + NVR0_SIZE)
#define NVR1_HALL_CAL_GYRO_GAIN_1_SIZE 0x40
#define NVR1_HALL_CAL_GYRO_GAIN_2_OFFSET (NVR1_HALL_CAL_GYRO_GAIN_1_OFFSET + NVR1_HALL_CAL_GYRO_GAIN_1_SIZE)
#define NVR1_HALL_CAL_GYRO_GAIN_2_SIZE 0x40
#define NVR1_PRI_AFMIDPOS_COO_ETC_DATA_OFFSET (NVR1_HALL_CAL_GYRO_GAIN_2_OFFSET + NVR1_HALL_CAL_GYRO_GAIN_2_SIZE)
#define NVR1_PRI_AFMIDPOS_COO_ETC_DATA_SIZE 0x30
#define NVR1_PRI_AFMIDPOS_OFFSET NVR1_PRI_AFMIDPOS_COO_ETC_DATA_OFFSET
#define NVR1_PRI_AFMIDPOS_SIZE 0x02
#define NVR1_COORDINATE_ETC_DATA_OFFSET (NVR1_PRI_AFMIDPOS_COO_ETC_DATA_OFFSET + 0x10)
#define NVR1_COORDINATE_ETC_DATA_SIZE 0x20
#define OIS_OTP_SIZE (NVR0_SIZE+NVR1_HALL_CAL_GYRO_GAIN_1_SIZE+NVR1_HALL_CAL_GYRO_GAIN_2_SIZE+NVR1_PRI_AFMIDPOS_COO_ETC_DATA_SIZE)
#define OIS_OTP_DUMP

int len_sysboot_check(char * ois_otp_buf_p);
#endif
