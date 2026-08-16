#define _POSIX_C_SOURCE 200809L

/*
 *  TU/e 5EID0 - Venus Project -- ROVER firmware (single-file / run-on-boot build).
 *
 *  Consolidated build: the sensor module (sensors.c), the VL53L0X ToF driver
 *  (vl53l0x.c) and the partner-robot / mission-control MQTT link (mqtt.c) are all
 *  INLINED below, so this is one self-contained translation unit -- it includes
 *  none of the project's own headers and links against nothing but libpynq + libm.
 *  That makes it trivial to drop on the board and launch from a run-on-boot service:
 *
 *      gcc main.c  -o robot1 $(pkg-config --cflags --libs libpynq) -lm
 *      gcc main2.c -o robot2 $(pkg-config --cflags --libs libpynq) -lm
 *
 *  AUTONOMOUS / MQTT-ONLY. At boot there is no operator and no ws_bridge, so the
 *  stdin command interface and the stdout UI JSON stream have been removed: the
 *  robot powers straight into autonomous EXPLORE and reports everything (pose,
 *  status, mountains/tape, scanned cubes) to mission control + the partner robot
 *  over the MQTT/UART0 link only. Manual / target tele-op went with the command
 *  interface. Diagnostics go to the boot log (firmware status on stderr; the
 *  inlined sensor bring-up keeps its own stdout traces).
 *  Responds to SIGINT/SIGTERM with a clean shutdown (motors disabled,
 *  IIC/UART/pynq released) so a service stop never leaves the steppers live.
 *
 *  This is ROBOT 1 (starts facing +y). Robot 2 is main2.c, which re-#includes this
 *  file with a -y start heading. Build ONE main per robot (never link both).
 */

#include <libpynq.h>
#include <stepper.h>   /* stepper_init/enable/steps/... (not pulled in by libpynq.h) */
#include <iic.h>       /* IIC0 ToF bus (was pulled in by the inlined sensors.c)       */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>   /* SIGINT/SIGTERM clean shutdown */

/* =====================================================================
 *  INLINED LIBRARY DECLARATIONS  (were vl53l0x.h / sensors.h / mqtt.h)
 *  The robot no longer ships or includes those headers; their types + API
 *  live here and the implementations follow further down.
 * ===================================================================== */

#ifndef _TOFLIB_H_
#define _TOFLIB_H_

/**
 * @brief Internal type, do not modify directly. 
 */
typedef struct _vl53_sensor_ {
    iic_index_t iic_index;
    uint8_t baseAddr;
    uint8_t stop_variable;
    uint32_t measurement_timing_budget_us;
} vl53x;

/**
 * @brief Set IIC address of a VL53L0X Sensor
 * @param addr IIC Address of the sensor
 * @param iic IIC Index
 * @param newAddr IIC Target Address of the sensor
 * @return 0 if successful, 1 on error
 */
extern int tofSetAddress(iic_index_t iic, uint8_t addr, uint8_t newAddr);

/**
 * @brief Connection test for a VL53L0X Sensor
 * @note USE BEFORE `tofInit`!
 * @param addr IIC Address of the sensor
 * @param iic IIC Index
 * @return 0 if successful, 1 on error
 */
extern int tofPing(iic_index_t iic, uint8_t addr);

/**
 * @brief Initialize the VL53L0X Sensor
 * @param sensor Handle to the sensor.
 * @param iic IIC Index
 * @param addr IIC Address of the sensor
 * @param bLongRange 0 => default mode (30-800mm) or 1 => long range mode (30-2000mm)
 * @return 0 if successful, 1 on error
 */
extern int tofInit(vl53x *sensor, iic_index_t iic, uint8_t addr, int bLongRange);

/**
 * @brief Read the model and revision of the VL53L0X Sensor
 * @param sensor Handle to the sensor.
 * @param model pointer to store model
 * @param revision pointer to store revision
 * @return 0 if successful, 1 on error
 */
extern int tofGetModel(vl53x *sensor, uint8_t *model, uint8_t *revision);

/**
 * @brief Read current distance in mm
 * @param sensor Handle to the sensor.
 * @returns distance in mm
 */
extern uint32_t tofReadDistance(vl53x *sensor);

#endif // _TOFLIB_H
#ifndef SENSORS_H
#define SENSORS_H

/*
 *  TU/e 5EID0 - Venus Project, Group 10
 *  Embedded Software - Sensor Module
 *
 *  Public API used by the robot main loop. Each underlying sensor
 *  (3x VL53L0X ToF, 2x TCRT5000 IR, 1x TCS3200 colour, 1x NTC-10K
 *  thermistor) has its own init + read function, plus there is a
 *  single update_sensor_data() that polls everything in one go.
 */

/* This is THE struct the robot main code uses. Replace the local
 * definition in robot main.c with `#include "sensors.h"`.        */
typedef struct {
    /* The three ToF sensors are stacked vertically and aimed forward:
     *   bottom -> small (3x3) rocks, middle -> large (6x6) rocks,
     *   top    -> mountains. mm; 0 means out-of-range / invalid.        */
    uint16_t tof_bottom_mm;
    uint16_t tof_middle_mm;
    uint16_t tof_top_mm;
    bool     ir_left_detects_tape;
    bool     ir_right_detects_tape;
    uint32_t color_freq_r;
    uint32_t color_freq_g;
    uint32_t color_freq_b;
    float    temp_celsius;
} SensorData;

/* ---------- Top-level ---------- */

/* Brings up every sensor in the right order.
 * Returns 0 if everything came online, non-zero if any sensor failed. */
int  sensors_init_all(void);

/* Polls every sensor and writes the readings into `out`. Never blocks
 * for long; safe to call once per main-loop iteration.                */
void update_sensor_data(SensorData *out);

/* ---------- Time-of-Flight (3x VL53L0X on IIC0) ---------- */

/* Brings the three ToF sensors up via the XSHUT address-reassign
 * sequence (A is always-on, B on AR0, C on AR1).                */
int  tof_init(void);

/* Reads all three ToF distances in mm. Vertical stack aimed forward:
 *   bottom -> small (3x3) rocks, middle -> large (6x6) rocks, top -> mountains.
 * Zero means out-of-range / read error - treat 0 as "invalid". */
void tof_read(uint16_t *bottom, uint16_t *middle, uint16_t *top);

/* ---------- IR tape detector (2x TCRT5000) ---------- */

int  ir_init(void);
void ir_read(bool *left_tape, bool *right_tape);

/* ---------- Colour (TCS3200) ---------- */

/* Discrete colour label produced by the classifier. SCAN_COLOR_NONE means
 * "nothing scannable in view" (too dark / no dominant channel).            */
typedef enum {
    SCAN_COLOR_NONE = 0,
    SCAN_COLOR_RED,
    SCAN_COLOR_GREEN,
    SCAN_COLOR_BLUE,
    SCAN_COLOR_YELLOW,
    SCAN_COLOR_WHITE,
    SCAN_COLOR_BLACK
} scan_color_t;

int  color_init(void);

/* Raw per-filter HIGH-pulse counts (longer = darker through that filter). */
void color_read(uint32_t *r, uint32_t *g, uint32_t *b);

/* Turn three filter frequencies into a colour label. When non-NULL,
 * *confidence is filled in [0..1] (share of total the winning channel holds).
 * Tunables COLOR_MIN_TOTAL_HZ / COLOR_DOMINANT_MARGIN live in sensors.c.    */
scan_color_t color_classify(uint32_t r, uint32_t g, uint32_t b, float *confidence);

/* Convenience: one shot read + classify. */
scan_color_t color_read_label(float *confidence);

/* Human-readable name for a label (e.g. for JSON / logging). */
const char  *color_label_name(scan_color_t label);

/* ---------- Temperature (NTC-10K via ADC, on ADC5) ---------- */

int  temp_init(void);

/* Read the NTC and return degrees Celsius directly (Beta equation:
 * B = 4050, R0 = 10k @ 25C, 10k series resistor, 3.3V, divider on ADC5).
 * Returns a large negative value if the reading is invalid. */
float temp_read_celsius(void);

#endif /* SENSORS_H */
#ifndef MQTT
#define MQTT

/**
 * @brief Send a raw JSON string to the ESP32 over UART0 with a 4-byte
 *        little-endian length header (the partner-robot / mission-control link).
 *        Used for pose / status / object telemetry; send_mqtt() builds on it.
 */
void mqtt_publish_json(const char* json);

/**
 * @brief Format rock data as JSON and send it to the ESP32 over UART0 with a
 *        4-byte little-endian length header (partner-robot MQTT link).
 */
void send_mqtt(int x, int y, const char* color, int size, float temp);

/**
 * @brief Non-blocking-ish poll: if a framed message is waiting on UART0, parse
 *        the partner robot's rock (x, y, size, temp, color) into the outputs.
 *        color_buf must be at least 32 bytes. Leaves outputs untouched when no
 *        frame is available.
 * @return true if a complete frame was parsed into the outputs this call, false
 *         otherwise. Call in a `while (recv_mqtt(...))` loop to drain every queued
 *         frame in one poll (one frame is parsed per call).
 */
bool recv_mqtt(int* xz, int* yz, int* sizez, float* tempz, char* color_buf);

/**
 * @brief Route UART0 to AR0/AR1 and bring it up. NOTE: robot_main sets UART0 up
 *        itself (to control switchbox ordering vs the sensors), so this is
 *        provided for standalone use and is not called there.
 */
void init_mqtt(void);

#endif

/* =====================================================================
 *  INLINED IMPLEMENTATION -- vl53l0x.c  (VL53L0X ToF driver)
 * ===================================================================== */
 /*
 *  TU/e 5EID0::LIBPYNQ Driver for VL53L0X TOF Sensor
 *
 *  Original C library: Larry Bank
 *  Adapted for PYNQ: Walthzer
 * 
 */

//
// VL53L0X time of flight range sensor
// Library to read the distance
// from the I2C bus
//
// by Larry Bank
//
// This code is based on Pololu's Arduino library
// https://github.com/pololu/vl53l0x-arduino
// (see LICENSE.txt for more info)
//
// My version is an attempt to simplify that code and 
// create a generic C library for Linux
//

static uint8_t readReg(vl53x *ptr_s, uint8_t ucAddr);
static unsigned short readReg16(vl53x *ptr_s, uint8_t ucAddr);
static void writeReg16(vl53x *ptr_s, uint8_t ucAddr, unsigned short usValue);
static void writeReg(vl53x *ptr_s, uint8_t ucAddr, uint8_t ucValue);
static void writeRegList(vl53x *ptr_s, uint8_t *ucList);
static int initSensor(vl53x *ptr_s, int);
static int performSingleRefCalibration(vl53x *ptr_s, uint8_t vhv_init_byte);
static int setMeasurementTimingBudget(vl53x *ptr_s, uint32_t budget_us);

#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)
// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

#define VL53L0X_SEQUENCE_ENABLE_FINAL_RANGE 0x80
#define VL53L0X_SEQUENCE_ENABLE_PRE_RANGE   0x40
#define VL53L0X_SEQUENCE_ENABLE_TCC         0x10
#define VL53L0X_SEQUENCE_ENABLE_DSS         0x08
#define VL53L0X_SEQUENCE_ENABLE_MSRC        0x04

typedef enum vcselperiodtype { VcselPeriodPreRange, VcselPeriodFinalRange } vcselPeriodType;
static int setVcselPulsePeriod(vl53x *ptr_s, vcselPeriodType type, uint8_t period_pclks);

typedef struct tagSequenceStepTimeouts
{
  uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

  uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
  uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
} SequenceStepTimeouts;

// VL53L0X internal registers
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID		0xc0
#define VL53L0X_EXPECTED_MODEL_ID 0xEE
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID		0xc2
#define VL53L0X_REG_SYSRANGE_START			0x00

#define VL53L0X_REG_RESULT_INTERRUPT_STATUS 		0x13
#define VL53L0X_RESULT_RANGE_STATUS      		0x14
#define VL53L0X_ALGO_PHASECAL_LIM                       0x30
#define VL53L0X_ALGO_PHASECAL_CONFIG_TIMEOUT            0x30

#define VL53L0X_GLOBAL_CONFIG_VCSEL_WIDTH               0x32
#define VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_LOW      0x47
#define VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH     0x48

#define VL53L0X_PRE_RANGE_CONFIG_VCSEL_PERIOD           0x50
#define VL53L0X_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI      0x51
#define VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_LOW        0x56
#define VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_HIGH       0x57

#define VL53L0X_REG_MSRC_CONFIG_CONTROL                 0x60
#define VL53L0X_FINAL_RANGE_CONFIG_VCSEL_PERIOD         0x70
#define VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI    0x71
#define VL53L0X_MSRC_CONFIG_TIMEOUT_MACROP              0x46
#define VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT  0x44
#define VL53L0X_SYSRANGE_START                          0x00
#define VL53L0X_SYSTEM_SEQUENCE_CONFIG                  0x01
#define VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO            0x0A
#define VL53L0X_RESULT_INTERRUPT_STATUS                 0x13
#define VL53L0X_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV       0x89
#define VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0        0xB0
#define VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH                 0x84
#define VL53L0X_SYSTEM_INTERRUPT_CLEAR                  0x0B

#define VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS 0x8A

//
// Set IIC address of a VL53L0X Sensor
//
int tofSetAddress(iic_index_t iic, uint8_t addr, uint8_t newAddr)
{
  return iic_write_register(iic, addr, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, &newAddr, 1);
}

//
// Connection test for a VL53L0X Sensor
// Model should be equal for all VL53L0X
//
int tofPing(iic_index_t iic, uint8_t addr)
{
  uint8_t model;
  iic_read_register(iic, addr, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model, 1);
  return (model != VL53L0X_EXPECTED_MODEL_ID);
}

//
// reads the calibration data and sets the device
// into auto sensing mode
//
int tofInit(vl53x *ptr_s, iic_index_t iic, uint8_t addr, int bLongRange)
{
  ptr_s->iic_index = iic;
  ptr_s->baseAddr = addr;
	return initSensor(ptr_s, bLongRange); // finally, initialize the magic numbers in the sensor

} /* tofInit() */

//
// Read a pair of registers as a 16-bit value
//
static unsigned short readReg16(vl53x *ptr_s, uint8_t ucAddr)
{
  uint8_t ucTemp[2];
  iic_read_register(ptr_s->iic_index, ptr_s->baseAddr, ucAddr, ucTemp, 2);

	return (unsigned short)((ucTemp[0]<<8) + ucTemp[1]);
} /* readReg16() */

//
// Read a single register value from I2C device
//
static uint8_t readReg(vl53x *ptr_s, uint8_t ucAddr)
{
  uint8_t ucTemp;

  iic_read_register(ptr_s->iic_index, ptr_s->baseAddr, ucAddr, &ucTemp, 1);

	return ucTemp;
} /* ReadReg() */

static void readMulti(vl53x *ptr_s, uint8_t ucAddr, uint8_t *pBuf, int iCount)
{

  iic_read_register(ptr_s->iic_index, ptr_s->baseAddr, ucAddr, pBuf, iCount);

} /* readMulti() */

static void writeMulti(vl53x *ptr_s, uint8_t ucAddr, uint8_t *pBuf, int iCount)
{

  iic_write_register(ptr_s->iic_index, ptr_s->baseAddr, ucAddr, pBuf, iCount);

} /* writeMulti() */
//
// Write a 16-bit value to a register
//
static void writeReg16(vl53x *ptr_s, uint8_t ucAddr, unsigned short usValue)
{
  uint8_t pBuf[2];

	pBuf[0] = (uint8_t)(usValue >> 8); // MSB first
	pBuf[1] = (uint8_t) usValue;

  iic_write_register(ptr_s->iic_index, ptr_s->baseAddr, ucAddr, pBuf, 2);
} /* writeReg16() */
//
// Write a single register/value pair
//
static void writeReg(vl53x *ptr_s, uint8_t ucAddr, uint8_t ucValue)
{

  iic_write_register(ptr_s->iic_index, ptr_s->baseAddr, ucAddr, &ucValue, 1);

} /* writeReg() */

//
// Write a list of register/value pairs to the I2C device
//
static void writeRegList(vl53x *ptr_s, uint8_t *ucList)
{
  uint8_t ucCount = *ucList++; // count is the first element in the list

	while (ucCount)
	{
    iic_write_register(ptr_s->iic_index, ptr_s->baseAddr, ucList[0], &(ucList[1]), 1);
		ucList += 2;
		ucCount--;
	}
} /* writeRegList() */

//
// Register init lists consist of the count followed by register/value pairs
//
uint8_t ucI2CMode[] = {4, 0x88,0x00, 0x80,0x01, 0xff,0x01, 0x00,0x00};
uint8_t ucI2CMode2[] = {3, 0x00,0x01, 0xff,0x00, 0x80,0x00};
uint8_t ucSPAD0[] = {4, 0x80,0x01, 0xff,0x01, 0x00,0x00, 0xff,0x06};
uint8_t ucSPAD1[] = {5, 0xff,0x07, 0x81,0x01, 0x80,0x01, 0x94,0x6b, 0x83,0x00};
uint8_t ucSPAD2[] = {4, 0xff,0x01, 0x00,0x01, 0xff,0x00, 0x80,0x00};
uint8_t ucSPAD[] = {5, 0xff,0x01, 0x4f,0x00, 0x4e,0x2c, 0xff,0x00, 0xb6,0xb4};
uint8_t ucDefTuning[] = {80, 0xff,0x01, 0x00,0x00, 0xff,0x00, 0x09,0x00,
0x10,0x00, 0x11,0x00, 0x24,0x01, 0x25,0xff, 0x75,0x00, 0xff,0x01, 0x4e,0x2c,
0x48,0x00, 0x30,0x20, 0xff,0x00, 0x30,0x09, 0x54,0x00, 0x31,0x04, 0x32,0x03,
0x40,0x83, 0x46,0x25, 0x60,0x00, 0x27,0x00, 0x50,0x06, 0x51,0x00, 0x52,0x96,
0x56,0x08, 0x57,0x30, 0x61,0x00, 0x62,0x00, 0x64,0x00, 0x65,0x00, 0x66,0xa0,
0xff,0x01, 0x22,0x32, 0x47,0x14, 0x49,0xff, 0x4a,0x00, 0xff,0x00, 0x7a,0x0a,
0x7b,0x00, 0x78,0x21, 0xff,0x01, 0x23,0x34, 0x42,0x00, 0x44,0xff, 0x45,0x26,
0x46,0x05, 0x40,0x40, 0x0e,0x06, 0x20,0x1a, 0x43,0x40, 0xff,0x00, 0x34,0x03,
0x35,0x44, 0xff,0x01, 0x31,0x04, 0x4b,0x09, 0x4c,0x05, 0x4d,0x04, 0xff,0x00,
0x44,0x00, 0x45,0x20, 0x47,0x08, 0x48,0x28, 0x67,0x00, 0x70,0x04, 0x71,0x01,
0x72,0xfe, 0x76,0x00, 0x77,0x00, 0xff,0x01, 0x0d,0x01, 0xff,0x00, 0x80,0x01,
0x01,0xf8, 0xff,0x01, 0x8e,0x01, 0x00,0x01, 0xff,0x00, 0x80,0x00};

int getSpadInfo(vl53x *ptr_s, uint8_t *pCount, uint8_t *pTypeIsAperture)
{
  int iTimeout;
  uint8_t ucTemp;
  #define VL53L0X_SPAD_MAX_TIMEOUT 50

  writeRegList(ptr_s, ucSPAD0);
  writeReg(ptr_s, 0x83, readReg(ptr_s, 0x83) | 0x04);
  writeRegList(ptr_s, ucSPAD1);
  iTimeout = 0;
  while(iTimeout < VL53L0X_SPAD_MAX_TIMEOUT)
  {
    if (readReg(ptr_s, 0x83) != 0x00) break;
    iTimeout++;
    sleep_msec(5);
  }
  if (iTimeout == VL53L0X_SPAD_MAX_TIMEOUT)
  {
    fprintf(stderr, "Timeout while waiting for SPAD info\n");
    return 0;
  }
  writeReg(ptr_s, 0x83,0x01);
  ucTemp = readReg(ptr_s, 0x92);
  *pCount = (ucTemp & 0x7f);
  *pTypeIsAperture = (ucTemp & 0x80);
  writeReg(ptr_s, 0x81,0x00);
  writeReg(ptr_s, 0xff,0x06);
  writeReg(ptr_s, 0x83, readReg(ptr_s, 0x83) & ~0x04);
  writeRegList(ptr_s, ucSPAD2);
  
  return 1;
} /* getSpadInfo() */

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
static uint16_t decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
static uint16_t encodeTimeout(uint16_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

static void getSequenceStepTimeouts(vl53x *ptr_s, uint8_t enables, SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = ((readReg(ptr_s, VL53L0X_PRE_RANGE_CONFIG_VCSEL_PERIOD) +1) << 1);

  timeouts->msrc_dss_tcc_mclks = readReg(ptr_s, VL53L0X_MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us =
    timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks =
    decodeTimeout(readReg16(ptr_s, VL53L0X_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us =
    timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = ((readReg(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_VCSEL_PERIOD) +1) << 1);

  timeouts->final_range_mclks =
    decodeTimeout(readReg16(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables & VL53L0X_SEQUENCE_ENABLE_PRE_RANGE)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
    timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
} /* getSequenceStepTimeouts() */

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
static int setVcselPulsePeriod(vl53x *ptr_s, vcselPeriodType type, uint8_t period_pclks)
{
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

  uint8_t enables;
  SequenceStepTimeouts timeouts;

  enables = readReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG);
  getSequenceStepTimeouts(ptr_s, enables, &timeouts);

  // "Apply specific settings for the requested clock period"
  // "Re-calculate and apply timeouts, in macro periods"

  // "When the VCSEL period for the pre or final range is changed,
  // the corresponding timeout must be read from the device using
  // the current VCSEL period, then the new VCSEL period can be
  // applied. The timeout then must be written back to the device
  // using the new VCSEL period.
  //
  // For the MSRC timeout, the same applies - this timeout being
  // dependant on the pre-range vcsel period."

  if (type == VcselPeriodPreRange)
  {
    // "Set phase check limits"
    switch (period_pclks)
    {
      case 12:
        writeReg(ptr_s, VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;

      case 14:
        writeReg(ptr_s, VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;

      case 16:
        writeReg(ptr_s, VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;

      case 18:
        writeReg(ptr_s, VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;

      default:
        // invalid period
        return 0;
    }
    writeReg(ptr_s, VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

    // apply new VCSEL period
    writeReg(ptr_s, VL53L0X_PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

    uint16_t new_pre_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

    writeReg16(ptr_s, VL53L0X_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_pre_range_timeout_mclks));

    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

    uint16_t new_msrc_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

    writeReg(ptr_s, VL53L0X_MSRC_CONFIG_TIMEOUT_MACROP,
      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

    // set_sequence_step_timeout() end
  }
  else if (type == VcselPeriodFinalRange)
  {
    switch (period_pclks)
    {
      case 8:
        writeReg(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        writeReg(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(ptr_s, VL53L0X_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        writeReg(ptr_s, VL53L0X_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        writeReg(ptr_s, 0xFF, 0x01);
        writeReg(ptr_s, VL53L0X_ALGO_PHASECAL_LIM, 0x30);
        writeReg(ptr_s, 0xFF, 0x00);
        break;

      case 10:
        writeReg(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        writeReg(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(ptr_s, VL53L0X_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ptr_s, VL53L0X_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        writeReg(ptr_s, 0xFF, 0x01);
        writeReg(ptr_s, VL53L0X_ALGO_PHASECAL_LIM, 0x20);
        writeReg(ptr_s, 0xFF, 0x00);
        break;

      case 12:
        writeReg(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        writeReg(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(ptr_s, VL53L0X_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ptr_s, VL53L0X_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        writeReg(ptr_s, 0xFF, 0x01);
        writeReg(ptr_s, VL53L0X_ALGO_PHASECAL_LIM, 0x20);
        writeReg(ptr_s, 0xFF, 0x00);
        break;

      case 14:
        writeReg(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        writeReg(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(ptr_s, VL53L0X_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ptr_s, VL53L0X_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        writeReg(ptr_s, 0xFF, 0x01);
        writeReg(ptr_s, VL53L0X_ALGO_PHASECAL_LIM, 0x20);
        writeReg(ptr_s, 0xFF, 0x00);
        break;

      default:
        // invalid period
        return 0;
    }

    // apply new VCSEL period
    writeReg(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t new_final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables & VL53L0X_SEQUENCE_ENABLE_PRE_RANGE)
    {
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    writeReg16(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
    encodeTimeout(new_final_range_timeout_mclks));

    // set_sequence_step_timeout end
  }
  else
  {
    // invalid type
    return 0;
  }

  // "Finally, the timing budget must be re-applied"

  setMeasurementTimingBudget(ptr_s, ptr_s->measurement_timing_budget_us);

  // "Perform the phase calibration. This is needed after changing on vcsel period."
  // VL53L0X_perform_phase_calibration() begin

  uint8_t sequence_config = readReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG);
  writeReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0x02);
  performSingleRefCalibration(ptr_s, 0x0);
  writeReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG, sequence_config);

  // VL53L0X_perform_phase_calibration() end

  return 1;
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
int setMeasurementTimingBudget(vl53x *ptr_s, uint32_t budget_us)
{
uint32_t used_budget_us;
uint32_t final_range_timeout_us;
uint16_t final_range_timeout_mclks;

  uint8_t enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return 0; }

  used_budget_us = StartOverhead + EndOverhead;

  enables = readReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG);
  getSequenceStepTimeouts(ptr_s, enables, &timeouts);

  if (enables & VL53L0X_SEQUENCE_ENABLE_TCC)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables & VL53L0X_SEQUENCE_ENABLE_DSS)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables & VL53L0X_SEQUENCE_ENABLE_MSRC)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables & VL53L0X_SEQUENCE_ENABLE_PRE_RANGE)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables & VL53L0X_SEQUENCE_ENABLE_FINAL_RANGE)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return 0;
    }

    final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables & VL53L0X_SEQUENCE_ENABLE_PRE_RANGE)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    writeReg16(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(final_range_timeout_mclks));

    // set_sequence_step_timeout() end

    ptr_s->measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return 1;
}

uint32_t getMeasurementTimingBudget(vl53x *ptr_s)
{
  uint8_t enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  enables = readReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG);
  getSequenceStepTimeouts(ptr_s, enables, &timeouts);

  if (enables & VL53L0X_SEQUENCE_ENABLE_TCC)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables & VL53L0X_SEQUENCE_ENABLE_DSS)
  {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables & VL53L0X_SEQUENCE_ENABLE_MSRC)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables & VL53L0X_SEQUENCE_ENABLE_PRE_RANGE)
  {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables & VL53L0X_SEQUENCE_ENABLE_FINAL_RANGE)
  {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  ptr_s->measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

int performSingleRefCalibration(vl53x *ptr_s, uint8_t vhv_init_byte)
{
  writeReg(ptr_s, VL53L0X_SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

  int iTimeout = 0;
  while ((readReg(ptr_s, VL53L0X_RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    iTimeout++;
    sleep_msec(5);
    if (iTimeout > 100) { return 0; }
  }

  writeReg(ptr_s, VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01);

  writeReg(ptr_s, VL53L0X_SYSRANGE_START, 0x00);

  return 1;
} /* performSingleRefCalibration() */

//
// Initialize the vl53l0x
//
int initSensor(vl53x *ptr_s, int bLongRangeMode)
{
  uint8_t spad_count=0, spad_type_is_aperture=0, ref_spad_map[6];
  uint8_t ucFirstSPAD, ucSPADsEnabled;
  int i;

// set 2.8V mode
  writeReg(ptr_s, VL53L0X_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
  readReg(ptr_s, VL53L0X_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0
  // Set I2C standard mode
  writeRegList(ptr_s, ucI2CMode);
  ptr_s->stop_variable = readReg(ptr_s, 0x91);
  writeRegList(ptr_s, ucI2CMode2);
  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  writeReg(ptr_s, VL53L0X_REG_MSRC_CONFIG_CONTROL, readReg(ptr_s, VL53L0X_REG_MSRC_CONFIG_CONTROL) | 0x12);
  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  writeReg16(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32); // 0.25
  writeReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0xFF);
  getSpadInfo(ptr_s, &spad_count, &spad_type_is_aperture);
  readMulti(ptr_s, VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
  //printf("initial spad map: %02x,%02x,%02x,%02x,%02x,%02x\n", ref_spad_map[0], ref_spad_map[1], ref_spad_map[2], ref_spad_map[3], ref_spad_map[4], ref_spad_map[5]);
  writeRegList(ptr_s, ucSPAD);
  ucFirstSPAD = (spad_type_is_aperture) ? 12: 0;
  ucSPADsEnabled = 0;
// clear bits for unused SPADs
  for (i=0; i<48; i++)
  {
    if (i < ucFirstSPAD || ucSPADsEnabled == spad_count)
    {
      ref_spad_map[i>>3] &= ~(1<<(i & 7));
    }
    else if (ref_spad_map[i>>3] & (1<< (i & 7)))
    {
      ucSPADsEnabled++;
    }
  } // for i
  writeMulti(ptr_s, VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
  //printf("final spad map: %02x,%02x,%02x,%02x,%02x,%02x\n", ref_spad_map[0], ref_spad_map[1], ref_spad_map[2], ref_spad_map[3], ref_spad_map[4], ref_spad_map[5]);

// load default tuning settings
  writeRegList(ptr_s, ucDefTuning); // long list of magic numbers

// change some settings for long range mode
  if (bLongRangeMode)
  {
	writeReg16(ptr_s, VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 13); // 0.1
	setVcselPulsePeriod(ptr_s, VcselPeriodPreRange, 18);
	setVcselPulsePeriod(ptr_s, VcselPeriodFinalRange, 14);
  }

// set interrupt configuration to "new sample ready"
  writeReg(ptr_s, VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  writeReg(ptr_s, VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH, readReg(ptr_s, VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
  writeReg(ptr_s, VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01);
  ptr_s->measurement_timing_budget_us = getMeasurementTimingBudget(ptr_s);
  writeReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0xe8);
  setMeasurementTimingBudget(ptr_s, ptr_s->measurement_timing_budget_us);
  writeReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0x01);

  if (!performSingleRefCalibration(ptr_s, 0x40)) { return 1; }
  writeReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!performSingleRefCalibration(ptr_s, 0x00)) { return 1; }
  writeReg(ptr_s, VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0xe8);
  return 0;
} /* initSensor() */

uint16_t readRangeContinuousMillimeters(vl53x *ptr_s)
{
int iTimeout = 0;
uint16_t range;

  while ((readReg(ptr_s, VL53L0X_RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    iTimeout++;
    sleep_msec(50);
    if (iTimeout > 50)
    {
      return -1;
    }
  }

  // assumptions: Linearity Corrective Gain is 1000 (default);
  // fractional ranging is not enabled
  range = readReg16(ptr_s, VL53L0X_RESULT_RANGE_STATUS + 10);

  writeReg(ptr_s, VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01);

  return range;
}
//
// Read the current distance in mm
//
uint32_t tofReadDistance(vl53x *sensor)
{
int iTimeout;

  writeReg(sensor, 0x80, 0x01);
  writeReg(sensor, 0xFF, 0x01);
  writeReg(sensor, 0x00, 0x00);
  writeReg(sensor, 0x91, sensor->stop_variable);
  writeReg(sensor, 0x00, 0x01);
  writeReg(sensor, 0xFF, 0x00);
  writeReg(sensor, 0x80, 0x00);

  writeReg(sensor, VL53L0X_SYSRANGE_START, 0x01);

  // "Wait until start bit has been cleared"
  iTimeout = 0;
  while (readReg(sensor, VL53L0X_SYSRANGE_START) & 0x01)
  {
    iTimeout++;
    sleep_msec(50);
    if (iTimeout > 50)
    {
      return -1;
    }
  }

  return readRangeContinuousMillimeters(sensor);

} /* tofReadDistance() */

int tofGetModel(vl53x *sensor, uint8_t *model, uint8_t *revision)
{
uint8_t ucTemp[2];
int i;

	if (model)
	{
		i = iic_read_register(sensor->iic_index, sensor->baseAddr, VL53L0X_REG_IDENTIFICATION_MODEL_ID, ucTemp, 1);
		if (i == 0) //0 on succes aka no error
			*model = ucTemp[0];
    else
      return 1;
	}
	if (revision)
	{
      i = iic_read_register(sensor->iic_index, sensor->baseAddr, VL53L0X_REG_IDENTIFICATION_REVISION_ID, ucTemp, 1);
		if (i == 0)
			*revision = ucTemp[0];
    else
      return 1;
	}
	return 0;

} /* tofGetModel() */

/* =====================================================================
 *  INLINED IMPLEMENTATION -- sensors.c  (ToF / IR / colour / temperature)
 * ===================================================================== */
/*
 *  TU/e 5EID0 - Venus Project, Group 10
 *  Embedded Software - Sensor Module Implementation
 *
 *  - ToF (3x VL53L0X)        : fully implemented (uses the working
 *                              XSHUT-staggered bring-up sequence)
 *  - IR  (2x TCRT5000)       : skeleton, GPIO pin numbers are TODO
 *  - Colour (TCS3200)        : skeleton, frequency counting is TODO
 *  - Temperature (NTC-10K)   : ADC read skeleton + full Celsius math
 *
 *  Pin assignments are gathered here at the top so they are easy to
 *  edit when the electronics sub-team finalises wiring.
 */

/* =====================================================================
 * Pin map (edit here, not scattered through the code)
 * ===================================================================== */

/* --- VL53L0X ToF --- (ROBOT 2 wiring) */
#define TOF_XSHUT_B   IO_AR5   /* XSHUT of ToF sensor B (middle) -> AR5 */
#define TOF_XSHUT_C   IO_AR4   /* XSHUT of ToF sensor C (bottom) -> AR4 */
/* Sensor A's XSHUT is tied high on the board (always on) -> top.
 * Mapping below: A->top, B->middle, C->bottom (verify in sensortest). */

/* --- TCRT5000 IR tape detectors --- (ROBOT 2 wiring) */
#define IR_LEFT_PIN   IO_AR9    /* IR left  DO -> AR9  */
#define IR_RIGHT_PIN  IO_AR10   /* IR right DO -> AR10 */

/* --- TCS3200 colour --- (ROBOT 2 wiring; S0/S1 now wired) */
#define COLOR_S0_PIN  IO_AR11  /* output frequency-scaling select S0 -> AR11 */
#define COLOR_S1_PIN  IO_AR12  /* output frequency-scaling select S1 -> AR12 */
#define COLOR_S2_PIN  IO_AR7   /* photodiode filter-select line S2 -> AR7 */
#define COLOR_S3_PIN  IO_AR8   /* photodiode filter-select line S3 -> AR8 */
#define COLOR_OUT_PIN IO_AR6   /* frequency output -> AR6 */
#define COLOR_SETTLE_MS 10 /* filter settle after S2/S3 flip */

/* Pulse-width method + map_to_rgb calibration (raw HIGH-pulse loop counts).
 * Per channel: <= WHITE -> 255, >= BLACK -> 0, linear in between.
 * Measured 2026-06-18 (debug_color, ~2 cm, mission lighting):
 * white block (10,13,12), black block (17,32,30).
 * Verified red/green/blue all classify correctly with these. */
#define COL_R_WHITE  10u
#define COL_R_BLACK  17u
#define COL_G_WHITE  13u
#define COL_G_BLACK  32u
#define COL_B_WHITE  12u
#define COL_B_BLACK  30u

/* Guard so a stuck OUT pin can't hang the read (returns 0 on timeout). */
#define COL_PULSE_TIMEOUT 1000000u

/* --- NTC-10K thermistor --- (confirmed on ADC5 / A5) */
#define TEMP_ADC_CHANNEL ADC5

/* NTC Beta-equation constants (matches design report eq. 4.1). */
#define NTC_R0      10000.0f
#define NTC_T0_K    298.15f
#define NTC_BETA    4050.0f
#define NTC_SERIES  10000.0f
#define NTC_VREF    3.3f
#define ADC_MAX     4095.0f   /* 12-bit ADC assumed; adjust if wrong   */

/* ToF bring-up addresses. Each sensor ends up at a unique address; we use the
 * team convention (0x68/0x69/0x6A) so sensors left powered between software runs
 * are already where we expect them. */
#define TOF_ADDR_DEFAULT 0x29
#define TOF_ADDR_A       0x68   /* top    (always-on, XSHUT tied high) */
#define TOF_ADDR_B       0x69   /* middle (XSHUT on AR5) */
#define TOF_ADDR_C       0x6A   /* bottom (XSHUT on AR4) */

#define XSHUT_SETTLE_MS 100

/* =====================================================================
 * Module state (static so it is not visible outside this file)
 * ===================================================================== */

static vl53x s_tof_a, s_tof_b, s_tof_c;
static bool  s_tof_a_ok = false, s_tof_b_ok = false, s_tof_c_ok = false;
static bool  s_tof_ready = false;
static bool  s_ir_ready    = false;
static bool  s_color_ready = false;
static bool  s_temp_ready  = false;

/* =====================================================================
 * Internal helpers: VL53L0X bring-up.
 *
 * KEY: the sensors keep their reassigned I2C address as long as they stay
 * powered, so a plain software restart does NOT put them back at 0x29. The
 * bring-up therefore NEVER assumes 0x29 and NEVER pauses/fails the system --
 * it finds each sensor wherever it currently answers, (re)assigns it to its
 * target, and continues regardless.
 * ===================================================================== */

/* Addresses a freshly-powered VL53L0X could be sitting at, in probe order. */
static const uint8_t TOF_ADDR_CANDIDATES[] = {
    TOF_ADDR_DEFAULT,                       /* 0x29 fresh / after a power-cycle */
    TOF_ADDR_A, TOF_ADDR_B, TOF_ADDR_C,     /* our targets (this code, prior run) */
    0x30, 0x31, 0x32                        /* older-firmware targets, just in case */
};

/* Always-on sensor (XSHUT tied high): we can't reset it to 0x29, so find it at
 * whatever address it answers (it's the ONLY device on the bus while B/C are
 * held off) and move it to target. Idempotent + non-fatal. */
/* Try to bring a sensor that answers at cur_addr up at target_addr, printing
 * every step so we can see exactly where it fails. Returns true on success. */
static bool tof_try_at(const char *label, uint8_t cur_addr, uint8_t target_addr, vl53x *handle)
{
    if (cur_addr != target_addr) {
        const int s = tofSetAddress(IIC0, cur_addr, target_addr);
        printf("[%s]   setAddress 0x%02X -> 0x%02X = %d\n", label, cur_addr, target_addr, s);
        fflush(NULL);
    }
    const int r = tofInit(handle, IIC0, target_addr, 0);
    printf("[%s]   tofInit 0x%02X = %d\n", label, target_addr, r);
    fflush(NULL);
    return (r == 0);
}

static bool tof_bring_up_always_on(const char *label, uint8_t target_addr, vl53x *handle)
{
    for (size_t i = 0; i < sizeof(TOF_ADDR_CANDIDATES) / sizeof(TOF_ADDR_CANDIDATES[0]); ++i) {
        const uint8_t a = TOF_ADDR_CANDIDATES[i];
        const int p = tofPing(IIC0, a);
        printf("[%s] ping 0x%02X = %d\n", label, a, p);
        fflush(NULL);
        if (p != 0) continue;
        if (tof_try_at(label, a, target_addr, handle)) {
            printf("[%s] READY at 0x%02X\n", label, target_addr);
            fflush(NULL);
            return true;
        }
    }
    printf("[%s] offline (skipped)\n", label);
    fflush(NULL);
    return false;
}

static bool tof_bring_up_xshut(const char *label, uint8_t target_addr, vl53x *handle)
{
    const int p29 = tofPing(IIC0, TOF_ADDR_DEFAULT);
    printf("[%s] ping 0x29 = %d\n", label, p29);
    fflush(NULL);
    if (p29 == 0 && tof_try_at(label, TOF_ADDR_DEFAULT, target_addr, handle)) {
        printf("[%s] READY at 0x%02X\n", label, target_addr);
        fflush(NULL);
        return true;
    }
    const int pt = tofPing(IIC0, target_addr);
    printf("[%s] ping 0x%02X = %d\n", label, target_addr, pt);
    fflush(NULL);
    if (pt == 0 && tof_try_at(label, target_addr, target_addr, handle)) {
        printf("[%s] READY at 0x%02X (already)\n", label, target_addr);
        fflush(NULL);
        return true;
    }
    printf("[%s] offline (skipped)\n", label);
    fflush(NULL);
    return false;
}

/* =====================================================================
 * Time-of-Flight
 * ===================================================================== */

int tof_init(void)
{
    /* Switchbox + GPIO direction for the XSHUT lines on B and C. */
    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
    switchbox_set_pin(TOF_XSHUT_B, SWB_GPIO);   /* AR5 (middle) */
    switchbox_set_pin(TOF_XSHUT_C, SWB_GPIO);   /* AR4 (bottom) */

    gpio_set_direction(TOF_XSHUT_B, GPIO_DIR_OUTPUT);
    gpio_set_direction(TOF_XSHUT_C, GPIO_DIR_OUTPUT);

    /* Isolate A: hold B and C in shutdown (XSHUT low resets them to 0x29
     * when later re-enabled, so any stale address on B/C is cleared). */
    gpio_set_level(TOF_XSHUT_B, GPIO_LEVEL_LOW);
    gpio_set_level(TOF_XSHUT_C, GPIO_LEVEL_LOW);
    sleep_msec(30);

    iic_init(IIC0);

    /* A = top, always-on. B/C are held off, so A is the only device on the bus;
     * find it wherever it is and move it to TOF_ADDR_A. */
    s_tof_a_ok = tof_bring_up_always_on("ToF-A", TOF_ADDR_A, &s_tof_a);

    /* B = middle: wake via AR5 (resets it to 0x29), then assign TOF_ADDR_B. */
    gpio_set_level(TOF_XSHUT_B, GPIO_LEVEL_HIGH);
    sleep_msec(XSHUT_SETTLE_MS);
    s_tof_b_ok = tof_bring_up_xshut("ToF-B", TOF_ADDR_B, &s_tof_b);

    /* C = bottom: wake via AR4 (resets it to 0x29), then assign TOF_ADDR_C. */
    gpio_set_level(TOF_XSHUT_C, GPIO_LEVEL_HIGH);
    sleep_msec(XSHUT_SETTLE_MS);
    s_tof_c_ok = tof_bring_up_xshut("ToF-C", TOF_ADDR_C, &s_tof_c);

    s_tof_ready = s_tof_a_ok || s_tof_b_ok || s_tof_c_ok;
    printf("[ToF] online: A=%d B=%d C=%d\n", s_tof_a_ok, s_tof_b_ok, s_tof_c_ok);
    fflush(NULL);

    /* OK as long as at least one sensor came up. */
    return s_tof_ready ? 0 : 1;
}

/*
 * Physical-to-logical mapping for the vertical ToF stack (confirmed by test).
 * EDIT these three assignments if the layout changes.
 *   A -> top     (mountains)
 *   B -> middle  (large 6x6 rocks)
 *   C -> bottom  (small 3x3 rocks)
 */
void tof_read(uint16_t *bottom, uint16_t *middle, uint16_t *top)
{
    /* Read only the sensors that actually came up; others report 0 (invalid). */
    uint32_t dA = s_tof_a_ok ? tofReadDistance(&s_tof_a) : (uint32_t)-1;
    uint32_t dB = s_tof_b_ok ? tofReadDistance(&s_tof_b) : (uint32_t)-1;
    uint32_t dC = s_tof_c_ok ? tofReadDistance(&s_tof_c) : (uint32_t)-1;

    /* tofReadDistance returns (uint32_t)-1 on error. Map that to 0
     * so the main loop's "0 == invalid" convention works.            */
    if (dA == (uint32_t)-1) dA = 0;
    if (dB == (uint32_t)-1) dB = 0;
    if (dC == (uint32_t)-1) dC = 0;

    if (top)    *top    = (uint16_t)(dA > 65535 ? 65535 : dA);   /* A -> top    */
    if (middle) *middle = (uint16_t)(dB > 65535 ? 65535 : dB);   /* B -> middle */
    if (bottom) *bottom = (uint16_t)(dC > 65535 ? 65535 : dC);   /* C -> bottom */
}

/* =====================================================================
 * IR tape detectors (TCRT5000)
 * Most breakouts expose a digital "DO" pin that goes LOW when the IR
 * is reflected back (light surface) and HIGH on dark surface (= tape).
 * Verify polarity with the electronics team.
 * ===================================================================== */

int ir_init(void)
{
    switchbox_set_pin(IR_LEFT_PIN,  SWB_GPIO);   /* AR9  */
    switchbox_set_pin(IR_RIGHT_PIN, SWB_GPIO);   /* AR10 */
    gpio_set_direction(IR_LEFT_PIN,  GPIO_DIR_INPUT);
    gpio_set_direction(IR_RIGHT_PIN, GPIO_DIR_INPUT);

    s_ir_ready = true;
    return 0;
}

void ir_read(bool *left_tape, bool *right_tape)
{
    if (!s_ir_ready) {
        if (left_tape)  *left_tape  = false;
        if (right_tape) *right_tape = false;
        return;
    }

    /* TODO: confirm active-high vs active-low. Currently assumes HIGH
     * on DO = tape (dark surface = no reflection).                    */
    if (left_tape)
        *left_tape  = (gpio_get_level(IR_LEFT_PIN)  == GPIO_LEVEL_HIGH);
    if (right_tape)
        *right_tape = (gpio_get_level(IR_RIGHT_PIN) == GPIO_LEVEL_HIGH);
}

/* =====================================================================
 * Colour sensor (TCS3200)
 * Frequency-output sensor. S2/S3 select the filter:
 *      S2 S3   Filter
 *       0  0   Red
 *       0  1   Blue
 *       1  0   Clear (no filter)
 *       1  1   Green
 * OUT pin emits a square wave whose frequency is proportional to the
 * light intensity through the selected filter. Count edges over a
 * fixed window to get a "frequency" reading.
 * ===================================================================== */

static void color_select_filter(int s2, int s3)
{
    gpio_set_level(COLOR_S2_PIN, s2 ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW);
    gpio_set_level(COLOR_S3_PIN, s3 ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW);
}

/*
 * Measure one HIGH-pulse duration on OUT in loop counts. Longer pulse =
 * lower frequency = less light through the selected filter. Guarded against a
 * stuck pin so it can never hang the caller; returns 0 on timeout.
 */
static uint32_t color_measure_pulse(void)
{
    uint32_t guard = 0;
    while (gpio_get_level(COLOR_OUT_PIN) == GPIO_LEVEL_HIGH) {
        if (++guard > COL_PULSE_TIMEOUT) return 0;
    }
    guard = 0;
    while (gpio_get_level(COLOR_OUT_PIN) == GPIO_LEVEL_LOW) {
        if (++guard > COL_PULSE_TIMEOUT) return 0;
    }
    uint32_t count = 0;
    while (gpio_get_level(COLOR_OUT_PIN) == GPIO_LEVEL_HIGH) {
        if (++count > COL_PULSE_TIMEOUT) return 0;
    }
    return count;
}

/* Raw pulse count -> 0..255 using per-channel white/black calibration. */
static int color_map_to_rgb(uint32_t raw, uint32_t white, uint32_t black)
{
    if (raw <= white) return 255;
    if (raw >= black) return 0;
    const float intensity = (float)(black - raw) / (float)(black - white);
    return (int)(intensity * 255.0f);
}

int color_init(void)
{
    switchbox_set_pin(COLOR_S0_PIN,  SWB_GPIO);   /* AR11 */
    switchbox_set_pin(COLOR_S1_PIN,  SWB_GPIO);   /* AR12 */
    switchbox_set_pin(COLOR_S2_PIN,  SWB_GPIO);   /* AR7 */
    switchbox_set_pin(COLOR_S3_PIN,  SWB_GPIO);   /* AR8 */
    switchbox_set_pin(COLOR_OUT_PIN, SWB_GPIO);   /* AR6 */
    gpio_set_direction(COLOR_S0_PIN,  GPIO_DIR_OUTPUT);
    gpio_set_direction(COLOR_S1_PIN,  GPIO_DIR_OUTPUT);
    gpio_set_direction(COLOR_S2_PIN,  GPIO_DIR_OUTPUT);
    gpio_set_direction(COLOR_S3_PIN,  GPIO_DIR_OUTPUT);
    gpio_set_direction(COLOR_OUT_PIN, GPIO_DIR_INPUT);

    /* Output frequency scaling via S0/S1: 20% (S0=HIGH, S1=LOW). Slower pulses
     * = more loop counts in color_measure_pulse() = better resolution for the
     * busy-loop method. (S0=L,S1=L powers the sensor DOWN -- never do that.)
     * NOTE: this scaling sets the raw counts, so the white/black calibration
     * (COL_*_WHITE / COL_*_BLACK) MUST be redone for this robot. */
    gpio_set_level(COLOR_S0_PIN, GPIO_LEVEL_HIGH);
    gpio_set_level(COLOR_S1_PIN, GPIO_LEVEL_LOW);

    s_color_ready = true;
    return 0;
}

void color_read(uint32_t *r, uint32_t *g, uint32_t *b)
{
    if (!s_color_ready) {
        if (r) *r = 0;
        if (g) *g = 0;
        if (b) *b = 0;
        return;
    }

    color_select_filter(0, 0);                          /* Red    */
    sleep_msec(COLOR_SETTLE_MS);
    uint32_t fr = color_measure_pulse();

    color_select_filter(1, 1);                          /* Green  */
    sleep_msec(COLOR_SETTLE_MS);
    uint32_t fg = color_measure_pulse();

    color_select_filter(0, 1);                          /* Blue   */
    sleep_msec(COLOR_SETTLE_MS);
    uint32_t fb = color_measure_pulse();

    if (r) *r = fr;   /* raw HIGH-pulse counts (not Hz) */
    if (g) *g = fg;
    if (b) *b = fb;
}

/* ---------------------------------------------------------------------
 * Colour classification: raw pulse counts -> 0..255 RGB (via the white/black
 * calibration), then nearest palette colour by Euclidean distance. Includes
 * WHITE and BLACK so neutral surfaces classify instead of returning NONE.
 * ------------------------------------------------------------------- */
scan_color_t color_classify(uint32_t r_raw, uint32_t g_raw, uint32_t b_raw, float *confidence)
{
    /* All-zero raw = pulse-measure timeout / dead OUT pin. Report NONE rather
     * than letting it map to (255,255,255) and masquerade as a WHITE rock. */
    if (r_raw == 0 && g_raw == 0 && b_raw == 0) {
        if (confidence) *confidence = 0.0f;
        return SCAN_COLOR_NONE;
    }

    const int r = color_map_to_rgb(r_raw, COL_R_WHITE, COL_R_BLACK);
    const int g = color_map_to_rgb(g_raw, COL_G_WHITE, COL_G_BLACK);
    const int b = color_map_to_rgb(b_raw, COL_B_WHITE, COL_B_BLACK);

    /* Anchors are the MEASURED normalised RGB of each real cube (debug_color
     * 2026-06-18), NOT idealised corners: at ~2 cm the cubes read desaturated,
     * so pure-corner anchors collapse every cube onto WHITE. No yellow cube in
     * this mission, so yellow is intentionally absent from the palette. */
    static const struct { scan_color_t label; int r, g, b; } palette[] = {
        { SCAN_COLOR_RED,    255, 161, 170 },
        { SCAN_COLOR_GREEN,  182, 174, 127 },
        { SCAN_COLOR_BLUE,   255, 174, 212 },
        { SCAN_COLOR_WHITE,  255, 255, 255 },
        { SCAN_COLOR_BLACK,    0,   0,   0 },
    };
    const int n = (int)(sizeof(palette) / sizeof(palette[0]));

    int best = 0;
    long best_d = 0x7fffffffL;
    for (int i = 0; i < n; ++i) {
        const long dr = (long)r - palette[i].r;
        const long dg = (long)g - palette[i].g;
        const long db = (long)b - palette[i].b;
        const long d = dr * dr + dg * dg + db * db;
        if (d < best_d) { best_d = d; best = i; }
    }

    if (confidence) {
        const float far = 3.0f * 255.0f * 255.0f;   /* corner-to-corner */
        *confidence = 1.0f - (float)best_d / far;
    }
    return palette[best].label;
}

scan_color_t color_read_label(float *confidence)
{
    uint32_t r = 0, g = 0, b = 0;
    color_read(&r, &g, &b);
    return color_classify(r, g, b, confidence);
}

const char *color_label_name(scan_color_t label)
{
    switch (label) {
        case SCAN_COLOR_RED:    return "red";
        case SCAN_COLOR_GREEN:  return "green";
        case SCAN_COLOR_BLUE:   return "blue";
        case SCAN_COLOR_YELLOW: return "yellow";
        case SCAN_COLOR_WHITE:  return "white";
        case SCAN_COLOR_BLACK:  return "black";
        case SCAN_COLOR_NONE:
        default:                return "none";
    }
}

/* =====================================================================
 * NTC-10K temperature
 * Voltage divider: 3.3V -- 10k(fixed) -- Vout -- NTC -- GND
 * Vout sampled through unity-gain op-amp buffer into the ADC.
 * ===================================================================== */

int temp_init(void)
{
    /* NTC divider buffered into ADC5. Matches the working stand-alone test
     * (which also called buttons_init alongside adc_init). */
    adc_init();
    buttons_init();
    s_temp_ready = true;
    return 0;
}

/*
 * Read the NTC and return degrees Celsius.
 * adc_read_channel() returns the channel VOLTAGE directly. Divider is
 *   3.3V -- 10k(fixed) -- Vout -- NTC -- GND, so R_ntc = R2 * Vout/(Vref-Vout),
 * then the Beta equation gives temperature.
 */
float temp_read_celsius(void)
{
    if (!s_temp_ready) return -273.15f;

    const float v_out = (float)adc_read_channel(TEMP_ADC_CHANNEL);

    /* Guard against rail values that would blow up the maths. */
    if (v_out <= 0.001f || v_out >= (NTC_VREF - 0.001f)) {
        return -273.15f;
    }

    const float r_ntc = NTC_SERIES * (v_out / (NTC_VREF - v_out));
    const float inv_T = (1.0f / NTC_T0_K)
                      + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R0);
    return (1.0f / inv_T) - 273.15f;
}

/* =====================================================================
 * Top-level wrappers used by the robot main loop
 * ===================================================================== */

int sensors_init_all(void)
{
    int err = 0;

    gpio_init();

    if (tof_init()   != 0) { printf("ToF init failed\n");    err |= 1; }
    if (ir_init()    != 0) { printf("IR init failed\n");     err |= 2; }
    if (color_init() != 0) { printf("Colour init failed\n"); err |= 4; }
    if (temp_init()  != 0) { printf("Temp init failed\n");   err |= 8; }

    return err;
}

void update_sensor_data(SensorData *out)
{
    if (out == NULL) return;

    tof_read(&out->tof_bottom_mm,
             &out->tof_middle_mm,
             &out->tof_top_mm);

    ir_read(&out->ir_left_detects_tape,
            &out->ir_right_detects_tape);

    color_read(&out->color_freq_r,
               &out->color_freq_g,
               &out->color_freq_b);

    out->temp_celsius = temp_read_celsius();
}

/* =====================================================================
 *  INLINED IMPLEMENTATION -- mqtt.c  (partner-robot / mission-control link)
 * ===================================================================== */
/*
 *  Partner-robot comms over UART0 (AR0/AR1) -> ESP32 -> MQTT broker.
 *  Wire format: 4-byte little-endian length, then a JSON payload.
 *
 *  Debug output goes to stderr (the boot log); the framed JSON payloads are the
 *  only thing put on the wire.
 */

void init_mqtt(void) {
    switchbox_set_pin(IO_AR1, SWB_UART0_TX);
    switchbox_set_pin(IO_AR0, SWB_UART0_RX);
    uart_init(UART0);
    /* NOTE: do NOT call switchbox_init() here -- it resets the WHOLE switchbox
     * to defaults, wiping the UART pins just set above plus every sensor/stepper
     * routing. robot_main sets UART0 up itself and never calls this, but the
     * stray reset made init_mqtt a landmine for anyone who did. */
}

void mqtt_publish_json(const char* json) {
    uint32_t len = (uint32_t)strlen(json);

    uart_send(UART0, (len >> 0) & 0xFF);
    uart_send(UART0, (len >> 8) & 0xFF);
    uart_send(UART0, (len >> 16) & 0xFF);
    uart_send(UART0, (len >> 24) & 0xFF);

    for (uint32_t i = 0; i < len; i++) {
        uart_send(UART0, json[i]);
    }

    fprintf(stderr, "[mqtt] sent (%u bytes): %s\n", len, json);
}

void send_mqtt(int x, int y, const char* color, int size, float temp) {
    char json_payload[160];

    snprintf(json_payload, sizeof(json_payload),
             "{\"type\":\"ROCK\",\"x\":\"%i\",\"y\":\"%i\",\"color\":\"%s\","
             "\"size\":%d,\"temp\":%.2f}",
             x, y, color, size, temp);
    mqtt_publish_json(json_payload);
}

/*
 * NON-BLOCKING. Drains the bytes currently waiting on UART0 into a persistent
 * buffer, then extracts ONE complete JSON object if a whole one has arrived.
 *
 * It locates the object by scanning for '{' .. '}' rather than trusting a fixed
 * length header, so it works whether or not the pynqbridge keeps the 4-byte
 * length prefix in front of the payload (any leading framing bytes before '{'
 * are simply skipped). This mirrors how mission_control.py finds the JSON, and
 * is what lets robot 1 reliably ingest robot 2's relayed cube reports.
 *
 * Our messages are flat (no nested braces, no '}' inside string values), so the
 * first '}' closes the object. Returns true once per parsed object; call it in a
 * `while (recv_mqtt(...))` loop to drain every queued frame. color_buf is set
 * only for messages that carry a "color" (i.e. ROCK cube reports); it is cleared
 * otherwise, so the caller can ignore relayed pose/status/object messages.
 */
bool recv_mqtt(int* xz, int* yz, int* sizez, float* tempz, char* color_buf) {
    static uint8_t buf[2048];
    static size_t  buf_len = 0;

    /* Pull in whatever is waiting -- bounded, never blocks. */
    while (uart_has_data(UART0) && buf_len < sizeof(buf)) {
        buf[buf_len++] = uart_recv(UART0);
    }
    if (buf_len == 0) return false;

    /* Find the start of a JSON object (skip any length/framing bytes). */
    size_t s = 0;
    while (s < buf_len && buf[s] != '{') s++;
    if (s >= buf_len) { buf_len = 0; return false; }   /* only noise -> drop it */

    /* Find the end of the (flat) object. */
    size_t e = s + 1;
    while (e < buf_len && buf[e] != '}') e++;
    if (e >= buf_len) {                                /* not fully arrived yet */
        if (s > 0) { memmove(buf, buf + s, buf_len - s); buf_len -= s; }  /* drop junk */
        else if (buf_len == sizeof(buf)) buf_len = 0;  /* overflow w/o close -> reset */
        return false;
    }

    const size_t json_len = e - s + 1;
    char json_buffer[json_len + 1];
    memcpy(json_buffer, buf + s, json_len);
    json_buffer[json_len] = '\0';

    /* Consume everything through this object (incl. any leading framing bytes). */
    const size_t consumed = e + 1;
    memmove(buf, buf + consumed, buf_len - consumed);
    buf_len -= consumed;

    /* Parse -- tolerant of quoted ("x":"15") or bare ("x":15) values. */
    int x = 0, y = 0, size = 0;
    float temp = 0.0f;
    char *p;
    color_buf[0] = '\0';   /* only ROCK carries colour; cleared so non-ROCK is ignored */

    if ((p = strstr(json_buffer, "\"x\":")))    { p += 4; if (*p == '"') p++; x = atoi(p); }
    if ((p = strstr(json_buffer, "\"y\":")))    { p += 4; if (*p == '"') p++; y = atoi(p); }
    if ((p = strstr(json_buffer, "\"size\":"))) { p += 7; if (*p == '"') p++; size = atoi(p); }
    if ((p = strstr(json_buffer, "\"temp\":"))) { p += 7; if (*p == '"') p++; temp = (float)atof(p); }

    /* Treat this as a partner CUBE only if it is an explicit ROCK message. The
     * caller (poll_partner_mqtt) registers a cube only when color_buf is set, so
     * relayed pose/status/object frames (no ROCK, no colour) are ignored -- even
     * if a status "detail" string happened to contain the substring "color". */
    if (strstr(json_buffer, "\"ROCK\"") &&
        (p = strstr(json_buffer, "\"color\":\""))) {
        char *start = p + 9;
        char *end = strchr(start, '\"');
        if (end) {
            size_t color_len = (size_t)(end - start);
            if (color_len > 31) color_len = 31;
            strncpy(color_buf, start, color_len);
            color_buf[color_len] = '\0';
        }
    }

    fprintf(stderr, "[mqtt] rx frame: x=%d y=%d color=%s size=%d temp=%.2f\n",
            x, y, color_buf, size, temp);

    *xz = x;
    *yz = y;
    *sizez = size;
    *tempz = temp;
    return true;
}

/* =====================================================================
 *  ROBOT FIRMWARE  (was main.c) -- autonomous, MQTT-only run-on-boot
 * ===================================================================== */

#define AS_SECONDS(time_ticks) ((time_ticks) / 1.0e8)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef ROBOT_ID
#define ROBOT_ID "r1"
#endif

/* Fixed start pose, defining the shared world frame for BOTH robots. The origin
 * (0,0) is where the robot is switched on; the two robots start right next to each
 * other so they share that origin. Their headings are FIXED by placement: robot 1
 * (this file) faces +y (theta = +pi/2); robot 2 (main2.c) faces -y (theta = -pi/2).
 * Because both then integrate odometry in the same world frame, their cube
 * coordinates line up directly -- that is what makes the UI map and the
 * cross-robot dedup work with no per-robot transform. */
#ifndef START_X_M
#define START_X_M 0.0
#endif
#ifndef START_Y_M
#define START_Y_M 0.0
#endif
#ifndef START_THETA_RAD
#define START_THETA_RAD (M_PI / 2.0)   /* +pi/2: robot 1 looks along +y */
#endif

/* ===================== Pin map -- ROBOT 2 ==========================
 * sensors.c owns the sensor pins:
 *     IIC0 (ToF) .... IO_AR_SCL / IO_AR_SDA
 *     ToF XSHUT ..... AR5 (B = middle), AR4 (C = bottom); A (top) tied high @0x29
 *     TCS3200 ....... AR6 (OUT), AR7 (S2), AR8 (S3), AR11 (S0), AR12 (S1)
 *     IR tape ....... AR9 (left), AR10 (right)
 *
 * Comms now go ONLY over the board UART0 (AR0/AR1 -> ESP32 -> MQTT broker); the
 * stdio UI link was removed for the run-on-boot build. Odometry is dead-reckoning,
 * so the pulse-counter pins below are unused; they default to AR2/AR13 (both free
 * on robot 2) only so the disabled odometry block compiles without clashing with
 * the sensor pins (AR4-AR12).
 * =================================================================== */
#define LEFT_PULSE_PIN  IO_AR2    /* unused (odometry off); free pin */
#define RIGHT_PULSE_PIN IO_AR13   /* unused (odometry off); free pin */

/* Encoders aren't wired, so default to dead-reckoning: pose is integrated from
 * the COMMANDED steps. Set this to 1 only if real wheel encoders are added on
 * free pins (and update LEFT/RIGHT_PULSE_PIN to match). */
#define USE_PULSECOUNTER_ODOMETRY 0

/* --------------------------- Calibration --------------------------- *
 * Matches the measured wheel library: 1600 steps/rev, 23.56 cm wheel
 * circumference (=> Ø 7.5 cm), 12 cm wheel track. With these, this file's
 * meters_to_steps / update_odometry produce the exact same motion + pose
 * as your move_and_track / update_location.                             */
#define STEPS_PER_REV 1600.0
#define MICROSTEP_FACTOR 1.0
#define WHEEL_DIAMETER_M 0.075   /* 23.56 cm circumference / pi */
#define WHEEL_BASE_M 0.12        /* track */

#define CELL_M 0.03              /* 3 cm/cell: small cube = 1 cell, big cube = 2x2 */
#define BACKUP_M 0.08
#define TAPE_REPORT_OFFSET_M 0.05
#define HEADING_TOLERANCE_RAD 0.18

/* The robot publishes its telemetry to mission control over the MQTT/UART0 link
 * so that PC can map our location/orientation and every object. Publishes are
 * rate-limited so the UART link to the ESP32 isn't flooded -- mission control
 * only needs a steady trickle. */
#define POSE_MQTT_MIN_MS    400  /* min gap between pose publishes over MQTT      */
#define STATUS_MQTT_MIN_MS  600  /* min gap between status publishes over MQTT    */
/* Objects (mountains/tape) are de-duplicated by position, not time, so two
 * DISTINCT objects seen close together are never dropped (see send_observation). */
#define PUB_OBJ_MAX         128  /* distinct object cells remembered for the MQTT publish */

/* Stepper pulse delay in ticks (larger = slower). Matches the wheel
 * library's tested speeds: 20000 driving, 25000 turning. */
#define STEPPER_PULSE_DELAY_TICKS 20000
#define STEPPER_TURN_PULSE_DELAY_TICKS 25000

#define LOOP_SLEEP_MS 20
#define POSE_REPORT_PERIOD_MS 500
#define STATUS_REPORT_PERIOD_MS 2000

#define MOUNTAIN_STOP_M 0.15

/* --------------------- Autopilot (frontier + SLAM) ----------------- *
 * Mirrors the UI auto-pilot: build an occupancy grid from the forward
 * sensors, drive to the nearest frontier (free cell next to unknown), and
 * react to what the sensors see:
 *   - top ToF  -> mountains (boundaries): map them; if one is in the way, avoid.
 *   - IR tape  -> boundary in the way: avoid.
 *   - bottom ToF -> small 3x3 rock; bottom+middle ToF -> large 6x6 (within ROCK_DETECT_M,
 *     MEASURED ~6-11 cm). Approach, read colour, report the sample, then re-plan.
 * Bottom/middle readings beyond ROCK_DETECT_M are floor/mountain, not a cube.
 *
 * Mission layer on top of that (see the located-cube registry below):
 *   - On boot it scans in place and approaches the first cube it sees.
 *   - Every NEW cube is logged locally and shared with the partner / mission
 *     control over MQTT; a cube already located (by us or the partner) is
 *     ignored instead of re-scanned.
 *   - Hitting a boundary/crater (black tape) backtracks, re-scans, and -- if
 *     nothing new is found -- roams on instead of freezing.
 *   - Once all six cubes are known, the robot stops; since finds are shared,
 *     both robots stop together.                                            */
#define GRID_W 40                /* 40 * 3cm = 1.2 m square map (~1m arena + margin) */
#define GRID_H 40
#define GRID_CX 20               /* world (0,0) -> grid centre */
#define GRID_CY 20
#define ROCK_DETECT_M       0.12 /* bottom/middle ToF range that counts as a cube.
                                  * MEASURED (debug_tof): a cube in front reads ~6-11 cm on
                                  * bottom (and middle too for a 6x6); a mountain reads >=17 cm
                                  * there and lights the TOP beam instead, so 12 cm cleanly
                                  * separates cubes from mountains. */
#define ROCK_APPROACH_M     0.02 /* final gap: stop ~this close to the cube. The ToF cannot
                                  * read the last few cm (short-range floor ~6 cm), so once the
                                  * cube is within COLOR_ALIGN_DIST_M the final approach is driven
                                  * OPEN-LOOP (commanded distance), not by waiting for a near read. */
#define COLOR_ALIGN_DIST_M    0.10   /* final-approach trigger: once the cube reads within this, drive the rest OPEN-LOOP. Kept above the ToF ~6cm short-range floor so it triggers reliably */
#define COLOR_SENSOR_OFFSET_M 0.02   /* colour sensor sits this far RIGHT of the bottom ToF; strafe LEFT this much to align it onto the cube */
#define CUBE_CONFIRM_TOL_M    0.05   /* burst reads must agree within this (m) */
#define CUBE_CONFIRM_READS     6      /* reads taken to confirm a cube candidate */
#define CUBE_CONFIRM_MIN_HITS  2      /* ...of which this many must agree to commit */
#define MOUNTAIN_MAP_MAX_M  0.45 /* map mountains the top ToF sees up to this range  */
/* Top ToF detection band. The top sensor is angled down, so on EMPTY floor it
 * reads its far baseline (measured ~0.57 m); a real mountain blocks the beam and
 * reads CLOSER than that. So a reading is only a mountain inside [MIN,MAX], where
 * MAX sits safely BELOW the empty-floor baseline. Beyond MAX = floor -> ignored,
 * exploration keeps going. TUNE MAX to ~0.10 m below the top sensor's empty read. */
#define MOUNTAIN_DETECT_MIN_M 0.05
#define MOUNTAIN_DETECT_MAX_M 0.45
#define NAV_TARGET_REACHED_M 0.06 /* within this of the target -> reached, re-plan    */
#define SENSE_HORIZON_M     0.25 /* trust open floor this far ahead when sensors read
                                  * clear, so the frontier map keeps growing          */
/* Frontier exploration thresholds, in GRID CELLS (ported from the UI prototype's
 * 100x100 algorithm, scaled to this 40-cell map). These are what make it actually
 * explore instead of crawling one cell straight ahead. */
#define FRONTIER_MIN_CELLS     4   /* a frontier target must be >= this many cells away */
#define DIST_FROM_LAST_CELLS   3   /* reject frontiers within this of the last target   */
#define BLACKLIST_RADIUS_CELLS 3   /* a failed frontier bans this radius around it      */
#define BLACKLIST_TTL_STEPS  250   /* ...for this many autopilot steps                  */
#define EDGE_BUFFER_CELLS      2   /* stay this far off the grid edge                   */
#define MEMORY_SCAN_CELLS   12.0   /* how far the side-scan probes the occupancy map    */
#define STUCK_STEPS            8   /* no-progress steps before blacklisting + recovery  */
#define SCAN_INCREMENTS       72   /* in-place sweep steps per 360deg = 5deg/step (fine scan; slower sweep) */
#define SCAN_SETTLE_MS       120   /* dwell after each scan rotation so wobble decays    */
#define APPROACH_MAX_STEPS    30   /* creep ticks before giving up on a phantom cube     */
#define ROAM_DISTANCE_M     0.40   /* straight-line distance per roam leg (longer = explores more ground) */
#define AVOID_TURN_RAD  (M_PI/3.0) /* turn aside this much to go around an obstacle      */

/* Cube colour commit (AP_EVALUATE). Average several reads, and only trust a
 * reading whose classifier confidence clears the floor before it is logged +
 * broadcast (a misread permanently fills a registry slot and can trip the
 * six-cube stop early). After EVAL_MAX_RETRIES low-confidence re-reads we accept
 * the best effort so a real cube is never abandoned and the loop always ends. */
#define COLOR_EVAL_SAMPLES     8   /* averaged reads per cube evaluation                 */
#define COLOR_CONFIDENCE_MIN 0.50  /* [0..1] min classifier confidence to commit a cube  */
#define EVAL_MAX_RETRIES       2   /* low-confidence re-approaches before accepting       */

#define CELL_UNKNOWN  0
#define CELL_FREE     1
#define CELL_MOUNTAIN 2
#define CELL_BOUNDARY 3

/* ----------------------------- Types ------------------------------- */

typedef enum { MODE_STOP = 0, MODE_EXPLORE, MODE_TARGET, MODE_MANUAL } robot_mode_t;

/* Autopilot state machine (same shape as the UI's). */
typedef enum {
  AP_SCAN = 0, AP_APPROACH, AP_EVALUATE, AP_ROAM
} ap_state_t;

typedef enum {
  COLOR_NONE = 0, COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_YELLOW, COLOR_WHITE
} color_label_t;

typedef struct { double x, y, theta; } pose_t;

/* Vertical ToF stack: bottom -> 3x3 rocks, middle -> 6x6 rocks, top -> mountains. */
typedef struct {
  bool bottom_valid, middle_valid, top_valid;
  double bottom_m, middle_m, top_m;
} distance_readings_t;

typedef struct {
  bool detected;
  color_label_t label;
  double confidence;
} color_detection_t;

typedef struct {
  pulsecounter_index_t index;
  uint32_t past_count;
  uint32_t past_time;
  bool initialized;
} pulse_tracker_t;

typedef struct {
  pose_t pose;
  robot_mode_t mode;
  bool has_target;
  double target_x, target_y;
  uint64_t last_pose_report_ms;
  uint64_t last_status_report_ms;
  pulse_tracker_t left_counter;
  pulse_tracker_t right_counter;
  int active_left_dir;
  int active_right_dir;
  ap_state_t ap_state;   /* autopilot state machine */
  int scan_rotations;    /* in-place sweep steps taken while looking for a frontier */
  int last_target_gx, last_target_gy; /* last frontier picked (cells) */
  bool has_last_target;
  int stuck_counter;     /* navigate ticks with no distance progress */
  double last_dist_to_target;
  int rock_is_large;     /* 1 = 6x6 (middle sensor), 0 = 3x3 (bottom)  */
  int approach_steps;    /* creep ticks in the current AP_APPROACH (anti-livelock) */
  int eval_attempts;     /* low-confidence colour re-reads on the current cube      */
  bool scan_has_best;    /* a candidate cube was seen during the current 360 sweep  */
  double scan_best_dist; /* ...its ToF range (m); we approach the CLOSEST one       */
  double scan_best_theta;/* ...the heading (world rad) it was seen at               */
  int scan_best_large;   /* ...whether it read as a large (6x6) cube                */
} robot_state_t;

static void poll_partner_mqtt(robot_state_t *s);  /* drained during blocking moves too */

/* ----------------------------- Utils ------------------------------- */

/* Set false by SIGINT/SIGTERM so the main loop exits and cleanup_hardware()
 * actually runs (motors disabled, IIC/UART/pynq released) instead of the process
 * being killed mid-move with the steppers left energised. Read in the main loop
 * and inside the blocking step loop (execute_step_command) for a prompt halt. */
static volatile sig_atomic_t g_keep_running = 1;
static void handle_signal(int sig) { (void)sig; g_keep_running = 0; }

static uint64_t monotonic_ms(void) {
  struct timespec ts;
  if (clock_gettime(CLOCK_MONOTONIC, &ts) == 0) {
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
  }
  return (uint64_t)((double)clock() * 1000.0 / (double)CLOCKS_PER_SEC);
}

static double normalize_angle(double a) {
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static double steps_to_meters(int steps) {
  const double revs = (double)steps / (STEPS_PER_REV * MICROSTEP_FACTOR);
  return revs * M_PI * WHEEL_DIAMETER_M;
}

static int meters_to_steps(double meters) {
  const double revs = meters / (M_PI * WHEEL_DIAMETER_M);
  return (int)lround(revs * STEPS_PER_REV * MICROSTEP_FACTOR);
}

static int sign_of_steps(int steps) {
  return (steps > 0) - (steps < 0);
}

static void update_odometry(robot_state_t *s, int left_steps, int right_steps) {
  const double dl = steps_to_meters(left_steps);
  const double dr = steps_to_meters(right_steps);
  const double dc = 0.5 * (dl + dr);
  const double dtheta = (dr - dl) / WHEEL_BASE_M;
  const double mid = s->pose.theta + 0.5 * dtheta;
  s->pose.x += dc * cos(mid);
  s->pose.y += dc * sin(mid);
  s->pose.theta = normalize_angle(s->pose.theta + dtheta);
}

/* ------------------------- Pulse-counter odometry ------------------ */

static void pulse_tracker_init(pulse_tracker_t *t, pulsecounter_index_t idx) {
  t->index = idx; t->past_count = 0; t->past_time = 0; t->initialized = false;
}

#if USE_PULSECOUNTER_ODOMETRY
static int pulse_tracker_read_delta(pulse_tracker_t *t) {
  uint32_t time = 0;
  const uint32_t count = pulsecounter_get_count(t->index, &time);
  if (!t->initialized) {
    t->past_count = count; t->past_time = time; t->initialized = true;
    return 0;
  }
  const uint32_t d = count - t->past_count;
  t->past_count = count; t->past_time = time;
  return (int)d;
}
#endif

static void sync_odometry_from_pulsecounters(robot_state_t *s) {
#if USE_PULSECOUNTER_ODOMETRY
  const int ld = pulse_tracker_read_delta(&s->left_counter);
  const int rd = pulse_tracker_read_delta(&s->right_counter);
  if (ld == 0 && rd == 0) return;
  update_odometry(s, s->active_left_dir * ld, s->active_right_dir * rd);
#else
  (void)s;
#endif
}

/* ------------------------ Telemetry TX (MQTT-only) ---------------- *
 * All telemetry (pose / status / observations / scanned cubes) leaves the robot
 * over the MQTT/UART0 link only -- there is no stdout UI stream any more.
 * Diagnostics still go to stderr (-> the boot log).                      */

static const char *mode_name(robot_mode_t m) {
  switch (m) {
    case MODE_EXPLORE: return "explore";
    case MODE_TARGET:  return "target";
    case MODE_MANUAL:  return "manual";
    default:           return "stop";
  }
}

/* Capitalised names for the partner-robot MQTT schema (Red/Blue/Green/Yellow). */
static const char *color_label_to_mqtt(color_label_t c) {
  switch (c) {
    case COLOR_RED:    return "Red";
    case COLOR_GREEN:  return "Green";
    case COLOR_BLUE:   return "Blue";
    case COLOR_YELLOW: return "Yellow";
    case COLOR_WHITE:  return "White";
    default:           return "Unknown";
  }
}

/* True (and stamps *last_ms) at most once per min_ms -- throttles MQTT publishes
 * so the UART link to the ESP32 isn't flooded. */
static bool mqtt_rate_ok(uint64_t *last_ms, uint64_t min_ms) {
  const uint64_t now = monotonic_ms();
  if (now - *last_ms < min_ms) return false;
  *last_ms = now;
  return true;
}

static void send_pose(const robot_state_t *s) {

  /* Relay location + orientation to mission control over MQTT (rate-limited).
   * x/y in metres, theta in rad -- mission_control scales pose m->cm. */
  static uint64_t last = 0;
  if (mqtt_rate_ok(&last, POSE_MQTT_MIN_MS)) {
    char m[160];
    snprintf(m, sizeof(m),
             "{\"type\":\"pose\",\"robot\":\"%s\",\"x\":%.4f,\"y\":%.4f,\"theta\":%.4f}",
             ROBOT_ID, s->pose.x, s->pose.y, s->pose.theta);
    mqtt_publish_json(m);
  }
}

static void send_status(const robot_state_t *s, const char *detail) {

  /* Publish mode/detail to mission control over MQTT (rate-limited) for its panel. */
  static uint64_t last = 0;
  if (mqtt_rate_ok(&last, STATUS_MQTT_MIN_MS)) {
    char m[224];
    snprintf(m, sizeof(m),
             "{\"type\":\"status\",\"robot\":\"%s\",\"mode\":\"%s\",\"detail\":\"%s\"}",
             ROBOT_ID, mode_name(s->mode), detail);
    mqtt_publish_json(m);
  }

  /* Every transition also publishes a fresh pose over MQTT (rate-limited) so
   * mission control's dead-reckoning trail stays current at turns/stops. */
  send_pose(s);
}

/* Objects already mirrored to mission control, keyed by kind + a ~5 cm cell, so
 * each distinct object is published exactly ONCE (a time gate would drop two
 * distinct objects seen in the same window, and re-spam one the robot lingers by).
 * Ring buffer: if more than PUB_OBJ_MAX distinct cells appear, the oldest is
 * forgotten and may be re-sent later (harmless -- mission control keys by pos). */
static struct { char k; int qx, qy; bool used; } g_pub_obj[PUB_OBJ_MAX];
static int g_pub_obj_head = 0;

static bool obj_publish_once(const char *kind, double x, double y) {
  const int qx = (int)lround(x * 20.0);   /* metres -> 5 cm cells */
  const int qy = (int)lround(y * 20.0);
  const char k = kind[0];
  for (int i = 0; i < PUB_OBJ_MAX; ++i) {
    if (g_pub_obj[i].used && g_pub_obj[i].k == k &&
        g_pub_obj[i].qx == qx && g_pub_obj[i].qy == qy) {
      return false;                         /* already mirrored this object */
    }
  }
  g_pub_obj[g_pub_obj_head].k = k;
  g_pub_obj[g_pub_obj_head].qx = qx;
  g_pub_obj[g_pub_obj_head].qy = qy;
  g_pub_obj[g_pub_obj_head].used = true;
  g_pub_obj_head = (g_pub_obj_head + 1) % PUB_OBJ_MAX;
  return true;
}

static void send_observation(const char *kind, double x, double y, double conf) {
  (void)conf;   /* confidence is UI-only; the MQTT object schema omits it */

  /* Relay each distinct object to mission control over MQTT so terrain
   * (mountains, boundaries/craters) maps too. The MQTT "type" is the kind, and
   * x/y are in cm to match mission_control's object/cube handling. */
  if (obj_publish_once(kind, x, y)) {
    char m[160];
    snprintf(m, sizeof(m),
             "{\"type\":\"%s\",\"robot\":\"%s\",\"x\":%d,\"y\":%d}",
             kind, ROBOT_ID, (int)lround(x * 100.0), (int)lround(y * 100.0));
    mqtt_publish_json(m);
  }
}

static void project_from_pose(const pose_t *p, double range_m, double bearing,
                              double *x, double *y) {
  const double wb = p->theta + bearing;
  *x = p->x + range_m * cos(wb);
  *y = p->y + range_m * sin(wb);
}

/* ----------------------- Located-cube registry --------------------- *
 * The mission is to find SIX cubes. We keep a local list of every cube located
 * so far -- both the ones THIS robot scans and the ones the partner reports over
 * MQTT (folded in by poll_partner_mqtt). It does two jobs:
 *   1. De-duplication: a cube whose position is already in the list is ignored,
 *      per the spec ("if the robot sees a cube already located by either itself
 *      or the other robot, it should be ignored").
 *   2. Completion: once SIX distinct cubes are known, the robot stops -- and
 *      because every find is shared over MQTT, BOTH robots independently reach
 *      six and stop together.
 * Positions are compared in metres; two finds within CUBE_DEDUP_RADIUS_M are
 * treated as the same cube (covers colour-sensor jitter + odometry drift). Both
 * robots run odometry in ONE shared world frame -- they start adjacent at (0,0)
 * with fixed opposite headings (r1 faces +y, r2 faces -y; see START_THETA_RAD and
 * main2.c) -- so the partner's x/y are already in our frame and register directly. */
#define MAX_CUBES           16
#define TARGET_CUBE_COUNT    6
#define CUBE_DEDUP_RADIUS_M  0.12   /* finds within 12 cm are the same cube */

static struct { double x, y; int hits; bool used; } g_cubes[MAX_CUBES];
static int g_cube_count = 0;

/* Index of the CLOSEST known cube within CUBE_DEDUP_RADIUS_M of (x,y), else -1. */
static int cube_nearest(double x, double y) {
  int best = -1;
  double best_d = CUBE_DEDUP_RADIUS_M;
  for (int i = 0; i < MAX_CUBES; ++i) {
    if (!g_cubes[i].used) continue;
    const double d = hypot(x - g_cubes[i].x, y - g_cubes[i].y);
    if (d <= best_d) { best_d = d; best = i; }
  }
  return best;
}

static bool cube_is_known(double x, double y) { return cube_nearest(x, y) >= 0; }

/* Add a cube at (x,y) unless it is already known. A repeat find (ours or the
 * partner's) instead refines the stored centre by running average -- which
 * smooths the inter-robot frame/odometry bias on the shared position. Returns
 * true ONLY for a genuinely new cube, so the caller reports + counts it once. */
static bool cube_register(double x, double y) {
  const int i = cube_nearest(x, y);
  if (i >= 0) {                                /* already known -> refine centre */
    g_cubes[i].x = (g_cubes[i].x * g_cubes[i].hits + x) / (g_cubes[i].hits + 1);
    g_cubes[i].y = (g_cubes[i].y * g_cubes[i].hits + y) / (g_cubes[i].hits + 1);
    ++g_cubes[i].hits;
    return false;
  }
  for (int j = 0; j < MAX_CUBES; ++j) {
    if (!g_cubes[j].used) {
      g_cubes[j].x = x; g_cubes[j].y = y; g_cubes[j].hits = 1; g_cubes[j].used = true;
      ++g_cube_count;
      return true;
    }
  }
  return false;   /* registry full (won't happen for six cubes) */
}

/* Halt once all six cubes are accounted for. Called after every new find --
 * local or from the partner -- so the robots stop together. */
static void stop_if_mission_complete(robot_state_t *s) {
  if (g_cube_count < TARGET_CUBE_COUNT) return;
  s->mode = MODE_STOP;
  stepper_reset();
  send_status(s, "mission_complete_all_cubes_found");
}

/* ----------------------- Sensor wrappers (real HW) ----------------- */

static distance_readings_t read_distance_sensors(void) {
  uint16_t b = 0, m = 0, t = 0;
  tof_read(&b, &m, &t);                 /* mm; 0 == invalid/out-of-range */
  distance_readings_t d = {0};
  d.bottom_valid = (b != 0); d.bottom_m = (double)b / 1000.0;  /* 3x3 rocks */
  d.middle_valid = (m != 0); d.middle_m = (double)m / 1000.0;  /* 6x6 rocks */
  d.top_valid    = (t != 0); d.top_m    = (double)t / 1000.0;  /* mountains */
  return d;
}

static bool tape_detected_now(void) {
  bool left = false, right = false;
  ir_read(&left, &right);
  return left || right;
}

static color_detection_t color_det_from_label(scan_color_t c, float conf) {
  color_detection_t det = {0};
  switch (c) {
    case SCAN_COLOR_RED:    det.label = COLOR_RED;    det.detected = true;  break;
    case SCAN_COLOR_GREEN:  det.label = COLOR_GREEN;  det.detected = true;  break;
    case SCAN_COLOR_BLUE:   det.label = COLOR_BLUE;   det.detected = true;  break;
    case SCAN_COLOR_YELLOW: det.label = COLOR_YELLOW; det.detected = true;  break;
    case SCAN_COLOR_WHITE:  det.label = COLOR_WHITE;  det.detected = true;  break;
    /* BLACK / NONE -> treat as background (floor/tape), not a scannable rock. */
    default:                det.label = COLOR_NONE;   det.detected = false; break;
  }
  det.confidence = (double)conf;
  return det;
}

/* Averaged read -- used only at the cube-commit point (AP_EVALUATE), where the
 * extra few hundred ms while stationary buys a far more reliable classification.
 * Built on the existing public sensor API (color_read + color_classify) so the
 * sensor driver itself needs no change: average several raw reads, discarding the
 * all-zero (timed-out) ones, then classify the mean. */
static color_detection_t read_color_detection_averaged(int samples) {
  uint64_t rs = 0, gs = 0, bs = 0;
  int valid = 0;
  if (samples < 1) samples = 1;
  for (int i = 0; i < samples; ++i) {
    uint32_t r = 0, g = 0, b = 0;
    color_read(&r, &g, &b);
    if (r == 0 && g == 0 && b == 0) continue;   /* timed-out read -> skip */
    rs += r; gs += g; bs += b; ++valid;
  }
  if (valid == 0) return color_det_from_label(SCAN_COLOR_NONE, 0.0f);
  float conf = 0.0f;
  const scan_color_t c = color_classify((uint32_t)(rs / valid), (uint32_t)(gs / valid),
                                        (uint32_t)(bs / valid), &conf);
  return color_det_from_label(c, conf);
}

/* --------------------------- Hazard helpers ------------------------ */

/* ------------------------------ Motion ----------------------------- */

static bool execute_step_command(robot_state_t *s, int left_steps, int right_steps,
                                 bool safety_enabled, uint16_t pulse_delay) {
  s->active_left_dir  = sign_of_steps(left_steps);
  s->active_right_dir = sign_of_steps(right_steps);
  stepper_set_speed(pulse_delay, pulse_delay);
  stepper_steps((int16_t)left_steps, (int16_t)right_steps);

  while (!stepper_steps_done()) {
    poll_partner_mqtt(s);   /* keep draining the partner link DURING the move so its
                             * relayed cube frames aren't lost to a UART FIFO overflow
                             * (a step can block for seconds); may also trip the
                             * six-cube stop, caught by the MODE_STOP check below. */
    sync_odometry_from_pulsecounters(s);

    if (!g_keep_running || s->mode == MODE_STOP) {   /* shutdown or stop: halt now */
      stepper_reset();
      sync_odometry_from_pulsecounters(s);
      s->active_left_dir = s->active_right_dir = 0;
      return false;
    }

    if (safety_enabled) {
      /* Tape = boundary: stop mid-move. (Mountains are handled at the state
       * level by the autopilot, not by aborting a step.) */
      if (tape_detected_now()) {
        stepper_reset();
        sync_odometry_from_pulsecounters(s);
        double x = 0.0, y = 0.0;
        project_from_pose(&s->pose, TAPE_REPORT_OFFSET_M, 0.0, &x, &y);
        send_observation("black_tape", x, y, 1.0);
        send_status(s, "tape_avoidance");
        s->active_left_dir = s->active_right_dir = 0;
        return false;
      }
    }
    sleep_msec(LOOP_SLEEP_MS);
  }

  sync_odometry_from_pulsecounters(s);
#if !USE_PULSECOUNTER_ODOMETRY
  /* Dead-reckoning: integrate pose from the commanded steps (no encoders). */
  update_odometry(s, left_steps, right_steps);
#endif
  s->active_left_dir = s->active_right_dir = 0;
  return true;
}

static bool move_forward_m(robot_state_t *s, double meters, bool safety) {
  const int steps = meters_to_steps(meters);
  return execute_step_command(s, steps, steps, safety, STEPPER_PULSE_DELAY_TICKS);
}

static bool rotate_rad(robot_state_t *s, double radians, bool safety) {
  const double wheel_m = 0.5 * WHEEL_BASE_M * radians;
  const int l = meters_to_steps(-wheel_m);
  const int r = meters_to_steps(wheel_m);
  return execute_step_command(s, l, r, safety, STEPPER_TURN_PULSE_DELAY_TICKS);
}


/* -------------------------- Reporting / scan ----------------------- */

static void report_periodic(robot_state_t *s) {
  const uint64_t now = monotonic_ms();
  if (now - s->last_pose_report_ms >= POSE_REPORT_PERIOD_MS) {
    send_pose(s);
    s->last_pose_report_ms = now;
  }
  if (now - s->last_status_report_ms >= STATUS_REPORT_PERIOD_MS) {
    send_status(s, "alive");
    s->last_status_report_ms = now;
  }
}

/* Poll the partner robot's MQTT link and fold any rock it reports into our shared
 * cube registry (dedup + the six-cube finish). cm -> m. */
static void poll_partner_mqtt(robot_state_t *s) {
  int px = 0, py = 0, psize = 0;
  float ptemp = 0.0f;
  char pcolor[32];
  pcolor[0] = '\0';

  /* Drain EVERY complete partner frame queued this tick. After a multi-second
   * blocking move several can be buffered; recv_mqtt parses one per call and
   * returns false once none remain (so finds aren't trickled out one-per-loop). */
  while (recv_mqtt(&px, &py, &psize, &ptemp, pcolor)) {
    if (pcolor[0] != '\0' && strcmp(pcolor, "Unknown") != 0) {
      /* Shared world frame (see the cube-registry note + START_THETA_RAD): the
       * partner's x/y (cm) are already in our frame, so register them directly. */
      const double wx = (double)px / 100.0, wy = (double)py / 100.0;

      /* Fold into the shared registry: this is how a cube the partner located is
       * ignored if we later see it, and how we count toward the six-cube finish.
       * Only a genuinely new cube advances the count. */
      if (cube_register(wx, wy)) stop_if_mission_complete(s);
    }
    pcolor[0] = '\0';   /* reset so a frame with no colour field can't reuse the last */
  }
}

/* Closest cube within ROCK_DETECT_M (-1 if none). Sizing follows the vertical beam
 * geometry MEASURED with debug_tof: a 3x3 cube only trips the BOTTOM beam, while a 6x6
 * is tall enough to ALSO trip the MIDDLE beam -- so "middle in range" => large. (A
 * mountain reads past ROCK_DETECT_M on bottom/middle and shows up on the TOP beam
 * instead, so it is handled by the mountain logic, not mistaken for a cube here.) */
static double ap_rock_dist(const distance_readings_t *d, int *is_large) {
  const bool b = d->bottom_valid && d->bottom_m > 0.0 && d->bottom_m <= ROCK_DETECT_M;
  const bool m = d->middle_valid && d->middle_m > 0.0 && d->middle_m <= ROCK_DETECT_M;
  const bool t = d->top_valid    && d->top_m    > 0.0 && d->top_m    <= ROCK_DETECT_M;
  /* A cube is seen by the BOTTOM and/or MIDDLE beam -- accept EITHER. On this rover
   * the bottom beam grazes the floor and often MISSES a cube the middle beam sees
   * cleanly (observed: bottom 8.19 m while middle reads ~9 cm on a cube right in
   * front), so requiring the bottom beam made it ignore real cubes. */
  if (!b && !m) return -1.0;
  /* The TOP beam (8 cm) is too high for a 3-6 cm cube, so a cube leaves it reading
   * the far background. If the top beam is ALSO blocked this close, it's a ~10 cm
   * mountain, not a cube -> reject. */
  if (t) return -1.0;
  if (is_large) *is_large = (b && m) ? 1 : 0;   /* both low beams blocked -> 6x6, just one -> 3x3 */
  if (b && m) return fmin(d->bottom_m, d->middle_m);
  return b ? d->bottom_m : d->middle_m;
}

/* Report the objects we sense to the map / mission control (localization). The
 * simplified roamer does NOT avoid mountains -- it only logs them so they appear
 * on the map -- and logs boundaries/craters too. Cubes are reported separately
 * when scanned (AP_EVALUATE). */
static void report_objects(robot_state_t *s, const distance_readings_t *d, bool tape) {
  if (d->top_valid && d->top_m >= MOUNTAIN_DETECT_MIN_M && d->top_m <= MOUNTAIN_DETECT_MAX_M) {
    double hx = 0.0, hy = 0.0;
    project_from_pose(&s->pose, d->top_m, 0.0, &hx, &hy);
    send_observation("mountain", hx, hy, 0.85);
  }
  if (tape) {
    double bx = 0.0, by = 0.0;
    project_from_pose(&s->pose, TAPE_REPORT_OFFSET_M, 0.0, &bx, &by);
    send_observation("black_tape", bx, by, 1.0);   /* boundary or crater */
  }
}

/* Read the cube's colour at the current spot. This one line is the ONLY per-robot
 * difference in the autopilot below: robotv2 averages several reads for a more
 * reliable classification (robot/ uses a single read). */
static color_detection_t read_cube_color(void) {
  return read_color_detection_averaged(COLOR_EVAL_SAMPLES);
}

/* The autopilot, run once per main-loop iteration in MODE_EXPLORE.
 *
 * State-machine roamer focused on object classification + localization:
 *   SCAN     - rotate 360deg in place; break off to the FIRST unscanned cube
 *              seen, remembering its position as the approach target.
 *   APPROACH - head to that cube while AVOIDING collisions: only cubes trigger
 *              this state (mountains never do), but a mountain or crater in the
 *              way is gone AROUND, keeping the cube as the goal.
 *   EVALUATE - read its colour, log + report colour/size/temperature, re-scan.
 *   ROAM     - a full sweep found nothing: turn a RANDOM direction, drive a set
 *              distance, then re-scan.
 * Evasive interrupt (any state): a boundary or crater (black tape) -> back off,
 * turn away a randomised angle, ABANDON any approach, and re-enter SCAN (which
 * re-scans for objects and roams if none). Runs until the six-cube stop. */
static void autopilot_step(robot_state_t *s) {
  const distance_readings_t d = read_distance_sensors();
  const bool tape = tape_detected_now();

  report_objects(s, &d, tape);   /* localise mountains + boundaries/craters */

  if (tape) {
    /* Boundary or crater (black tape): GO BACK. Reverse straight off the tape and
     * turn ~180deg to face the way we came (back toward the interior), then re-scan.
     * We deliberately do NOT drive forward here -- the old forward "step clear" kept
     * shoving the robot back onto the border and it got stuck there. If we're still
     * on tape next tick this interrupt fires again and reverses further until clear.
     * Any approach is dropped (a cube is never across a boundary; craters are avoided). */
    move_forward_m(s, -BACKUP_M, false);                            /* reverse straight off the tape */
    const double jitter = (((double)rand() / (double)RAND_MAX) - 0.5) * (M_PI / 3.0);  /* +/-30 deg */
    rotate_rad(s, M_PI + jitter, false);                            /* ~180deg -> face back to the interior */
    s->scan_rotations = 0;
    s->ap_state = AP_SCAN;                                          /* re-scan; nothing new -> roam */
    send_status(s, "evade_tape");
    return;
  }

  switch (s->ap_state) {
    case AP_SCAN: {
      /* 360deg in-place sweep; the FIRST not-yet-known cube seen wins. Remember
       * its position so APPROACH can steer back to it after dodging an obstacle. */
      int is_large = 0;
      const double rock = ap_rock_dist(&d, &is_large);
      if (rock > 0.0) {
        /* IMMEDIATE-APPROACH variant (robotv5 from robotv3, robotv6 from robotv4):
         * the INSTANT a single read sees a cube, switch to APPROACH -- NO burst-vote
         * confirmation. Whenever a cube is seen we commit to approaching it. The
         * cube_is_known dedup stays so we don't loop re-approaching a logged cube. */
        double rx = 0.0, ry = 0.0;
        project_from_pose(&s->pose, rock, 0.0, &rx, &ry);
        if (!cube_is_known(rx, ry)) {
          s->rock_is_large = is_large;
          s->target_x = rx;
          s->target_y = ry;
          s->scan_rotations = 0;
          s->approach_steps = 0;
          s->ap_state = AP_APPROACH;
          send_status(s, "cube_detected");
          break;
        }
      }
      if (s->scan_rotations < SCAN_INCREMENTS) {
        rotate_rad(s, 2.0 * M_PI / (double)SCAN_INCREMENTS, false);
        s->scan_rotations++;
      } else {
        s->scan_rotations = 0;
        s->ap_state = AP_ROAM;               /* full sweep, nothing new -> roam */
      }
      break;
    }

    case AP_APPROACH: {
      if (++s->approach_steps > APPROACH_MAX_STEPS) {   /* can't reach it -> give up */
        s->ap_state = AP_SCAN;
        s->scan_rotations = 0;
        send_status(s, "approach_timeout");
        break;
      }

      const double rock = ap_rock_dist(&d, NULL);

      /* Mountain in the way -> go AROUND it (avoid the collision), keep the cube.
       * Only if the mountain is NEARER than the cube (i.e. actually between us and
       * it); a mountain just behind the cube must not abort a reachable approach. */
      const bool mountain_ahead = d.top_valid &&
          d.top_m >= MOUNTAIN_DETECT_MIN_M && d.top_m < MOUNTAIN_STOP_M &&
          (rock < 0.0 || d.top_m < rock);
      if (mountain_ahead) {
        move_forward_m(s, -BACKUP_M, false);
        rotate_rad(s, AVOID_TURN_RAD, false);
        move_forward_m(s, CELL_M * 2.0, true);          /* sidestep; safety: don't cross tape */
        send_status(s, "approach_avoid_mountain");
        break;                                          /* stay AP_APPROACH; re-aim next tick */
      }

      /* How far still to go: prefer the live bottom ToF, fall back to the
       * remembered cube position when the (noisy) beam has dropped out. */
      const double tdx = s->target_x - s->pose.x, tdy = s->target_y - s->pose.y;
      const double to_target = hypot(tdx, tdy);
      const double near_m = (rock > 0.0) ? rock : to_target;

      if (near_m <= COLOR_ALIGN_DIST_M) {
        /* Close enough -> drive the last gap straight in and evaluate. */
        if (near_m > ROCK_APPROACH_M) {
          move_forward_m(s, near_m - ROCK_APPROACH_M, true);
        }
        s->ap_state = AP_EVALUATE;
        break;
      }

      if (rock > 0.0) {
        /* Cube in view straight ahead -> creep toward it. */
        move_forward_m(s, fmin(0.03, rock - ROCK_APPROACH_M), true);
      } else {
        /* Cube not in view (e.g. just dodged an obstacle) -> steer back toward
         * where we saw it. */
        const double err = normalize_angle(atan2(tdy, tdx) - s->pose.theta);
        if (fabs(err) > HEADING_TOLERANCE_RAD) rotate_rad(s, err, false);
        else move_forward_m(s, fmin(0.03, to_target), true);
      }
      break;
    }

    case AP_EVALUATE: {
      /* Register on the strength of the (burst-confirmed) DETECTION, not the colour
       * read -- the colour sensor is the weak link, so we never drop a confirmed cube
       * just because the colour is uncertain. We still read + report the best colour
       * we can ("Unknown" if none); position + size + temperature go out regardless. */
      const color_detection_t color = read_cube_color();
      double rx = 0.0, ry = 0.0;
      project_from_pose(&s->pose, ROCK_APPROACH_M, 0.0, &rx, &ry);
      if (cube_register(rx, ry)) {                              /* NEW cube -> log + report */
        const float temp_c = temp_read_celsius();               /* environment temp at the cube */
        send_mqtt((int)lround(rx * 100.0), (int)lround(ry * 100.0),       /* colour (best-effort) + size + temp */
                  color_label_to_mqtt(color.label),
                  s->rock_is_large ? 6 : 3, temp_c);
        send_status(s, "cube_located");
        stop_if_mission_complete(s);                            /* sixth cube -> stop */
      } else {
        send_status(s, "cube_duplicate_ignored");               /* already known -> ignore */
      }
      if (s->mode == MODE_STOP) break;                          /* mission complete: halt */
      move_forward_m(s, -0.05, false);                          /* back off the cube */
      s->scan_rotations = 0;
      s->ap_state = AP_SCAN;
      break;
    }

    case AP_ROAM: {
      /* Pick a RANDOM forward direction, drive a set distance, then re-scan. */
      const double ang = ((double)rand() / (double)RAND_MAX) * 2.0 * M_PI - M_PI;
      rotate_rad(s, ang, false);
      send_status(s, "roaming");
      move_forward_m(s, ROAM_DISTANCE_M, true);
      s->scan_rotations = 0;
      s->ap_state = AP_SCAN;
      break;
    }
  }
}

/* ----------------------------- Hardware ---------------------------- */

static void setup_hardware(void) {
  /* Headless / run-on-boot: no operator, no ws_bridge, no stdin. The only comms
   * link is MQTT over UART0 (brought up below); diagnostics go to the boot log. */
  pynq_init();
  fprintf(stderr, "[robot] pynq_init done\n");

  /* Steppers FIRST. stepper_init() resets the switchbox, so it must run BEFORE
   * the sensors route their pins -- otherwise it wipes the IR/colour GPIO
   * routing and the IR floats HIGH (reads tape forever; works in sensortest,
   * which never inits the stepper). sensors_init_all() only sets its own pins
   * and does not reset the whole switchbox, so the motor routing survives it. */
  stepper_init();
  stepper_enable();
  stepper_set_speed(STEPPER_PULSE_DELAY_TICKS, STEPPER_PULSE_DELAY_TICKS);
  fprintf(stderr, "[robot] steppers up\n");

  /* Sensor module brings up GPIO, IIC0 + the three ToF sensors (XSHUT on
   * AR4/AR5), the IR tape pair (AR9/AR10), and the TCS3200 colour sensor
   * (AR6/AR7/AR8). Runs after the stepper so its pin routing wins. */
  const int serr = sensors_init_all();
  if (serr != 0) {
    fprintf(stderr, "[robot] sensors_init_all returned 0x%X (some sensor offline)\n", serr);
  }
  fprintf(stderr, "[robot] sensors up -- autonomous MQTT-only mode.\n");

  /* UART0 on AR0/AR1 for the ESP32 / MQTT link to the partner robot. */
  switchbox_set_pin(IO_AR0, SWB_UART0_RX);
  switchbox_set_pin(IO_AR1, SWB_UART0_TX);
  uart_init(UART0);
  fprintf(stderr, "[robot] MQTT link up on UART0 (AR0/AR1).\n");

#if USE_PULSECOUNTER_ODOMETRY
  switchbox_set_pin(LEFT_PULSE_PIN,  SWB_TIMER_IC0);
  switchbox_set_pin(RIGHT_PULSE_PIN, SWB_TIMER_IC1);
  pulsecounter_init(PULSECOUNTER0);
  pulsecounter_init(PULSECOUNTER1);
  pulsecounter_set_edge(PULSECOUNTER0, GPIO_LEVEL_HIGH);
  pulsecounter_set_edge(PULSECOUNTER1, GPIO_LEVEL_HIGH);
  pulsecounter_reset_count(PULSECOUNTER0);
  pulsecounter_reset_count(PULSECOUNTER1);
#endif
}

static void cleanup_hardware(void) {
  stepper_reset();         /* stop any in-flight motion before disabling the driver */
  stepper_disable();
  stepper_destroy();
#if USE_PULSECOUNTER_ODOMETRY
  pulsecounter_destroy(PULSECOUNTER0);
  pulsecounter_destroy(PULSECOUNTER1);
#endif
  iic_destroy(IIC0);
  uart_reset_fifos(UART0); /* release UART0 (brought up in setup_hardware) */
  uart_destroy(UART0);
  pynq_destroy();
}

/* ------------------------------- main ------------------------------ */

int main(void) {
  robot_state_t state;
  memset(&state, 0, sizeof(state));
  state.pose.x = START_X_M;
  state.pose.y = START_Y_M;
  state.pose.theta = START_THETA_RAD;
  state.mode = MODE_EXPLORE;
  pulse_tracker_init(&state.left_counter, PULSECOUNTER0);
  pulse_tracker_init(&state.right_counter, PULSECOUNTER1);
  state.last_pose_report_ms = monotonic_ms();
  state.last_status_report_ms = monotonic_ms();

  signal(SIGINT, handle_signal);    /* Ctrl+C / kill -> clean shutdown (motors off) */
  signal(SIGTERM, handle_signal);

  /* Seed the RNG for the random roam direction. Mix in ROBOT_ID so the two
   * robots roam differently even if they boot in the same second. */
  {
    unsigned seed = (unsigned)time(NULL);
    for (const char *p = ROBOT_ID; *p; ++p) seed = seed * 31u + (unsigned char)*p;
    srand(seed);
  }

  setup_hardware();
  sync_odometry_from_pulsecounters(&state);

  send_status(&state, "boot");
  send_pose(&state);

  uint64_t last_loop_hb = 0;

  while (g_keep_running) {
    poll_partner_mqtt(&state);
    report_periodic(&state);

    /* Heartbeat to stderr (-> /tmp/ws.log): loop alive, mode, ap-state, pose,
     * the three ToF heights, tape, temp. */
    const uint64_t hb_now = monotonic_ms();
    if (hb_now - last_loop_hb >= 1000) {
      const distance_readings_t hb_d = read_distance_sensors();
      const bool hb_tape = tape_detected_now();
      fprintf(stderr,
              "[robot] tick: mode=%s ap=%d pose=(%.2f, %.2f, %.0fdeg) "
              "tof[bot %.2f mid %.2f top %.2f]m tape=%d temp=%.1fC\n",
              mode_name(state.mode), (int)state.ap_state, state.pose.x, state.pose.y,
              state.pose.theta * 180.0 / M_PI,
              hb_d.bottom_m, hb_d.middle_m, hb_d.top_m, hb_tape,
              temp_read_celsius());
      last_loop_hb = hb_now;
    }

    if (state.mode == MODE_STOP) {
      sleep_msec(LOOP_SLEEP_MS);
      continue;
    }
    autopilot_step(&state);   /* always autonomous EXPLORE: frontier + SLAM */
  }

  cleanup_hardware();
  return EXIT_SUCCESS;
}
