#include "mma8452q.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mma8452q {

static const char *const TAG = "mma8452q";

const uint8_t MMA8452Q_REGISTER_WHO_AM_I = 0x75;
const uint8_t MMA8452Q_REGISTER_POWER_MANAGEMENT_1 = 0x6B;
const uint8_t MMA8452Q_REGISTER_GYRO_CONFIG = 0x1B;
const uint8_t MMA8452Q_REGISTER_ACCEL_CONFIG = 0x1C;
const uint8_t MMA8452Q_REGISTER_ACCEL_XOUT_H = 0x3B;
const uint8_t MMA8452Q_CLOCK_SOURCE_X_GYRO = 0b001;
const uint8_t MMA8452Q_SCALE_2000_DPS = 0b11;
const float MMA8452Q_SCALE_DPS_PER_DIGIT_2000 = 0.060975f;
const uint8_t MMA8452Q_RANGE_2G = 0b00;
const float MMA8452Q_RANGE_PER_DIGIT_2G = 0.000061f;
const uint8_t MMA8452Q_BIT_SLEEP_ENABLED = 6;
const uint8_t MMA8452Q_BIT_TEMPERATURE_DISABLED = 3;
const float GRAVITY_EARTH = 9.80665f;




// WRITE A SINGLE REGISTER
// 	Write a single byte of data to a register in the MMA8452Q.
void MMA8452QComponent::writeRegister(MMA8452Q_Register reg, byte data)
{
    write_register( reg, &data, 1);
	//writeRegisters(reg, &data, 1);
}

// WRITE MULTIPLE REGISTERS
//	Write an array of "len" bytes ("buffer"), starting at register "reg", and
//	auto-incrmenting to the next.
void MMA8452QComponent::writeRegisters(MMA8452Q_Register reg, byte *buffer, byte len)
{
#if 1

    this->write((uint8_t*)&reg, sizeof(reg), 0);
    this->write(buffer, len);

#else
	_i2cPort->beginTransmission(_deviceAddress);
	_i2cPort->write(reg);
	for (int x = 0; x < len; x++)
		_i2cPort->write(buffer[x]);
	_i2cPort->endTransmission(); //Stop transmitting
#endif
}

// READ A SINGLE REGISTER
//	Read a byte from the MMA8452Q register "reg".
byte MMA8452QComponent::readRegister(MMA8452Q_Register reg)
{


#if 1
    uint8_t byte = 0;

    if(!this->read_byte(reg, &byte, false)) {
       this->mark_failed();
       return 0;
    }

    return byte;

#else


#ifdef _VARIANT_ARDUINO_DUE_X_
	_i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)1, (uint32_t)reg, (uint8_t)1, true);
#else


	_i2cPort->beginTransmission(_deviceAddress);
	_i2cPort->write(reg);
	_i2cPort->endTransmission(false); //endTransmission but keep the connection active

	_i2cPort->requestFrom(_deviceAddress, (byte)1); //Ask for 1 byte, once done, bus is released by default
#endif
	if (_i2cPort->available())
	{							 //Wait for the data to come back
		return _i2cPort->read(); //Return this one byte
	}
	else
	{
		return 0;
	}
#endif
}



void MMA8452QComponent::readRegisters(MMA8452Q_Register reg, byte *buffer, byte len)
{
#if 1

    //this->write((byte*)&reg, sizeof(reg), 0);
    //memset(buffer, 0, len);
    //this->read(buffer, len);

    this->read_register(reg, buffer, len, 0);



#else
#ifdef _VARIANT_ARDUINO_DUE_X_
	_i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)len, (uint32_t)reg, (uint8_t)1, true);
#else
	_i2cPort->beginTransmission(_deviceAddress);
	_i2cPort->write(reg);
	_i2cPort->endTransmission(false);			//endTransmission but keep the connection active
	_i2cPort->requestFrom(_deviceAddress, len); //Ask for bytes, once done, bus is released by default
#endif
	if (_i2cPort->available() == len)
	{
		for (int x = 0; x < len; x++)
			buffer[x] = _i2cPort->read();
	}
#endif
}






void MMA8452QComponent::setup() {

    ESP_LOGCONFIG(TAG, "Setting up MMA8452Q...");
	byte c = readRegister(WHO_AM_I); // Read WHO_AM_I register
    ESP_LOGE(TAG, "WHO_AM_I = %d", c);

	if (c != 0x2A) // WHO_AM_I should always be 0x2A
	{
        this->mark_failed();
        return;
	}
    this->standby();

	scale = SCALE_2G;
	odr = ODR_6;

	setScale(scale);  // Set up accelerometer scale
	setDataRate(odr); // Set up output data rate
	setupPL();		  // Set up portrait/landscape detection

	// Multiply parameter by 0.0625g to calculate threshold.
	setupTap(0x80, 0x80, 0x08); // Disable x, y, set z to 0.5g

	this->active(); // Set to active to start reading



	/*
  ESP_LOGCONFIG(TAG, "Setting up MMA8452Q...");
  uint8_t who_am_i;
  if (!this->read_byte(MMA8452Q_REGISTER_WHO_AM_I, &who_am_i) || who_am_i != 0x68) {
    this->mark_failed();
    return;
  }

  ESP_LOGV(TAG, "  Setting up Power Management...");
  // Setup power management
  uint8_t power_management;
  if (!this->read_byte(MMA8452Q_REGISTER_POWER_MANAGEMENT_1, &power_management)) {
    this->mark_failed();
    return;
  }
  ESP_LOGV(TAG, "  Input power_management: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(power_management));
  // Set clock source - X-Gyro
  power_management &= 0b11111000;
  power_management |= MMA8452Q_CLOCK_SOURCE_X_GYRO;
  // Disable sleep
  power_management &= ~(1 << MMA8452Q_BIT_SLEEP_ENABLED);
  // Enable temperature
  power_management &= ~(1 << MMA8452Q_BIT_TEMPERATURE_DISABLED);
  ESP_LOGV(TAG, "  Output power_management: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(power_management));
  if (!this->write_byte(MMA8452Q_REGISTER_POWER_MANAGEMENT_1, power_management)) {
    this->mark_failed();
    return;
  }

  ESP_LOGV(TAG, "  Setting up Gyro Config...");
  // Set scale - 2000DPS
  uint8_t gyro_config;
  if (!this->read_byte(MMA8452Q_REGISTER_GYRO_CONFIG, &gyro_config)) {
    this->mark_failed();
    return;
  }
  ESP_LOGV(TAG, "  Input gyro_config: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(gyro_config));
  gyro_config &= 0b11100111;
  gyro_config |= MMA8452Q_SCALE_2000_DPS << 3;
  ESP_LOGV(TAG, "  Output gyro_config: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(gyro_config));
  if (!this->write_byte(MMA8452Q_REGISTER_GYRO_CONFIG, gyro_config)) {
    this->mark_failed();
    return;
  }

  ESP_LOGV(TAG, "  Setting up Accel Config...");
  // Set range - 2G
  uint8_t accel_config;
  if (!this->read_byte(MMA8452Q_REGISTER_ACCEL_CONFIG, &accel_config)) {
    this->mark_failed();
    return;
  }
  ESP_LOGV(TAG, "    Input accel_config: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(accel_config));
  accel_config &= 0b11100111;
  accel_config |= (MMA8452Q_RANGE_2G << 3);
  ESP_LOGV(TAG, "    Output accel_config: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(accel_config));
  if (!this->write_byte(MMA8452Q_REGISTER_GYRO_CONFIG, gyro_config)) {
    this->mark_failed();
    return;
  }
  */

}
void MMA8452QComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "MMA8452Q:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with MMA8452Q failed!");
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_);
}


// SET STANDBY MODE
//	Sets the MMA8452 to standby mode. It must be in standby to change most register settings
void MMA8452QComponent::standby()
{
	byte c = readRegister(CTRL_REG1);
	writeRegister(CTRL_REG1, c & ~(0x01)); //Clear the active bit to go into standby
}


// SET FULL-SCALE RANGE
//	This function sets the full-scale range of the x, y, and z axis accelerometers.
//	Possible values for the fsr variable are SCALE_2G, SCALE_4G, or SCALE_8G.
void MMA8452QComponent::setScale(MMA8452Q_Scale fsr)
{
	// Must be in standby mode to make changes!!!
	// Change to standby if currently in active state
	if (isActive() == true)
		standby();

	byte cfg = readRegister(XYZ_DATA_CFG);
	cfg &= 0xFC;	   // Mask out scale bits
	cfg |= (fsr >> 2); // Neat trick, see page 22. 00 = 2G, 01 = 4A, 10 = 8G
	writeRegister(XYZ_DATA_CFG, cfg);

	// Return to active state when done
	// Must be in active state to read data
	active();
}

// SET THE OUTPUT DATA RATE
//	This function sets the output data rate of the MMA8452Q.
//	Possible values for the odr parameter are: ODR_800, ODR_400, ODR_200,
//	ODR_100, ODR_50, ODR_12, ODR_6, or ODR_1
void MMA8452QComponent::setDataRate(MMA8452Q_ODR odr)
{
	// Must be in standby mode to make changes!!!
	// Change to standby if currently in active state
	if (isActive() == true)
		standby();

	byte ctrl = readRegister(CTRL_REG1);
	ctrl &= 0xC7; // Mask out data rate bits
	ctrl |= (odr << 3);
	writeRegister(CTRL_REG1, ctrl);

	// Return to active state when done
	// Must be in active state to read data
	active();
}


// SET ACTIVE MODE
//	Sets the MMA8452 to active mode. Needs to be in this mode to output data
void MMA8452QComponent::active()
{
	byte c = readRegister(CTRL_REG1);
	writeRegister(CTRL_REG1, c | 0x01); //Set the active bit to begin detection
}


// CHECK STATE (ACTIVE or STANDBY)
//	Returns true if in Active State, otherwise return false
bool MMA8452QComponent::isActive()
{
	byte currentState = readRegister(SYSMOD);
	currentState &= 0b00000011;

	// Wake and Sleep are both active SYSMOD states (pg. 10 datasheet)
	if (currentState == SYSMOD_STANDBY)
		return false;
	return true;
}

// SET UP PORTRAIT/LANDSCAPE DETECTION
//	This function sets up portrait and landscape detection.
void MMA8452QComponent::setupPL()
{
	// Must be in standby mode to make changes!!!
	// Change to standby if currently in active state
	if (isActive() == true)
		standby();

	// For more info check out this app note:
	//	http://cache.freescale.com/files/sensors/doc/app_note/AN4068.pdf
	// 1. Enable P/L
	writeRegister(PL_CFG, readRegister(PL_CFG) | 0x40); // Set PL_EN (enable)
	// 2. Set the debounce rate
	writeRegister(PL_COUNT, 0x50); // Debounce counter at 100ms (at 800 hz)

	// Return to active state when done
	// Must be in active state to read data
	active();
}


void MMA8452QComponent::setupTap(byte xThs, byte yThs, byte zThs)
{
	// Must be in standby mode to make changes!!!
	// Change to standby if currently in active state
	if (isActive() == true)
		standby();

	// Set up single and double tap - 5 steps:
	// for more info check out this app note:
	// http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf
	// Set the threshold - minimum required acceleration to cause a tap.
	byte temp = 0;
	if (!(xThs & 0x80)) // If top bit ISN'T set
	{
		temp |= 0x3;					 // Enable taps on x
		writeRegister(PULSE_THSX, xThs); // x thresh
	}
	if (!(yThs & 0x80))
	{
		temp |= 0xC;					 // Enable taps on y
		writeRegister(PULSE_THSY, yThs); // y thresh
	}
	if (!(zThs & 0x80))
	{
		temp |= 0x30;					 // Enable taps on z
		writeRegister(PULSE_THSZ, zThs); // z thresh
	}
	// Set up single and/or double tap detection on each axis individually.
	writeRegister(PULSE_CFG, temp | 0x40);
	// Set the time limit - the maximum time that a tap can be above the thresh
	writeRegister(PULSE_TMLT, 0x30); // 30ms time limit at 800Hz odr
	// Set the pulse latency - the minimum required time between pulses
	writeRegister(PULSE_LTCY, 0xA0); // 200ms (at 800Hz odr) between taps min
	// Set the second pulse window - maximum allowed time between end of
	//	latency and start of second pulse
	writeRegister(PULSE_WIND, 0xFF); // 5. 318ms (max value) between taps max

	// Return to active state when done
	// Must be in active state to read data
	active();
}







void MMA8452QComponent::update() {

  short x, y, z;
  float accel_x, accel_y, accel_z;
  float roll, pitch;

  ESP_LOGV(TAG, "    Updating MMA8452Q...");


  byte rawData[6]; // x/y/z accel register data stored here
  
  readRegisters(OUT_X_MSB, rawData, 6); // Read the six raw data registers into data array
  
  x = ((short)(rawData[0] << 8 | rawData[1])) >> 4;
  y = ((short)(rawData[2] << 8 | rawData[3])) >> 4;
  z = ((short)(rawData[4] << 8 | rawData[5])) >> 4;
  accel_x = (float)x / (float)(1 << 11) * (float)(scale);
  accel_y = (float)y / (float)(1 << 11) * (float)(scale);
  accel_z = (float)z / (float)(1 << 11) * (float)(scale);

  //roll    = atan2(accel_y , accel_z) * 57.3;
  //pitch   = atan2((- accel_x) , sqrt(accel_y * accel_y + accel_z * accel_z)) * 57.3;
  roll   = atan2 ( -accel_x, accel_z) * 57.3;
  pitch  = atan2 (  accel_y, sqrt(accel_z * accel_z + accel_x * accel_x) ) * 57.3;
  if ( always_look_down_ && ( accel_z < 0 ) ) {
    accel_z = -accel_z;
    accel_x = -accel_x;
    accel_y = -accel_y;
  }


  ESP_LOGD(TAG,
           "Got accel={x=%.3f m/s², y=%.3f m/s², z=%.3f m/s²}, ",
           accel_x, accel_y, accel_z);

  if (this->accel_x_sensor_ != nullptr)
    this->accel_x_sensor_->publish_state(accel_x);
  if (this->accel_y_sensor_ != nullptr)
    this->accel_y_sensor_->publish_state(accel_y);
  if (this->accel_z_sensor_ != nullptr)
    this->accel_z_sensor_->publish_state(accel_z);

  if (this->roll_sensor_ != nullptr)
    this->roll_sensor_->publish_state(roll);
  if (this->pitch_sensor_ != nullptr)
    this->pitch_sensor_->publish_state(pitch);


  this->status_clear_warning();






/*
  uint16_t raw_data[7];
  if (!this->read_bytes_16(MMA8452Q_REGISTER_ACCEL_XOUT_H, raw_data, 7)) {
    this->status_set_warning();
    return;
  }
  auto *data = reinterpret_cast<int16_t *>(raw_data);

  float accel_x = data[0] * MMA8452Q_RANGE_PER_DIGIT_2G * GRAVITY_EARTH;
  float accel_y = data[1] * MMA8452Q_RANGE_PER_DIGIT_2G * GRAVITY_EARTH;
  float accel_z = data[2] * MMA8452Q_RANGE_PER_DIGIT_2G * GRAVITY_EARTH;


  ESP_LOGD(TAG,
           "Got accel={x=%.3f m/s², y=%.3f m/s², z=%.3f m/s²}, ",
           accel_x, accel_y, accel_z);

  if (this->accel_x_sensor_ != nullptr)
    this->accel_x_sensor_->publish_state(accel_x);
  if (this->accel_y_sensor_ != nullptr)
    this->accel_y_sensor_->publish_state(accel_y);
  if (this->accel_z_sensor_ != nullptr)
    this->accel_z_sensor_->publish_state(accel_z);

  this->status_clear_warning();
*/

}
float MMA8452QComponent::get_setup_priority() const { return setup_priority::DATA; }

}  // namespace mma8452q
}  // namespace esphome
