/* Includes */

#include <iostream>
#include <fcntl.h>
extern "C"{
    #include<linux/i2c-dev.h>
    #include <i2c/smbus.h>
}
#include <sys/ioctl.h>
#include <unistd.h>
#include <cmath>
#include "INA233.h"

/* Definitions */

/**************************************************************************/
/*!
    @brief Instantiates a new INA233 class
*/
/**************************************************************************/
INA233::INA233(int bus, uint8_t address) : bus(bus), address(address){
    char filename[20];
    snprintf(filename,  19, "/dev/i2c-%d", bus);

    // Open the I2C bus
    file = open(filename, O_RDWR);
    if (file <  0) {
        std::cerr << "Failed to open the I2C bus" << '\n';
        exit(1);
    }

    // Set the I2C slave address
    if (ioctl(file, I2C_SLAVE, address) <  0) {
        std::cerr << "Failed to acquire bus access and/or talk to slave" << '\n';
        exit(1);
    }
}

/**************************************************************************/
/*!
    @brief Destructor of the class INA233.
*/
/**************************************************************************/
INA233::~INA233(){
    close(file);
}

/**************************************************************************/
/*!
    @brief Writes a 16-bit word to a register on the INA233.
*/
/**************************************************************************/
int INA233::write_word_data(int fd, uint8_t reg, uint16_t data){
    char charReg = (char) reg;
    int result = i2c_smbus_write_word_data(fd, charReg, data);
    if (result <  0 && errno != EOPNOTSUPP) {
        // Handle the error, i2c_smbus_write_word_data returns -1 on failure
        // errno is set to the error number
        perror("Error in write_word_data");
    }
    return result;
}

/**************************************************************************/
/*!
    @brief Reads a 16-bit word to a register on the INA233.
*/
/**************************************************************************/
int16_t INA233::read_word_data(uint8_t reg){
    int result = i2c_smbus_read_word_data(file, reg);
    if (result <  0 && errno != EOPNOTSUPP) {
        // Handle the error, i2c_smbus_write_word_data returns -1 on failure
        // errno is set to the error number
        perror("Error in read_word_data");
    }
    return static_cast<int16_t>(result);
}

/**************************************************************************/
/*!
    @brief Calibration and scaling values per section 7.5.2 of TI INA233.
*/
/**************************************************************************/
void INA233::calibrate(double R_shunt, double I_max){
    // Calibration and scaling values per section 7.5.2 of TI INA233 datasheet
    Current_LSB = 0;
    Power_LSB = 0;
    ERROR = 0;
    double CAL = 0;
    int32_t aux = 0;
    bool round_done = false;
    float m_c_F = 0;
    float m_p_F = 0;
    int8_t local_R_c = 0;
    int8_t local_R_p = 0;

    Current_LSB = I_max / pow(2, 15);
    Power_LSB = 25 * Current_LSB;
    Current_LSB *= 1000000;  // Convert to microamps
    Power_LSB *= 1000;         // Convert to milliwatts
    CAL = 0.00512 / (R_shunt * Current_LSB);

    // Check if CAL is in the uint16 range
    if (CAL > 0xFFFF){
        ERROR = 1;
    }else{
        ERROR = 0;  
        int result = write_word_data(file, MFR_CALIBRATION, static_cast<uint16_t>(CAL));
        if (result < 0) {
            std::cerr << "Failed to write to the I2C bus" << '\n';
        }
    }

    m_c_F = 1 / Current_LSB;
    m_p_F = 1 / Power_LSB;

    // Calculate m and R for maximum accuracy in current measurement
    aux = static_cast<int32_t>(m_c_F);
    while((aux > 32768) || (aux < -32768)){
        m_c_F = m_c_F / 10;
        local_R_c++;
        aux = static_cast<int32_t>(m_c_F);
    }

    while(!round_done){
        aux = static_cast<int32_t>(m_c_F);
        if(aux == m_c_F){
            round_done = true;
        }else{
            aux = static_cast<int32_t>(m_c_F * 10); // Shift decimal to the right
            if((aux > 32768) || (aux < -32768)){ // m_c is out of int16 (-32768 to 32768)
                round_done = true;
            }else{
                m_c_F = m_c_F * 10;
                local_R_c--;
            }
        }
    }
    round_done = false;

    // Calculate m and R for maximum accuracy in power measurement
    aux = static_cast<int32_t>(m_p_F);
    while(aux > 32768 || aux < -32768){
        m_p_F = m_p_F / 10;
        local_R_p++;
        aux = static_cast<int32_t>(m_p_F);
    }
    while(!round_done){
        aux = static_cast<int32_t>(m_p_F);
        if(aux == m_p_F){
            round_done = true;
        }else{
            aux = static_cast<int32_t>(m_p_F * 10); // Shift decimal to the right
            if(aux > 32768 || aux < -32768){ // m_p is out of int16 (-32768 to 32768)
                round_done = true;
            }else{
                m_p_F = m_p_F * 10;
                local_R_p--;
            }
        }
    }

    m_p = static_cast<uint16_t>(m_p_F);
    m_c = static_cast<uint16_t> (m_c_F);
    R_c = local_R_c;
    R_p = local_R_p;
    CAL = static_cast<uint16_t>(CAL);
}

/**************************************************************************/
/*!
    @brief  Gets the raw bus voltage input (2-byte, two's complement integer
    received from the device)
*/
/**************************************************************************/
int16_t INA233::getBusVoltageIn_raw(){
    uint16_t value;
    value = i2c_smbus_read_word_data(file, READ_VIN);

    return static_cast<int16_t>(value);
}

/**************************************************************************/
/*!
    @brief  Gets the raw bus voltage input (2-byte, two's complement integer
    received from the device)
*/
/**************************************************************************/
int16_t INA233::getBusVoltageOut_raw(){
    uint16_t value;
    value = i2c_smbus_read_word_data(file, READ_VOUT);

    return static_cast<int16_t>(value);
}

/**************************************************************************/
/*!
    @brief  Gets the raw shunt voltage (2-byte, two's complement integer
    received from the device)
*/
/**************************************************************************/
int16_t INA233::getShuntVoltage_raw(){
  uint16_t value;
  value = i2c_smbus_read_word_data(file, MFR_READ_VSHUNT);

  return static_cast<int16_t>(value);   
}

/**************************************************************************/
/*!
    @brief  Gets the raw current input value (2-byte, two's complement integer
    received from the device)
*/
/**************************************************************************/
int16_t INA233::getCurrentIn_raw(){
  uint16_t value;
  value = i2c_smbus_read_word_data(file, READ_IIN);

  return static_cast<int16_t>(value);  
}

/**************************************************************************/
/*!
    @brief  Gets the raw current output value (2-byte, two's complement integer
    received from the device)
*/
/**************************************************************************/
int16_t INA233::getCurrentOut_raw(){
  uint16_t value;
  value = i2c_smbus_read_word_data(file, READ_IOUT);

  return static_cast<int16_t>(value);  
}

/**************************************************************************/
/*!
    @brief  Gets the raw power value (2-byte, two's complement integer
    received from the device)
*/
/**************************************************************************/
int16_t INA233::getPower_raw(){
  uint16_t value;
  value = i2c_smbus_read_word_data(file, READ_PIN);

  return static_cast<int16_t>(value);  
}

/**************************************************************************/
/*!
    @brief  Gets the raw energy info from READ_EIN register power accumulator
    (2-byte), power accumulator roll over (1byte) and sample count (3bytes)

*/
/**************************************************************************/
void INA233::getEnergy_raw(uint16_t* accumulator, uint8_t* roll_over, uint32_t* sample_count){
    uint8_t value[6];
    uint32_t aux;

    // Read the six bytes starting from the READ_EIN register
    for(int i =  0; i <  6; ++i){
        value[i] = i2c_smbus_read_byte_data(file, READ_EIN + i);
    }

    // Combine the read bytes to form the accumulator, roll_over, and sample_count
    *accumulator = (value[1] <<  8) | value[0];
    *roll_over = value[2];
    *sample_count = static_cast<uint32_t>(value[5]) <<  16;
    *sample_count |= static_cast<uint32_t>(value[4]) <<  8;
    *sample_count |= static_cast<uint32_t>(value[3]);
}

/**************************************************************************/
/*!
    @brief  Gets the averaged power from last reading of READ_EIN in mW
*/
/**************************************************************************/
float INA233::getAv_Power_mW(){
  uint16_t accumulator = 0;
  uint8_t roll_over = 0;
  uint32_t sample_count = 0;
  uint32_t accumulator_24 = 0;
  uint32_t raw_av_power = 0;
  float av_power = 0;
  getEnergy_raw(&accumulator,&roll_over, &sample_count);
  accumulator_24 = uint32_t(roll_over)*65536 + uint32_t(accumulator);
  raw_av_power = accumulator_24 / sample_count;

  av_power = (raw_av_power*pow(10,-R_p)-b_p)/m_p;
  return av_power * 1000;
}

/**************************************************************************/
/*!
    @brief  Gets the shunt voltage in mV
*/
/**************************************************************************/
float INA233::getShuntVoltage_mV(){
  int16_t value = getShuntVoltage_raw();
  float vshunt;
  vshunt = (value*pow(10,-R_vs)-b_vs)/m_vs;
  return vshunt * 1000;
}

/**************************************************************************/
/*!
    @brief  Gets the shunt voltage in volts
*/
/**************************************************************************/
float INA233::getBusVoltage_V(){
  int16_t value = getBusVoltageIn_raw();
  float vbus;
  vbus = (value*pow(10,-R_vb)-b_vb)/m_vb;
  return vbus;
}

/**************************************************************************/
/*!
    @brief  Gets the current value in mA, taking into account the
            config settings and current LSB
*/
/**************************************************************************/
float INA233::getCurrent_mA(){
  int16_t value = getCurrentIn_raw();
  float current;
  current = (value*pow(10,-R_c)-b_c)/m_c;
  return current*1000;
}

/**************************************************************************/
/*!
    @brief  Gets the power value in mW, taking into account the
            config settings and power LSB
*/
/**************************************************************************/
float INA233::getPower_mW(){
  int16_t value = getPower_raw();
  float power;
  power = (value*pow(10,-R_p)-b_p)/m_p;
  return power*1000;
}