/**
 * @file wb-idf-i2c.h
 * @brief Whirlingbits I2C Driver API for ESP-IDF.
 * @author Whirlingbits
 * @date 2024
 *
 * This file provides high-level I2C master functions for ESP-IDF projects.
 */

#ifndef _I2C_BUS_H_
#define _I2C_BUS_H_

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "driver/i2c_master.h"


#ifdef __cplusplus
extern "C" {
#endif

extern i2c_master_bus_handle_t bus_handle;

/**************************************** Public Functions (Application level)*********************************************/

/**
 * @brief Initializes the I2C master bus.
 *
 * This function initializes an I2C master bus using the provided SCL and SDA GPIO pins.
 * It configures the bus with default clock source, internal pull-ups enabled, glitch ignore count,
 * and uses the configuration specified by CONFIG_I2C_NUM. If the bus is already initialized or if
 * initialization fails, it returns ESP_FAIL.
 *
 * @param i2c_scl The GPIO number for the I2C SCL pin.
 * @param i2c_sda The GPIO number for the I2C SDA pin.
 * @return esp_err_t Returns ESP_OK on success, or an error code if initialization fails.
 */
esp_err_t wb_i2c_master_bus_init(i2c_port_num_t i2c_port, gpio_num_t i2c_scl, gpio_num_t i2c_sda);

/**
 * @brief Creates a new I2C master device handle.
 *
 * This function creates a new I2C master device handle for the specified bus and device address. The clock speed is also set to the default value of 100 kHz.
 *
 * @param bus_handle The I2C bus handle.
 * @param dev_addr The device address.
 * @param clk_speed The clock speed in Hz.
 * @return i2c_master_dev_handle_t The new I2C master device handle.
 */
i2c_master_dev_handle_t wb_i2c_master_device_create(i2c_master_bus_handle_t bus_handle, uint8_t dev_addr, uint32_t clk_speed);

/**
 * @brief Probes the device at the specified address on the I2C bus.
 *
 * This function probes the device at the specified address on the I2C bus to determine if it is present and responding.
 *
 * @param bus_handle Handle to the I2C master bus.
 * @param dev_addr Address of the device to probe.
 * @param timeout Timeout in milliseconds for the probe operation.
 *
 * @return ESP_OK if successful, error code otherwise.
 */
esp_err_t wb_i2c_master_bus_probe_device(i2c_master_bus_handle_t bus_handle, uint16_t dev_addr, uint32_t timeout);

/**
 * @brief Deletes an I2C master bus handle.
 *
 * This function deletes an I2C master bus handle and releases any resources associated with it.
 *
 * @param[in] bus_handle Handle to the I2C master bus to delete.
 *
 * @return
 * - ESP_OK on success
 * - ESP_ERR_INVALID_ARG if the handle is invalid
 */
esp_err_t wb_i2c_master_bus_delete(i2c_master_bus_handle_t bus_handle);

/**
 * @brief Delete an I2C master device handle.
 *
 * This function deletes an I2C master device handle created by `wb_i2c_master_device_create()`.
 *
 * @param dev_handle Handle to the I2C master device to delete.
 *
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t wb_i2c_master_device_delete(i2c_master_dev_handle_t dev_handle);

/*------------------- I2C master byte operations -------------------*/

/**
 * @brief Reads a byte from the I2C bus.
 *
 * This function reads a single byte from the I2C bus using the specified device handle and memory address.
 *
 * @param dev_handle Handle to the I2C master device.
 * @param mem_address Memory address on the I2C bus.
 * @param data Pointer to the location where the read data will be stored.
 *
 * @return ESP_OK if successful, error code otherwise.
 */
esp_err_t wb_i2c_master_bus_read_byte(i2c_master_dev_handle_t dev_handle, uint8_t mem_address, uint8_t *data);

/**
 * @brief Write a byte to the I2C bus.
 *
 * This function writes a single byte to the specified memory address on the I2C bus.
 *
 * @param dev_handle Handle to the I2C master device.
 * @param mem_address Memory address to write to.
 * @param data Byte to write.
 *
 * @return ESP_OK if successful, error code otherwise.
 */
esp_err_t wb_i2c_master_bus_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t mem_address, uint8_t data);

/**
 * @brief Read multiple bytes from an I2C device
 *
 * This function reads multiple bytes from an I2C device.
 *
 * @param dev_handle Handle to the I2C device
 * @param mem_address Memory address of the first byte to read
 * @param data Pointer to the buffer where the read data will be stored
 * @param length Number of bytes to read
 *
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t wb_i2c_master_bus_read_multiple_bytes(i2c_master_dev_handle_t dev_handle, uint8_t mem_address, uint8_t data[], uint8_t length);

/**
 * @brief Write multiple bytes to an I2C device
 *
 * This function writes multiple bytes to an I2C device.
 *
 * @param dev_handle Handle of the I2C device
 * @param mem_address Memory address of the first byte to write
 * @param data Pointer to the data buffer to write
 * @param length Number of bytes to write
 *
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t wb_i2c_master_bus_write_multiple_bytes(i2c_master_dev_handle_t dev_handle, uint8_t mem_address, uint8_t* data, uint8_t length);

/**
 * @brief Reads a single byte from the I2C bus and returns the value of the bit at the specified position.
 *
 * @param dev_handle Handle to the I2C master device.
 * @param mem_address Memory address to read from.
 * @param bit_num Bit number to read from (0-7).
 * @param data Pointer to the variable where the read value will be stored.
 *
 * @return ESP_OK if successful, error code otherwise.
 */
esp_err_t wb_i2c_master_bus_read_byte_bit(i2c_master_dev_handle_t dev_handle, uint8_t mem_address, uint8_t bit_num, uint8_t *data);

/**
 * @brief Write a single byte to an I2C device at a specific bit position.
 *
 * This function writes a single byte to an I2C device at the specified memory address and bit position.
 * The bit position is specified as an offset from the least significant bit of the byte.
 *
 * @param dev_handle Handle to the I2C master device.
 * @param mem_address Memory address of the I2C device.
 * @param bit_num Bit position within the byte to write to.
 * @param data Byte value to write to the specified bit position.
 *
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t wb_i2c_master_bus_write_byte_bit(i2c_master_dev_handle_t dev_handle, uint8_t mem_address, uint8_t bit_num, uint8_t data);

/**
 * @brief Reads a byte from an I2C device and returns the value of the specified bits.
 *
 * This function reads a byte from an I2C device and returns the value of the specified bits. The `mem_address` parameter specifies the memory address to read from, and the `bit_start` and `length` parameters specify which bits to extract from the read data.
 *
 * @param dev_handle Handle to the I2C device.
 * @param mem_address Memory address to read from.
 * @param bit_start Bit number of the first bit to extract (0-7).
 * @param length Number of bits to extract (1-8).
 * @param data Pointer to a variable where the extracted bits will be stored.
 *
 * @return ESP_OK if successful, error code otherwise.
 */
esp_err_t wb_i2c_master_bus_read_byte_bits(i2c_master_dev_handle_t dev_handle, uint8_t mem_address, uint8_t bit_start, uint8_t length, uint8_t *data);
/**
 * @brief Write a byte to an I2C device with bit-level addressing.
 *
 * This function writes a byte to an I2C device using bit-level addressing. The `mem_address` parameter specifies the memory address of the device, and the `bit_start` parameter specifies the starting bit position within the byte. The `length` parameter specifies the number of bits to write, and the `data` parameter specifies the data to be written.
 *
 * @param dev_handle Handle to the I2C master driver instance.
 * @param mem_address Memory address of the device.
 * @param bit_start Starting bit position within the byte.
 * @param length Number of bits to write.
 * @param data Data to be written.
 *
 * @return ESP_OK if successful, otherwise an error code.
 */

esp_err_t wb_i2c_master_bus_write_byte_bits(i2c_master_dev_handle_t dev_handle, uint8_t mem_address, uint8_t bit_start, uint8_t length, uint8_t data);

/*------------------- I2C master word operations -------------------*/

/**
 * @brief Reads a word from the I2C bus.
 *
 * This function reads a word from the I2C bus using the specified device handle and memory address.
 *
 * @param dev_handle Handle to the I2C master device.
 * @param mem_address Memory address on the I2C slave device.
 * @param data Pointer to the variable where the read data will be stored.
 *
 * @return ESP_OK if successful, error code otherwise.
 */
esp_err_t wb_i2c_master_bus_read_word(i2c_master_dev_handle_t dev_handle, uint8_t mem_address, uint16_t *data);

/**
 * @brief Write a word to the I2C bus.
 *
 * This function writes a word to the I2C bus.
 *
 * @param dev_handle Handle to the I2C master device.
 * @param mem_address Memory address on the I2C slave device.
 * @param data Word to be written to the I2C slave device.
 *
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t wb_i2c_master_bus_write_word(i2c_master_dev_handle_t dev_handle, uint8_t mem_address, uint16_t data);

/**
 * @brief Reads a word from the I2C bus and returns the bit value at the specified position.
 *
 * This function reads a word from the I2C bus and returns the bit value at the specified position. The bit numbering starts from 0, where the least significant bit is bit 0.
 *
 * @param dev_handle Handle to the I2C master device.
 * @param mem_address Memory address of the word to read.
 * @param bit_num Bit number within the word to read.
 * @param data Pointer to a variable where the read value will be stored.
 *
 * @return ESP_OK if successful, error code otherwise.
 */
esp_err_t wb_i2c_master_bus_read_word_bit(i2c_master_dev_handle_t dev_handle, uint8_t mem_address, uint8_t bit_num, uint8_t *data);

/**
 * @brief Write a word to an I2C device at a specific bit position.
 *
 * This function writes a word to an I2C device at a specific bit position. The bit position is specified by the `bit_num` parameter, and the data to be written is specified by the `data` parameter.
 *
 * @param dev_handle Handle to the I2C master driver instance.
 * @param mem_address Memory address of the device on the bus.
 * @param bit_num Bit position within the word to write to.
 * @param data Data to be written to the specified bit position.
 *
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t wb_i2c_master_bus_write_word_bit(i2c_master_dev_handle_t dev_handle, uint8_t mem_address, uint8_t bit_num, uint8_t data);

/**
 * @brief Reads a word from the I2C bus and returns the data as an unsigned 16-bit integer.
 *
 * This function reads a word from the I2C bus using the specified device handle, memory address, bit start position, and length. The read data is returned in the `data` parameter as an unsigned 16-bit integer.
 *
 * @param dev_handle Handle to the I2C master device.
 * @param mem_address Memory address on the I2C bus.
 * @param bit_start Bit start position of the word to read.
 * @param length Length of the word to read in bits.
 * @param data Pointer to the location where the read data will be stored.
 *
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t wb_i2c_master_bus_read_word_bits(i2c_master_dev_handle_t dev_handle, uint8_t mem_address, uint8_t bit_start, uint8_t length, uint16_t *data);

/**
 * @brief Write a word to the I2C bus with bit-level addressing.
 *
 * This function writes a 16-bit word to the I2C bus with bit-level addressing. The `mem_address` parameter specifies the memory address of the device, and the `bit_start` parameter specifies the starting bit position within the word. The `length` parameter specifies the number of bits to write, and the `data` parameter specifies the data to be written.
 *
 * @param dev_handle Handle to the I2C master driver instance.
 * @param mem_address Memory address of the device.
 * @param bit_start Starting bit position within the word.
 * @param length Number of bits to write.
 * @param data Data to be written.
 *
 * @return ESP_OK if successful, error code otherwise.
 */
esp_err_t wb_i2c_master_bus_write_word_bits(i2c_master_dev_handle_t dev_handle, uint8_t mem_address, uint8_t bit_start, uint8_t length, uint16_t data);

#ifdef __cplusplus
}
#endif

#endif