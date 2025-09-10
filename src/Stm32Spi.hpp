/*
 * SPDX-FileCopyrightText: 2024 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef LIBSMART_STM32SPI_STM32SPI_HPP
#define LIBSMART_STM32SPI_STM32SPI_HPP

#include "libsmart_config.hpp"
#include <main.h>

#ifdef HAL_SPI_MODULE_ENABLED

#include "HalStatus.hpp"
#include "Loggable.hpp"
#include "PinDigital.hpp"
#include "PinDigitalOut.hpp"
#include "spi.h"

namespace Stm32Spi {
    enum class spiState {
        STATE_RESET = 0x00U, /*!< Peripheral not Initialized                         */
        STATE_READY = 0x01U, /*!< Peripheral Initialized and ready for use           */
        STATE_BUSY = 0x02U, /*!< an internal process is ongoing                     */
        STATE_BUSY_TX = 0x03U, /*!< Data Transmission process is ongoing               */
        STATE_BUSY_RX = 0x04U, /*!< Data Reception process is ongoing                  */
        STATE_BUSY_TX_RX = 0x05U, /*!< Data Transmission and Reception process is ongoing */
        STATE_ERROR = 0x06U, /*!< SPI error state                                    */
        STATE_ABORT = 0x07U /*!< SPI abort is ongoing                               */
    };

    enum class spiError : uint32_t {
        ERROR_NONE = 0,
        ERROR_MODF = (0x00000001U),
        ERROR_CRC = (0x00000002U),
        ERROR_OVR = (0x00000004U),
        ERROR_FRE = (0x00000008U),
        ERROR_DMA = (0x00000010U),
        ERROR_FLAG = (0x00000020U),
        ERROR_ABORT = (0x00000040U),
        ERROR_INVALID_CALLBACK = (0x00000080U)
    };


    class Spi : public Stm32ItmLogger::Loggable {
    public:
        explicit Spi(SPI_HandleTypeDef *spi)
            : spi(spi) { ; }

        Spi(SPI_HandleTypeDef *spi, Stm32Gpio::PinDigitalOut *pinSS)
            : spi(spi), pinSS(pinSS) { ; }

        Spi(SPI_HandleTypeDef *spi, Stm32ItmLogger::LoggerInterface *logger)
            : Loggable(logger),
              spi(spi) { ; }

        Spi(SPI_HandleTypeDef *spi, Stm32Gpio::PinDigitalOut *pinSS, Stm32ItmLogger::LoggerInterface *logger)
            : Loggable(logger), spi(spi), pinSS(pinSS) { ; }


        /**
         * @brief Get the state of the SPI peripheral.
         *
         * This method returns the current state of the SPI peripheral.
         *
         * @return The current state of the SPI peripheral.
         */
        spiState getState() {
            return static_cast<spiState>(HAL_SPI_GetState(spi));
        }


        /**
         * @brief Check if the SPI peripheral is ready.
         *
         * This method checks if the SPI peripheral is in the ready state.
         *
         * @return true if the SPI peripheral is ready, false otherwise.
         */
        bool isReady() { return getState() == spiState::STATE_READY; }


        /**
         * @brief Get the error status of the SPI peripheral.
         *
         * This method returns the current error status of the SPI peripheral. The error status indicates any encountered errors during the SPI communication.
         *
         * @return The current error status of the SPI peripheral.
         */
        [[nodiscard]] spiError getError() const;

        /**
         * @brief Transmit data over SPI.
         *
         * This method transmits data over the SPI peripheral.
         *
         * @param pData   Pointer to the data buffer to be transmitted.
         * @param size    Size of the data buffer in bytes.
         * @param Timeout Timeout value in milliseconds for the transmission.
         *
         * @return The status of the transmission.
         */
        Stm32Common::HalStatus transmit(const uint8_t *pData, const uint16_t size, const uint32_t Timeout) const;


        /**
         * @brief Transmit data over SPI.
         *
         * This method transmits data over the SPI peripheral.
         *
         * @param pData   Pointer to the data buffer to be transmitted.
         * @param size    Size of the data buffer in bytes.
         *
         * @return The status of the transmission.
         */
        Stm32Common::HalStatus transmit(const uint8_t *pData, const uint16_t size) const;


        /**
         * @brief Transmit data over SPI.
         *
         * This method transmits data over the SPI peripheral.
         *
         * @param data    The single byte of data to be transmitted.
         *
         * @return The status of the transmission.
         */
        Stm32Common::HalStatus transmit(const uint8_t data) const;


        /**
         * @brief Transmit data over SPI.
         *
         * This method transmits data over the SPI peripheral.
         *
         * @param data The 32-bit data to be transmitted.
         *
         * @return The status of the transmission.
         */
        // Stm32Common::HalStatus transmit(const uint32_t data) const {
        //     return transmit(reinterpret_cast<const uint8_t *>(&data), 4, LIBSMART_STM32SPI_DEFAULT_TIMEOUT);
        // }


        /**
         * @brief Transmit a 32-bit data to the SPI peripheral using big-endian format.
         *
         * This method converts the given 32-bit data to a byte array in big-endian format,
         * and then transmits it to the SPI peripheral using the `transmit` method with a
         * default timeout value.
         *
         * @param data The 32-bit data to be transmitted.
         * @return The status of the transmission operation.
         */
        Stm32Common::HalStatus transmit_be(const uint32_t data) const;


        /**
         * @brief Transmit a 16-bit data to the SPI peripheral using big-endian format.
         *
         * This method converts the given 16-bit data to a byte array in big-endian format,
         * and then transmits it to the SPI peripheral using the `transmit` method with a
         * default timeout value.
         *
         * @param data The 16-bit data to be transmitted.
         * @return The status of the transmission operation.
         */
        Stm32Common::HalStatus transmit_be(const uint16_t data) const;

        /**
         * @brief Transmit data using the SPI peripheral.
         *
         * This method transmits the provided data using the SPI peripheral.
         *
         * @param data The data to transmit.
         * @param size The size of the data in bytes.
         * @return The status of the transmission.
         */
        Stm32Common::HalStatus transmit(const char *data, const uint16_t size) const;


        /**
         * @brief Transmit data using SPI.
         *
         * This method transmits data using the SPI peripheral. It accepts a char array as input
         * data and internally converts it to a uint8_t array before transmitting. The length of
         * the data is determined by computing the string length of the input data. The timeout
         * value used for the transmission is the default timeout value provided by the library.
         *
         * @param data C string containing the data to transmit.
         * @return The status of the transmission.
         */
        Stm32Common::HalStatus transmit(const char *data) const;


        /**
         * @brief Receive data from the SPI peripheral.
         *
         * This method receives data from the SPI peripheral and stores it in the provided buffer.
         *
         * @param pData A pointer to the buffer where the received data will be stored.
         * @param size The size of the buffer in bytes.
         * @param timeout The timeout duration for the receive operation in milliseconds.
         *
         * @return The status of the receive operation.
         */
        Stm32Common::HalStatus receive(uint8_t *pData, const uint16_t size, const uint32_t timeout) const;


        /**
         * @brief Receive data from the SPI peripheral.
         *
         * This method is used to receive data from the SPI peripheral.
         *
         * @param pData Pointer to a buffer where the received data will be stored.
         * @param size The number of bytes to receive.
         * @return The status of the receive operation.
         */
        Stm32Common::HalStatus receive(uint8_t *pData, const uint16_t size) const;


        void select() const;

        void unselect() const;

    protected:
        SPI_HandleTypeDef *spi;
        Stm32Gpio::PinDigitalOut *pinSS = {};
    };
}
#endif
#endif
