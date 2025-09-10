/*
 * SPDX-FileCopyrightText: 2024 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Stm32Spi.hpp"

#ifdef HAL_SPI_MODULE_ENABLED

using namespace Stm32Spi;


spiError Spi::getError() const {
    return static_cast<spiError>(HAL_SPI_GetError(spi));
}

Stm32Common::HalStatus Spi::transmit(const uint8_t *pData, const uint16_t size, const uint32_t Timeout) const {
    log(Stm32ItmLogger::LoggerInterface::Severity::DEBUGGING)
            ->print("Stm32Spi::Spi::transmit(");
    for (uint8_t i = 0; i < std::min(size, static_cast<uint16_t>(8)); i++) {
        log(Stm32ItmLogger::LoggerInterface::Severity::DEBUGGING)
                ->printf("%s0x%02x", i == 0 ? "" : " ", pData[i]);
    }
    log(Stm32ItmLogger::LoggerInterface::Severity::DEBUGGING)
            ->println(size > 8 ? "...)" : ")");


    const auto ret = HAL_SPI_Transmit(spi, const_cast<uint8_t *>(pData), size, Timeout);
    return static_cast<Stm32Common::HalStatus>(ret);
}

Stm32Common::HalStatus Spi::transmit(const uint8_t *pData, const uint16_t size) const {
    return transmit(pData, size, LIBSMART_STM32SPI_DEFAULT_TIMEOUT);
}

Stm32Common::HalStatus Spi::transmit(const uint8_t data) const {
    return transmit(&data, 1, LIBSMART_STM32SPI_DEFAULT_TIMEOUT);
}

Stm32Common::HalStatus Spi::transmit_be(const uint32_t data) const {
    uint8_t d[4];
    d[0] = (data & 0xff000000) >> 24u;
    d[1] = (data & 0x00ff0000) >> 16u;
    d[2] = (data & 0x0000ff00) >> 8u;
    d[3] = (data & 0x000000ff);
    return transmit(d, 4, LIBSMART_STM32SPI_DEFAULT_TIMEOUT);
}

Stm32Common::HalStatus Spi::transmit_be(const uint16_t data) const {
    uint8_t d[2];
    d[0] = (data & 0x0000ff00) >> 8u;
    d[1] = (data & 0x000000ff);
    return transmit(d, 2, LIBSMART_STM32SPI_DEFAULT_TIMEOUT);
}

Stm32Common::HalStatus Spi::transmit(const char *data, const uint16_t size) const {
    return transmit(reinterpret_cast<const uint8_t *>(data), size, LIBSMART_STM32SPI_DEFAULT_TIMEOUT);
}

Stm32Common::HalStatus Spi::transmit(const char *data) const {
    return transmit(reinterpret_cast<const uint8_t *>(data), strlen(data), LIBSMART_STM32SPI_DEFAULT_TIMEOUT);
}

Stm32Common::HalStatus Spi::receive(uint8_t *pData, const uint16_t size, const uint32_t timeout) const {
    log(Stm32ItmLogger::LoggerInterface::Severity::DEBUGGING)
            ->printf("Stm32Spi::Spi::receive(%p, %lu, %lu)\r\n", &pData, size, timeout);

    const auto ret = HAL_SPI_Receive(spi, pData, size, timeout);
    if (ret != static_cast<uint32_t>(Stm32Common::HalStatus::HAL_OK)) {
        log(Stm32ItmLogger::LoggerInterface::Severity::ERROR)
                ->printf("HAL_SPI_Receive() = 0x%02x\r\n", ret);
    }
    return static_cast<Stm32Common::HalStatus>(ret);
}

Stm32Common::HalStatus Spi::receive(uint8_t *pData, const uint16_t size) const {
    return receive(pData, size, LIBSMART_STM32SPI_DEFAULT_TIMEOUT);
}

void Spi::select() const { pinSS == nullptr ? (void) 0 : pinSS->setOn(); }

void Spi::unselect() const { pinSS == nullptr ? (void) 0 : pinSS->setOff(); }

#endif
