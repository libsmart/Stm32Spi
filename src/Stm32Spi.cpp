/*
 * SPDX-FileCopyrightText: 2026 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Stm32Spi.hpp"

#ifdef HAL_SPI_MODULE_ENABLED

using namespace Stm32Spi;
using namespace Stm32Common;
using Severity = Stm32ItmLogger::LoggerInterface::Severity;

spiError Spi::getError() const {
    return static_cast<spiError>(HAL_SPI_GetError(spi));
}

HalStatus Spi::transmit(const uint8_t *pData, const uint16_t size, const uint32_t timeout) {
    log(Severity::DEBUGGING)->print("Stm32Spi::Spi::transmit(");
    for (uint8_t i = 0; i < std::min(size, static_cast<uint16_t>(8)); i++) {
        log(Severity::DEBUGGING)->printf("%s0x%02x", i == 0 ? "" : " ", pData[i]);
    }
    log(Severity::DEBUGGING)->println(size > 8 ? "...)" : ")");

    // if (waitForReadyState(timeout) != Stm32Common::HalStatus::HAL_OK) return Stm32Common::HalStatus::HAL_TIMEOUT;

    // __disable_irq();
    // const auto ret = HAL_SPI_Transmit(spi, const_cast<uint8_t *>(pData), size, timeout);
    // __enable_irq();
    // if (ret != static_cast<uint32_t>(Stm32Common::HalStatus::HAL_OK)) {
    // log(Severity::ERROR)->printf("HAL_SPI_Transmit() = 0x%02x\r\n", ret);
    // }


    const auto ret = HAL_SPI_Transmit_DMA(spi, const_cast<uint8_t *>(pData), size);
    if (ret != static_cast<uint32_t>(HalStatus::HAL_OK)) {
        log(Severity::ERROR)->printf("HAL_SPI_Transmit_DMA() = 0x%02x\r\n", ret);
        return static_cast<HalStatus>(ret);
    }

    try {
        txCpltIsr.get(timeout);
    } catch (...) {
        return HalStatus::HAL_TIMEOUT;
    }

    return static_cast<HalStatus>(ret);
}

HalStatus Spi::transmit(const uint8_t *pData, const uint16_t size) {
    return transmit(pData, size, DEFAULT_TIMEOUT);
}

HalStatus Spi::transmit(const uint8_t data) {
    return transmit(&data, 1, DEFAULT_TIMEOUT);
}

HalStatus Spi::transmit_be(const uint32_t data) {
    uint8_t d[4];
    d[0] = (data & 0xff000000) >> 24u;
    d[1] = (data & 0x00ff0000) >> 16u;
    d[2] = (data & 0x0000ff00) >> 8u;
    d[3] = (data & 0x000000ff);
    return transmit(d, 4, DEFAULT_TIMEOUT);
}

HalStatus Spi::transmit_be(const uint16_t data) {
    uint8_t d[2];
    d[0] = (data & 0x0000ff00) >> 8u;
    d[1] = (data & 0x000000ff);
    return transmit(d, 2, DEFAULT_TIMEOUT);
}

HalStatus Spi::transmit(const char *data, const uint16_t size) {
    return transmit(reinterpret_cast<const uint8_t *>(data), size, DEFAULT_TIMEOUT);
}

HalStatus Spi::transmit(const char *data) {
    return transmit(reinterpret_cast<const uint8_t *>(data), strlen(data), DEFAULT_TIMEOUT);
}

HalStatus Spi::receive(uint8_t *pData, const uint16_t size, const uint32_t timeout) {
    log(Severity::DEBUGGING)->printf("Stm32Spi::Spi::receive(%p, %lu, %lu)\r\n", &pData, size, timeout);

    // if (waitForReadyState(timeout) != Stm32Common::HalStatus::HAL_OK) return Stm32Common::HalStatus::HAL_TIMEOUT;

    // __disable_irq();
    // const auto ret = HAL_SPI_Receive(spi, pData, size, timeout);
    // __enable_irq();
    // if (ret != static_cast<uint32_t>(Stm32Common::HalStatus::HAL_OK)) {
    // log(Stm32ItmLogger::LoggerInterface::Severity::ERROR)
    // ->printf("HAL_SPI_Receive() = 0x%02x\r\n", ret);
    // }

    const auto ret = HAL_SPI_Receive_DMA(spi, pData, size);
    if (ret != static_cast<uint32_t>(HalStatus::HAL_OK)) {
        log(Severity::ERROR)->printf("HAL_SPI_Receive_DMA() = 0x%02x\r\n", ret);
        return static_cast<HalStatus>(ret);
    }
    try {
        rxCpltIsr.get(timeout);
    } catch (...) {
        return HalStatus::HAL_TIMEOUT;
    }

    return static_cast<HalStatus>(ret);
}

HalStatus Spi::receive(uint8_t *pData, const uint16_t size) {
    return receive(pData, size, DEFAULT_TIMEOUT);
}

HalStatus Spi::waitForReadyState(const uint32_t timeout) const {
    const auto startMillis = millis();
    while (millis() - startMillis < timeout) {
        if (HAL_SPI_GetState(spi) == HAL_SPI_STATE_READY) return HalStatus::HAL_OK;
        delay(1);
    }
    return HalStatus::HAL_TIMEOUT;
}

void Spi::select() const { pinSS == nullptr ? (void) 0 : pinSS->setOn(); }

void Spi::unselect() const { pinSS == nullptr ? (void) 0 : pinSS->setOff(); }

void Spi::setup() {
    rxCpltIsr.setup();
    txCpltIsr.setup();
    txRxCpltIsr.setup();
}

#endif
