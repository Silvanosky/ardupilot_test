#pragma once

#include <inttypes.h>

#include <AP_HAL/HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/OwnPtr.h>
#include "AP_HAL_ESP32.h"

#if HAL_USE_I2C == TRUE

#include "Semaphores.h"
#include "DeviceBus.h"

#include "driver/i2c.h"

namespace ESP32
{

class I2CBus : public DeviceBus
{
public:
        I2CBus();
        ~I2CBus();

        void init();

        i2c_config_t cfg;
        uint8_t bus;
};

class I2CDevice : public AP_HAL::I2CDevice
{
public:
        static I2CDevice *from(AP_HAL::I2CDevice *dev)
        {
                return static_cast<I2CDevice *>(dev);
        }


        I2CDevice(uint8_t busnum, uint8_t address, uint32_t bus_clock,
                  bool use_smbus,
                  uint32_t timeout_ms);

        ~I2CDevice();

        /* See AP_HAL::I2CDevice::set_address() */
        void set_address(uint8_t address) override
        {
                _address = address;
        }

        /* See AP_HAL::I2CDevice::set_retries() */
        void set_retries(uint8_t retries) override
        {
                _retries = retries;
        }

        /* See AP_HAL::Device::set_speed(): Empty implementation, not supported. */
        bool set_speed(enum Device::Speed speed) override
        {
                return true;
        }

        /* See AP_HAL::Device::transfer() */
        bool transfer(const uint8_t *send, uint32_t send_len,
                      uint8_t *recv, uint32_t recv_len) override;

        bool read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                     uint32_t recv_len, uint8_t times) override;

        /* See AP_HAL::Device::register_periodic_callback() */
        AP_HAL::Device::PeriodicHandle register_periodic_callback(
                uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

        /* See AP_HAL::Device::adjust_periodic_callback() */
        bool adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h,
                                      uint32_t period_usec) override;

        AP_HAL::Semaphore *get_semaphore() override
        {
                // if asking for invalid bus number use bus 0 semaphore
                return &bus.semaphore;
        }

        void set_split_transfers(bool set) override
        {
                _split_transfers = set;
        }

private:
        I2CBus &bus;
        bool _transfer(const uint8_t *send, uint32_t send_len,
                       uint8_t *recv, uint32_t recv_len);

        /* I2C interface #2 */
        uint8_t _retries;
        uint8_t _address;

        bool _split_transfers;
        uint32_t _timeout_ms;
        bool _use_smbus;
};

class I2CDeviceManager : public AP_HAL::I2CDeviceManager
{
public:
        friend class I2CDevice;

        // constructor
        I2CDeviceManager();

        AP_HAL::OwnPtr<AP_HAL::I2CDevice> get_device(uint8_t bus, uint8_t address,
                        uint32_t bus_clock = 400000,
                        bool use_smbus = false,
                        uint32_t timeout_ms = 4) override;

        /*
          get mask of bus numbers for all configured I2C buses
         */
        uint32_t get_bus_mask(void) const override;

        /*
          get mask of bus numbers for all configured external I2C buses
         */
        uint32_t get_bus_mask_external(void) const override;

        /*
          get mask of bus numbers for all configured internal I2C buses
         */
        uint32_t get_bus_mask_internal(void) const override;


private:
        static I2CBus businfo[];

};
}

#endif // HAL_USE_I2C
