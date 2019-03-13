#include "I2CDevice.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/OwnPtr.h>
#include "Scheduler.h"
#include "Semaphores.h"
#include <stdio.h>

static const struct I2CInfo {
        i2c_mode_t mode;       /*!< I2C mode */
        gpio_num_t sda_io_num;        /*!< GPIO number for I2C sda signal */
        gpio_num_t scl_io_num;        /*!< GPIO number for I2C scl signal */
        gpio_pullup_t sda_pullup_en;  /*!< Internal GPIO pull mode for I2C sda signal*/
        gpio_pullup_t scl_pullup_en;  /*!< Internal GPIO pull mode for I2C scl signal*/
        uint32_t clk_speed;     /*!< I2C clock frequency for master mode, (no higher than 1MHz for now) */
} I2CD[] = {
        {
                I2C_MODE_MASTER,
                (gpio_num_t) 25,
                (gpio_num_t) 26,

                GPIO_PULLUP_ENABLE,
                GPIO_PULLUP_ENABLE,

                100000,
        },
        {
                I2C_MODE_MASTER,
                (gpio_num_t) 18,
                (gpio_num_t) 19,

                GPIO_PULLUP_ENABLE,
                GPIO_PULLUP_ENABLE,

                100000,
        }
};

using namespace ESP32;

I2CBus I2CDeviceManager::businfo[ARRAY_SIZE(I2CD)];

#ifndef HAL_I2C_BUS_BASE
#define HAL_I2C_BUS_BASE 0
#endif

// default to 100kHz clock for maximum reliability. This can be
// changed in hwdef.dat
#ifndef HAL_I2C_MAX_CLOCK
#define HAL_I2C_MAX_CLOCK 100000
#endif


#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            (i2c_ack_type_t)0x0              /*!< I2C ack value */
#define NACK_VAL                           (i2c_ack_type_t)0x1              /*!< I2C nack value */


I2CBus::I2CBus():
        DeviceBus(Scheduler::I2C_PRIORITY)
{}

void I2CBus::init()
{
        i2c_param_config((i2c_port_t)bus, &cfg);
        //No buffer for the master i2c
        i2c_driver_install((i2c_port_t)bus, cfg.mode, 0, 0, 0);
}

I2CDevice::I2CDevice(uint8_t busnum, uint8_t address, uint32_t bus_clock,
                     bool use_smbus, uint32_t timeout_ms) :
        _retries(2),
        _address(address),
        _use_smbus(use_smbus),
        _timeout_ms(timeout_ms),
        bus(I2CDeviceManager::businfo[busnum])
{
        set_device_bus(bus.bus);
        set_device_address(address);
        set_address(address);

        //TODO handle clock speed

	printf("i2c device constructed address: %x  bus: %x\n", address, busnum);
}

I2CDevice::~I2CDevice()
{}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
        if (_split_transfers) {
                /*
                  splitting the transfer() into two pieces avoids a stop condition
                  with SCL low which is not supported on some devices (such as
                  LidarLite blue label)
                */
                if (send && send_len) {
                        if (!_transfer(send, send_len, nullptr, 0)) {
                                return false;
                        }
                }
                if (recv && recv_len) {
                        if (!_transfer(nullptr, 0, recv, recv_len)) {
                                return false;
                        }
                }
        } else {
                // combined transfer
                if (!_transfer(send, send_len, recv, recv_len)) {
                        return false;
                }
        }

        return true;
}

bool I2CDevice::_transfer(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len)
{
        for (uint8_t i = 0; i <= _retries; i++) {
                i2c_cmd_handle_t cmd = i2c_cmd_link_create();
                i2c_master_start(cmd);

                if (0 < send_len) {
                        i2c_master_write_byte(cmd, (_address << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);

			//Because i2c_master_write doesn't have const data parameter
			for (uint32_t j = 0; j < send_len; ++j)
				i2c_master_write_byte(cmd, send[j], ACK_CHECK_EN);
                }

                if (0 < recv_len) {
                        i2c_master_write_byte(cmd, (_address << 1 ) | I2C_MASTER_READ, ACK_CHECK_EN);
                        if (1 < recv_len) {
                                i2c_master_read(cmd, recv, recv_len - 1, ACK_VAL);
                        }
                        i2c_master_read_byte(cmd, recv + recv_len - 1, NACK_VAL);
                }

                i2c_master_stop(cmd);
                esp_err_t ret = i2c_master_cmd_begin((i2c_port_t)bus.bus, cmd, 1000 / portTICK_RATE_MS);
                i2c_cmd_link_delete(cmd);
                if (ret == ESP_OK) {
                        return true;
                }
        }

        return false;
}

bool I2CDevice::read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                        uint32_t recv_len, uint8_t times)
{
        return false;
}

AP_HAL::Device::PeriodicHandle I2CDevice::register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
        return bus.register_periodic_callback(period_usec, cb, this);
}

bool I2CDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h,
                uint32_t period_usec)
{
        return bus.adjust_timer(h, period_usec);
}

// setup I2C buses
I2CDeviceManager::I2CDeviceManager(void)
{
        for (uint8_t i = 0; i < ARRAY_SIZE(I2CD); ++i) {
		businfo[i].bus = i;
                auto &_cfg = I2CD[i];
                i2c_config_t conf;
                conf.mode = _cfg.mode;
                conf.sda_io_num = _cfg.sda_io_num;
                conf.sda_pullup_en = _cfg.sda_pullup_en;
                conf.scl_io_num = _cfg.sda_io_num;
                conf.scl_pullup_en = _cfg.sda_pullup_en;
                conf.master.clk_speed = _cfg.clk_speed;
                businfo[i].cfg = conf;

		businfo[i].init();
        }
}

AP_HAL::OwnPtr<AP_HAL::I2CDevice>
I2CDeviceManager::get_device(uint8_t bus, uint8_t address,
                             uint32_t bus_clock,
                             bool use_smbus,
                             uint32_t timeout_ms)
{
        bus -= HAL_I2C_BUS_BASE;
        if (bus >= ARRAY_SIZE(I2CD)) {
                return AP_HAL::OwnPtr<AP_HAL::I2CDevice>(nullptr);
        }
        auto dev = AP_HAL::OwnPtr<AP_HAL::I2CDevice>(new I2CDevice(bus, address,
                        bus_clock, use_smbus, timeout_ms));
        return dev;

}
