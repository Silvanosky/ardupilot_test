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
                .mode = I2C_MODE_MASTER,
                .sda_io_num = (gpio_num_t) 23,
                .scl_io_num = (gpio_num_t) 22,

                .sda_pullup_en = GPIO_PULLUP_ENABLE,
                .scl_pullup_en = GPIO_PULLUP_ENABLE,

                .clk_speed = 400000,
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
        if (i2c_param_config((i2c_port_t)bus, &cfg) != ESP_OK)
                printf("Cannot configure i2c for bus: %d", bus);

        //No buffer for the master i2c
        if (i2c_driver_install((i2c_port_t)bus, cfg.mode, 0, 0, 0) != ESP_OK)
                printf("Cannot install driver on bus: %d in mode: %d\n", bus, cfg.mode);
}

I2CBus::~I2CBus()
{
        if (i2c_driver_delete((i2c_port_t)bus) != ESP_OK)
                printf("Error: cannort delete bus %d. \n", bus);
        printf("Deleted bus %d correctly.\n", bus);
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

        printf("i2c: added addr: 0x%x  bus: %x\n", address, busnum);
}

I2CDevice::~I2CDevice()
{}


static void disp_buf(const uint8_t *buf, const int len)
{
        int i;
        for (i = 0; i < len; i++) {
                printf("%02x ", buf[i]);
                if ((i + 1) % 16 == 0) {
                        printf("\n");
                }
        }
        printf("\n");
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{

        if (_split_transfers && send_len > 0 && recv_len > 0) {
                return transfer(send, send_len, nullptr, 0) &&
                       transfer(nullptr, 0, recv, recv_len);
        }

        // calculate a timeout as twice the expected transfer time, and set as min of 4ms
        uint32_t timeout_ms = 1 + 2 * (((8 * 1000000UL / bus.cfg.master.clk_speed) *
                                        MAX(send_len,
                                            recv_len)) / 1000);
        timeout_ms = MAX(timeout_ms, _timeout_ms);

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);

        uint8_t op = send_len ? I2C_MASTER_WRITE : I2C_MASTER_READ;
        i2c_master_write_byte(cmd, (_address << 1 ) | op, ACK_CHECK_EN);

        if (send && send_len != 0) {
                //Because i2c_master_write doesn't have const data parameter
                i2c_master_write(cmd, const_cast<uint8_t *>(send), send_len, ACK_CHECK_EN);
        }

        if (recv && recv_len != 0) {
                if (recv_len > 1) {
                        i2c_master_read(cmd, recv, recv_len - 1, ACK_VAL);
                }
                i2c_master_read_byte(cmd, recv + recv_len - 1, NACK_VAL);
        }

        i2c_master_stop(cmd);
        printf("Waiting : %d \n", pdMS_TO_TICKS(timeout_ms));

        esp_err_t ret;
        unsigned retries = _retries;
        do {

		bus.semaphore.take(timeout_ms);
                ret = i2c_master_cmd_begin((i2c_port_t)bus.bus, cmd, timeout_ms / portTICK_RATE_MS);

		bus.semaphore.give();
        } while (ret != ESP_OK && retries-- > 0);

        printf("Send i2c operation code: %d to addr: 0x%x\n", op, _address);
        printf("Buffer sended %d:\n", send_len);
        disp_buf(send, send_len);
        printf("Buffer received %d:\n", recv_len);
        disp_buf(recv, recv_len);

        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
                printf("OK ----------------  \n");
                return true;
        }
        switch (ret) {
        case ESP_ERR_INVALID_ARG:
                printf("i2c: Parameter error\n");
                break;
        case ESP_FAIL:
                printf("i2c: Sending command error, slave doesn’t ACK the transfer.\n");
                break;
        case ESP_ERR_INVALID_STATE:
                printf("i2c: I2C driver not installed or not in master mode.\n");
                break;
        case ESP_ERR_TIMEOUT:
                printf("i2c: Operation timeout because the bus is busy.\n");
                break;
	default:
		break;
        }

        printf("NOPE --------------- \n");

        return false;

}


bool I2CDevice::_transfer(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len)
{
        return false;
}

bool I2CDevice::read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                        uint32_t recv_len, uint8_t times)
{
        printf("ok ------------------- \n");
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
                conf.scl_io_num = _cfg.scl_io_num;
                conf.scl_pullup_en = _cfg.scl_pullup_en;
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

/*
  get mask of bus numbers for all configured I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask(void) const
{
        return ((1U << ARRAY_SIZE(I2CD)) - 1) << HAL_I2C_BUS_BASE;
}

/*
  get mask of bus numbers for all configured internal I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_internal(void) const
{
        // assume first bus is internal
        return get_bus_mask() & HAL_I2C_INTERNAL_MASK;
}

/*
  get mask of bus numbers for all configured external I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_external(void) const
{
        // assume first bus is internal
        return get_bus_mask() & ~HAL_I2C_INTERNAL_MASK;
}
