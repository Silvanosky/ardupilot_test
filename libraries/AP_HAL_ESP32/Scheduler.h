/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include "HAL_ESP32_Namespace.h"

#define ESP32_SCHEDULER_MAX_TIMER_PROCS 10
#define ESP32_SCHEDULER_MAX_IO_PROCS 10


/* Scheduler implementation: */
class ESP32::Scheduler : public AP_HAL::Scheduler {

public:
    Scheduler();
    /* AP_HAL::Scheduler methods */
    void     init() override;
    void     set_callbacks(AP_HAL::HAL::Callbacks *cb)
    {
        callbacks = cb;
    };
    void     delay(uint16_t ms) override;
    void     delay_microseconds(uint16_t us) override;
    void     register_timer_process(AP_HAL::MemberProc) override;
    void     register_io_process(AP_HAL::MemberProc) override;
    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override;
    void     reboot(bool hold_in_bootloader) override;
    bool     in_main_thread() const override;
    void     system_initialized() override;

    static const int SPI_PRIORITY = 21;
    static const int MAIN_PRIO = 20;
    static const int I2C_PRIORITY = 19;
    static const int TIMER_PRIO = 15;
    static const int RCIN_PRIO = 10;
    static const int WIFI_PRIO = 7;
    static const int UART_PRIO = 6;
    static const int IO_PRIO = 5;
    static const int STORAGE_PRIO = 4;

    static const int TIMER_SS = 4096;
    static const int MAIN_SS = 4096;
    static const int RCIN_SS = 4096;
    static const int WIFI_SS = 4096;
    static const int UART_SS = 1024;
    static const int DEVICE_SS = 4096;
    static const int IO_SS = 4096;
    static const int STORAGE_SS = 4096;

private:
    AP_HAL::HAL::Callbacks *callbacks;
    AP_HAL::Proc _failsafe;

    AP_HAL::MemberProc _timer_proc[ESP32_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_timer_procs;

    AP_HAL::MemberProc _io_proc[ESP32_SCHEDULER_MAX_IO_PROCS];
    uint8_t _num_io_procs;

    static bool _initialized;



    void *_main_task_handle;
    void *_timer_task_handle;
    void *_rcin_task_handle;
    void *_uart_task_handle;
    void *_io_task_handle;
    void *test_task_handle;
    void *_storage_task_handle;

    static void _main_thread(void *arg);
    static void _timer_thread(void *arg);
    static void _rcin_thread(void *arg);
    static void _uart_thread(void *arg);
    static void _io_thread(void *arg);
    static void _storage_thread(void *arg);

	static void set_position(void* arg);


    static void _print_profile(void* arg);

    static void test_esc(void* arg);

    bool _in_timer_proc;
    void _run_timers();
    Semaphore _timer_sem;

    bool _in_io_proc;
    void _run_io();
    Semaphore _io_sem;
};
