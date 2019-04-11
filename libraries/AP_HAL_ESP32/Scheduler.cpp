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

#include "AP_HAL_ESP32/Scheduler.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

using namespace ESP32;

extern const AP_HAL::HAL& hal;

bool Scheduler::_initialized = true;

Scheduler::Scheduler()
{
    _initialized = false;
}

void Scheduler::init()
{
    xTaskCreate(_main_thread, "APM_MAIN", Scheduler::MAIN_SS, this, Scheduler::MAIN_PRIO, &_main_task_handle);
    xTaskCreate(_timer_thread, "APM_TIMER", TIMER_SS, this, TIMER_PRIO, &_timer_task_handle);
    xTaskCreate(_rcin_thread, "APM_RCIN", RCIN_SS, this, RCIN_PRIO, &_rcin_task_handle);
    xTaskCreate(_uart_thread, "APM_UART", UART_SS, this, UART_PRIO, &_uart_task_handle);
    xTaskCreate(_io_thread, "APM_IO", IO_SS, this, IO_PRIO, &_io_task_handle);
    xTaskCreate(test_esc, "APM_TEST", IO_SS, this, IO_PRIO, nullptr);
    xTaskCreate(_storage_thread, "APM_STORAGE", STORAGE_SS, this, STORAGE_PRIO, &_storage_task_handle);
}

void Scheduler::delay(uint16_t ms)
{
    uint64_t start = AP_HAL::micros64();
    while ((AP_HAL::micros64() - start)/1000 < ms) {
        delay_microseconds(1000);
        if (_min_delay_cb_ms <= ms) {
            if (in_main_thread()) {
                call_delay_cb();
            }
        }
    }
}

void Scheduler::delay_microseconds(uint16_t us)
{
    uint64_t start = AP_HAL::micros64();
    uint16_t tick = portTICK_PERIOD_MS * 1000;
    vTaskDelay((us+tick/2)/tick);
    uint64_t now = AP_HAL::micros64();
    if(start + us > now) {
        ets_delay_us((start + us) - now);
    }
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }
    if (_num_timer_procs >= ESP32_SCHEDULER_MAX_TIMER_PROCS) {
        printf("Out of timer processes\n");
        return;
    }
    _timer_sem.take_blocking();
    _timer_proc[_num_timer_procs] = proc;
    _num_timer_procs++;
    _timer_sem.give();
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            return;
        }
    }

    if (_num_io_procs < ESP32_SCHEDULER_MAX_IO_PROCS) {
        _io_sem.take_blocking();
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
        _io_sem.give();
    } else {
        printf("Out of IO processes\n");
    }
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void Scheduler::reboot(bool hold_in_bootloader)
{
    printf("Restarting now...\n");
    esp_restart();
}

bool Scheduler::in_main_thread() const
{
    return _main_task_handle == xTaskGetCurrentTaskHandle();
}

void Scheduler::system_initialized()
{
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::system_initialized called more than once");
    }

    _initialized = true;
}

void Scheduler::_timer_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (!_initialized) {
        sched->delay_microseconds(1000);
    }
    while (true) {
        sched->delay_microseconds(1000);
        sched->_run_timers();
        // process any pending RC output requests
        //hal.rcout->timer_tick();
        //analog in
        //((AnalogIn *)hal.analogin)->_timer_tick();
    }
}

void Scheduler::_run_timers()
{
    if (_in_timer_proc) {
        return;
    }
    _in_timer_proc = true;

    int num_procs = 0;

    _timer_sem.take_blocking();
    num_procs = _num_timer_procs;
    _timer_sem.give();

    // now call the timer based drivers
    for (int i = 0; i < num_procs; i++) {
        if (_timer_proc[i]) {
            _timer_proc[i]();
        }
    }

    // and the failsafe, if one is setup
    if (_failsafe != nullptr) {
        _failsafe();
    }

    _in_timer_proc = false;
}

void Scheduler::_rcin_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (!_initialized) {
        sched->delay_microseconds(20000);
    }
    while (true) {
        sched->delay_microseconds(20000);
        //((RCInput *)hal.rcin)->_timer_tick();
    }
}


static const char* TAG = "MOTOR";

void Scheduler::test_esc(void* arg)
{
    Scheduler *sched = (Scheduler *)arg;
    for (long i = 0; i < 5000; ++i)
    {
	hal.rcout->write(0, 1600);
	sched->delay_microseconds(1000);
    }
    sched->delay(500);

    long n = 0;
    int v = 1;
    while (true)
    {
	hal.rcout->write(0, 1600 + n);
	sched->delay(100);
	n += v;
	if (n > 300)
	    v = -1;
	if (n < 10)
	    v = 1;
	ESP_LOGI(TAG, "PWM value: %d", 1600 + n);
    }
}

void Scheduler::_run_io(void)
{
    if (_in_io_proc) {
        return;
    }

    _in_io_proc = true;

    int num_procs = 0;
    _io_sem.take_blocking();
    num_procs = _num_io_procs;
    _io_sem.give();
    // now call the IO based drivers
    for (int i = 0; i < num_procs; i++) {
        if (_io_proc[i]) {
            _io_proc[i]();
        }
    }
    _in_io_proc = false;
}

void Scheduler::_io_thread(void* arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (!sched->_initialized) {
        sched->delay_microseconds(1000);
    }
    while (true) {
        sched->delay_microseconds(1000);
        // run registered IO processes
        sched->_run_io();
    }
}

void Scheduler::_storage_thread(void* arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (sched->_initialized) {
        sched->delay_microseconds(10000);
    }
    while (true) {
        sched->delay_microseconds(10000);
        // process any pending storage writes
        hal.storage->_timer_tick();
    }
}

void Scheduler::_uart_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (!_initialized) {
        sched->delay_microseconds(20000);
    }
    while (true) {
        sched->delay_microseconds(1000);
        hal.uartA->_timer_tick();
        hal.uartB->_timer_tick();
        hal.uartC->_timer_tick();
        hal.uartD->_timer_tick();
        hal.console->_timer_tick();
    }
}

void print_stats()
{
    static int64_t last_run = 0;
    if (AP_HAL::millis64() - last_run > 60000) {
        char buffer[1024];
        vTaskGetRunTimeStats(buffer);
        printf("\n\n%s\n", buffer);
        heap_caps_print_heap_info(0);
        last_run = AP_HAL::millis64();
    }
}

void Scheduler::_main_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    hal.uartA->begin(115200);
    hal.uartB->begin(38400);
    hal.uartC->begin(57600);
    hal.analogin->init();
    hal.rcout->init();

    sched->callbacks->setup();
    sched->system_initialized();

    while (true) {
        sched->callbacks->loop();
        sched->delay_microseconds(250);
        //print_stats();
    }
}

