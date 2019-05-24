/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   Code by Charles Villard.
 */

#include "AP_Baro_Analog.h"

#include <utility>
#include <stdio.h>

#include <AP_Math/AP_Math.h>

#if BARO_ANALOG_DEBUG
# define Debug(fmt, args ...)  do {printf(fmt "\n", ## args);} while(0)
#else
# define Debug(fmt, args ...)
#endif

extern const AP_HAL::HAL &hal;

AP_Baro_Analog::AP_Baro_Analog(AP_Baro &baro, int source) :
	AP_Baro_Backend(baro)
{
    _src = hal.analogin->channel(source);
}

AP_Baro_Backend *AP_Baro_Analog::probe(AP_Baro &baro, int source)
{
	if (!source) {
		return nullptr;
	}

	AP_Baro_Analog *sensor = new AP_Baro_Analog(baro, std::move(source));

	if (!sensor || !sensor->_init()) {
		delete sensor;
		return nullptr;
	}

	return sensor;
}

bool AP_Baro_Analog::_init()
{
	if (!_src) {
		return false;
	}

	_instance = _frontend.register_sensor();

	//_frontend.sensors[_instance].healthy

    _frontend.set_type(_instance, AP_Baro::BARO_TYPE_WATER);

	return true;
}

void AP_Baro_Analog::update()
{
	double raw_pressure = _src->read_average();
	double pressure = (((raw_pressure - 97.) / 1024. )*5./0.0266)*0.069;

	pressure = 1.0117 * pressure + 0.00995; //Calibrate sensor

    pressure *= 100000; // bar -> Pascal
	printf("Raw sensor value: %f Pressure: %lf Voltage: %lf mv\n", raw_pressure, pressure, raw_pressure * (3.3/4096));

    _copy_to_frontend(_instance, -pressure, 0.0);
}



