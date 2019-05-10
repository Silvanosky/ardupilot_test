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

#pragma once

#include "AP_Baro_Backend.h"

class AP_Baro_Analog : public AP_Baro_Backend
{
public:
	void update() override;

	static AP_Baro_Backend *probe(AP_Baro &baro, int src);

private:
	AP_Baro_Analog(AP_Baro &baro, int source);

	bool _init();

    AP_HAL::AnalogSource* _src;

    struct {
        uint32_t sum_pressure;
        uint32_t sum_temperature;
        uint8_t num_samples;
    } _accum;


    uint8_t _instance;
};
