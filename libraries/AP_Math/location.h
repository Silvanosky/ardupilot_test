#pragma once

#include <inttypes.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#include "vector2.h"
#include "vector3.h"
#include <AP_Common/Location.h>

// scaling factor from 1e-7 degrees to meters at equator
// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
#define LOCATION_SCALING_FACTOR 0.011131884502145034f
// inverse of LOCATION_SCALING_FACTOR
#define LOCATION_SCALING_FACTOR_INV 89.83204953368922f

/*
 * LOCATION
 */

// return horizontal distance in centimeters between two positions
float        get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination);

// return bearing in centi-degrees between two positions
float        get_bearing_cd(const Vector3f &origin, const Vector3f &destination);

// see if location is past a line perpendicular to
// the line between point1 and point2. If point1 is
// our previous waypoint and point2 is our target waypoint
// then this function returns true if we have flown past
// the target waypoint
bool        location_passed_point(const struct Location & location,
                                  const struct Location & point1,
                                  const struct Location & point2);

/*
  return the proportion we are along the path from point1 to
  point2. This will be less than >1 if we have passed point2
 */
float       location_path_proportion(const struct Location &location,
                               const struct Location &point1,
                               const struct Location &point2);


// Converts from WGS84 geodetic coordinates (lat, lon, height)
// into WGS84 Earth Centered, Earth Fixed (ECEF) coordinates
// (X, Y, Z)
void        wgsllh2ecef(const Vector3d &llh, Vector3d &ecef);

// Converts from WGS84 Earth Centered, Earth Fixed (ECEF)
// coordinates (X, Y, Z), into WHS84 geodetic
// coordinates (lat, lon, height)
void        wgsecef2llh(const Vector3d &ecef, Vector3d &llh);

// return true when lat and lng are within range
bool        check_lat(float lat);
bool        check_lng(float lng);
bool        check_lat(int32_t lat);
bool        check_lng(int32_t lng);
bool        check_latlng(float lat, float lng);
bool        check_latlng(int32_t lat, int32_t lng);
