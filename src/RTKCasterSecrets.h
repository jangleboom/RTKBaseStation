#ifndef RTK_CASTER_SECRETS_H
#define RTK_CASTER_SECRETS_H
#include <Arduino.h>
// A place for your default caster credentials

/**
 * While a survey-in is easy to set up and fine for an in-the-field way to establish the location 
 * of a base, it’s not recommended for getting the fixed location of a static base station as it is 
 * less accurate.
 * 
 * Earth's equatorial circumference is about 40,000 kilometers.
 * A latitude/longitude value breaks that distance up into 360 degrees, 
 * starting at -180 and ending at 180.
 * This means that one degree is 40,000 km divided by 360: 40,000 / 360 = 111
 * So, one degree is 111 kilometres. For fractions of a degree, you divide it 
 * by 10 for each decimal place:
 * 
 * decimal places 	degrees 	distance
            0 	     1 	        111 km
            1 	     0.1 	    11.1 km
            2        0.01 	    1.11 km
            3        0.001 	    111 m
            4        0.0001 	11.1 m
            5 	     0.00001 	1.11 m
            6 	     0.000001 	0.111 m
            7 	     0.0000001 	1.11 cm
            8 	     0.00000001 1.11 mm

Units are cm with a high precision extension so -1234.5678 should be called: (-123456, -78)
For more infomation see Example12_setStaticPosition
Note: If you leave these coordinates in place and setup your antenna *not* at SparkFun, your receiver
will be very confused and fail to generate correction data because, well, you aren't at SparkFun...
See this tutorial on getting PPP coordinates: https://learn.sparkfun.com/tutorials/how-to-build-a-diy-gnss-reference-station/all
response &= myGNSS.setStaticPosition(-128020830, -80, -471680384, -70, 408666581, 10);  With high precision 0.1mm parts

ECEF coordinates: Example tiny office Brieslang

Get LatLongHight coords from map: https://www.gpskoordinaten.de/ (and verify the coords on new Google Maps cards)
Latitude : 52.6013104707097 | Longitude : 13.001770401773417 | Höhe : 39 Meter
Convert LLH coords into ECEF XYZ coords: https://tool-online.com/en/coordinate-converter.php
(1 mm resolution only)
WGS84_XYZ (geocentric)  ECEF-X 3782519.925 or 3782521.875
WGS84_XYZ (geocentric)  ECEF-Y 873386.641  or 873386.472
WGS84_XYZ (geocentric)  ECEF-Z 5043750.593 or 5043752.317

after running look here: http://new.rtk2go.com:2101/SNIP::STATUS
*/

/*
const int32_t ECEF_X_CM = 378252188;   // Earth-centered-X in cm
const int8_t ECEF_X_HP = 02;           // Earth-centered-X, high precision extension 0.1mm
const int32_t ECEF_Y_CM = 87338647;    // Earth-centered-Y in cm
const int8_t ECEF_Y_HP = 32;           // Earth-centered-Y, high precision extension 0.1mm
const int32_t ECEF_Z_CM = 504375230;   // Earth-centered-Z in cm
const int8_t ECEF_Z_HP = 10;           // Earth-centered-Z, high precision extension 0.1mm

const int32_t LATITUDE =  526013065;   // 7-digits
const int8_t LATITUDE_HP = 90;         // high precision extension
const int32_t LONGITUDE = 130017615;   // 7-digits
const int8_t LONGITUDE_HP = 00;        // high precision extension
const int32_t ALTITUDE = 41500;        // Unit: mm
const int8_t ALTITUDE_HP = 0;          // high precision extension
*/

const float kDesiredAccuracy = 0.06;
//RTK2Go MountPoint 1 http://new.rtk2go.com:2101/SNIP::STATUS
// Email: mr.markuese@gmail.com
const char kCasterHost[] = "rtk2go.com";
const uint16_t kCasterPort = 2101;
const char KMountPoint[] = "rtkbrieselang"; //The mount point you want to push data to
const char kMountPointPW[] = "WEEK2208";

#endif /*** RTK_CASTER_SECRETS_H ***/