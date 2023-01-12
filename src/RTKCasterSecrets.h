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
Convert LLH coords into ECEF XYZ coords: https://tool-online.com/en/coordinate-converter.php
(1 mm resolution only)
WGS84_XYZ (geocentric)  ECEF-X 
WGS84_XYZ (geocentric)  ECEF-Y 
WGS84_XYZ (geocentric)  ECEF-Z 

after running look here: http://new.rtk2go.com:2101/SNIP::STATUS
*/


const float kDesiredAccuracy = 0.01;
// http://new.rtk2go.com:2101/SNIP::STATUS

const char kCasterHost[] = "rtk2go.com";
const uint16_t kCasterPort = 2101;
const char kMountPoint[] = "YOUR_RTK2GO_MOUNTPOINT";            //The mount point you want to push data to
const char kMountPointPW[] = "YOUR_RTK2GO_MOUNTPOINT_PASSWORD";

#endif /*** RTK_CASTER_SECRETS_H ***/