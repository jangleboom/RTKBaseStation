### Real Time Kinematics Base Station (RTK Server)
Hardware used:   
* Sparkfun ESP32 Thing Plus 
* SparkFun GPS-RTK-SMA Breakout - ZED-F9P (Qwiic)
* ublox ANN-MB1 antenna

Infrastructure:
* WiFi (e. g. a personal hotspot)
* free line of sight between antenna (horizontal placed) an sky
* high-precision ECEF coordinates of the location of the antenna (put it into the code)

To connect to a caster you need to create a secrets.h file in your `lib/` dir with your credentials that looks like this:

````
#ifndef SECRETS_H
#define SECRETS_H
// A place for your caster credentials

// RTK2Go MountPoint http://www.rtk2go.com:2101/SNIP::STATUS#uptime
Email: YOUR_RTK2GO_ACCOUNT_EMAIL.com
const char casterHost[] = "rtk2go.com";
const uint16_t casterPort = 2101;
const char mountPoint[] = "YOUR_RTK2GO_MOUNT_POINT"; //The mount point you want to push data to
// Use "WEEK2208" until your registration is confirmed by the caster
const char mountPointPW[] = "YOUR_RTK2GO_MOUNT_POINT_PASSWORD"; 

// Or

// Another free NTRIP Caster is Emlid
// Emlid Caster MountPoint
Email: YOUR_EMLID_ACCOUNT_EMAIL.COM
const char casterHost[] = "caster.emlid.com";
const uint16_t casterPort = 2101;
const char mountPoint[] = "YOUR_EMLID_MOUNT_POINT"; //The mount point you want to push data to
const char mountPointPW[] = "YOUR_EMLID_MOUNT_POINT_PASSWORD";

// Use only one of this choices!

#endif /*** SECRETS_H ***/

````
