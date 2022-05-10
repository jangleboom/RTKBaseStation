Headtracker + Real Time Kinematics

To conntect to a caster you need a secrets.h file with your credentials that looks like this:

````
#ifndef SECRETS_H
#define SECRETS_H
// A place for your caster credentials

// RTK2Go MountPoint 1 http://www.rtk2go.com:2101/SNIP::STATUS#uptime
Email: YOUR_ACCOUNT_EMAIL.com
const char casterHost[] = "rtk2go.com";
const uint16_t casterPort = 2101;
const char mountPoint[] = "YOUR_MOUNT_POINT"; //The mount point you want to push data to
// Use "WEEK2208" until your registration is confirmed by the caster
const char mountPointPW[] = "YOUR_MOUNT_POINT_PASSWORD"; 

// Or

// Another free NTRIP Caster is Emlid
// Emlid Caster MountPoint 1
Email: YOUR_ACCOUNT_EMAIL.com
const char casterHost[] = "caster.emlid.com";
const uint16_t casterPort = 2101;
const char mountPoint[] = "YOUR_MOUNT_POINT"; //The mount point you want to push data to
const char mountPointPW[] = "YOUR_MOUNT_POINT_PASSWORD";

// Use only one of this choices!

#endif /*** SECRETS_H ***/

````
