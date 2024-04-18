# gpsd2nmea2000
convert nmea0183 data from gpsd to a nmea2000 compatible gps

Uses [NMEA2000](https://github.com/ttlappalainen/NMEA2000) library using socketCAN Linux CAN devices.

See https://github.com/thomasonw/NMEA2000_socketCAN

## Building
It can be build using cmake on liunx, including raspberry pi and also on WSL

```
sudo apt install cmake g++ libgps-dev gpsd git
git clone --recurse-submodules https://github.com/mlaiacker/gpsd2nmea2000.git
cd gpsd2nmea2000
mkdir build
cd build
cmake ..
make
```
then you can run 

```
./gpsd2nmea2000 -?
Usage: gpsd2nmea2000 [OPTION...]
convert data from gpsd to a nmea2000 compatible gps

  -d, --device[=can0]        can device name
  -h, --hostname[=localhost] gpsd hostname
  -p, --port[=2947]          gpsd port
  -v, --verbose              Produce verbose output
  -?, --help                 Give this help list
      --usage                Give a short usage message

Mandatory or optional arguments to long options are also mandatory or optional
for any corresponding short options.
```

## Running
the default setup is a gpsd running on localhost and using can0 to write the nmea2000 data to
```
gpsd2nmea2000 --device=can1 --host=localhost --port=2947
```

## Dependencies
socketCAN must be installed and working on your system - refer to the adapters users guide.  And make sure to 'start up' the CAN port, example:
```
$ sudo /sbin/ip link set can0 up type can bitrate 250000
```
you can also use a virtual can (vcan) device to test it

