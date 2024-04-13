/* 
 * File:   main.cpp
 * Author: al
 *
 * Testing for CAN and RPI 
 * 
 * See: https://github.com/thomasonw/NMEA2000_socketCAN
 *
 * Created on February 12, 2017, 2:37 PM
 */

#include <cstdlib>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <cmath>
#include <time.h>

#include <NMEA2000_SocketCAN.h>       // https://github.com/thomasonw/NMEA2000_socketCAN
#include <N2kMessages.h>


tNMEA2000 &NMEA2000=*(new tNMEA2000_SocketCAN());


using namespace std;

#include <libgpsmm.h>
gpsmm *gps_rec;
struct gps_data_t gps_data_last;


#ifndef NSEC_PER_SEC
#define NSEC_PER_SEC 1000000000
#endif

void time_to_tm(time_t totalsecs, int offset, struct tm *result);
/* stolen from the linux kernel, I tried to make it as less performant as possible
 * despite's the kernel's attempts */
void set_normalized_timespec(struct timespec *ts, time_t sec, long long nsec) {
    while (nsec >= NSEC_PER_SEC) {
        /*
         * The following asm() prevents the compiler from
         * optimising this loop into a modulo operation. See
         * also __iter_div_u64_rem() in include/linux/time.h
         */
        /* asm("" : "+rm"(nsec)); */
        nsec -= NSEC_PER_SEC;
        ++sec;
    }
    while (nsec < 0) {
        /* asm("" : "+rm"(nsec)); */
        nsec += NSEC_PER_SEC;
        --sec;
    }
    ts->tv_sec = sec;
    ts->tv_nsec = nsec;
}

/*
 * sub = lhs - rhs, in normalized form
 */
static inline struct timespec timespec_sub(struct timespec lhs,
						struct timespec rhs)
{
	struct timespec ts_delta;
	set_normalized_timespec(&ts_delta, lhs.tv_sec - rhs.tv_sec,
				lhs.tv_nsec - rhs.tv_nsec);
	return ts_delta;
}

/*
int main(void)
{
    cout << "Starting CAN watching" << endl;

    setvbuf (stdout, NULL, _IONBF, 0);                                          // No buffering on stdout, just send chars as they come.
    tSocketStream* stdoutstream = new tSocketStream();

    nmea2000.SetForwardStream(stdoutstream);                                      // Connect bridge function for streaming output.
    nmea2000.SetForwardType(tNMEA2000::fwdt_Text);                              // Show in clear text (for now)
       
    if (!nmea2000.Open()) {
       cout << "Failed to open CAN port" << endl;
       return 1;
   }
    
    cout  << endl << "CAN started, going to watch it now" << endl;

     while(1) {
         nmea2000.ParseMessages();                                               // Will send out CAN messages in open text 
    }
    
    return 0;
}
*/

// List here messages your device will transmit.
 //Setting up PGN 129025 Message "Position, Rapid Update"
 //Setting up PGN 129026 Message "COG SOG rapid update"
 //Setting up PGN 129029 Message "GNSS Position Data"
const unsigned long TransmitMessages[] PROGMEM={129025L,129026L,129029L,0};

// Define schedulers for messages. Define schedulers here disabled. Schedulers will be enabled
// on OnN2kOpen so they will be synchronized with system.
// We use own scheduler for each message so that each can have different offset and period.
// Setup periods according PGN definition (see comments on IsDefaultSingleFrameMessage and
// IsDefaultFastPacketMessage) and message first start offsets. Use a bit different offset for
// each message so they will not be sent at same time.
tN2kSyncScheduler SchedulerRapid(false,250,100);
tN2kSyncScheduler SchedulerSOG(false,250,120);
tN2kSyncScheduler SchedulerPosition(false,1000,110);

// *****************************************************************************
// Call back for NMEA2000 open. This will be called, when library starts bus communication.
// See NMEA2000.SetOnOpen(OnN2kOpen); on setup()
void OnN2kOpen() {
    cout << "Starting nmea2000 gps device" << endl;
  // Start schedulers now.
  SchedulerPosition.UpdateNextTime();
  SchedulerRapid.UpdateNextTime();
  SchedulerSOG.UpdateNextTime();
}

// *****************************************************************************
void setup() {
    gps_rec = new gpsmm("localhost", DEFAULT_GPSD_PORT);

    if (gps_rec->stream(WATCH_ENABLE|WATCH_JSON) == NULL) {
        cerr << "No GPSD running.\n";
        exit(1);
    }
    cout << "Start reading from gpsd" << endl;


    setvbuf (stdout, NULL, _IONBF, 0);                                          // No buffering on stdout, just send chars as they come.

  // Set Product information
  NMEA2000.SetProductInformation("86723598", // Manufacturer's Model serial code
                                 142, // Manufacturer's product code
                                 "maxGPS",  // Manufacturer's Model ID
                                 __DATE__,  // Manufacturer's Software version code
                                 "1.0.0.0 (2024-04-12)" // Manufacturer's Model version
                                 );
  // Set device information
  NMEA2000.SetDeviceInformation(162739, // Unique number. Use e.g. Serial number.
                                145, // Device function=145 Ownship Position (GNSS) . See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                60, // Device class=Navigation Communication Interface. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2141 // Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );
  // Uncomment 2 rows below to see, what device will send to bus. Use e.g. OpenSkipper or Actisense NMEA Reader                           
  //Serial.begin(115200);
  //NMEA2000.SetForwardStream(&Serial);
  // If you want to use simple ascii monitor like Arduino Serial Monitor, uncomment next line
  //NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.
    NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 21);
  //NMEA2000.SetDebugMode(tNMEA2000::dm_Actisense); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
  // Here we tell library, which PGNs we transmit
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  // Define OnOpen call back. This will be called, when CAN is open and system starts address claiming.
  NMEA2000.SetOnOpen(OnN2kOpen);
  NMEA2000.Open();
}

// *****************************************************************************
void SendN2kPosition(gps_data_t *gpsd_data) {
  tN2kMsg N2kMsg;

  if ( SchedulerRapid.IsTime() ) {
    SchedulerRapid.UpdateNextTime();
    if(gpsd_data->fix.mode>=MODE_2D)
    {
        SetN2kLatLonRapid(N2kMsg, gpsd_data->fix.latitude, gpsd_data->fix.longitude);
        NMEA2000.SendMsg(N2kMsg);
    }
  }
  
  if ( SchedulerSOG.IsTime() ) {
    SchedulerSOG.UpdateNextTime();
    if(gpsd_data->fix.mode>=MODE_2D)
    {
        if(!isnan(gpsd_data->fix.track)){
          //printf("COG=%f, SOG=%f\r\n", gpsd_data->fix.track, gpsd_data->fix.speed);
          SetN2kCOGSOGRapid(N2kMsg, 1, tN2kHeadingReference::N2khr_true, DegToRad(gpsd_data->fix.track), gpsd_data->fix.speed);
          NMEA2000.SendMsg(N2kMsg);
        }
    }
  }
  
  if ( SchedulerPosition.IsTime() ) {
    SchedulerPosition.UpdateNextTime();
    uint16_t  dayssince1970 = gpsd_data->fix.time.tv_sec/3600/24;
    struct  tm tm_now;
    time_to_tm(gpsd_data->fix.time.tv_sec, 0, &tm_now);
    tm_now.tm_sec = 0;
    tm_now.tm_min = 0;
    tm_now.tm_hour = 0;
    struct timespec time_midnight;
    time_midnight.tv_sec = mkgmtime(&tm_now);

    struct  timespec since_midnight = timespec_sub(gpsd_data->fix.time, time_midnight);
    //printf("dayssince1970=%i sec since midnight=%li fix.mode=%i\r\n", dayssince1970, since_midnight.tv_sec, gpsd_data->fix.mode);
    if(gpsd_data->fix.mode>=MODE_2D){
      SetN2kGNSS(N2kMsg, 1, dayssince1970, since_midnight.tv_sec, 
      gpsd_data->fix.latitude, gpsd_data->fix.longitude, gpsd_data->fix.altHAE, tN2kGNSStype::N2kGNSSt_GPSSBASWAASGLONASS, 
      (tN2kGNSSmethod)(gpsd_data->fix.mode), gpsd_data->satellites_used, gpsd_data->dop.hdop, gpsd_data->dop.pdop);
      NMEA2000.SendMsg(N2kMsg);

    }
  }
}

void loop() {
  NMEA2000.ParseMessages();
 	struct gps_data_t* newdata;

	if (gps_rec->waiting(100000))
  {
    if((newdata = gps_rec->read()) != NULL){
      memcpy(newdata, &gps_data_last, sizeof(gps_data_last));
    } else {
	    cerr << "Read error.\n";
    }
  } 
    SendN2kPosition(&gps_data_last);
}

int main(){
    setup();
    while (true)
    {
        loop();
        delay(10);
    }
    
}