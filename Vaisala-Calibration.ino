/*
   sketch.ino

   Author:   Hiromasa Ihara (taisyo)
   Created:  2016-02-18
*/

#include <Ethernet.h>
#include <EthernetUdp.h>

#include <PFFIAPUploadAgent.h>
#include <TimeLib.h>
#include <LocalTimeLib.h>
#include <SerialCLI.h>
#include "SHT2x.h"
#include <NTP.h>
#include <Wire.h>
#include "HMP155.h"

#define GLED 22
#define RLED 23

//cli
SerialCLI      commandline(Serial);
//  ethernet
MacEntry       mac("MAC", "B0:12:66:01:02:AE", "mac address");
//  ip
BoolEntry      dhcp("DHCP", "true", "DHCP enable/disable");
IPAddressEntry ip("IP", "192.168.0.2", "IP address");
IPAddressEntry gw("GW", "192.168.0.1", "default gateway IP address");
IPAddressEntry sm("SM", "255.255.255.0", "subnet mask");
IPAddressEntry dns_server("DNS", "8.8.8.8", "dns server");
//  ntp
StringEntry    ntp("NTP", "ntp.nict.jp", "ntp server");
//  fiap
StringEntry    host("HOST", "202.15.110.21", "host of ieee1888 server end point");
IntegerEntry   port("PORT", "80", "port of ieee1888 server end point");
StringEntry    path("PATH", "/axis2/services/FIAPStorage", "path of ieee1888 server end point");
StringEntry    prefix("PREFIX", "http://iwata.com/vaisala_calibration/", "prefix of point id");
//  debug
int debug = 0;

//ntp
NTPClient ntpclient;
TimeZone localtimezone = { 9 * 60 * 60, 0, "+09:00" };

//fiap
FIAPUploadAgent fiap_upload_agent;
char hmp155_temp_str[16];
char hmp155_humi_str[16];
char hmp155_vp_str[16];
char hmp155_vpd_str[16];
char  sht21_temp_str[16];
char  sht21_humi_str[16];
char  sht21_vp_str[16];
char  sht21_vpd_str[16];
char sht21g_humi_str[16];
char sht21g_vp_str[16];
char sht21g_vpd_str[16];
struct fiap_element fiap_elements [] = {
  { "HMP155_Temperature", hmp155_temp_str, 0, &localtimezone, },
  { "HMP155_Humidity", hmp155_humi_str, 0, &localtimezone, },
  { "HMP155_VP", hmp155_vp_str, 0, &localtimezone, },
  { "HMP155_VPD", hmp155_vpd_str, 0, &localtimezone, },
  { "SHT21_Temperature", sht21_temp_str, 0, &localtimezone, },
  { "SHT21_Humidity", sht21_humi_str, 0, &localtimezone, },
  { "SHT21_VP", sht21_vp_str, 0, &localtimezone, },
  { "SHT21_VPD", sht21_vpd_str, 0, &localtimezone, },
  { "SHT21_GuessHumidity", sht21g_humi_str, 0, &localtimezone, },
  { "SHT21_GuessVP", sht21g_vp_str, 0, &localtimezone, },
  { "SHT21_GuessVPD", sht21g_vpd_str, 0, &localtimezone, },
};

// sensor
SHT2x sht_sensor;
HMP155 vaisala(Serial2, 24);

// Return the satureted Vapour pressure at the temperature
double temp2svp( double temp ) {
  temp = temp + 273.15;
  double a = -6096.9385 / temp;
  double b = 16.635794;
  double c = -2.711193 / 100.0 * temp;
  double d = 1.673952 / 100000.0 * temp * temp;
  double e = 2.433502 * log(temp);
  return ( exp( a + b + c + d + e ) * 100.0 );
}


void enable_debug()
{
  debug = 1;
}

void disable_debug()
{
  debug = 0;
}

void setup()
{
  int ret;
  commandline.add_entry(&mac);

  commandline.add_entry(&dhcp);
  commandline.add_entry(&ip);
  commandline.add_entry(&gw);
  commandline.add_entry(&sm);
  commandline.add_entry(&dns_server);

  commandline.add_entry(&ntp);

  commandline.add_entry(&host);
  commandline.add_entry(&port);
  commandline.add_entry(&path);
  commandline.add_entry(&prefix);

  commandline.add_command("debug", enable_debug);
  commandline.add_command("nodebug", disable_debug);

  commandline.begin(9600, "Vaisala HMP155 Gateway");

  // ethernet & ip connection
  if (dhcp.get_val() == 1) {
    ret = Ethernet.begin(mac.get_val());
    if (ret == 0) {
      restart("Failed to configure Ethernet using DHCP", 10);
    }
  } else {
    Ethernet.begin(mac.get_val(), ip.get_val(), dns_server.get_val(), gw.get_val(), sm.get_val());
  }

  // fetch time
  uint32_t unix_time;
  ntpclient.begin();
  ret = ntpclient.getTime(ntp.get_val(), &unix_time);
  if (ret < 0) {
    restart("Failed to configure time using NTP", 10);
  }
  setTime(unix_time);

  // fiap
  fiap_upload_agent.begin(host.get_val(), path.get_val(), port.get_val(), prefix.get_val());

  // sensor
  Wire.begin();
  sht_sensor.begin();
  sht_sensor.heaterOn();
  Serial.println("SHT21 Ready");
  Serial2.begin(4800, SERIAL_7E1);
  pinMode(24, OUTPUT);
  vaisala.begin();
  Serial.println("Vaisala Ready");
  pinMode(GLED, OUTPUT);
  pinMode(RLED, OUTPUT);
  digitalWrite(RLED, HIGH); // LED on when pin is High
}

void loop()
{
  static unsigned long old_epoch = 0, epoch;

  commandline.process();
  epoch = now();
  if (dhcp.get_val() == 1) {
    Ethernet.maintain();
  }

  if (epoch != old_epoch) {
    char buf[120] = ",HMP155,";
    char bufbuf[8];

    vaisala.read();
    float hmp155_temp = vaisala.ta();
    dtostrf(hmp155_temp, -1, 2, hmp155_temp_str);
    float hmp155_humi = vaisala.rh();
    dtostrf(hmp155_humi, -1, 2, hmp155_humi_str);
    float hmp155_svp = temp2svp(hmp155_temp); // Saturated Vapour Pressure [Pa]
    float hmp155_vp = hmp155_svp * hmp155_humi / 100.0; // Vapour Pressure [Pa]
    dtostrf(hmp155_vp, -1, 2, hmp155_vp_str);
    float hmp155_vpd = (hmp155_svp - hmp155_vp) / 1000.0; // Vapour Pressure Deficit [hPa]
    dtostrf(hmp155_vpd, -1, 4, hmp155_vpd_str);

    float sht21_temp = sht_sensor.readTemperature();
    dtostrf(sht21_temp, -1, 2, sht21_temp_str);
    float sht21_humi = sht_sensor.readHumidity();
    dtostrf(sht21_humi, -1, 2, sht21_humi_str);
    float sht21_svp = temp2svp(sht21_temp); // Saturated Vapour Pressure [Pa]
    float sht21_vp = sht21_svp * sht21_humi / 100.0;  // Vapour Pressure [Pa]
    dtostrf(sht21_vp, -1, 2, sht21_vp_str);
    float sht21_vpd = (sht21_svp - sht21_vp) / 1000.0; // Vapour Pressure Deficit [hPa]
    dtostrf(sht21_vpd, -1, 4, sht21_vpd_str);

    //    float sht21g_humi = 1.2225 * sht21_humi -16.229; // approximate Vaisala (Cloudy)
    float sht21g_humi = 1.2419 * sht21_humi - 18.064; // approximate Vaisala (Sunny)
    dtostrf(sht21g_humi, -1, 2, sht21g_humi_str);
    float sht21g_svp = temp2svp(hmp155_temp); // Saturated Vapour Pressure [Pa]
    float sht21g_vp = sht21g_svp * sht21g_humi / 100.0; // Vapour Pressure [Pa]
    dtostrf(sht21g_vp, -1, 2, sht21g_vp_str);
    float sht21g_vpd = (sht21g_svp - sht21g_vp) / 1000.0; // Vapour Pressure Deficit [hPa]
    dtostrf(sht21g_vpd, -1, 4, sht21g_vpd_str);

    strcat(buf, hmp155_temp_str);
    strcat(buf, ",");
    strcat(buf, hmp155_humi_str);
    strcat(buf, ",");
    strcat(buf, hmp155_vp_str);
    strcat(buf, ",");
    strcat(buf, hmp155_vpd_str);
    strcat(buf, ",SHT21,");
    strcat(buf, sht21_temp_str);
    strcat(buf, ",");
    strcat(buf, sht21_humi_str);
    strcat(buf, ",");
    strcat(buf, sht21_vp_str);
    strcat(buf, ",");
    strcat(buf, sht21_vpd_str);
    strcat(buf, ",SHT21Guess,");
    strcat(buf, sht21g_humi_str);
    strcat(buf, ",");
    strcat(buf, sht21g_vp_str);
    strcat(buf, ",");
    strcat(buf, sht21g_vpd_str);
    debug_msg(buf);
    //    Serial.println(buf);

    if (epoch % 30 == 0) {
      debug_msg("uploading...");
      digitalWrite(GLED, HIGH); // LED on when pin is High

      for (int i = 0; i < sizeof(fiap_elements) / sizeof(fiap_elements[0]); i++) {
        fiap_elements[i].time = epoch;
      }
      int ret = fiap_upload_agent.post(fiap_elements, sizeof(fiap_elements) / sizeof(fiap_elements[0]));
      if (ret == 0) {
        debug_msg("done");
      } else {
        debug_msg("failed");
        Serial.println(ret);
      }
      digitalWrite(GLED, LOW); // LED on when pin is High
    }
  }

  old_epoch = epoch;
}

void debug_msg(String msg)
{
  if (debug == 1) {
    Serial.print("[");
    print_time();
    Serial.print("]");
    Serial.println(msg);
  }
}

void print_time()
{
  char print_time_buf[32];
  TimeElements* tm;
  tm = localtime();
  sprintf(print_time_buf, "%04d/%02d/%02d %02d:%02d:%02d",
          tm->Year + 1970, tm->Month, tm->Day, tm->Hour, tm->Minute, tm->Second);
  Serial.print(print_time_buf);
}

void restart(String msg, int restart_minutes)
{
  Serial.println(msg);
  Serial.print("This system will restart after ");
  Serial.print(restart_minutes);
  Serial.print("minutes.");

  unsigned int start_ms = millis();
  while (1) {
    commandline.process();
    if (millis() - start_ms > restart_minutes * 60UL * 1000UL) {
      commandline.reboot();
    }
  }
}
