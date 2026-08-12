#pragma once

#include <Mesh.h>
#include <helpers/SensorManager.h>
#include <helpers/sensors/LocationProvider.h>

class EnvironmentSensorManager : public SensorManager {
protected:
  static const int MAX_ACTIVE_SENSORS = 16;

  // Query function pointer + sub-channel index (for multi-channel sensors like INA3221).
  // Sub-channel is 0 for all single-output sensors.
  struct ActiveSensor {
    void    (*query)(uint8_t channel, uint8_t sub_channel, CayenneLPP& telemetry);
    uint8_t   sub_channel;
  };

  ActiveSensor _active_sensors[MAX_ACTIVE_SENSORS];
  int          _active_sensor_count = 0;
  uint8_t      next_available_channel = TELEM_CHANNEL_SELF + 1;

  static const uint32_t GPS_MAX_FIX_WAIT_SECONDS = 600;  // 10 minutes

  bool     gps_detected = false;
  bool     gps_active = false;
  bool     gps_loc_enabled = false;     // maps from gps_enabled; gates lat/lon copy
  bool     gps_sleep_during_interval = false;
  bool     in_fix_window = false;
  uint32_t gps_update_interval_sec = 1; // raw user value (0 disables sleep cycle)
  unsigned long next_gps_action = 0;
  unsigned long gps_fix_timeout = 0;

  uint32_t _gpsTickSec() const { return gps_update_interval_sec > 0 ? gps_update_interval_sec : 1; }

  #if ENV_INCLUDE_GPS
  LocationProvider* _location;
  void start_gps();
  void stop_gps();
  void initBasicGPS();
  #ifdef RAK_BOARD
  void rakGPSInit();
  bool gpsIsAwake(uint8_t ioPin);
  #endif
  #endif

public:
  #if ENV_INCLUDE_GPS
  EnvironmentSensorManager(LocationProvider &location): _location(&location){};
  LocationProvider* getLocationProvider() { return _location; }
  #else
  EnvironmentSensorManager(){};
  #endif
  bool begin() override;
  bool querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) override;
  #if ENV_INCLUDE_GPS || defined(ENV_INCLUDE_BME680_BSEC)
  void loop() override;
  #endif
  int getNumSettings() const override;
  const char* getSettingName(int i) const override;
  const char* getSettingValue(int i) const override;
  bool setSettingValue(const char* name, const char* value) override;
};
