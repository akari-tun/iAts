#if defined(PROTOCOL_NMEA) || defined(PROTOCOL_UBLOX)
bool gps_read()
{
    GPS.Read();

    if (GPS.NewData) {// New GPS data?
        //digitalWrite(LED_GPS_PIN, HIGH);

        uav_satellites_visible = GPS.NumSats;
        uav_fix_type = GPS.Fix;
        if (uav_fix_type == 1)
        {
            uav_fix_type = 3; // 3D fix
        }
        else
            uav_fix_type == 1; // no fix
        uav_lat = GPS.Lattitude;
        uav_lon = GPS.Longitude;
#if defined(PROTOCOL_NMEA)
        uav_alt = round(GPS.Altitude / 10.0f); //from mm to cm
#else
#if defined(PROTOCOL_UBLOX)
        uav_alt = round(GPS.Altitude / 10.0f);
#else
        uav_alt = GPS.Altitude; //in cm
#endif
#endif
        uav_groundspeed = (uint8_t)round(GPS.Ground_Speed / 100.0f); // in m/s
        //uav_groundspeed = GPS.Ground_Speed; 
        uav_heading = (int16_t)round(GPS.Ground_Course / 100.0f);    // in deg
        //uav_heading = GPS.Ground_Course;

        GPS.NewData = 0;

        return true;
    }
    else { 
        return false;
    }
}
#endif
