import re
from pynmea.utils import checksum_calc, NMEADeserializer

class NMEASentence(object):
    """ Base sentence class. This is used to pull apart a sentence.
        It will not have any real reference to what things mean. Things that
        subclass this base class should all the additional functionality.
    """

    def __init__(self, parse_map):
        self.sen_type = None
        self.parse_map = parse_map
        self.deserializer = NMEADeserializer()

    def _parse(self, nmea_str):
        """ Tear the sentence apart, grabbing the name on the way. Create a
            parts attribute on the class and fill in the sentence type in
            sen_type
        """
        self.nmea_sentence = nmea_str
        self.parts = nmea_str.split(',')

        chksum_regex = re.compile(r".+((\*{1})(?i)(?P<chksum>[0-9a-f]{2}))$")
        m = chksum_regex.match(nmea_str)

        if m:
            self.checksum = m.groupdict()['chksum']
            d, par, ck = self.parts.pop().rpartition('*')
            self.parts.extend([d])

        #if '*' in self.parts[-1]:
            #d, par, ck = self.parts.pop().rpartition('*')
            #self.parts.extend([d])

        if self.parts[0].startswith('$'):
            self.parts[0] = self.parts[0][1:]
        self.sen_type = self.parts[0][-3:]
        self.talker_id = self.parts[0][:2]

    def parse(self, nmea_str, ignore_err=False):
        """ Use the parse map. Parse map should be in the format:
            (('Field name', 'field_name'),
             ('Field name', 'field_name'))

             Where the first entry in the tuple is the human readable name
             and the second is the parameter name
        """

        self._parse(nmea_str)

        #assert len(self.parts[1:]) <= len(self.parse_map)
        parts_len = len(self.parts) - 1

        for index, item in enumerate(self.parse_map):
            if index + 1 > parts_len:
                break
            if len(item) == 3:
                #If parse map contains type element, deserialize the value
                val = self.deserializer.deserialize(self.parts[index+1],item[2])
                setattr(self,item[1],val)
            else:
                setattr(self, item[1], self.parts[index + 1])
        #for index, item in enumerate(self.parts[1:]):
            #setattr(self, self.parse_map[index][1], item)

    def check_chksum(self):
        # If there is no checksum, raise AssertionError
        assert hasattr(self, 'checksum')

        result = checksum_calc(self.nmea_sentence)
        return (result.upper() == self.checksum.upper())




# ---------------------------------------------------------------------------- #
# Here are all the currently supported sentences. All should eventually be
# supported. They are being added as properties and other useful functions are
# implimented. Unit tests are also provided.
# ---------------------------------------------------------------------------- #
class AAM(NMEASentence):
    """ Waypoint Arrival Alarm
    """
    def __init__(self):
        parse_map = (
            ("Arrival Circle Entered", "arrival_circ_entered"),
            ("Perpendicular Passed", "perp_passed"),
            ("Circle Radius", "circle_rad"),
            ("Nautical Miles", "circle_rad_unit"),
            ("Waypoint ID", "waypoint_id"))

        super(AAM, self).__init__(parse_map)


class ALM(NMEASentence):
    """ GPS Almanac data
    """
    def __init__(self):
        parse_map = (("Total number of messages", "total_num_msgs"),
                     ("Message number", "msg_num"),
                     ("Satellite PRN number", "sat_prn_num"), # 01 - 32
                     ("GPS week number", "gps_week_num"), # Week since Jan 6 1980
                     ("SV Health, bits 17-24 of each almanac page", "sv_health"),
                     ("Eccentricity", "eccentricity"),
                     ("Almanac Reference Time", "alamanac_ref_time"),
                     ("Inclination Angle", "inc_angle"),
                     ("Rate of right ascension", "rate_right_asc"),
                     ("Root of semi-major axis", "root_semi_major_axis"),
                     ("Argument of perigee", "arg_perigee"),
                     ("Longitude of ascension node", "lat_asc_node"),
                     ("Mean anomaly", "mean_anom"),
                     ("F0 Clock parameter", "f0_clock_param"),
                     ("F1 Clock parameter", "f1_clock_param"))

        super(ALM, self).__init__(parse_map)


class APA(NMEASentence):
    """ Autopilot Sentence "A"
    """

    def __init__(self):
        parse_map = (
            ("General Status", "status_gen"),
            ("Cycle lock Status", "status_cycle_lock"),
            ("Cross Track Error Magnitude", "cross_track_err_mag"),
            ("Direction to Steer (L or R)", "dir_steer"),
            ("Cross Track Units (Nautical Miles or KM)", "cross_track_unit"),
            ("Arrival Circle Entered", "arr_circle_entered"), # A = True
            ("Perpendicular passed at waypoint", "perp_passed"), # A = True
            ("Bearing origin to destination", "bearing_to_dest"),
            ("Bearing type", "bearing_type"), # M = Magnetic, T = True
            ("Destination waypoint ID", "dest_waypoint_id"))

        super(APA, self).__init__(parse_map)


class APB(NMEASentence):
    """ Autopilot Sentence "B"
    """

    def __init__(self):
        parse_map = (
            ("General Status", "status_gen"),
            ("Cycle lock Status", "status_cycle_lock"),
            ("Cross Track Error Magnitude", "cross_track_err_mag"),
            ("Direction to Steer (L or R)", "dir_steer"),
            ("Cross Track Units (Nautical Miles or KM)", "cross_track_unit"),
            ("Arrival Circle Entered", "arr_circle_entered"), # A = True
            ("Perpendicular passed at waypoint", "perp_passed"), # A = True
            ("Bearing origin to destination", "bearing_to_dest"),
            ("Bearing type", "bearing_type"), # M = Magnetic, T = True
            ("Destination waypoint ID", "dest_waypoint_id"),
            ("Bearing, present position to dest", "bearing_pres_dest"),
            ("Bearing to destination, type", "bearing_pres_dest_type"), # M = Magnetic, T = True
            ("Heading to steer to destination", "heading_to_dest"),
            ("Heading to steer to destination type", "heading_to_dest_type")) # M = Magnetic, T = True

        super(APB, self).__init__(parse_map)


class BEC(NMEASentence):
    """ Bearing & Distance to Waypoint, Dead Reckoning
    """
    def __init__(self):
        parse_map = (
            ("Timestamp", "timestamp"),
            ("Waypoint Latitude", "waypoint_lat"),
            ("Waypoint Latitude direction", "waypoint_lat_dir"),
            ("Waypoint Longitude", "waypoint_lon"),
            ("Waypoint Longitude direction", "waypoint_lon_dir"),
            ("Bearing, true", "bearing_true"),
            ("Bearing True symbol", "bearing_true_sym"), # T = true
            ("Bearing Magnetic", "bearing_mag"),
            ("Bearing Magnetic symbol", "bearing_mag_sym"),
            ("Nautical Miles", "nautical_miles"),
            ("Nautical Miles symbol", "nautical_miles_sym"),
            ("Waypoint ID", "waypoint_id"),
            ("FAA mode indicator", "faa_mode"))

        super(BEC, self).__init__(parse_map)


class BOD(NMEASentence):
    def __init__(self):
        # 045.,T,023.,M,DEST,START
        parse_map = (('Bearing True', 'bearing_t'),
                     ('Bearing True Type', 'bearing_t_type'),
                     ('Bearing Magnetic', 'bearing_mag'),
                     ('Bearing Magnetic Type', 'bearing_mag_type'),
                     ('Destination', 'dest'),
                     ('Start', 'start'))

        super(BOD, self).__init__(parse_map)

    @property
    def bearing_true(self):
        return ','.join([self.bearing_t, self.bearing_t_type])

    @property
    def bearing_magnetic(self):
        return ','.join([self.bearing_mag, self.bearing_mag_type])

    @property
    def destination(self):
        return self.dest

    @property
    def origin(self):
        return self.start


class BWC(NMEASentence):
    def __init__(self):
        parse_map = (
            ('Timestamp', 'timestamp'),
            ('Latitude of next Waypoint', 'lat_next'),
            ('Latitude of next Waypoint Direction', 'lat_next_direction'),
            ('Longitude of next Waypoint', 'lon_next'),
            ('Longitude of next Waypoint Direction', 'lon_next_direction'),
            ('True track to waypoint', 'true_track'),
            ('True Track Symbol', 'true_track_sym'),
            ('Magnetic track to waypoint', 'mag_track'),
            ('Magnetic Symbol', 'mag_sym'),
            ('Range to waypoint', 'range_next'),
            ('Unit of range', 'range_unit'),
            ('Waypoint Name', 'waypoint_name'))
            #('Checksum', 'checksum'))

        super(BWC, self).__init__(parse_map)


class BWR(NMEASentence):
    def __init__(self):
        parse_map = (
            ('Timestamp', 'timestamp'),
            ('Latitude of next Waypoint', 'lat_next'),
            ('Latitude of next Waypoint Direction', 'lat_next_direction'),
            ('Longitude of next Waypoint', 'lon_next'),
            ('Longitude of next Waypoint Direction', 'lon_next_direction'),
            ('True track to waypoint', 'true_track'),
            ('True Track Symbol', 'true_track_sym'),
            ('Magnetic track to waypoint', 'mag_track'),
            ('Magnetic Symbol', 'mag_sym'),
            ('Range to waypoint', 'range_next'),
            ('Unit of range', 'range_unit'),
            ('Waypoint Name', 'waypoint_name'))
            #('Checksum', 'checksum'))

        super(BWR, self).__init__(parse_map)


class GGA(NMEASentence):
    def __init__(self):
        parse_map = (
            ('Timestamp', 'timestamp'),
            ('Latitude', 'latitude'),
            ('Latitude Direction', 'lat_direction'),
            ('Longitude', 'longitude'),
            ('Longitude Direction', 'lon_direction'),
            ('GPS Quality Indicator', 'gps_qual'),
            ('Number of Satellites in use', 'num_sats'),
            ('Horizontal Dilution of Precision', 'horizontal_dil'),
            ('Antenna Alt above sea level (mean)', 'antenna_altitude'),
            ('Units of altitude (meters)', 'altitude_units'),
            ('Geoidal Separation', 'geo_sep'),
            ('Units of Geoidal Separation (meters)', 'geo_sep_units'),
            ('Age of Differential GPS Data (secs)', 'age_gps_data'),
            ('Differential Reference Station ID', 'ref_station_id'))
            #('Checksum', 'checksum'))

        super(GGA, self).__init__(parse_map)


class BWW(NMEASentence):
    """ Bearing, Waypoint to Waypoint
    """
    def __init__(self):
        parse_map = (
            ("Bearing degrees True", "bearing_deg_true"),
            ("Bearing degrees True Symbol", "bearing_deg_true_sym"),
            ("Bearing degrees Magnitude", "bearing_deg_mag"),
            ("Bearing degrees Magnitude Symbol", "bearing_deg_mag_sym"),
            ("Destination Waypoint ID", "waypoint_id_dest"),
            ("Origin Waypoint ID", "waypoint_id_orig"))

        super(BWW, self).__init__(parse_map)


class GLL(NMEASentence):
    def __init__(self):
        parse_map = (
            ('Latitude', 'lat'),
            ('Latitude Direction', 'lat_dir'),
            ('Longitude', 'lon'),
            ('Longitude Direction', 'lon_dir'),
            ('Timestamp', 'timestamp'),
            ('Data Validity', "data_valid"))

        super(GLL, self).__init__(parse_map)

        self._use_data_validity = False

    #def _parse(self, nmea_str):
        #""" GGL Allows for a couple of different formats.
            #The all have lat,direction,lon,direction

            #but one may have timestamp,data_validity
            #while the other has only checksum

            #We shall treat data_validity as a checksum and always
            #add in a timestamp field

        #"""
        #self.nmea_sentence = nmea_str
        #self.parts = nmea_str.split(',')

        #chksum_regex = re.compile(r".+((\*{1})(?i)(?P<chksum>[0-9a-f]{2}))$")
        #m = chksum_regex.match(nmea_str)

        #if m:
            #self.checksum = m.groupdict()['chksum']


        ##if '*' in self.parts[-1]:
            ### There is a checksum but no timestamp + data_validity.
            ### Add an empty field for the timestamp and indicate that when
            ### validating the checksum, we should use validity, not a
            ### calculation
            ##d, par, ck = self.parts.pop().rpartition('*')
            ##self.parts.extend([d, ''])
            ##self._use_data_validity = True

        #self.sen_type = self.parts[0]
        #if self.parts[0].startswith('$'):
            #self.parts[0] = self.parts[0][1:]
        #self.sen_type = self.parts[0]

    #def check_chksum(self):
        #""" Override check_checksum. If it has been detected that
            #the checksum field contains "A" for valid data and something else
            #for invalid, do a check based on thsi information. Otherwise, call
            #to original checksum code from the superclass
        #"""
        ## If we are looking for an "A" character
        #if self._use_data_validity:
            #if self.checksum == 'A':
                #return True
            #else:
                #return False

        #else:
            ## Otherwise, call the superclass version
            #return super(GLL, self).check_chksum()

    @property
    def latitude(self):
        return float(self.lat)

    @property
    def longitude(self):
        return float(self.lon)

    @property
    def lat_direction(self):
        mapping = {'N': 'North', 'S': 'South'}
        return mapping[self.lat_dir.upper()]

    @property
    def lon_direction(self):
        mapping = {"E": "East", "W": "West"}
        return mapping[self.lon_dir.upper()]


class GSA(NMEASentence):
    def __init__(self):
        parse_map = (
            ('Mode', 'mode'),
            ('Mode fix type', 'mode_fix_type'),
            ('SV ID01', 'sv_id01'),
            ('SV ID02', 'sv_id02'),
            ('SV ID03', 'sv_id03'),
            ('SV ID04', 'sv_id04'),
            ('SV ID05', 'sv_id05'),
            ('SV ID06', 'sv_id06'),
            ('SV ID07', 'sv_id07'),
            ('SV ID08', 'sv_id08'),
            ('SV ID09', 'sv_id09'),
            ('SV ID10', 'sv_id10'),
            ('SV ID11', 'sv_id11'),
            ('SV ID12', 'sv_id12'),
            ('PDOP (Dilution of precision)', 'pdop'),
            ('HDOP (Horizontal DOP)', 'hdop'),
            ('VDOP (Vertical DOP)', 'vdop'))
            #('Checksum', 'checksum'))

        super(GSA, self).__init__(parse_map)


class GSV(NMEASentence):
    def __init__(self):
        parse_map = (
            ('Number of messages of type in cycle', 'num_messages'),
            ('Message Number', 'msg_num'),
            ('Total number of SVs in view', 'num_sv_in_view'),
            ('SV PRN number 1', 'sv_prn_num_1'),
            ('Elevation in degrees 1', 'elevation_deg_1'), # 90 max
            ('Azimuth, deg from true north 1', 'azimuth_1'), # 000 to 159
            ('SNR 1', 'snr_1'), # 00-99 dB
            ('SV PRN number 2', 'sv_prn_num_2'),
            ('Elevation in degrees 2', 'elevation_deg_2'), # 90 max
            ('Azimuth, deg from true north 2', 'azimuth_2'), # 000 to 159
            ('SNR 2', 'snr_2'), # 00-99 dB
            ('SV PRN number 3', 'sv_prn_num_3'),
            ('Elevation in degrees 3', 'elevation_deg_3'), # 90 max
            ('Azimuth, deg from true north 3', 'azimuth_3'), # 000 to 159
            ('SNR 3', 'snr_3'), # 00-99 dB
            ('SV PRN number 4', 'sv_prn_num_4'),
            ('Elevation in degrees 4', 'elevation_deg_4'), # 90 max
            ('Azimuth, deg from true north 4', 'azimuth_4'), # 000 to 159
            ('SNR 4', 'snr_4'))  # 00-99 dB
            #('Checksum', 'checksum'))

        super(GSV, self).__init__(parse_map)


class HDG(NMEASentence):
    """ NOTE! This is a GUESS as I cannot find an actual spec
        telling me the fields. Updates are welcome!
    """
    def __init__(self):
        parse_map = (
            ("Heading", "heading"),
            ("Deviation", "deviation"),
            ("Deviation Direction", "dev_dir"),
            ("Variation", "variation"),
            ("Variation Direction", "var_dir"))
            #("Checksum", "checksum"))

        super(HDG, self).__init__(parse_map)


class HDT(NMEASentence):
    def __init__(self):
        parse_map = (
            ("Heading", "heading","decimal"),
            ("True", "hdg_true"))
            #("Checksum", "checksum"))

        super(HDT, self).__init__(parse_map)


class R00(NMEASentence):
    def __init__(self):
        parse_map = (
            ("Waypoint List", "waypoint_list"),)
            #("Checksum", "checksum"))

        super(R00, self).__init__(parse_map)

    def parse(self, nmea_str):
        """ As the length of the sentence is variable (there can be many or few
            waypoints), parse is overridden to do something special with the
            different parts
        """
        self._parse(nmea_str)

        new_parts = [self.parts[0]]
        new_parts.append(self.parts[1:])
        #new_parts.append(self.parts[-1])

        self.parts = new_parts

        for index, item in enumerate(self.parts[1:]):
            setattr(self, self.parse_map[index][1], item)


class RMA(NMEASentence):
    def __init__(self):
        parse_map = (
            ("Data status", "data_status"),
            ("Latitude", "lat"),
            ("Latitude Direction", "lat_dir"),
            ("Longitude", "lon"),
            ("Longitude Direction", "lon_dir"),
            ("Not Used 1", "not_used_1"),
            ("Not Used 2", "not_used_2"),
            ("Speed over ground", "spd_over_grnd"), # Knots
            ("Course over ground", "crse_over_grnd"),
            ("Variation", "variation"),
            ("Variation Direction", "var_dir"))
            #("Checksum", "checksum"))

        super(RMA, self).__init__(parse_map)


class RMB(NMEASentence):
    """ Recommended Minimum Navigation Information
    """
    def __init__(self):
        parse_map = (
            ("Data Validity", "validity"),
            ("Cross Track Error", "cross_track_error"), # nautical miles, 9.9 max
            ("Cross Track Error, direction to corrent", "cte_correction_dir"),
            ("Origin Waypoint ID", "origin_waypoint_id"),
            ("Destination Waypoint ID", "dest_waypoint_id"),
            ("Destination Waypoint Latitude", "dest_lat"),
            ("Destination Waypoint Lat Direction", "dest_lat_dir"),
            ("Destination Waypoint Longitude", "dest_lon"),
            ("Destination Waypoint Lon Direction", "dest_lon_dir"),
            ("Range to Destination", "dest_range"), # Nautical Miles
            ("True Bearing to Destination", "dest_true_bearing"),
            ("Velocity Towards Destination", "dest_velocity"), # Knots
            ("Arrival Alarm", "arrival_alarm")) # A = Arrived, V = Not arrived
            #("Checksum", "checksum"))
        super(RMB, self).__init__(parse_map)


class RMC(NMEASentence):
    """ Recommended Minimum Specific GPS/TRANSIT Data
    """
    def __init__(self):
        parse_map = (("Timestamp", "timestamp"),
                     ("Data Validity", "data_validity"),
                     ("Latitude", "lat"),
                     ("Latitude Direction", "lat_dir"),
                     ("Longitude", "lon"),
                     ("Longitude Direction", "lon_dir"),
                     ("Speed Over Ground", "spd_over_grnd"),
                     ("True Course", "true_course"),
                     ("Datestamp", "datestamp"),
                     ("Magnetic Variation", "mag_variation"),
                     ("Magnetic Variation Direction", "mag_var_dir"))
                     #("Checksum", "checksum"))
        super(RMC, self).__init__(parse_map)


class RTE(NMEASentence):
    """ Routes
    """
    def __init__(self):
        parse_map = (
            ("Number of sentences in sequence", "num_in_seq"),
            ("Sentence Number", "sen_num"),
            ("Start Type", "start_type"), # The first in the list is either current route or waypoint
            ("Name or Number of Active Route", "active_route_id"),
            ("Waypoint List", "waypoint_list"))
            #("Checksum", "checksum"))

        super(RTE, self).__init__(parse_map)

    def parse(self, nmea_str):
        """ As the length of the sentence is variable (there can be many or few
            waypoints), parse is overridden to do something special with the
            different parts
        """
        self._parse(nmea_str)

        new_parts = []
        new_parts.extend(self.parts[0:5])
        new_parts.append(self.parts[5:])

        self.parts = new_parts

        for index, item in enumerate(self.parts[1:]):
            setattr(self, self.parse_map[index][1], item)


class STN(NMEASentence):
    """ NOTE: No real data could be found for examples of the actual spec so
            it is a guess that there may be a checksum on the end
    """
    def __init__(self):
        parse_map = (
            ("Talker ID Number", "talker_id"),) # 00 - 99
            #("Checksum", "checksum"))


        super(STN, self).__init__(parse_map)


class TRF(NMEASentence):
    """ Transit Fix Data
    """
    def __init__(self):
        parse_map = (
            ("Timestamp (UTC)", "timestamp"),
            ("Date (DD/MM/YY", "date"),
            ("Latitude", "lat"),
            ("Latitude Direction", "lat_dir"),
            ("Longitude", "lon"),
            ("Longitude Direction", "lon_dir"),
            ("Elevation Angle", "ele_angle"),
            ("Number of Iterations", "num_iterations"),
            ("Number of Doppler Intervals", "num_doppler_intervals"),
            ("Update Distance", "update_dist"), # Nautical Miles
            ("Satellite ID", "sat_id"))

        super(TRF, self).__init__(parse_map)


class VBW(NMEASentence):
    """ Dual Ground/Water Speed
    """
    def __init__(self):
        parse_map = (
            ("Longitudinal Water Speed", "lon_water_spd"), # Knots
            ("Transverse Water Speed", "trans_water_spd"), # Knots
            ("Water Speed Data Validity", "data_validity_water_spd"),
            ("Longitudinal Ground Speed", "lon_grnd_spd"), # Knots
            ("Transverse Ground Speed", "trans_grnd_spd"), # Knots
            ("Ground Speed Data Validity", "data_validity_grnd_spd"))
            #("Checksum", "checksum"))
        super(VBW, self).__init__(parse_map)


class VTG(NMEASentence):
    """ Track Made Good and Ground Speed
    """
    def __init__(self):
        parse_map = (
            ("True Track made good", "true_track"),
            ("True Track made good symbol", "true_track_sym"),
            ("Magnetic Track made good", "mag_track"),
            ("Magnetic Track symbol", "mag_track_sym"),
            ("Speed over ground knots", "spd_over_grnd_kts"),
            ("Speed over ground symbol", "spd_over_grnd_kts_sym"),
            ("Speed over ground kmph", "spd_over_grnd_kmph"),
            ("Speed over ground kmph symbol", "spd_over_grnd_kmph_sym"))

        super(VTG, self).__init__(parse_map)


class WCV(NMEASentence):
    """ Waypoint Closure Velocity
    """
    def __init__(self):
        parse_map = (
            ("Velocity", "velocity"),
            ("Velocity Units", "vel_units"), # Knots
            ("Waypoint ID", "waypoint_id"))

        super(WCV, self).__init__(parse_map)


class WNC(NMEASentence):
    """ Distance, Waypoint to Waypoint
    """
    def __init__(self):
        parse_map = (
            ("Distance, Nautical Miles", "dist_nautical_miles"),
            ("Distance Nautical Miles Unit", "dist_naut_unit"),
            ("Distance, Kilometers", "dist_km"),
            ("Distance, Kilometers Unit", "dist_km_unit"),
            ("Origin Waypoint ID", "waypoint_origin_id"),
            ("Destination Waypoint ID", "waypoint_dest_id"))

        super(WNC, self).__init__(parse_map)


class WPL(NMEASentence):
    """ Waypoint Location
    """
    def __init__(self):
        parse_map = (
            ("Latitude", "lat"),
            ("Latitude Direction", "lat_dir"),
            ("Longitude", "lon"),
            ("Longitude Direction", "lon_dir"),
            ("Waypoint ID", "waypoint_id"))

        super(WPL, self).__init__(parse_map)


class XTE(NMEASentence):
    """ Cross-Track Error, Measured
    """
    def __init__(self):
        parse_map = (("General Warning Flag", "warning_flag"),
                     ("Lock flag (Not Used)", "lock_flag"),
                     ("Cross Track Error Distance", "cross_track_err_dist"),
                     ("Correction Direction (L or R)", "correction_dir"),
                     ("Distance Units", "dist_units"))

        super(XTE, self).__init__(parse_map)


class ZDA(NMEASentence):
    def __init__(self):
        parse_map = (
            ("Timestamp", "timestamp"), # hhmmss.ss = UTC
            ("Day", "day"), # 01 to 31
            ("Month", "month"), # 01 to 12
            ("Year", "year"), # Year = YYYY
            ("Local Zone Description", "local_zone"), # 00 to +/- 13 hours
            ("Local Zone Minutes Description", "local_zone_minutes")) # same sign as hours
        #("Checksum", "checksum"))

        super(ZDA, self).__init__(parse_map)

# Implemented by Janez Stupar for Visionect
class RSA(NMEASentence):
    """ Rudder Sensor Angle
    """
    def __init__(self):
        parse_map = (
            ("Starboard rudder sensor","rsa_starboard","decimal"),
            ("Starboard rudder sensor status","rsa_starboard_status"),
            ("Port rudder sensor","rsa_port","decimal"),
            ("Port rudder sensor status","rsa_port_status"),
        )
        super(RSA,self).__init__(parse_map)

class HSC(NMEASentence):
    """ Heading Steering Command
    """
    def __init__(self):
        parse_map = (
            ("Heading","heading_true","decimal"),
            ("True","true"),
            ("Heading Magnetic","heading_magnetic","decimal"),
            ("Magnetic","magnetic"),
        )
        super(HSC,self).__init__(parse_map)

class MWD(NMEASentence):
    """ Wind Direction
    """
    def __init__(self):
        parse_map = (
            ("Wind direction true","direction_true","decimal"),
            ("True","true"),
            ("Wind direction magnetic","direction_magnetic","decimal"),
            ("Magnetic","magnetic"),
            ("Wind speed knots","wind_speed_knots","decimal"),
            ("Knots","knots"),
            ("Wind speed meters/second","wind_speed_meters","decimal"),
            ("Wind speed","meters"),
        )
        super(MWD,self).__init__(parse_map)

class MWV(NMEASentence):
    """ Wind Speed and Angle
    """
    def __init__(self):
        parse_map = (
            ("Wind angle","wind_angle","decimal"),
            ("Reference","reference"), # relative (R)/true(T)
            ("Wind speed","wind_speed","decimal"),
            ("Wind speed units","wind_speed_units"), # K/M/N
            ("Status","status"),
        )
        super(MWV,self).__init__(parse_map)

# ---------------------------------- Not Yet Implimented --------------------- #
# ---------------------------------------------------------------------------- #

#class DBT(NMEASentence):
#    """ Depth Below Transducer
#    """
#    def __init__(self):
#        parse_map = ()
#        super(DBT).__init__(parse_map)

#class DPT(NMEASentence):
#    """ Heading - Deviation and Variation
#    """
#    def __init__(self):
#        parse_map = ()
#        super(DPT).__init__(parse_map)

#class FSI(NMEASentence):
#    """ Frequency Set Information
#    """
#    def __init__(self):
#        parse_map = ()
#        super(FSI).__init__(parse_map)

#class GLC(NMEASentence):
#    """ Geographic Position, Loran-C
#    """
#    def __init__(self):
#        parse_map = ()
#        super(GLC).__init__(parse_map)

#class GXA(NMEASentence):
#    """ TRANSIT Position
#    """
#    def __init__(self):
#        parse_map = ()
#        super(GXA).__init__(parse_map)

#class HSC(NMEASentence):
#    """ Heading Steering Command
#    """
#    def __init__(self):
#        parse_map = ()
#        super(HSC).__init__(parse_map)

#class LCD(NMEASentence):
#    """ Loran-C Signal Data
#    """
#    def __init__(self):
#        parse_map = ()
#        super(LCD).__init__(parse_map)

#class MTA(NMEASentence):
#    """ Air Temperature (to be phased out)
#    """
#    def __init__(self):
#        parse_map = ()
#        super(MTA).__init__(parse_map)

#class MTW(NMEASentence):
#    """ Water Temperature
#    """
#    def __init__(self):
#        parse_map = ()
#        super(MTW).__init__(parse_map)

#class MWD(NMEASentence):
#    """ Wind Direction
#    """
#    def __init__(self):
#        parse_map = ()
#        super(MWD).__init__(parse_map)

#class MWV(NMEASentence):
#    """ Wind Speed and Angle
#    """
#    def __init__(self):
#        parse_map = ()
#        super(MWV).__init__(parse_map)

#class OLN(NMEASentence):
#    """ Omega Lane Numbers
#    """
#    def __init__(self):
#        parse_map = ()
#        super(OLN).__init__(parse_map)

#class OSD(NMEASentence):
#    """ Own Ship Data
#    """
#    def __init__(self):
#        parse_map = ()
#        super(OSD).__init__(parse_map)

#class ROT(NMEASentence):
#    """ Rate of Turn
#    """
#    def __init__(self):
#        parse_map = ()
#        super(ROT).__init__(parse_map)

#class RPM(NMEASentence):
#    """ Revolutions
#    """
#    def __init__(self):
#        parse_map = ()
#        super(RPM).__init__(parse_map)

#class RSA(NMEASentence):
#    """ Rudder Sensor Angle
#    """
#    def __init__(self):
#        parse_map = ()
#        super(RSA).__init__(parse_map)

#class RSD(NMEASentence):
#    """ RADAR System Data
#    """
#    def __init__(self):
#        parse_map = ()
#        super(RSD).__init__(parse_map)

#class SFI(NMEASentence):
#    """ Scanning Frequency Information
#    """
#    def __init__(self):
#        parse_map = ()
#        super(SFI).__init__(parse_map)

#class TTM(NMEASentence):
#    """ Tracked Target Message
#    """
#    def __init__(self):
#        parse_map = ()
#        super(TTM).__init__(parse_map)

#class VDR(NMEASentence):
#    """ Set and Drift
#    """
#    def __init__(self):
#        parse_map = ()
#        super(VDR).__init__(parse_map)

#class VHW(NMEASentence):
#    """ Water Speed and Heading
#    """
#    def __init__(self):
#        parse_map = ()
#        super(VHW).__init__(parse_map)

#class VLW(NMEASentence):
#    """ Distance Traveled through the Water
#    """
#    def __init__(self):
#        parse_map = ()
#        super(VLW).__init__(parse_map)

#class VPW(NMEASentence):
#    """ Speed, Measured Parallel to Wind
#    """
#    def __init__(self):
#        parse_map = ()
#        super(VPW).__init__(parse_map)

#class XDR(NMEASentence):
#    """ Transducer Measurements
#    """
#    def __init__(self):
#        parse_map = ()
#        super(XDR).__init__(parse_map)

#class XTR(NMEASentence):
#    """ Cross-Track Error, Dead Reckoning
#    """
#    def __init__(self):
#        parse_map = ()
#        super(XTR).__init__(parse_map)

#class ZFO(NMEASentence):
#    """ UTC & Time from Origin Waypoint
#    """
#    def __init__(self):
#        parse_map = ()
#        super(ZFO).__init__(parse_map)

#class ZTG(NMEASentence):
#    """ UTC & Time to Destination Waypoint
#    """
#    def __init__(self):
#        parse_map = ()
#        super(ZTG).__init__(parse_map)


# ---------------------------------------------------------------------------- #
# -------------------------- Unknown Formats --------------------------------- #
# ---------------------------------------------------------------------------- #

#class ASD(NMEASentence):
#    """ Auto-pilot system data (Unknown format)
#    """
#    def __init__(self):
#        parse_map = ()
#        super(ASD).__init__()

# ---------------------------------------------------------------------------- #
# -------------------------- Obsolete Formats -------------------------------- #
# ---------------------------------------------------------------------------- #

#class DCN(NMEASentence):
#    """ Decca Position (obsolete)
#    """
#    def __init__(self):
#        parse_map = ()
#        super(DCN).__init__(parse_map)


# PROPRIETRY SENTENCES

# -- GARMIN -- #
class RME(NMEASentence):
    """ GARMIN Estimated position error
    """
    def __init__(self):
        parse_map = (("Estimated Horiz. Position Error", "hpe"),
                     ("Estimated Horiz. Position Error Unit (M)", "hpe_unit"),
                     ("Estimated Vert. Position Error", "vpe"),
                     ("Estimated Vert. Position Error Unit (M)", "vpe_unit"),
                     ("Estimated Horiz. Position Error", "osepe"),
                     ("Overall Spherical Equiv. Position Error", "osepe_unit"))

        super(RME, self).__init__(parse_map)


class RMM(NMEASentence):
    """ GARMIN Map Datum
    """
    def __init__(self):
        parse_map = (('Currently Active Datum', 'datum'),)

        super(RMM, self).__init__(parse_map)


class RMZ(NMEASentence):
    """ GARMIN Altitude Information
    """
    def __init__(self):
        parse_map = (("Altitude", "altitude"),
                     ("Altitude Units (Feet)", "altitude_unit"),
                     ("Positional Fix Dimension (2=user, 3=GPS)",
                      "pos_fix_dim"))

        super(RMZ, self).__init__(parse_map)
