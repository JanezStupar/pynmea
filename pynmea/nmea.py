# coding=utf-8
import decimal
import re
from pynmea.utils import checksum_calc, NMEADeserializer
from pynmea import exceptions


class NMEASentence(object):
    """ Base sentence class. This is used to pull apart a sentence.
        It will not have any real reference to what things mean. Things that
        subclass this base class should all the additional functionality.
    """

    def __init__(self, parse_map,**kwargs):
        self.sen_type = None
        self.parse_map = parse_map
        self.deserialize = kwargs.get('deserialize',False)
        if self.deserialize:
            self.deserializer = NMEADeserializer()

    def _parse(self, nmea_str):
        """ Tear the sentence apart, grabbing the name on the way. Create a
            parts attribute on the class and fill in the sentence type in
            sen_type
        """
        self.parts = nmea_str.split(',')

        self._parse_checksum(nmea_str)
        #if '*' in self.parts[-1]:
            #d, par, ck = self.parts.pop().rpartition('*')
            #self.parts.extend([d])

        if self.parts[0].startswith('$'):
            self.parts[0] = self.parts[0][1:]
        self.sen_type = self.parts[0][-3:]
        self.talker_id = self.parts[0][:2]

    def _parse_checksum(self, nmea_str):
        chksum_regex = re.compile(r".+((\*{1})(?i)(?P<chksum>[0-9a-f]{2}))$")
        
        m = chksum_regex.search(nmea_str)

        if m:
            self.checksum = m.groupdict()['chksum']
            d, par, ck = self.parts.pop().rpartition('*')
            self.parts.extend([d])
        

    def construct(self,**kwargs):
        """
        Construct an empty object for serialization.

        The object is built according to the parse map. Defaults are taken
        from the kwargs.
        """

        talker_id = kwargs.get('talker_id','II')

        if not hasattr(self,'parts'):
            self.parts = []

        self.talker_id = talker_id
        self.sen_type = str(self.__class__).split('.')[-1].split('\'')[0]
        self.parts.insert(0, self.talker_id + self.sen_type)
        for index,item in enumerate(self.parse_map,1):

            #get value from the kwargs according to the parse_map key
            value = kwargs.get(item[1],None)
            if value is None:
                setattr(self,item[1],'')
                self.parts.insert(index,'')
            #use correct type
            elif self.deserialize and len(item) > 2:
                deserialized_val = self.deserializer.deserialize(value,item[2])
                setattr(self,item[1],deserialized_val)
                self.parts.insert(index,deserialized_val)
            else:
                setattr(self,item[1],value)
                self.parts.insert(index,value)


    @property
    def nmea_sentence(self):
        """
        Dump the object data into a NMEA sentence
        """
        tmp = []
        if isinstance(self,STALKSentence) or isinstance(self, TMQSentence):
            tmp = self.parts
        else:
            tmp.append(self.talker_id + self.sen_type)
            for i,elem in enumerate(self.parse_map):
                val = str(getattr(self,elem[1],''))
                if val is 'NaN':
                    val = ''
                tmp.append(val)
        tmp = (',').join(tmp)
        return '$' + tmp + '*' + checksum_calc(tmp)

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
            if self.deserialize and len(item) > 2:
                #If parse map contains type element, deserialize the value
                val = self.deserializer.deserialize(self.parts[index+1],item[2])
                setattr(self,item[1],val)
            else:
                setattr(self, item[1], self.parts[index + 1])

        #if deserialization is enabled some data (visavis checksum) may get lost (e.g. 010.4 becomes 10.4 in Decimal)
        #which means that dynamically constructed statements won't match the original data, which will mean a new
        #checksum.
        if self.deserialize:
            self.checksum = checksum_calc(self.nmea_sentence)

        #Check the checksum
        if hasattr(self,'checksum') and not self.check_chksum():
            raise exceptions.ChecksumException('Checksum error for nmea statement: %s' % nmea_str)

    def check_chksum(self):
        # If there is no checksum, raise AssertionError
        assert hasattr(self, 'checksum')

        result = checksum_calc(self.nmea_sentence)
        return (result.upper() == self.checksum.upper())

class TMQSentence(NMEASentence):

    # we use the same values for autopilot mode as SEATALK
    _AUTOPILOT_MODES = {
        'standby': '0',
        'auto': '2'
    }

    def __init__(self, **kwargs):
        self.talker_id = 'PT'

    def parse(self, tmq_str, ignore_err=False):
        self.talker_id = self.parts[0][:2]

        self._parse_checksum(tmq_str)

        #Check the checksum
        if hasattr(self,'checksum') and not self.check_chksum():
            raise exceptions.ChecksumException('Checksum error for nmea statement: %s' % tmq_str)

    def construct(self,**kwargs):
        raise NotImplementedError

    def _compute_heading(self,hbyte,lbyte):
        return  decimal.Decimal(((hbyte<<8) | lbyte) / 4)

    def _encode_heading(self, heading):
        hbyte = int(heading*4) >> 8
        lbyte = int(heading*4) & 255

        return chr(hbyte),chr(lbyte)

    def _compute_rudder_position(self, angle):
        # rudder angle Left(Port) 0 - 128 (Center) - 255 Right (Starboard)
        return decimal.Decimal(round(-45 + (angle/float(128) * 45), 1))

class STALKSentence(NMEASentence):
    def __init__(self,**kwargs):
        self.talker_id = 'ST'

    def parse(self, stalk_str, ignore_err=False):
        self._parse(stalk_str)

        #Check the checksum
        if hasattr(self,'checksum') and not self.check_chksum():
            raise exceptions.ChecksumException('Checksum error for nmea statement: %s' % stalk_str)

    def construct(self,**kwargs):
        raise NotImplementedError

    def _compute_compass_heading(self,U,VW):
        """
        Compass heading in degrees:
            The two lower  bits of  U * 90 +
            the six lower  bits of VW *  2 +
            number of bits set in the two higher bits of U =
            (U & 0x3)* 90 + (VW & 0x3F)* 2 + (U & 0xC ? (U & 0xC == 0xC ? 2 : 1): 0)
        """

        # Compute compass heading
        tmp = U & 0xC
        if not tmp == 0:
            tmp = ((U & 0xC == 0xC) and (2,) or (1,))[0]
        return decimal.Decimal((U & 0x3) * 90 + (VW & 0x3F) * 2 + tmp)

    def _compute_turning_direction(self,U):
        """
          Turning direction:
            Most significant bit of U = 1: Increasing heading, Ship turns right
            Most significant bit of U = 0: Decreasing heading, Ship turns left
        """
        return U & 0x8

    def _compute_rudder_position(self,RR):
        """
          Rudder position: RR degrees (positive values steer right,
            negative values steer left. Example: 0xFE = 2 degrees left)
        """
        #Rudder position, positive value = right, negative=left
        #conversion from hex to signed int has been copied from:
        # http://stackoverflow.com/questions/385572/need-help-typecasting-in-python
        return decimal.Decimal((RR + 2**7) % 2**8 - 2**7)

# ---------------------------------------------------------------------------- #
# SEATALK statement parsing according to the gadgetpool.de
# SEATALK/NMEA USB link.
# ---------------------------------------------------------------------------- #
class S84(STALKSentence):
    """
    SEATALK Datagram 84, Rudder position (see also command 9C)

    Interpreted according to http://www.thomasknauf.de/rap/seatalk2.htm
    """

    def parse(self,stalk_str,**kwargs):
        """
        SEATALK statement mask:
            STALK,84,U6,VW,XY,0Z,0M,RR,SS,TT

              Autopilot course in degrees:
                The two higher bits of  V * 90 + XY / 2
                 Z & 0x2 = 0 : Autopilot in Standby-Mode
                 Z & 0x2 = 2 : Autopilot in Auto-Mode
                 Z & 0x4 = 4 : Autopilot in Vane Mode (WindTrim), requires regular '10' datagrams
                 Z & 0x8 = 8 : Autopilot in Track Mode
              M: Alarms + audible beeps
                M & 0x04 = 4 : Off course
                M & 0x08 = 8 : Wind Shift
              SS & 0x01 : when set, turns off heading display on 600R control.
              SS & 0x02 : always on with 400G
              SS & 0x08 : displays 'NO DATA' on 600R
              SS & 0x10 : displays 'LARGE XTE' on 600R
              SS & 0x80 : Displays 'Auto Rel' on 600R
              TT : Always 0x08 on 400G computer, always 0x05 on 150(G) computer
        """
        super(S84,self).parse(stalk_str)

        self.sen_type = 'S84'

        # Compute compass heading
        U = int(self.parts[2][0],16)
        VW = int(self.parts[3],16)
        self.heading = self._compute_compass_heading(U,VW)

        self.turning_direction = self._compute_turning_direction(U)

        #Autopilot course in degrees
        V = int(self.parts[3][0],16)
        XY = int(self.parts[4],16)
        Z = int(self.parts[5][1],16)
        self.course = decimal.Decimal((V>>2) * 90 + XY/2)

        #Autopilot mode
        self.autopilot_mode = Z

        #Alarm
        M = int(self.parts[6][1],16)
        self.alarm = M

        #Rudder position, positive value = right, negative=left
        #conversion from hex to signed int has been copied from:
        # http://stackoverflow.com/questions/385572/need-help-typecasting-in-python
        RR = int(self.parts[7],16)
        self.rudder = self._compute_rudder_position(RR)

        #SS and TT not implemented at the moment, since it doesn't make sense

class S9C(STALKSentence):
    """
    SEATALK Datagram 9C,Compass heading and Rudder position (see also command 84)

    Interpreted according to http://www.thomasknauf.de/rap/seatalk2.htm
    """

    def parse(self,stalk_str,**kwargs):
        """
        SEATALK statement mask:
            STALK,9C,U1,VW,RR
        """

        super(S9C,self).parse(stalk_str)
        self.sen_type = 'S9C'

        U = int(self.parts[2][0],16)
        VW =  int(self.parts[3],16)
        RR = int(self.parts[4],16)


        self.heading = self._compute_compass_heading(U,VW)
        self.turning_direction = self._compute_turning_direction(U)
        self.rudder = self._compute_rudder_position(RR)

class S86(STALKSentence):
    """
    Seatalk Datagram 86, Remote control for autopilot.
    """

    # we use the same values for autopilot mode as SEATALK
    _AUTOPILOT_MODES = {
        'standby': '0',
        'auto': '2'
    }

    def parse(self,stalk_str,**kwargs):
        """
        WARNING! Not all options are necessarily implemented!

        This sentence is in a prototype phase.

        SEATALK statement mask:
            86  X1  YY  yy

        X=1: Sent by Z101 remote control to increment/decrement
                          course of autopilot
             11  05  FA     -1
             11  06  F9    -10
             11  07  F8     +1
             11  08  F7    +10
             11  20  DF     +1 &  -1
             11  21  DE     -1 & -10
             11  22  DD     +1 & +10
             11  28  D7    +10 & -10
             11  45  BA     -1        pressed longer than 1 second
             11  46  B9    -10        pressed longer than 1 second
             11  47  B8     +1        pressed longer than 1 second
             11  48  B7    +10        pressed longer than 1 second
             11  60  DF     +1 &  -1  pressed longer than 1 second
             11  61  9E     -1 & -10  pressed longer than 1 second
             11  62  9D     +1 & +10  pressed longer than 1 second
             11  64  9B    +10 & -10  pressed longer than 1 second (why not 11 68 97 ?)

        Sent by autopilot (X=0: ST 1000+,  X=2: ST4000+ or ST600R)
             X1  01  FE    Auto
             X1  02  FD    Standby
             X1  03  FC    Track
             X1  04  FB    disp (in display mode or page in auto chapter = advance)
             X1  05  FA     -1 (in auto mode)
             X1  06  F9    -10 (in auto mode)
             X1  07  F8     +1 (in auto mode)
             X1  08  F7    +10 (in auto mode)
             X1  09  F6     -1 (in resp or rudder gain mode)
             X1  0A  F5     +1 (in resp or rudder gain mode)
             X1  21  DE     -1 & -10 (port tack, doesn´t work on ST600R?)
             X1  22  DD     +1 & +10 (stb tack)
             X1  23  DC    Standby & Auto (wind mode)
             X1  28  D7    +10 & -10 (in auto mode)
             X1  2E  D1     +1 & -1 (Response Display)
             X1  41  BE    Auto pressed longer
             X1  42  BD    Standby pressed longer
             X1  43  BC    Track pressed longer
             X1  44  BB    Disp pressed longer
             X1  45  BA     -1 pressed longer (in auto mode)
             X1  46  B9    -10 pressed longer (in auto mode)
             X1  47  B8     +1 pressed longer (in auto mode)
             X1  48  B7    +10 pressed longer (in auto mode)
             X1  63  9C    Standby & Auto pressed longer (previous wind angle)
             X1  68  97    +10 & -10 pressed longer (in auto mode)
             X1  6E  91     +1 & -1 pressed longer (Rudder Gain Display)
             X1  80  7F     -1 pressed (repeated 1x per second)
             X1  81  7E     +1 pressed (repeated 1x per second)
             X1  82  7D    -10 pressed (repeated 1x per second)
             X1  83  7C    +10 pressed (repeated 1x per second)
             X1  84  7B     +1, -1, +10 or -10 released

        """
        raise NotImplementedError

    def construct(self,**kwargs):
        """
        This sentence is in a prototype phase
        """
        self.talker_id = 'ST'
        self.sen_type = 'S86'

        if not hasattr(self,'parts'):
            self.parts = []

        self.parts.insert(0, 'STALK')
        self.parts.insert(1,'86')
        self.parts.insert(2,'01')

        mode = kwargs.get('operation',None)
        if not mode:
            raise NotImplementedError

        if mode == 'autopilot_mode':
            new_mode=kwargs.get('autopilot_mode',None)

            if new_mode == self._AUTOPILOT_MODES['standby']:
                self.parts.insert(3,'02')
                self.parts.insert(4,'FD')
                return
            elif new_mode == self._AUTOPILOT_MODES['auto']:
                self.parts.insert(3,'01')
                self.parts.insert(4,'FE')
                return
            else:
                raise NotImplementedError

        # Note we currently only support auto mode!
        if mode == 'course_change':
            new_course = kwargs.get('course_change')

            if str(new_course) == '-10':
                self.parts.insert(3,'06')
                self.parts.insert(4,'F9')
                return
            elif str(new_course) == '-1':
                self.parts.insert(3,'05')
                self.parts.insert(4,'FA')
                return
            elif str(new_course) == '1':
                self.parts.insert(3,'07')
                self.parts.insert(4,'F8')
                return
            elif str(new_course) == '10':
                self.parts.insert(3,'08')
                self.parts.insert(4,'F7')
                return
            else:
                raise NotImplementedError

        raise NotImplementedError

# ---------------------------------------------------------------------------- #
#   TMQ Autopilot proprietary sentences.
#   According to the TMQ documentation
#
# Warning: If you are using
# ---------------------------------------------------------------------------- #
class MQA(TMQSentence):
    """
    TMQ PTMQA Sentence datagram

    Autopilot MCU (Master Control Unit) to head
    """
    mqa_re = re.compile('^\$(..MQA),(.{11}\*[0-9a-fA-F]{2})$', re.DOTALL)

    def parse(self, tmq_str, ignore_err=False):
        """
        NMEA Sentence spec:
        $PTMQA,123M56789MA*hh
            1 - Current Mode
            2 - Current CTS (Course To Sail) Hight Byte
            3 - Current CTS (Course To Sail) Low Byte
            M - CTS Magnetic/True
            5 - Rudder Adjust. Rudder Tolerance
            6 - Rudder Adjust. Sensitivity
            7 - Rudder Angle
            8 - Current Heading (High Byte)
            9 - Current Heading (Low Byte)
            M - Heading Magnetic/True
            A - Alarm State


        An example of raw sentence: "$PTMQA,\x01\x02$M\x08\x05\x91\x02$M\x00*E8\r\n"
        """
        m = self.mqa_re.match(tmq_str)
        
        self.parts = list(m.groups())
        super(MQA,self).parse(tmq_str)

        self.sen_type = 'MQA'

        # compute autopilot mode
        data = self.parts[1]
        mode = ord(data[0])

        #Check for supported modes, we currently only support auto and standby modes
        if not mode in [1,2]:
            raise exceptions.TMQException('Unsupported AutoPilot mode!')
        elif mode == 1:
            #Standby mode
            self.autopilot_mode = self._AUTOPILOT_MODES['standby']
        elif mode == 2:
            #Auto mode
            self.autopilot_mode = self._AUTOPILOT_MODES['auto']

        # compute course
        hbyte_course = ord(data[1])
        lbyte_course = ord(data[2])
        self.course = self._compute_heading(hbyte_course, lbyte_course)
        self.course_type = data[3] # M (magnetic) or T (true)

        # rudder tolerance 0 - 10
        self.rudder_tolerance = ord(data[4])

        # rudder sensitivity 0 - 10
        self.rudder_sensitivity = ord(data[5])

        # rudder angle
        self.rudder = self._compute_rudder_position(ord(data[6]))

        # compute heading
        hbyte_heading = ord(data[7])
        lbyte_heading = ord(data[8])
        self.heading = self._compute_heading(hbyte_heading, lbyte_heading)
        self.heading_type = data[9] # M (magnetic) or T (true)

        # compute alarm
        # Alarm can have following values:
        #   1 -> GPS Error, No XTE
        #   2 -> Steering error, Angle off course
        #   4 -> Compass Error, No External Compass Input
        #   5 -> Watch Alarm Timer. Time out occurred
        #   6 -> Alarm Timer. Time out occurred
        self.alarm = ord(data[10])

class MQH(TMQSentence):

    def parse(self, tmq_str, ignore_err=False):
        raise NotImplementedError

    def construct(self, **kwargs):
        """
        This sentence is in a prototype phase
        """
        self.talker_id = 'PT'
        self.sen_type = 'MQH'

        if not hasattr(self, 'parts'):
            self.parts = []

        self.parts.insert(0, 'PTMQH')

        # compute command
        mode = kwargs.get('operation', None)
        if not mode:
            raise NotImplementedError

        if mode == 'autopilot_mode':
            new_mode=kwargs.get('autopilot_mode',None)

            if new_mode == self._AUTOPILOT_MODES['standby']:
                self.parts.insert(1,'\x01\x01\x00\x00')
                return
            elif new_mode == self._AUTOPILOT_MODES['auto']:
                self.parts.insert(1,'\x01\x02\x00\x00')
                return
            else:
                raise NotImplementedError

        if mode == 'course_change':
            new_course = kwargs.get('course_change', None)

            if new_course:
                hbyte, lbyte = self._encode_heading(new_course)
                self.parts.insert(1,'\x02'+hbyte+lbyte+'\x00')
                return
            else:
                raise NotImplementedError

        raise NotImplementedError

# ---------------------------------------------------------------------------- #
# Here are all the currently supported NMEA sentences. All should eventually be
# supported. They are being added as properties and other useful functions are
# implimented. Unit tests are also provided.
# ---------------------------------------------------------------------------- #
class AAM(NMEASentence):
    """ Waypoint Arrival Alarm
    """
    def __init__(self,**kwargs):
        parse_map = (
            ("Arrival Circle Entered", "arrival_circ_entered"),
            ("Perpendicular Passed", "perp_passed"),
            ("Circle Radius", "circle_rad"),
            ("Nautical Miles", "circle_rad_unit"),
            ("Waypoint ID", "waypoint_id"))

        super(AAM, self).__init__(parse_map,**kwargs)


class ALM(NMEASentence):
    """ GPS Almanac data
    """
    def __init__(self,**kwargs):
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

        super(ALM, self).__init__(parse_map,**kwargs)


class APA(NMEASentence):
    """ Autopilot Sentence "A"
    """

    def __init__(self,**kwargs):
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

        super(APA, self).__init__(parse_map,**kwargs)


class APB(NMEASentence):
    """ Autopilot Sentence "B"
    """

    def __init__(self,**kwargs):
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

        super(APB, self).__init__(parse_map,**kwargs)


class BEC(NMEASentence):
    """ Bearing & Distance to Waypoint, Dead Reckoning
    """
    def __init__(self,**kwargs):
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

        super(BEC, self).__init__(parse_map,**kwargs)


class BOD(NMEASentence):
    def __init__(self,**kwargs):
        # 045.,T,023.,M,DEST,START
        parse_map = (('Bearing True', 'bearing_t','decimal'),
                     ('Bearing True Type', 'bearing_t_type'),
                     ('Bearing Magnetic', 'bearing_mag','decimal'),
                     ('Bearing Magnetic Type', 'bearing_mag_type'),
                     ('Destination', 'dest'),
                     ('Start', 'start'))

        super(BOD, self).__init__(parse_map,**kwargs)

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
    def __init__(self,**kwargs):
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

        super(BWC, self).__init__(parse_map,**kwargs)


class BWR(NMEASentence):
    def __init__(self,**kwargs):
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

        super(BWR, self).__init__(parse_map,**kwargs)


class GGA(NMEASentence):
    def __init__(self,**kwargs):
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

        super(GGA, self).__init__(parse_map,**kwargs)


class BWW(NMEASentence):
    """ Bearing, Waypoint to Waypoint
    """
    def __init__(self,**kwargs):
        parse_map = (
            ("Bearing degrees True", "bearing_deg_true"),
            ("Bearing degrees True Symbol", "bearing_deg_true_sym"),
            ("Bearing degrees Magnitude", "bearing_deg_mag"),
            ("Bearing degrees Magnitude Symbol", "bearing_deg_mag_sym"),
            ("Destination Waypoint ID", "waypoint_id_dest"),
            ("Origin Waypoint ID", "waypoint_id_orig"))

        super(BWW, self).__init__(parse_map,**kwargs)


class GLL(NMEASentence):
    def __init__(self,**kwargs):
        parse_map = (
            ('Latitude', 'lat'),
            ('Latitude Direction', 'lat_dir'),
            ('Longitude', 'lon'),
            ('Longitude Direction', 'lon_dir'),
            ('Timestamp', 'timestamp'),
            ('Data Validity', "data_valid"),
            ("FAA mode indicator", "faa_mode"))

        super(GLL, self).__init__(parse_map,**kwargs)

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
    def __init__(self,**kwargs):
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

        super(GSA, self).__init__(parse_map,**kwargs)


class GSV(NMEASentence):
    def __init__(self,**kwargs):
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

        super(GSV, self).__init__(parse_map,**kwargs)


class HDG(NMEASentence):
    """ NOTE! This is a GUESS as I cannot find an actual spec
        telling me the fields. Updates are welcome!
    """
    def __init__(self,**kwargs):
        parse_map = (
            ("Heading", "heading","decimal"),
            ("Deviation", "deviation","decimal"),
            ("Deviation Direction", "dev_dir"),
            ("Variation", "variation","decimal"),
            ("Variation Direction", "var_dir"))
            #("Checksum", "checksum"))

        super(HDG, self).__init__(parse_map,**kwargs)


class HDT(NMEASentence):
    def __init__(self,**kwargs):
        parse_map = (
            ("Heading", "heading","decimal"),
            ("True", "hdg_true"))
            #("Checksum", "checksum"))

        super(HDT, self).__init__(parse_map,**kwargs)


class R00(NMEASentence):
    def __init__(self,**kwargs):
        parse_map = (
            ("Waypoint List", "waypoint_list"),)
            #("Checksum", "checksum"))

        super(R00, self).__init__(parse_map,**kwargs)

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

    @property
    def nmea_sentence(self):
        """
        Dump the object data into a NMEA sentence

        This one is over ridden since this sentence has overriden parse method
        """
        parts = [self.parts[0]] + self.parts[1]
        tmp=(',').join(parts)
        return '$' + tmp + '*' + checksum_calc(tmp)

class RMA(NMEASentence):
    def __init__(self,**kwargs):
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

        super(RMA, self).__init__(parse_map,**kwargs)


class RMB(NMEASentence):
    """ Recommended Minimum Navigation Information
    """
    def __init__(self,**kwargs):
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
        super(RMB, self).__init__(parse_map,**kwargs)


class RMC(NMEASentence):
    """ Recommended Minimum Specific GPS/TRANSIT Data
    """
    def __init__(self,**kwargs):
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
        super(RMC, self).__init__(parse_map,**kwargs)


class RTE(NMEASentence):
    """ Routes
    """
    def __init__(self,**kwargs):
        parse_map = (
            ("Number of sentences in sequence", "num_in_seq"),
            ("Sentence Number", "sen_num"),
            ("Start Type", "start_type"), # The first in the list is either current route or waypoint
            ("Name or Number of Active Route", "active_route_id"),
            ("Waypoint List", "waypoint_list"))
            #("Checksum", "checksum"))

        super(RTE, self).__init__(parse_map,**kwargs)

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

    @property
    def nmea_sentence(self):
        """
        Dump the object data into a NMEA sentence

        This one is over ridden since this sentence has overriden parse method
        """
        parts = self.parts[0:5] + self.parts[5]
        tmp=(',').join(parts)
        return '$' + tmp + '*' + checksum_calc(tmp)

class STN(NMEASentence):
    """ NOTE: No real data could be found for examples of the actual spec so
            it is a guess that there may be a checksum on the end
    """
    def __init__(self,**kwargs):
        parse_map = (
            ("Talker ID Number", "talker_id_num"),) # 00 - 99
            #("Checksum", "checksum"))


        super(STN, self).__init__(parse_map,**kwargs)


class TRF(NMEASentence):
    """ Transit Fix Data
    """
    def __init__(self,**kwargs):
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

        super(TRF, self).__init__(parse_map,**kwargs)


class VBW(NMEASentence):
    """ Dual Ground/Water Speed
    """
    def __init__(self,**kwargs):
        parse_map = (
            ("Longitudinal Water Speed", "lon_water_spd","decimal"), # Knots
            ("Transverse Water Speed", "trans_water_spd","decimal"), # Knots
            ("Water Speed Data Validity", "data_validity_water_spd"),
            ("Longitudinal Ground Speed", "lon_grnd_spd","decimal"), # Knots
            ("Transverse Ground Speed", "trans_grnd_spd","decimal"), # Knots
            ("Ground Speed Data Validity", "data_validity_grnd_spd"))
            #("Checksum", "checksum"))
        super(VBW, self).__init__(parse_map,**kwargs)


class VTG(NMEASentence):
    """ Track Made Good and Ground Speed
    """
    def __init__(self,**kwargs):
        parse_map = (
            ("True Track made good", "true_track","decimal"),
            ("True Track made good symbol", "true_track_sym"),
            ("Magnetic Track made good", "mag_track","decimal"),
            ("Magnetic Track symbol", "mag_track_sym"),
            ("Speed over ground knots", "spd_over_grnd_kts","decimal"),
            ("Speed over ground symbol", "spd_over_grnd_kts_sym"),
            ("Speed over ground kmph", "spd_over_grnd_kmph","decimal"),
            ("Speed over ground kmph symbol", "spd_over_grnd_kmph_sym"),
            ("FAA mode indicator", "faa_mode"))

        super(VTG, self).__init__(parse_map,**kwargs)


class WCV(NMEASentence):
    """ Waypoint Closure Velocity
    """
    def __init__(self,**kwargs):
        parse_map = (
            ("Velocity", "velocity"),
            ("Velocity Units", "vel_units"), # Knots
            ("Waypoint ID", "waypoint_id"))

        super(WCV, self).__init__(parse_map,**kwargs)


class WNC(NMEASentence):
    """ Distance, Waypoint to Waypoint
    """
    def __init__(self,**kwargs):
        parse_map = (
            ("Distance, Nautical Miles", "dist_nautical_miles"),
            ("Distance Nautical Miles Unit", "dist_naut_unit"),
            ("Distance, Kilometers", "dist_km"),
            ("Distance, Kilometers Unit", "dist_km_unit"),
            ("Origin Waypoint ID", "waypoint_origin_id"),
            ("Destination Waypoint ID", "waypoint_dest_id"))

        super(WNC, self).__init__(parse_map,**kwargs)


class WPL(NMEASentence):
    """ Waypoint Location
    """
    def __init__(self,**kwargs):
        parse_map = (
            ("Latitude", "lat"),
            ("Latitude Direction", "lat_dir"),
            ("Longitude", "lon"),
            ("Longitude Direction", "lon_dir"),
            ("Waypoint ID", "waypoint_id"))

        super(WPL, self).__init__(parse_map,**kwargs)


class XTE(NMEASentence):
    """ Cross-Track Error, Measured
    """
    def __init__(self,**kwargs):
        parse_map = (("General Warning Flag", "warning_flag"),
                     ("Lock flag (Not Used)", "lock_flag"),
                     ("Cross Track Error Distance", "cross_track_err_dist"),
                     ("Correction Direction (L or R)", "correction_dir"),
                     ("Distance Units", "dist_units"))

        super(XTE, self).__init__(parse_map,**kwargs)


class ZDA(NMEASentence):
    def __init__(self,**kwargs):
        parse_map = (
            ("Timestamp", "timestamp"), # hhmmss.ss = UTC
            ("Day", "day","decimal"), # 01 to 31
            ("Month", "month", "decimal"), # 01 to 12
            ("Year", "year", "decimal"), # Year = YYYY
            ("Local Zone Description", "local_zone", "decimal"), # 00 to +/- 13 hours
            ("Local Zone Minutes Description", "local_zone_minutes", "decimal")) # same sign as hours
        #("Checksum", "checksum"))

        super(ZDA, self).__init__(parse_map,**kwargs)

# Implemented by Janez Stupar for Visionect
class RSA(NMEASentence):
    """ Rudder Sensor Angle
    """
    def __init__(self,**kwargs):
        parse_map = (
            ("Starboard rudder sensor","rsa_starboard","decimal"),
            ("Starboard rudder sensor status","rsa_starboard_status"),
            ("Port rudder sensor","rsa_port","decimal"),
            ("Port rudder sensor status","rsa_port_status"),
        )
        super(RSA,self).__init__(parse_map,**kwargs)

class HSC(NMEASentence):
    """ Heading Steering Command
    """
    def __init__(self,**kwargs):
        parse_map = (
            ("Heading","heading_true","decimal"),
            ("True","true"),
            ("Heading Magnetic","heading_magnetic","decimal"),
            ("Magnetic","magnetic"),
        )
        super(HSC,self).__init__(parse_map,**kwargs)

class MWD(NMEASentence):
    """ Wind Direction
    NMEA 0183 standard Wind Direction and Speed, with respect to north.
    """
    def __init__(self,**kwargs):
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
        super(MWD,self).__init__(parse_map,**kwargs)

class MWV(NMEASentence):
    """ Wind Speed and Angle
    NMEA 0183 standard Wind Speed and Angle, in relation to the vessel's
bow/centerline.
    """
    def __init__(self,**kwargs):
        parse_map = (
            ("Wind angle","wind_angle","decimal"), #in relation to vessels centerline
            ("Reference","reference"), # relative (R)/theoretical(T)
            ("Wind speed","wind_speed","decimal"),
            ("Wind speed units","wind_speed_units"), # K/M/N
            ("Status","status"),
        )
        super(MWV,self).__init__(parse_map,**kwargs)

class DBT(NMEASentence):
    """ Depth Below Transducer
    """
    def __init__(self,**kwargs):
        parse_map = (
            ("Depth feet","depth_feet","decimal"),
            ("Feet","feet"),
            ("Depth meters","depth_meters","decimal"),
            ("Meters","meters"),
            ("Depth fathoms","depth_fathoms","decimal"),
            ("fathoms","fathoms"),
        )
        super(DBT,self).__init__(parse_map,**kwargs)

class DPT(NMEASentence):
    """ Depth of Water
    """
    def __init__(self,**kwargs):
        parse_map = (
            ("Depth meters","depth","decimal"),
            ("Offset from transducer","offset","decimal"),
            ("Maximum range on scale","max_range","decimal")
        )
        super(DPT,self).__init__(parse_map,**kwargs)

class HDM(NMEASentence):
    """
    Heading, Magnetic
    """
    def __init__(self,**kwargs):
        parse_map = (
            ("Heading degrees","heading","decimal"),
            ("Magnetic","magnetic"),
        )
        super(HDM,self).__init__(parse_map,**kwargs)

class MTW(NMEASentence):
    """ Water Temperature
    """
    def __init__(self,**kwargs):
        parse_map = (
            ('Water temperature','temperature','decimal'),
            ('Unit of measurement','units')
        )
        super(MTW,self).__init__(parse_map,**kwargs)

class VHW(NMEASentence):
    """ Water Speed and Heading
    """
    def __init__(self,**kwargs):
        parse_map = (
            ('Heading true degrees','heading_true','decimal'),
            ('heading true','true'),
            ('Heading Magnetic degrees','heading_magnetic','decimal'),
            ('Magnetic','magnetic'),
            ('Water speed knots','water_speed_knots','decimal'),
            ('Knots','knots'),
            ('Water speed kilometers','water_speed_km','decimal'),
            ('Kilometers','kilometers'),
        )
        super(VHW,self).__init__(parse_map,**kwargs)

class VLW(NMEASentence):
    """ Distance Traveled through the Water
    """
    def __init__(self,**kwargs):
        parse_map = (
            ('Water trip distance','trip_distance','decimal'),
            ('Trip distance nautical miles','trip_distance_miles'),
            ('Water trip distance since reset','trip_distance_reset','decimal'),
            ('Trip distance nautical miles since reset','trip_distance_reset_miles'),
        )
        super(VLW,self).__init__(parse_map,**kwargs)

# ---------------------------------- Not Yet Implimented --------------------- #
# ---------------------------------------------------------------------------- #



#class FSI(NMEASentence):
#    """ Frequency Set Information
#    """
#    def __init__(self,**kwargs):
#        parse_map = ()
#        super(FSI).__init__(parse_map,**kwargs)

#class GLC(NMEASentence):
#    """ Geographic Position, Loran-C
#    """
#    def __init__(self,**kwargs):
#        parse_map = ()
#        super(GLC).__init__(parse_map,**kwargs)

#class GXA(NMEASentence):
#    """ TRANSIT Position
#    """
#    def __init__(self,**kwargs):
#        parse_map = ()
#        super(GXA).__init__(parse_map,**kwargs)

#class LCD(NMEASentence):
#    """ Loran-C Signal Data
#    """
#    def __init__(self,**kwargs):
#        parse_map = ()
#        super(LCD).__init__(parse_map,**kwargs)

#class MTA(NMEASentence):
#    """ Air Temperature (to be phased out)
#    """
#    def __init__(self,**kwargs):
#        parse_map = ()
#        super(MTA).__init__(parse_map,**kwargs)



#class OLN(NMEASentence):
#    """ Omega Lane Numbers
#    """
#    def __init__(self,**kwargs):
#        parse_map = ()
#        super(OLN).__init__(parse_map,**kwargs)

#class OSD(NMEASentence):
#    """ Own Ship Data
#    """
#    def __init__(self,**kwargs):
#        parse_map = ()
#        super(OSD).__init__(parse_map,**kwargs)

#class ROT(NMEASentence):
#    """ Rate of Turn
#    """
#    def __init__(self,**kwargs):
#        parse_map = ()
#        super(ROT).__init__(parse_map,**kwargs)

#class RPM(NMEASentence):
#    """ Revolutions
#    """
#    def __init__(self,**kwargs):
#        parse_map = ()
#        super(RPM).__init__(parse_map,**kwargs)


#class RSD(NMEASentence):
#    """ RADAR System Data
#    """
#    def __init__(self,**kwargs):
#        parse_map = ()
#        super(RSD).__init__(parse_map,**kwargs)

#class SFI(NMEASentence):
#    """ Scanning Frequency Information
#    """
#    def __init__(self,**kwargs):
#        parse_map = ()
#        super(SFI).__init__(parse_map,**kwargs)

#class TTM(NMEASentence):
#    """ Tracked Target Message
#    """
#    def __init__(self,**kwargs):
#        parse_map = ()
#        super(TTM).__init__(parse_map,**kwargs)

#class VDR(NMEASentence):
#    """ Set and Drift
#    """
#    def __init__(self,**kwargs):
#        parse_map = ()
#        super(VDR).__init__(parse_map,**kwargs)


#class VPW(NMEASentence):
#    """ Speed, Measured Parallel to Wind
#    """
#    def __init__(self,**kwargs):
#        parse_map = ()
#        super(VPW).__init__(parse_map,**kwargs)

#class XDR(NMEASentence):
#    """ Transducer Measurements
#    """
#    def __init__(self,**kwargs):
#        parse_map = ()
#        super(XDR).__init__(parse_map,**kwargs)

#class XTR(NMEASentence):
#    """ Cross-Track Error, Dead Reckoning
#    """
#    def __init__(self,**kwargs):
#        parse_map = ()
#        super(XTR).__init__(parse_map,**kwargs)

#class ZFO(NMEASentence):
#    """ UTC & Time from Origin Waypoint
#    """
#    def __init__(self,**kwargs):
#        parse_map = ()
#        super(ZFO).__init__(parse_map,**kwargs)

#class ZTG(NMEASentence):
#    """ UTC & Time to Destination Waypoint
#    """
#    def __init__(self,**kwargs):
#        parse_map = ()
#        super(ZTG).__init__(parse_map,**kwargs)


# ---------------------------------------------------------------------------- #
# -------------------------- Unknown Formats --------------------------------- #
# ---------------------------------------------------------------------------- #

#class ASD(NMEASentence):
#    """ Auto-pilot system data (Unknown format)
#    """
#    def __init__(self,**kwargs):
#        parse_map = ()
#        super(ASD).__init__()

# ---------------------------------------------------------------------------- #
# -------------------------- Obsolete Formats -------------------------------- #
# ---------------------------------------------------------------------------- #

#class DCN(NMEASentence):
#    """ Decca Position (obsolete)
#    """
#    def __init__(self,**kwargs):
#        parse_map = ()
#        super(DCN).__init__(parse_map,**kwargs)


# PROPRIETRY SENTENCES

# -- GARMIN -- #
class RME(NMEASentence):
    """ GARMIN Estimated position error
    """
    def __init__(self,**kwargs):
        parse_map = (("Estimated Horiz. Position Error", "hpe"),
                     ("Estimated Horiz. Position Error Unit (M)", "hpe_unit"),
                     ("Estimated Vert. Position Error", "vpe"),
                     ("Estimated Vert. Position Error Unit (M)", "vpe_unit"),
                     ("Estimated Horiz. Position Error", "osepe"),
                     ("Overall Spherical Equiv. Position Error", "osepe_unit"))

        super(RME, self).__init__(parse_map,**kwargs)


class RMM(NMEASentence):
    """ GARMIN Map Datum
    """
    def __init__(self,**kwargs):
        parse_map = (('Currently Active Datum', 'datum'),)

        super(RMM, self).__init__(parse_map,**kwargs)


class RMZ(NMEASentence):
    """ GARMIN Altitude Information
    """
    def __init__(self,**kwargs):
        parse_map = (("Altitude", "altitude"),
                     ("Altitude Units (Feet)", "altitude_unit"),
                     ("Positional Fix Dimension (2=user, 3=GPS)",
                      "pos_fix_dim"))

        super(RMZ, self).__init__(parse_map,**kwargs)

class EMR(NMEASentence):
    """ WEMAR anchorpilot
    """

    def __init__(self,**kwargs):
        parse_map = (
            ("Sentence type", "sentence_type"),
            ("Instruction type", "instruction_type"),
            ("Chain counter units", "chain_counter_units"),
            ("Chain counter", "chain_counter",'decimal'),
            ("Battery voltage", "battery_voltage",'decimal'),
            ("Tandem diff", "tandem_diff"),
            ("Light status", "light_status"),
            ("Water status", "water_status"),
            ("EEPROM status", "eeprom_status"),
            ("Base status", "base_status"),
            ("Limit flag", "limit_flag"),
            ("Base version", "base_version"),
            ("RSSI", "RSSI"))

        super(EMR, self).__init__(parse_map,**kwargs)