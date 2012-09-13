""" For dealing with streams of nmea data
"""
from pynmea.exceptions import NoDataGivenError, ChecksumException
import re


class NMEAStream(object):
    """ NMEAStream object is used to
    """
    def __init__(self, stream_obj=None, deserialize = False):
        """ stream_obj should be a file like object.
            If the requirement is just to split data in memory, no stream_obj
            is required. Simply create an instance of this class and
            call _split directly with the data.
        """
        self.stream = stream_obj
        self.head = ''
        self.deserialize = deserialize

    def get_strings(self, data=None, size=1024):
        """ Read and return sentences as strings
        """
        return self._read(data=data, size=size)

    def get_objects(self, data=None, size=1024):
        """ Get sentences but return list of NMEA objects
        """
        str_data = self._read(data=data, size=size)
        nmea_objects = []
        for nmea_str in str_data:
            try:
                nmea_ob = self._get_type(nmea_str)(deserialize=self.deserialize)
            except TypeError:
                # NMEA sentence was not recognised
                continue
            try:
                nmea_ob.parse(nmea_str)
                nmea_objects.append(nmea_ob)
            except ChecksumException:
                #NMEA object was invalid, do not add it to the object list
                pass

        return nmea_objects


    def _read(self, data=None, size=1024):
        """ read size bytes of data. always strip off the last record and
            append to the start of the data stream on the next call.
            this ensures that only full sentences are returned.
        """
        if not data and not self.stream and not self.head:
            # If there's no data and no stream, raise an error
            raise NoDataGivenError('No data was provided')

        if not data and self.stream:
            read_data = self.stream.read(size)
        else:
            read_data = data

        data = self.head + read_data
        raw_sentences = self._split(data)
        if not read_data:
            self.head = ''
            return raw_sentences
        try:
            self.head = raw_sentences[-1]
        except IndexError:
            pass
        full_sentences = raw_sentences[:-1]
        return full_sentences

    def _get_type(self, sentence):
        """ Get the NMEA type and return the appropriate object. Returns
            None if no such object was found.

            TODO: raise error instead of None. Failing silently is a Bad Thing.
            We can always catch the error later if the user wishes to supress
            errors.
        """

        #modified tu support Gadgetpool.de SEATALK/NMEA Usb bridge
        sentence = sentence.split(',')
        sen_type = sentence[0].lstrip('$')
        if sen_type == 'STALK':
            sen_type = 'S'+sentence[1]
        else:
            sen_type = sen_type[-3:]
        sen_mod = __import__('pynmea.nmea', fromlist=[sen_type])
        sen_obj = getattr(sen_mod, sen_type, None)
        return sen_obj

    def _split(self, data, separator=None):
        """ Take some data and split up based on the notion that a sentence
            looks something like:
            $x,y,z or $x,y,z*ab

            separator is for cases where there is something strange or
            non-standard as a separator between sentences.
            Without this, there is no real way to tell whether:
            $x,y,zSTUFF
            is legal or if STUFF should be stripped.
        """
        sentences = data.split('$')
        clean_sentences = []
        for item in sentences:
            cleaned_item = item.rstrip()
            if separator:
                cleaned_item = cleaned_item.rstrip(separator)
            if '*' in cleaned_item.split(',')[-1]:
                # There must be a checksum. Remove any trailing fluff:
                try:
                    first, checksum = cleaned_item.split('*')
                except ValueError:
                    # Some GPS data recorders have been shown to output
                    # run-together sentences (no leading $).
                    # In this case, ignore error and continue, discarding the
                    # erroneous data.
                    # TODO: try and fix the data.
                    continue
                cleaned_item = '*'.join([first, checksum[:2]])
            if cleaned_item:
                clean_sentences.append(cleaned_item)

        return clean_sentences

class TMQStream(NMEAStream):
    """
    This class is used to process "NMEA" data sent by TMQ devices.

    This privilege is bestowed upon TMQ, since they are a bunch of fuktards who can't
    get right as simple a task as implementing a simple serial protocol. Even.

    There are some major problems with TMQ data, which is sent in binary format. Such as stated
    below.

    The specification of NMEA protocol encoding: http://catb.org/gpsd/NMEA.html#_nmea_encoding_conventions

    TMQ devices commit some major crimes such as:

        # Start delimiter as data
        '$PTMQA,\x01\x02$M\x08\x05a\x02$M\x00*18\r\n'
        compass angle: 137

        # Newline as data
        '$PTMQA,\x01\x04\n'
        compass angle: 258

        # End delimiter (*) as data
        '$PTMQA,\x01\x02*M\x08\x05\\\x02*M\x00*25\r\n'
        compass angle: 138

    The strategy used to solve this mess is following:

        1. Treat statement name ('PTMQA') as delimiter.
        2. Treat statements as fixed length strings.
        3. Assume that TMQ statements always provide \r\n at the end of statement
    """
    def _get_type(self, sentence):
        """ Get the NMEA type and return the appropriate object. Returns
            None if no such object was found.

            TODO: raise error instead of None. Failing silently is a Bad Thing.
            We can always catch the error later if the user wishes to supress
            errors.
        """

        #modified tu support Gadgetpool.de SEATALK/NMEA Usb bridge
        sentence = sentence.split(',')
        sen_type = sentence[0].lstrip('$')
        if not sen_type[:-1] == 'PTMQ':
            raise TypeError('Not a TMQ sentence')
        sen_type = sen_type[-3:]
        sen_mod = __import__('pynmea.nmea', fromlist=[sen_type])
        sen_obj = getattr(sen_mod, sen_type, None)
        return sen_obj

    def _split(self, data, separator=None):
        """
        Rip apart TMQ sentences.

         We split them using TMQ's prefix "$PTMQ" as statement delimiter.
         Everything else is dropped. This is necessary as TMQ doesn't conform to the
         NMEA standard in regards of data encoding.

        Separator parameter is ignored.
        """
#        sentences = data.split('$PTMQ')
        sentences = re.split('\$(?=PTMQ)', data)
        clean_sentences = []
        for item in sentences:
            # 'clean' the item. Contrast this with NMEAStream's _split method. We cannot use rstrip, because
            # in TMQ format newline (\n) may be a part of valid data, which we don't want to remove.
            # Thus we might pass some technically incorrect data through (e.g. an ordinary NMEA statement),
            # because we have no way of discerning a partial TMQ statement from a valid NMEA statement
            # e.g.: "$GPHDT,100.2,T\n" cannot be differentiated from a part of TMQ statement that would contain
            # something like "$\x01\n" -> consider a hipothetical statement "$PTMQD,\x01$\x02\n\x04*7D"
            parts = item.split('\r\n')
            cleaned_item = parts[0]
            # Check for checksum, but just if item ends with \r\n and there is a * character as
            # [-3] of the cleaned item.
            # This is necessary since this library needs to support streaming operation and with TMQ
            # it is quite possible to have a partial statement with a * character as data point somewhere
            # where a checksum could be expected in a standards compiant NMEA statement.
            if len(parts) == 2 and  '*' == cleaned_item[-3]:
                # There must be a checksum. Remove any trailing fluff:
                try:
                    first = cleaned_item[:-3]
                    checksum = cleaned_item[-2:]
                except ValueError:
                    # Some GPS data recorders have been shown to output
                    # run-together sentences (no leading $).
                    # In this case, ignore error and continue, discarding the
                    # erroneous data.
                    continue
                cleaned_item = '*'.join([first, checksum[:2]])
            if cleaned_item:
                clean_sentences.append(cleaned_item)

        return clean_sentences

