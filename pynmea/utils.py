""" Functions that get used by multiple classes go in here
"""
import decimal
import sys

def checksum_calc(nmea_str):
    """ Loop through all of the given characters and xor the current to the
        previous (cumulatively).
    """
    chksum_val = 0
    nmea_str = nmea_str.replace('$', '')
    nmea_str = nmea_str.rsplit('*', 1)[0]
    for next_char in nmea_str:
        chksum_val ^= ord(next_char)

    return "%02X" % chksum_val

class NMEADeserializer(object):
    """
    Take a string and deserialize it into an appropriate type
    """
    def deserialize(self,data,type):
        method = getattr(self,'deserialize_'+type)
        if not method:
            raise Exception('Requested serializer not implemented')
        else:
            return method(data)


    def deserialize_decimal(self,data):
        try:
            if not data:
                return decimal.Decimal('NaN')
            return decimal.Decimal(data)
        except Exception:
            return decimal.Decimal('NaN')
