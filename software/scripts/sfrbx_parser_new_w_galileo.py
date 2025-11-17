"""
UBX SFRBX Navigation Data Parser
Parses u-blox UBX files to extract GPS and Galileo navigation messages (ephemeris and clock corrections)
according to IS-GPS-200N and Galileo OS-SIS-ICD specifications.
"""

import struct
import json
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
from enum import IntEnum
import logging


# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class GNSSId(IntEnum):
    """GNSS constellation identifiers"""
    GPS = 0
    SBAS = 1
    GALILEO = 2
    BEIDOU = 3
    IMES = 4
    QZSS = 5
    GLONASS = 6


class MessageType(IntEnum):
    """GPS Navigation Message Types (IS-GPS-200N)"""
    EPHEMERIS_1 = 10
    EPHEMERIS_2 = 11
    CLOCK_DIFF_CORR = 13
    EPH_DIFF_CORR = 14
    CLOCK_IONO_GROUP = 30
    CLOCK_EOP = 32
    CLOCK_UTC = 33
    CLOCK_DIFF = 34


@dataclass
class GPSEphemeris:
    """GPS Ephemeris parameters"""
    # Keplerian parameters
    a: float = 0.0  # Semi-major axis (meters)
    e: float = 0.0  # Eccentricity
    i0: float = 0.0  # Inclination angle at reference time (radians)
    Omega0: float = 0.0  # Longitude of ascending node (radians)
    omega: float = 0.0  # Argument of perigee (radians)
    M0: float = 0.0  # Mean anomaly at reference time (radians)

    # Harmonic correction coefficients
    Cis: float = 0.0  # Sine harmonic correction to inclination (radians)
    Cic: float = 0.0  # Cosine harmonic correction to inclination (radians)
    Crs: float = 0.0  # Sine harmonic correction to orbit radius (meters)
    Crc: float = 0.0  # Cosine harmonic correction to orbit radius (meters)
    Cus: float = 0.0  # Sine harmonic correction to argument of latitude (radians)
    Cuc: float = 0.0  # Cosine harmonic correction to argument of latitude (radians)

    # Rates
    IDOT: float = 0.0  # Rate of inclination angle (radians/sec)
    Omega_dot: float = 0.0  # Rate of right ascension (radians/sec)

    # Reference times
    toe: float = 0.0  # Ephemeris reference time (seconds)
    toc: float = 0.0  # Clock reference time (seconds)
    tow: float = 0.0  # Time of Week count (seconds)

    # Clock correction coefficients
    af0: float = 0.0  # SV clock bias (seconds)
    af1: float = 0.0  # SV clock drift (sec/sec)
    af2: float = 0.0  # SV clock drift rate (sec/sec^2)

    # Metadata
    WN: int = 0  # Week number
    svID: int = 0  # Satellite vehicle ID
    utc_time: Optional[float] = None  # UTC time when data was collected

    # Flags to track which messages have been received
    has_ephemeris_1: bool = False
    has_ephemeris_2: bool = False
    has_clock: bool = False


@dataclass
class UTCParameters:
    """UTC correction parameters from Message Type 33"""
    A0: float = 0.0
    A1: float = 0.0
    A2: float = 0.0
    Delta_t_LS: int = 0
    t_ot: float = 0.0
    WN_ot: int = 0
    WN_LSF: int = 0
    DN: int = 0
    Delta_t_LSF: int = 0


class CRC24Q:
    """CRC-24Q calculator for GPS navigation messages"""

    # Generator polynomial: x^24 + x^23 + x^18 + x^17 + x^14 + x^11 + x^10 + x^7 + x^6 + x^5 + x^4 + x^3 + x + 1
    POLYNOMIAL = 0x1864CFB  # Binary: 1 1000 0110 0100 1100 1111 1011

    @staticmethod
    def calculate(data: bytes, num_bits: int) -> int:
        """
        Calculate CRC-24Q for the given data

        Args:
            data: Input data bytes
            num_bits: Number of bits to process

        Returns:
            24-bit CRC value
        """
        crc = 0

        for bit_idx in range(num_bits):
            byte_idx = bit_idx // 8
            bit_pos = 7 - (bit_idx % 8)

            if byte_idx >= len(data):
                break

            bit = (data[byte_idx] >> bit_pos) & 1

            # XOR bit into MSB of CRC
            crc ^= (bit << 23)

            # If MSB is 1, XOR with polynomial
            if crc & 0x800000:
                crc ^= CRC24Q.POLYNOMIAL

        return crc & 0xFFFFFF


class UBXParser:
    """Parser for u-blox UBX binary files"""

    # UBX message structure
    SYNC_CHAR_1 = 0xB5
    SYNC_CHAR_2 = 0x62

    # Message classes
    CLASS_RXM = 0x02

    # Message IDs
    MSG_RXM_SFRBX = 0x13

    def __init__(self, filename: str):
        self.filename = filename
        self.file = None

    def __enter__(self):
        self.file = open(self.filename, 'rb')
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.file:
            self.file.close()

    def read_messages(self):
        """Generator that yields UBX messages from the file"""
        while True:
            # Find sync characters
            sync1 = self.file.read(1)
            if not sync1:
                break

            if sync1[0] != self.SYNC_CHAR_1:
                continue

            sync2 = self.file.read(1)
            if not sync2 or sync2[0] != self.SYNC_CHAR_2:
                continue

            # Read header
            header = self.file.read(4)
            if len(header) < 4:
                break

            msg_class, msg_id, length = struct.unpack('<BBH', header)

            # Read payload
            payload = self.file.read(length)
            if len(payload) < length:
                break

            # Read checksum
            checksum = self.file.read(2)
            if len(checksum) < 2:
                break

            # Verify checksum
            ck_a, ck_b = self.calculate_checksum(header + payload)
            if ck_a != checksum[0] or ck_b != checksum[1]:
                logger.warning(f"UBX checksum mismatch for class {msg_class:02X} id {msg_id:02X}")
                continue

            yield msg_class, msg_id, payload

    @staticmethod
    def calculate_checksum(data: bytes) -> Tuple[int, int]:
        """Calculate UBX checksum (Fletcher algorithm)"""
        ck_a = 0
        ck_b = 0
        for byte in data:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        return ck_a, ck_b


class RXMSFRBXParser:
    """Parser for UBX-RXM-SFRBX messages"""

    EXPECTED_GPS_WORDS = 10
    GPS_PREAMBLE = 0x8B

    @staticmethod
    def parse(payload: bytes) -> Optional[Dict[str, Any]]:
        """
        Parse RXM-SFRBX payload

        Args:
            payload: Raw message payload

        Returns:
            Dictionary with parsed fields or None if invalid
        """
        if len(payload) < 8:
            return None

        # Parse header
        gnssId = payload[0]
        svId = payload[1]
        sigId = payload[2]
        freqId = payload[3]
        numWords = payload[4]
        chn = payload[5]
        version = payload[6]
        reserved = payload[7]

        # Extract data words
        words = []
        for i in range(numWords):
            word_offset = 8 + i * 4
            if word_offset + 4 > len(payload):
                return None
            word = struct.unpack('<I', payload[word_offset:word_offset+4])[0]
            words.append(word)

        return {
            'gnssId': gnssId,
            'svId': svId,
            'sigId': sigId,
            'freqId': freqId,
            'numWords': numWords,
            'chn': chn,
            'version': version,
            'words': words,
            'raw_payload': payload
        }


class GPSNavDataParser:
    """Parser for GPS navigation data"""

    def __init__(self):
        self.ephemerides = {}
        self.partial_ephemerides = {}
        self.utc_params = None

    def extract_bits(self, data: bytearray, start_bit: int, length: int) -> int:
        """Extract unsigned bits from data"""
        result = 0
        for i in range(length):
            bit_pos = start_bit + i
            byte_idx = bit_pos // 8
            bit_idx = 7 - (bit_pos % 8)
            if byte_idx < len(data):
                bit = (data[byte_idx] >> bit_idx) & 1
                result = (result << 1) | bit
        return result

    def extract_signed_bits(self, data: bytearray, start_bit: int, length: int) -> int:
        """Extract signed bits from data using two's complement"""
        unsigned = self.extract_bits(data, start_bit, length)
        if unsigned & (1 << (length - 1)):
            return unsigned - (1 << length)
        return unsigned

    def get_or_create_partial_ephemeris(self, svId: int) -> GPSEphemeris:
        """Get or create a partial ephemeris for building"""
        if svId not in self.partial_ephemerides:
            self.partial_ephemerides[svId] = GPSEphemeris(svID=svId)
        return self.partial_ephemerides[svId]

    def finalize_ephemeris(self, svId: int):
        """Store ephemeris if complete"""
        if svId not in self.partial_ephemerides:
            return

        eph = self.partial_ephemerides[svId]

        if eph.has_ephemeris_1 and eph.has_ephemeris_2 and eph.has_clock:
            if svId not in self.ephemerides:
                self.ephemerides[svId] = {}
            self.ephemerides[svId][eph.toe] = eph
            logger.info(f"Finalized ephemeris for GPS PRN {svId} at toe={eph.toe}")
            self.partial_ephemerides[svId] = GPSEphemeris(svID=svId)

    def parse_gps_words(self, words: List[int], svId: int) -> Optional[int]:
        """Parse GPS navigation words and extract ephemeris"""
        if len(words) != 10:
            return None

        data = bytearray()
        for word in words:
            gps_word = word >> 6
            for i in range(3):
                data.append((gps_word >> (16 - i * 8)) & 0xFF)

        preamble = self.extract_bits(data, 0, 8)
        if preamble != 0x8B:
            return None

        msg_type = self.extract_bits(data, 14, 6)
        tow = self.extract_bits(data, 20, 17)

        if msg_type == MessageType.EPHEMERIS_1:
            self.parse_message_type_10(data, svId, tow)
        elif msg_type == MessageType.EPHEMERIS_2:
            self.parse_message_type_11(data, svId, tow)
        elif msg_type in [MessageType.CLOCK_IONO_GROUP, MessageType.CLOCK_EOP,
                          MessageType.CLOCK_UTC, MessageType.CLOCK_DIFF]:
            if msg_type == MessageType.CLOCK_IONO_GROUP:
                self.parse_message_type_30(data, svId, tow)
            elif msg_type == MessageType.CLOCK_EOP:
                self.parse_message_type_32(data, svId, tow)
            elif msg_type == MessageType.CLOCK_UTC:
                self.parse_message_type_33(data, svId, tow)
            elif msg_type == MessageType.CLOCK_DIFF:
                self.parse_message_type_34(data, svId, tow)
        else:
            return None

        return msg_type

    def parse_message_type_10(self, data: bytearray, svId: int, tow: float):
        """Parse Message Type 10 - Ephemeris 1"""
        eph = self.get_or_create_partial_ephemeris(svId)

        WN = self.extract_bits(data, 38, 13)
        toe = self.extract_bits(data, 61, 11) * 300
        M0 = self.extract_signed_bits(data, 72, 32) * 2**-31 * 3.1415926535898
        e = self.extract_bits(data, 104, 32) * 2**-33
        sqrtA = self.extract_bits(data, 136, 32) * 2**-19

        eph.WN = WN
        eph.toe = toe
        eph.tow = tow
        eph.M0 = M0
        eph.e = e
        eph.a = sqrtA ** 2
        eph.has_ephemeris_1 = True

        self.finalize_ephemeris(svId)

    def parse_message_type_11(self, data: bytearray, svId: int, tow: float):
        """Parse Message Type 11 - Ephemeris 2"""
        eph = self.get_or_create_partial_ephemeris(svId)

        Omega0 = self.extract_signed_bits(data, 38, 32) * 2**-31 * 3.1415926535898
        i0 = self.extract_signed_bits(data, 70, 32) * 2**-31 * 3.1415926535898
        omega = self.extract_signed_bits(data, 102, 32) * 2**-31 * 3.1415926535898
        IDOT = self.extract_signed_bits(data, 134, 14) * 2**-43 * 3.1415926535898
        Cis = self.extract_signed_bits(data, 148, 16) * 2**-29
        Cic = self.extract_signed_bits(data, 164, 16) * 2**-29
        Crs = self.extract_signed_bits(data, 180, 16) * 2**-5
        Crc = self.extract_signed_bits(data, 196, 16) * 2**-5
        Cus = self.extract_signed_bits(data, 212, 16) * 2**-29
        Cuc = self.extract_signed_bits(data, 228, 16) * 2**-29
        Omega_dot = self.extract_signed_bits(data, 244, 17) * 2**-43 * 3.1415926535898

        eph.Omega0 = Omega0
        eph.i0 = i0
        eph.omega = omega
        eph.IDOT = IDOT
        eph.Cis = Cis
        eph.Cic = Cic
        eph.Crs = Crs
        eph.Crc = Crc
        eph.Cus = Cus
        eph.Cuc = Cuc
        eph.Omega_dot = Omega_dot
        eph.has_ephemeris_2 = True

        self.finalize_ephemeris(svId)

    def parse_message_type_30(self, data: bytearray, svId: int, tow: float):
        """Parse Message Type 30 - Clock & Iono"""
        eph = self.get_or_create_partial_ephemeris(svId)

        toc = self.extract_bits(data, 61, 11) * 300
        af0 = self.extract_signed_bits(data, 72, 26) * 2**-35
        af1 = self.extract_signed_bits(data, 98, 20) * 2**-48
        af2 = self.extract_signed_bits(data, 118, 10) * 2**-60

        eph.toc = toc
        eph.tow = tow
        eph.af0 = af0
        eph.af1 = af1
        eph.af2 = af2
        eph.has_clock = True

        self.finalize_ephemeris(svId)

    def parse_message_type_32(self, data: bytearray, svId: int, tow: float):
        """Parse Message Type 32 - Clock & EOP"""
        eph = self.get_or_create_partial_ephemeris(svId)

        toc = self.extract_bits(data, 61, 11) * 300
        af0 = self.extract_signed_bits(data, 72, 26) * 2**-35
        af1 = self.extract_signed_bits(data, 98, 20) * 2**-48
        af2 = self.extract_signed_bits(data, 118, 10) * 2**-60

        eph.toc = toc
        eph.tow = tow
        eph.af0 = af0
        eph.af1 = af1
        eph.af2 = af2
        eph.has_clock = True

        self.finalize_ephemeris(svId)

    def parse_message_type_33(self, data: bytearray, svId: int, tow: float):
        """Parse Message Type 33 - Clock & UTC"""
        eph = self.get_or_create_partial_ephemeris(svId)

        toc = self.extract_bits(data, 61, 11) * 300
        af0 = self.extract_signed_bits(data, 72, 26) * 2**-35
        af1 = self.extract_signed_bits(data, 98, 20) * 2**-48
        af2 = self.extract_signed_bits(data, 118, 10) * 2**-60

        eph.toc = toc
        eph.tow = tow
        eph.af0 = af0
        eph.af1 = af1
        eph.af2 = af2
        eph.has_clock = True

        self.finalize_ephemeris(svId)

    def parse_message_type_34(self, data: bytearray, svId: int, tow: float):
        """Parse Message Type 34 - Clock & Differential Correction"""
        eph = self.get_or_create_partial_ephemeris(svId)

        toc = self.extract_bits(data, 61, 11) * 300
        af0 = self.extract_signed_bits(data, 72, 26) * 2**-35
        af1 = self.extract_signed_bits(data, 98, 20) * 2**-48
        af2 = self.extract_signed_bits(data, 118, 10) * 2**-60

        eph.toc = toc
        eph.tow = tow
        eph.af0 = af0
        eph.af1 = af1
        eph.af2 = af2
        eph.has_clock = True

        self.finalize_ephemeris(svId)

    def get_ephemerides_dict(self, save_all: bool = True) -> Dict[str, Any]:
        """Get ephemerides as dictionary for JSON export"""
        result = {}

        for svId, toe_dict in self.ephemerides.items():
            sat_key = f"G{svId:02d}"
            result[sat_key] = {}

            for toe, eph in sorted(toe_dict.items()):
                toe_key = f"G{svId:02d}_{int(toe)}"
                result[sat_key][toe_key] = {
                    "Cic": eph.Cic,
                    "Cis": eph.Cis,
                    "Crc": eph.Crc,
                    "Crs": eph.Crs,
                    "Cuc": eph.Cuc,
                    "Cus": eph.Cus,
                    "IDOT": eph.IDOT,
                    "M0": eph.M0,
                    "Omega0": eph.Omega0,
                    "Omega_dot": eph.Omega_dot,
                    "WN": eph.WN,
                    "a": eph.a,
                    "af0": eph.af0,
                    "af1": eph.af1,
                    "af2": eph.af2,
                    "e": eph.e,
                    "i0": eph.i0,
                    "omega": eph.omega,
                    "svID": eph.svID,
                    "toc": eph.toc,
                    "toe": eph.toe,
                    "tow": eph.tow,
                    "utc_time": eph.utc_time
                }

            if not save_all and result[sat_key]:
                latest_toe_key = max(result[sat_key].keys())
                result[sat_key] = {latest_toe_key: result[sat_key][latest_toe_key]}

        return result


class GalileoINavParser:
    """Parser for Galileo I/NAV navigation messages"""

    SCALE_FACTORS = {
        'toe': 60,
        'M0': 2**-31,
        'e': 2**-33,
        'sqrtA': 2**-19,
        'OMEGA0': 2**-31,
        'i0': 2**-31,
        'omega': 2**-31,
        'iDot': 2**-43,
        'OMEGAdot': 2**-43,
        'deltan': 2**-43,
        'Cuc': 2**-29,
        'Cus': 2**-29,
        'Crc': 2**-5,
        'Crs': 2**-5,
        'Cic': 2**-29,
        'Cis': 2**-29,
        'af0': 2**-34,
        'af1': 2**-46,
        'af2': 2**-59,
        'BGD_E1E5a': 2**-32,
        'BGD_E1E5b': 2**-32,
    }

    def __init__(self):
        self.ephemerides = {}

    @staticmethod
    def getbitu(buff: bytes, pos: int, length: int) -> int:
        """Extract unsigned integer from bit buffer"""
        bits = 0
        for i in range(pos, pos + length):
            bits = (bits << 1) + ((buff[i//8] >> (7 - i%8)) & 1)
        return bits

    @staticmethod
    def getbits(buff: bytes, pos: int, length: int) -> int:
        """Extract signed integer from bit buffer"""
        bits = GalileoINavParser.getbitu(buff, pos, length)
        if bits & (1 << (length - 1)):
            bits -= (1 << length)
        return bits

    def extract_inav_from_sfrbx(self, msg: bytes) -> Optional[bytes]:
        """Extract I/NAV data from u-blox RXM-SFRBX message"""
        if len(msg) < 40:
            return None

        payload = bytearray()
        data_offset = 14
        num_words = msg[10]

        if num_words < 8:
            return None

        expected_len = data_offset + (num_words * 4) + 2
        if len(msg) < expected_len:
            return None

        for i in range(num_words):
            word_offset = data_offset + i * 4
            payload.extend([
                msg[word_offset + 3],
                msg[word_offset + 2],
                msg[word_offset + 1],
                msg[word_offset + 0]
            ])

        inav = bytearray()
        for i in range(30):
            if 2 + i*8 + 8 <= len(payload) * 8:
                byte_val = self.getbitu(payload, 2 + i*8, 8)
                inav.append(byte_val)
            else:
                break

        if len(inav) < 16:
            return None

        return bytes(inav)

    def parse_inav_message(self, inav: bytes, svid: int) -> Optional[Dict]:
        """Parse I/NAV message and extract parameters based on word type"""
        word_type = self.getbitu(inav, 0, 6)

        if word_type == 1:
            return self.parse_word_type_1(inav, svid)
        elif word_type == 2:
            return self.parse_word_type_2(inav, svid)
        elif word_type == 3:
            return self.parse_word_type_3(inav, svid)
        elif word_type == 4:
            return self.parse_word_type_4(inav, svid)
        elif word_type == 5:
            return self.parse_word_type_5(inav, svid)
        else:
            return None

    def parse_word_type_1(self, inav: bytes, svid: int) -> Dict:
        """Parse Word Type 1: Ephemeris (1/4)"""
        data = {'word_type': 1, 'svid': svid}
        data['IODnav'] = self.getbitu(inav, 6, 10)
        data['toe'] = self.getbitu(inav, 16, 14) * self.SCALE_FACTORS['toe']
        data['M0'] = self.getbits(inav, 30, 32) * self.SCALE_FACTORS['M0']
        data['e'] = self.getbitu(inav, 62, 32) * self.SCALE_FACTORS['e']
        data['sqrtA'] = self.getbitu(inav, 94, 32) * self.SCALE_FACTORS['sqrtA']
        return data

    def parse_word_type_2(self, inav: bytes, svid: int) -> Dict:
        """Parse Word Type 2: Ephemeris (2/4)"""
        data = {'word_type': 2, 'svid': svid}
        data['IODnav'] = self.getbitu(inav, 6, 10)
        data['OMEGA0'] = self.getbits(inav, 16, 32) * self.SCALE_FACTORS['OMEGA0']
        data['i0'] = self.getbits(inav, 48, 32) * self.SCALE_FACTORS['i0']
        data['omega'] = self.getbits(inav, 80, 32) * self.SCALE_FACTORS['omega']
        data['iDot'] = self.getbits(inav, 112, 14) * self.SCALE_FACTORS['iDot']
        return data

    def parse_word_type_3(self, inav: bytes, svid: int) -> Dict:
        """Parse Word Type 3: Ephemeris (3/4) and SISA"""
        data = {'word_type': 3, 'svid': svid}
        data['IODnav'] = self.getbitu(inav, 6, 10)
        data['OMEGAdot'] = self.getbits(inav, 16, 24) * self.SCALE_FACTORS['OMEGAdot']
        data['deltan'] = self.getbits(inav, 40, 16) * self.SCALE_FACTORS['deltan']
        data['Cuc'] = self.getbits(inav, 56, 16) * self.SCALE_FACTORS['Cuc']
        data['Cus'] = self.getbits(inav, 72, 16) * self.SCALE_FACTORS['Cus']
        data['Crc'] = self.getbits(inav, 88, 16) * self.SCALE_FACTORS['Crc']
        data['Crs'] = self.getbits(inav, 104, 16) * self.SCALE_FACTORS['Crs']
        data['SISA'] = self.getbitu(inav, 120, 8)
        return data

    def parse_word_type_4(self, inav: bytes, svid: int) -> Dict:
        """Parse Word Type 4: Ephemeris (4/4) and Clock correction"""
        data = {'word_type': 4, 'svid': svid}
        data['IODnav'] = self.getbitu(inav, 6, 10)
        data['Cic'] = self.getbits(inav, 22, 16) * self.SCALE_FACTORS['Cic']
        data['Cis'] = self.getbits(inav, 38, 16) * self.SCALE_FACTORS['Cis']
        data['toc'] = self.getbitu(inav, 54, 14) * 60
        data['af0'] = self.getbits(inav, 68, 31) * self.SCALE_FACTORS['af0']
        data['af1'] = self.getbits(inav, 99, 21) * self.SCALE_FACTORS['af1']
        data['af2'] = self.getbits(inav, 120, 6) * self.SCALE_FACTORS['af2']
        return data

    def parse_word_type_5(self, inav: bytes, svid: int) -> Dict:
        """Parse Word Type 5: Ionospheric correction, BGD, signal health, GST"""
        data = {'word_type': 5, 'svid': svid}
        data['BGD_E1E5a'] = self.getbits(inav, 47, 10) * self.SCALE_FACTORS['BGD_E1E5a']
        data['BGD_E1E5b'] = self.getbits(inav, 57, 10) * self.SCALE_FACTORS['BGD_E1E5b']
        data['E5b_HS'] = self.getbitu(inav, 67, 2)
        data['E1B_HS'] = self.getbitu(inav, 69, 2)
        data['WN'] = self.getbitu(inav, 73, 12)
        data['TOW'] = self.getbitu(inav, 85, 20)
        return data

    def assemble_ephemeris(self, word_data: List[Dict]) -> Optional[Dict]:
        """Assemble complete ephemeris from multiple word types"""
        by_iodnav = {}
        for data in word_data:
            if 'IODnav' in data:
                iodnav = data['IODnav']
                if iodnav not in by_iodnav:
                    by_iodnav[iodnav] = {}
                by_iodnav[iodnav][data['word_type']] = data

        for iodnav in sorted(by_iodnav.keys(), reverse=True):
            words = by_iodnav[iodnav]
            if all(wt in words for wt in [1, 2, 3, 4]):
                eph = {
                    'constellation': 'Galileo',
                    'svid': words[1]['svid'],
                    'IODnav': iodnav,
                    'toe': words[1]['toe'],
                    'toc': words[4]['toc'],
                    'M0': words[1]['M0'],
                    'e': words[1]['e'],
                    'sqrtA': words[1]['sqrtA'],
                    'OMEGA0': words[2]['OMEGA0'],
                    'i0': words[2]['i0'],
                    'omega': words[2]['omega'],
                    'OMEGAdot': words[3]['OMEGAdot'],
                    'iDot': words[2]['iDot'],
                    'deltan': words[3]['deltan'],
                    'Cuc': words[3]['Cuc'],
                    'Cus': words[3]['Cus'],
                    'Crc': words[3]['Crc'],
                    'Crs': words[3]['Crs'],
                    'Cic': words[4]['Cic'],
                    'Cis': words[4]['Cis'],
                    'af0': words[4]['af0'],
                    'af1': words[4]['af1'],
                    'af2': words[4]['af2'],
                    'SISA': words[3]['SISA'],
                }

                if 5 in words:
                    eph.update({
                        'BGD_E1E5a': words[5]['BGD_E1E5a'],
                        'BGD_E1E5b': words[5]['BGD_E1E5b'],
                        'E5b_HS': words[5]['E5b_HS'],
                        'E1B_HS': words[5]['E1B_HS'],
                        'WN': words[5]['WN'],
                        'TOW': words[5]['TOW'],
                    })

                return eph

        return None

    def get_ephemerides_dict(self, save_all: bool = True) -> Dict[str, Any]:
        """Get ephemerides as dictionary for JSON export"""
        result = {}

        for svid, word_list in self.ephemerides.items():
            sat_key = f"E{svid:02d}"

            # Assemble complete ephemeris from word types
            eph = self.assemble_ephemeris(word_list)
            if eph:
                if sat_key not in result:
                    result[sat_key] = {}
                toe_key = f"E{svid:02d}_{int(eph['toe'])}"
                result[sat_key][toe_key] = eph

        return result


def parse_ubx_file(filename: str, save_all_ephemerides: bool = True) -> Dict[str, Any]:
    """Parse UBX file and extract GPS and Galileo navigation data"""
    gps_parser = GPSNavDataParser()
    galileo_parser = GalileoINavParser()

    gps_count = 0
    galileo_count = 0

    logger.info(f"Parsing UBX file: {filename}")

    with UBXParser(filename) as ubx:
        for msg_class, msg_id, payload in ubx.read_messages():
            if msg_class == UBXParser.CLASS_RXM and msg_id == UBXParser.MSG_RXM_SFRBX:
                sfrbx = RXMSFRBXParser.parse(payload)

                if sfrbx is None:
                    continue

                if sfrbx['gnssId'] == GNSSId.GPS:
                    if 1 <= sfrbx['svId'] <= 32:
                        msg_type = gps_parser.parse_gps_words(sfrbx['words'], sfrbx['svId'])
                        if msg_type is not None:
                            gps_count += 1

                elif sfrbx['gnssId'] == GNSSId.GALILEO and sfrbx['numWords'] == 8:
                    svId = sfrbx['svId']

                    # Reconstruct full message for Galileo extraction
                    msg = bytearray([0xB5, 0x62, 0x02, 0x13])
                    msg.extend(struct.pack('<H', len(payload)))
                    msg.extend(payload)
                    msg.extend([0, 0])  # Dummy checksum

                    inav = galileo_parser.extract_inav_from_sfrbx(bytes(msg))

                    if inav:
                        word_data = galileo_parser.parse_inav_message(inav, svId)

                        if word_data:
                            if svId not in galileo_parser.ephemerides:
                                galileo_parser.ephemerides[svId] = []
                            galileo_parser.ephemerides[svId].append(word_data)
                            galileo_count += 1

    logger.info(f"GPS: {gps_count} messages, {len(gps_parser.ephemerides)} satellites")
    logger.info(f"Galileo: {galileo_count} messages, {len(galileo_parser.ephemerides)} satellites")

    result = {}

    gps_data = gps_parser.get_ephemerides_dict(save_all=save_all_ephemerides)
    if gps_data:
        result['GPS'] = gps_data

    galileo_data = galileo_parser.get_ephemerides_dict(save_all=save_all_ephemerides)
    if galileo_data:
        result['Galileo'] = galileo_data

    return result


def main():
    """Main entry point"""
    import sys

    if len(sys.argv) < 2:
        print("Usage: python sfrbx_parser.py <input.ubx> [output.json]")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else "ephemerides.json"

    ephemerides = parse_ubx_file(input_file, save_all_ephemerides=True)

    with open(output_file, 'w') as f:
        json.dump(ephemerides, f, indent=2)

    logger.info(f"Saved ephemerides to {output_file}")

    gps_count = sum(len(sats) for sats in ephemerides.get('GPS', {}).values())
    galileo_count = sum(len(sats) for sats in ephemerides.get('Galileo', {}).values())

    print(f"\nProcessing complete!")
    print(f"Input: {input_file}")
    print(f"Output: {output_file}")
    print(f"GPS satellites: {gps_count}")
    print(f"Galileo satellites: {galileo_count}")


if __name__ == "__main__":
    main()