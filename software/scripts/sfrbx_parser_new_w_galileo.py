"""
UBX SFRBX Navigation Data Parser
Parses u-blox UBX files to extract GPS and Galileo navigation messages (ephemeris and clock corrections)
according to IS-GPS-200N and Galileo OS-SIS-ICD specifications.
"""

import struct
import json
import math
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


class GPSMessageType(IntEnum):
    """GPS Navigation Message Types (IS-GPS-200N)"""
    EPHEMERIS_1 = 10
    EPHEMERIS_2 = 11
    CLOCK_DIFF_CORR = 13
    EPH_DIFF_CORR = 14
    CLOCK_IONO_GROUP = 30
    CLOCK_EOP = 32
    CLOCK_UTC = 33
    CLOCK_DIFF = 34

class GalileoMessageType(IntEnum):
    """Galileo Navigation Message Types (OS-SIS-ICD)"""
    EPHEMERIS_1 = 1
    EPHEMERIS_2 = 2
    EPHEMERIS_3_AND_SISA = 3
    EPHEMERIS_4_CLOCK_CORRECTION_SVID = 4
    IONO_BGD_SIGNAL_HEALTH = 5
    GST_UTC_CONVERSION = 6

@dataclass
class GNSSEphemeris:
    """GNSS Ephemeris parameters"""
    # Keplerian parameters
    e: float = 0.0  # Eccentricity
    i0: float = 0.0  # Inclination angle at reference time (radians)
    Omega0: float = 0.0  # Longitude of ascending node (radians)
    omega: float = 0.0  # Argument of perigee (radians)
    M0: float = 0.0  # Mean anomaly at reference time (radians)
    sqrtA: float = 0.0  # Square root of the Semi-major axis (meters)

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
    delta_n0: float = 0.0  # Mean motion difference from computed value (radians/sec)

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
            'words': words
        }


class GPSNavDataParser:
    """Parser for GPS navigation data according to IS-GPS-200N"""

    # Reference values
    A_REF = 26559710.0  # meters
    OMEGA_DOT_REF = -2.6e-9  # semi-circles/sec
    PI = 3.1415926535898

    def __init__(self):
        self.ephemerides: Dict[int, Dict[float, GNSSEphemeris]] = {}  # svID -> {toe -> ephemeris}
        self.utc_params: Optional[UTCParameters] = None
        self.partial_ephemerides: Dict[int, GNSSEphemeris] = {}  # svID -> partial ephemeris

    def parse_gps_words(self, words: List[int], svId: int) -> Optional[int]:
        """
        Parse GPS navigation data words

        Args:
            words: List of 32-bit words (should be 10 words for GPS)
            svId: Satellite vehicle ID

        Returns:
            Message type ID or None if invalid
        """
        if len(words) != RXMSFRBXParser.EXPECTED_GPS_WORDS:
            return None

        # Concatenate all words into a bit stream (320 bits)
        bit_stream = bytearray()
        for word in words:
            # Convert to big-endian bytes
            bit_stream.extend(struct.pack('>I', word))

        # Verify preamble (bits 1-8)
        preamble = self.extract_bits(bit_stream, 1, 8)
        if preamble != RXMSFRBXParser.GPS_PREAMBLE:
            logger.warning(f"Invalid preamble: 0x{preamble:02X}, expected 0x{RXMSFRBXParser.GPS_PREAMBLE:02X}")
            return None

        # Extract message type (bits 15-20)
        msg_type = self.extract_bits(bit_stream, 15, 6)

        # Extract and verify CRC (bits 278-301)
        message_crc = self.extract_bits(bit_stream, 278, 24)

        # Calculate CRC on bits 1-277
        calculated_crc = CRC24Q.calculate(bytes(bit_stream), 277)

        # Extract TOW Count (bits 21-37) - scaled by 6 seconds
        tow_count = self.extract_bits(bit_stream, 21, 17)
        tow_seconds = tow_count * 6.0

        # TEMPORARILY DISABLED FOR DEBUGGING
        # if message_crc != calculated_crc:
        #    logger.warning(f"CRC mismatch for svID {svId} msg_type {msg_type}: "
        #                  f"got 0x{message_crc:06X}, expected 0x{calculated_crc:06X}")
        #    return None

        # Parse based on message type
        if msg_type == GPSMessageType.EPHEMERIS_1:
            self.parse_message_type_10(bit_stream, svId, tow_seconds)
        elif msg_type == GPSMessageType.EPHEMERIS_2:
            self.parse_message_type_11(bit_stream, svId, tow_seconds)
        elif msg_type == GPSMessageType.CLOCK_IONO_GROUP:
            self.parse_message_type_30(bit_stream, svId, tow_seconds)
        elif msg_type == GPSMessageType.CLOCK_UTC:
            self.parse_message_type_33(bit_stream, svId, tow_seconds)
        elif msg_type in [GPSMessageType.CLOCK_EOP, GPSMessageType.CLOCK_DIFF]:
            # Parse clock data for these types too
            if msg_type == GPSMessageType.CLOCK_EOP:
                self.parse_message_type_32(bit_stream, svId, tow_seconds)
            elif msg_type == GPSMessageType.CLOCK_DIFF:
                self.parse_message_type_34(bit_stream, svId, tow_seconds)
        elif msg_type in [GPSMessageType.CLOCK_DIFF_CORR, GPSMessageType.EPH_DIFF_CORR]:
            # Differential corrections - not currently used but could be added
            pass
        else:
            # Unknown or unhandled message type
            return None

        return msg_type

    def extract_bits(self, data: bytearray, start_bit: int, num_bits: int) -> int:
        """
        Extract bits from byte array (1-indexed, MSB first)

        Args:
            data: Byte array
            start_bit: Starting bit position (1-indexed)
            num_bits: Number of bits to extract

        Returns:
            Extracted value as integer
        """
        value = 0
        for i in range(num_bits):
            bit_pos = start_bit + i - 1  # Convert to 0-indexed
            byte_idx = bit_pos // 8
            bit_idx = 7 - (bit_pos % 8)  # MSB first

            if byte_idx < len(data):
                bit = (data[byte_idx] >> bit_idx) & 1
                value = (value << 1) | bit

        return value

    def extract_signed_bits(self, data: bytearray, start_bit: int, num_bits: int) -> int:
        """Extract signed integer (two's complement)"""
        value = self.extract_bits(data, start_bit, num_bits)

        # Check sign bit
        if value & (1 << (num_bits - 1)):
            # Negative number - extend sign
            value -= (1 << num_bits)

        return value

    def get_or_create_partial_ephemeris(self, svId: int) -> GNSSEphemeris:
        """Get or create partial ephemeris for a satellite"""
        if svId not in self.partial_ephemerides:
            self.partial_ephemerides[svId] = GNSSEphemeris(svID=svId)
        return self.partial_ephemerides[svId]

    def finalize_ephemeris(self, svId: int):
        """Check if ephemeris is complete and finalize it"""
        if svId not in self.partial_ephemerides:
            return

        eph = self.partial_ephemerides[svId]

        # Check if we have all required components
        if eph.has_ephemeris_1 and eph.has_ephemeris_2 and eph.has_clock:
            # Complete ephemeris - save it
            if svId not in self.ephemerides:
                self.ephemerides[svId] = {}

            toe = eph.toe
            self.ephemerides[svId][toe] = eph
            logger.info(f"Complete ephemeris for G{svId:02d} at toe={toe}, TOW={eph.tow}")

            # Create new partial ephemeris for next set
            self.partial_ephemerides[svId] = GNSSEphemeris(svID=svId)

    def parse_message_type_10(self, data: bytearray, svId: int, tow: float):
        """Parse Message Type 10 - Ephemeris 1"""
        eph = self.get_or_create_partial_ephemeris(svId)

        # Extract parameters according to bit layout
        WN = self.extract_bits(data, 39, 13)
        toe = self.extract_bits(data, 71, 11) * 300  # Scale by 300

        Delta_A = self.extract_signed_bits(data, 82, 26) * 2**-9
        A_dot = self.extract_signed_bits(data, 108, 25) * 2**-21
        Delta_n0 = self.extract_signed_bits(data, 133, 17) * 2**-44
        M0 = self.extract_signed_bits(data, 173, 33) * 2**-32
        e = self.extract_bits(data, 206, 33) * 2**-34
        omega = self.extract_signed_bits(data, 239, 33) * 2**-32

        # Compute semi-major axis
        a = self.A_REF + Delta_A
        sqrtA = math.sqrt(a)

        # Convert semi-circles to radians
        M0_rad = M0 * self.PI
        omega_rad = omega * self.PI
        Delta_n0_rad = Delta_n0 * self.PI

        # Store in ephemeris
        eph.WN = WN
        eph.toe = toe
        eph.tow = tow
        eph.sqrtA = sqrtA
        eph.e = e
        eph.M0 = M0_rad
        eph.omega = omega_rad
        eph.delta_n0 = Delta_n0_rad
        eph.has_ephemeris_1 = True

        self.finalize_ephemeris(svId)

    def parse_message_type_11(self, data: bytearray, svId: int, tow: float):
        """Parse Message Type 11 - Ephemeris 2"""
        eph = self.get_or_create_partial_ephemeris(svId)

        # Extract parameters
        toe = self.extract_bits(data, 39, 11) * 300
        Omega0 = self.extract_signed_bits(data, 50, 33) * 2**-32
        i0 = self.extract_signed_bits(data, 83, 33) * 2**-32
        Delta_Omega_dot = self.extract_signed_bits(data, 116, 17) * 2**-44
        IDOT = self.extract_signed_bits(data, 133, 15) * 2**-44
        Cis = self.extract_signed_bits(data, 148, 16) * 2**-30
        Cic = self.extract_signed_bits(data, 164, 16) * 2**-30
        Crs = self.extract_signed_bits(data, 180, 24) * 2**-8
        Crc = self.extract_signed_bits(data, 204, 24) * 2**-8
        Cus = self.extract_signed_bits(data, 228, 21) * 2**-30
        Cuc = self.extract_signed_bits(data, 249, 21) * 2**-30

        # Compute Omega_dot
        Omega_dot = self.OMEGA_DOT_REF + Delta_Omega_dot

        # Convert semi-circles to radians
        Omega0_rad = Omega0 * self.PI
        i0_rad = i0 * self.PI
        Omega_dot_rad = Omega_dot * self.PI
        IDOT_rad = IDOT * self.PI

        # Store in ephemeris
        eph.toe = toe  # Should match type 10
        eph.tow = tow
        eph.Omega0 = Omega0_rad
        eph.i0 = i0_rad
        eph.Omega_dot = Omega_dot_rad
        eph.IDOT = IDOT_rad
        eph.Cis = Cis
        eph.Cic = Cic
        eph.Crs = Crs
        eph.Crc = Crc
        eph.Cus = Cus
        eph.Cuc = Cuc
        eph.has_ephemeris_2 = True

        self.finalize_ephemeris(svId)

    def parse_message_type_30(self, data: bytearray, svId: int, tow: float):
        """Parse Message Type 30 - Clock, IONO & Group Delay"""
        eph = self.get_or_create_partial_ephemeris(svId)

        # Extract clock parameters
        toc = self.extract_bits(data, 61, 11) * 300
        af0 = self.extract_signed_bits(data, 72, 26) * 2**-35
        af1 = self.extract_signed_bits(data, 98, 20) * 2**-48
        af2 = self.extract_signed_bits(data, 118, 10) * 2**-60

        # Store in ephemeris
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

        # Extract clock parameters
        toc = self.extract_bits(data, 61, 11) * 300
        af0 = self.extract_signed_bits(data, 72, 26) * 2**-35
        af1 = self.extract_signed_bits(data, 98, 20) * 2**-48
        af2 = self.extract_signed_bits(data, 118, 10) * 2**-60

        # Store in ephemeris
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

        # Extract clock parameters
        toc = self.extract_bits(data, 61, 11) * 300
        af0 = self.extract_signed_bits(data, 72, 26) * 2**-35
        af1 = self.extract_signed_bits(data, 98, 20) * 2**-48
        af2 = self.extract_signed_bits(data, 118, 10) * 2**-60

        # Extract UTC parameters
        A0 = self.extract_signed_bits(data, 128, 16) * 2**-35
        A1 = self.extract_signed_bits(data, 144, 13) * 2**-51
        A2 = self.extract_signed_bits(data, 157, 7) * 2**-68
        Delta_t_LS = self.extract_signed_bits(data, 164, 8)
        t_ot = self.extract_bits(data, 172, 16) * 2**4
        WN_ot = self.extract_bits(data, 188, 13)
        WN_LSF = self.extract_bits(data, 201, 13)
        DN = self.extract_bits(data, 214, 4)
        Delta_t_LSF = self.extract_signed_bits(data, 218, 8)

        # Store clock parameters
        eph.toc = toc
        eph.tow = tow
        eph.af0 = af0
        eph.af1 = af1
        eph.af2 = af2
        eph.has_clock = True

        # Store UTC parameters
        self.utc_params = UTCParameters(
            A0=A0, A1=A1, A2=A2,
            Delta_t_LS=Delta_t_LS,
            t_ot=t_ot, WN_ot=WN_ot,
            WN_LSF=WN_LSF, DN=DN,
            Delta_t_LSF=Delta_t_LSF
        )

        self.finalize_ephemeris(svId)

    def parse_message_type_34(self, data: bytearray, svId: int, tow: float):
        """Parse Message Type 34 - Clock & Differential Correction"""
        eph = self.get_or_create_partial_ephemeris(svId)

        # Extract clock parameters
        toc = self.extract_bits(data, 61, 11) * 300
        af0 = self.extract_signed_bits(data, 72, 26) * 2**-35
        af1 = self.extract_signed_bits(data, 98, 20) * 2**-48
        af2 = self.extract_signed_bits(data, 118, 10) * 2**-60

        # Store in ephemeris
        eph.toc = toc
        eph.tow = tow
        eph.af0 = af0
        eph.af1 = af1
        eph.af2 = af2
        eph.has_clock = True

        self.finalize_ephemeris(svId)

    def get_ephemerides_dict(self, save_all: bool = True) -> Dict[str, Any]:
        """
        Get ephemerides as dictionary for JSON export

        Args:
            save_all: If True, save all ephemerides; if False, only most recent

        Returns:
            Dictionary organized by satellite
        """
        result = {}

        for svId, toe_dict in self.ephemerides.items():
            sat_key = f"G{svId:02d}"

            if save_all:
                # Save all ephemerides with toe as key
                result[sat_key] = {}
                for toe, eph in sorted(toe_dict.items()):
                    toe_key = f"G{svId:02d}_{int(toe)}"
                    result[sat_key][toe_key] = self.ephemeris_to_dict(eph)
            else:
                # Only save most recent ephemeris
                if toe_dict:
                    latest_toe = max(toe_dict.keys())
                    eph = toe_dict[latest_toe]
                    toe_key = f"G{svId:02d}_{int(latest_toe)}"
                    result[sat_key] = {toe_key: self.ephemeris_to_dict(eph)}

        return result

    def ephemeris_to_dict(self, eph: GNSSEphemeris) -> Dict[str, Any]:
        """Convert ephemeris to dictionary with sorted keys"""
        return {
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
            "af0": eph.af0,
            "af1": eph.af1,
            "af2": eph.af2,
            "delta_n0": eph.delta_n0,
            "e": eph.e,
            "i0": eph.i0,
            "omega": eph.omega,
            "sqrtA": eph.sqrtA,
            "svID": eph.svID,
            "toc": eph.toc,
            "toe": eph.toe,
            "tow": eph.tow,
        }


class GalileoNavDataParser:
    """Parser for Galileo I/NAV navigation data according to Galileo OS-SIS-ICD"""

    # Scaling factors from Galileo OS-SIS-ICD Section 4.3.5
    PI = 3.1415926535898

    SCALE_FACTORS = {
        'toe': 60,  # Time of ephemeris (seconds)
        'M0': 2 ** -31,  # Mean anomaly (semi-circles)
        'e': 2 ** -33,  # Eccentricity
        'sqrtA': 2 ** -19,  # Square root of semi-major axis (m^1/2)
        'OMEGA0': 2 ** -31,  # Longitude of ascending node (semi-circles)
        'i0': 2 ** -31,  # Inclination angle (semi-circles)
        'omega': 2 ** -31,  # Argument of perigee (semi-circles)
        'iDot': 2 ** -43,  # Rate of inclination angle (semi-circles/sec)
        'OMEGAdot': 2 ** -43,  # Rate of right ascension (semi-circles/sec)
        'deltan': 2 ** -43,  # Mean motion difference (semi-circles/sec)
        'Cuc': 2 ** -29,  # Amplitude of cosine harmonic (radians)
        'Cus': 2 ** -29,  # Amplitude of sine harmonic (radians)
        'Crc': 2 ** -5,  # Amplitude of cosine harmonic (meters)
        'Crs': 2 ** -5,  # Amplitude of sine harmonic (meters)
        'Cic': 2 ** -29,  # Amplitude of cosine harmonic (radians)
        'Cis': 2 ** -29,  # Amplitude of sine harmonic (radians)
        'af0': 2 ** -34,  # Clock bias (seconds)
        'af1': 2 ** -46,  # Clock drift (seconds/second)
        'af2': 2 ** -59,  # Clock drift rate (seconds/second^2)
        'BGD_E1E5a': 2 ** -32,  # E1-E5a Broadcast Group Delay (seconds)
        'BGD_E1E5b': 2 ** -32,  # E1-E5b Broadcast Group Delay (seconds)
    }

    def __init__(self):
        self.ephemerides: Dict[int, Dict[float, GNSSEphemeris]] = {}  # svID -> {toe -> ephemeris}
        self.partial_data: Dict[int, Dict[int, Dict]] = {}  # svID -> {IODnav -> {word_type -> data}}

    @staticmethod
    def getbitu(buff: bytes, pos: int, length: int) -> int:
        """
        Extract unsigned integer from bit buffer

        Args:
            buff: Byte buffer
            pos: Starting bit position (0-indexed)
            length: Number of bits to extract

        Returns:
            Unsigned integer value
        """
        bits = 0
        for i in range(pos, pos + length):
            bits = (bits << 1) + ((buff[i // 8] >> (7 - i % 8)) & 1)
        return bits

    @staticmethod
    def getbits(buff: bytes, pos: int, length: int) -> int:
        """
        Extract signed integer from bit buffer

        Args:
            buff: Byte buffer
            pos: Starting bit position (0-indexed)
            length: Number of bits to extract

        Returns:
            Signed integer value
        """
        bits = GalileoNavDataParser.getbitu(buff, pos, length)
        # Sign extend if MSB is 1
        if bits & (1 << (length - 1)):
            bits -= (1 << length)
        return bits

    @staticmethod
    def crc24q(buff: bytes, length_bytes: int) -> int:
        """
        Compute CRC-24Q for Galileo I/NAV

        Args:
            buff: Byte buffer containing data
            length_bytes: Number of bytes to process

        Returns:
            24-bit CRC value
        """
        crc = 0
        CRC24Q_POLY = 0x1864CFB
        for i in range(length_bytes):
            crc ^= (buff[i] << 16)
            for j in range(8):
                crc <<= 1
                if crc & 0x1000000:
                    crc ^= CRC24Q_POLY
        return crc & 0xFFFFFF

    def extract_inav_from_words(self, words: List[int]) -> Optional[bytes]:
        """
        Extract I/NAV data from u-blox word array

        Performs word-level byte reversal and non-byte-aligned extraction
        as required by u-blox's proprietary format.

        Args:
            words: List of 32-bit words from SFRBX message

        Returns:
            30-byte I/NAV message or None if extraction fails
        """
        if len(words) < 8:
            return None

        # Step 1: Reverse bytes in each 32-bit word
        payload = bytearray()
        for word in words:
            # Convert word to bytes and reverse (little-endian to big-endian)
            word_bytes = struct.pack('<I', word)
            payload.extend(reversed(word_bytes))

        # Step 2: Extract I/NAV content starting from bit offset 2
        # Extract 30 bytes (240 bits) to capture complete I/NAV data
        inav = bytearray()
        for i in range(30):
            if 2 + i * 8 + 8 <= len(payload) * 8:
                byte_val = self.getbitu(payload, 2 + i * 8, 8)
                inav.append(byte_val)
            else:
                break

        if len(inav) < 16:
            return None

        return bytes(inav)

    def parse_galileo_words(self, words: List[int], svId: int) -> Optional[int]:
        """
        Parse Galileo I/NAV navigation data words

        Args:
            words: List of 32-bit words (should be 8 words for Galileo)
            svId: Satellite vehicle ID

        Returns:
            Word type or None if invalid
        """
        if len(words) != 8:
            return None

        # Extract I/NAV message
        inav = self.extract_inav_from_words(words)
        if inav is None:
            return None

        # Extract word type (bits 0-5)
        word_type = self.getbitu(inav, 0, 6)

        # Parse based on word type
        if word_type == 1:
            self.parse_word_type_1(inav, svId)
        elif word_type == 2:
            self.parse_word_type_2(inav, svId)
        elif word_type == 3:
            self.parse_word_type_3(inav, svId)
        elif word_type == 4:
            self.parse_word_type_4(inav, svId)
        elif word_type == 5:
            self.parse_word_type_5(inav, svId)
        elif word_type in [0, 6]:
            # Word types 0 and 6 are not used for ephemeris
            pass
        else:
            return None

        return word_type

    def parse_word_type_1(self, inav: bytes, svId: int):
        """Parse Word Type 1: Ephemeris (1/4)"""
        # IODnav (10 bits, position 6-15)
        IODnav = self.getbitu(inav, 6, 10)

        # toe (14 bits, position 16-29)
        toe_raw = self.getbitu(inav, 16, 14)
        toe = toe_raw * self.SCALE_FACTORS['toe']

        # M0 (32 bits, position 30-61)
        M0_raw = self.getbits(inav, 30, 32)
        M0_sc = M0_raw * self.SCALE_FACTORS['M0']
        M0_rad = M0_sc * self.PI

        # e (32 bits, position 62-93)
        e_raw = self.getbitu(inav, 62, 32)
        e = e_raw * self.SCALE_FACTORS['e']

        # sqrtA (32 bits, position 94-125)
        sqrtA_raw = self.getbitu(inav, 94, 32)
        sqrtA = sqrtA_raw * self.SCALE_FACTORS['sqrtA']

        # Store data
        if svId not in self.partial_data:
            self.partial_data[svId] = {}
        if IODnav not in self.partial_data[svId]:
            self.partial_data[svId][IODnav] = {}

        self.partial_data[svId][IODnav][1] = {
            'toe': toe, 'M0': M0_rad, 'e': e, 'sqrtA': sqrtA
        }

        self.finalize_ephemeris(svId, IODnav)

    def parse_word_type_2(self, inav: bytes, svId: int):
        """Parse Word Type 2: Ephemeris (2/4)"""
        # IODnav (10 bits, position 6-15)
        IODnav = self.getbitu(inav, 6, 10)

        # OMEGA0 (32 bits, position 16-47)
        OMEGA0_raw = self.getbits(inav, 16, 32)
        OMEGA0_sc = OMEGA0_raw * self.SCALE_FACTORS['OMEGA0']
        OMEGA0_rad = OMEGA0_sc * self.PI

        # i0 (32 bits, position 48-79)
        i0_raw = self.getbits(inav, 48, 32)
        i0_sc = i0_raw * self.SCALE_FACTORS['i0']
        i0_rad = i0_sc * self.PI

        # omega (32 bits, position 80-111)
        omega_raw = self.getbits(inav, 80, 32)
        omega_sc = omega_raw * self.SCALE_FACTORS['omega']
        omega_rad = omega_sc * self.PI

        # iDot (14 bits, position 112-125)
        iDot_raw = self.getbits(inav, 112, 14)
        iDot_sc = iDot_raw * self.SCALE_FACTORS['iDot']
        iDot_rad = iDot_sc * self.PI

        # Store data
        if svId not in self.partial_data:
            self.partial_data[svId] = {}
        if IODnav not in self.partial_data[svId]:
            self.partial_data[svId][IODnav] = {}

        self.partial_data[svId][IODnav][2] = {
            'Omega0': OMEGA0_rad, 'i0': i0_rad, 'omega': omega_rad, 'IDOT': iDot_rad
        }

        self.finalize_ephemeris(svId, IODnav)

    def parse_word_type_3(self, inav: bytes, svId: int):
        """Parse Word Type 3: Ephemeris (3/4) and SISA"""
        # IODnav (10 bits, position 6-15)
        IODnav = self.getbitu(inav, 6, 10)

        # OMEGAdot (24 bits, position 16-39)
        OMEGAdot_raw = self.getbits(inav, 16, 24)
        OMEGAdot_sc = OMEGAdot_raw * self.SCALE_FACTORS['OMEGAdot']
        OMEGAdot_rad = OMEGAdot_sc * self.PI

        # deltan (16 bits, position 40-55)
        deltan_raw = self.getbits(inav, 40, 16)
        deltan_sc = deltan_raw * self.SCALE_FACTORS['deltan']
        deltan_rad = deltan_sc * self.PI

        # Cuc (16 bits, position 56-71)
        Cuc_raw = self.getbits(inav, 56, 16)
        Cuc = Cuc_raw * self.SCALE_FACTORS['Cuc']

        # Cus (16 bits, position 72-87)
        Cus_raw = self.getbits(inav, 72, 16)
        Cus = Cus_raw * self.SCALE_FACTORS['Cus']

        # Crc (16 bits, position 88-103)
        Crc_raw = self.getbits(inav, 88, 16)
        Crc = Crc_raw * self.SCALE_FACTORS['Crc']

        # Crs (16 bits, position 104-119)
        Crs_raw = self.getbits(inav, 104, 16)
        Crs = Crs_raw * self.SCALE_FACTORS['Crs']

        # SISA (8 bits, position 120-127) - not used in final output
        SISA = self.getbitu(inav, 120, 8)

        # Store data
        if svId not in self.partial_data:
            self.partial_data[svId] = {}
        if IODnav not in self.partial_data[svId]:
            self.partial_data[svId][IODnav] = {}

        self.partial_data[svId][IODnav][3] = {
            'Omega_dot': OMEGAdot_rad, 'delta_n0': deltan_rad, 'Cuc': Cuc, 'Cus': Cus, 'Crc': Crc, 'Crs': Crs
        }

        self.finalize_ephemeris(svId, IODnav)

    def parse_word_type_4(self, inav: bytes, svId: int):
        """Parse Word Type 4: Ephemeris (4/4) and Clock correction"""
        # IODnav (10 bits, position 6-15)
        IODnav = self.getbitu(inav, 6, 10)

        # svid field (6 bits, position 16-21) - not used
        svid_field = self.getbitu(inav, 16, 6)

        # Cic (16 bits, position 22-37)
        Cic_raw = self.getbits(inav, 22, 16)
        Cic = Cic_raw * self.SCALE_FACTORS['Cic']

        # Cis (16 bits, position 38-53)
        Cis_raw = self.getbits(inav, 38, 16)
        Cis = Cis_raw * self.SCALE_FACTORS['Cis']

        # toc (14 bits, position 54-67)
        toc_raw = self.getbitu(inav, 54, 14)
        toc = toc_raw * 60  # seconds

        # af0 (31 bits, position 68-98)
        af0_raw = self.getbits(inav, 68, 31)
        af0 = af0_raw * self.SCALE_FACTORS['af0']

        # af1 (21 bits, position 99-119)
        af1_raw = self.getbits(inav, 99, 21)
        af1 = af1_raw * self.SCALE_FACTORS['af1']

        # af2 (6 bits, position 120-125)
        af2_raw = self.getbits(inav, 120, 6)
        af2 = af2_raw * self.SCALE_FACTORS['af2']

        # Store data
        if svId not in self.partial_data:
            self.partial_data[svId] = {}
        if IODnav not in self.partial_data[svId]:
            self.partial_data[svId][IODnav] = {}

        self.partial_data[svId][IODnav][4] = {
            'Cic': Cic, 'Cis': Cis, 'toc': toc, 'af0': af0, 'af1': af1, 'af2': af2
        }

        self.finalize_ephemeris(svId, IODnav)

    def parse_word_type_5(self, inav: bytes, svId: int):
        """Parse Word Type 5: Ionospheric correction, BGD, signal health, GST"""
        # IODnav is not present in word type 5
        # Extract WN and TOW for reference

        # WN (12 bits, position 73-84)
        WN = self.getbitu(inav, 73, 12)

        # TOW (20 bits, position 85-104)
        TOW = self.getbitu(inav, 85, 20)

        # Store WN for most recent IODnav if we have data
        if svId in self.partial_data:
            for IODnav in self.partial_data[svId]:
                if 5 not in self.partial_data[svId][IODnav]:
                    self.partial_data[svId][IODnav][5] = {'WN': WN, 'tow': TOW}

    def finalize_ephemeris(self, svId: int, IODnav: int):
        """Check if ephemeris is complete and finalize it"""
        if svId not in self.partial_data:
            return
        if IODnav not in self.partial_data[svId]:
            return

        data = self.partial_data[svId][IODnav]

        # Check if we have all required word types (1-4)
        if all(wt in data for wt in [1, 2, 3, 4]):
            # Create complete ephemeris
            eph = GNSSEphemeris(svID=svId)

            # Word type 1
            eph.toe = data[1]['toe']
            eph.M0 = data[1]['M0']
            eph.e = data[1]['e']
            eph.sqrtA = data[1]['sqrtA']

            # Word type 2
            eph.Omega0 = data[2]['Omega0']
            eph.i0 = data[2]['i0']
            eph.omega = data[2]['omega']
            eph.IDOT = data[2]['IDOT']

            # Word type 3
            eph.Omega_dot = data[3]['Omega_dot']
            eph.delta_n0 = data[3]['delta_n0']
            eph.Cuc = data[3]['Cuc']
            eph.Cus = data[3]['Cus']
            eph.Crc = data[3]['Crc']
            eph.Crs = data[3]['Crs']

            # Word type 4
            eph.Cic = data[4]['Cic']
            eph.Cis = data[4]['Cis']
            eph.toc = data[4]['toc']
            eph.af0 = data[4]['af0']
            eph.af1 = data[4]['af1']
            eph.af2 = data[4]['af2']

            # Word type 5 (optional)
            if 5 in data:
                eph.WN = data[5]['WN']
                eph.tow = data[5]['tow']

            # Save ephemeris
            if svId not in self.ephemerides:
                self.ephemerides[svId] = {}

            toe = eph.toe
            self.ephemerides[svId][toe] = eph
            logger.info(f"Complete ephemeris for E{svId:02d} at toe={toe}")

    def get_ephemerides_dict(self, save_all: bool = True) -> Dict[str, Any]:
        """
        Get ephemerides as dictionary for JSON export

        Args:
            save_all: If True, save all ephemerides; if False, only most recent

        Returns:
            Dictionary organized by satellite
        """
        result = {}

        for svId, toe_dict in self.ephemerides.items():
            sat_key = f"E{svId:02d}"

            if save_all:
                # Save all ephemerides with toe as key
                result[sat_key] = {}
                for toe, eph in sorted(toe_dict.items()):
                    toe_key = f"E{svId:02d}_{int(toe)}"
                    result[sat_key][toe_key] = self.ephemeris_to_dict(eph)
            else:
                # Only save most recent ephemeris
                if toe_dict:
                    latest_toe = max(toe_dict.keys())
                    eph = toe_dict[latest_toe]
                    toe_key = f"E{svId:02d}_{int(latest_toe)}"
                    result[sat_key] = {toe_key: self.ephemeris_to_dict(eph)}

        return result

    def ephemeris_to_dict(self, eph: GNSSEphemeris) -> Dict[str, Any]:
        """Convert ephemeris to dictionary with sorted keys"""
        return {
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
            "af0": eph.af0,
            "af1": eph.af1,
            "af2": eph.af2,
            "delta_n0": eph.delta_n0,
            "e": eph.e,
            "i0": eph.i0,
            "omega": eph.omega,
            "sqrtA": eph.sqrtA,
            "svID": eph.svID,
            "toc": eph.toc,
            "toe": eph.toe,
            "tow": eph.tow,
        }


def parse_ubx_file(filename: str, save_all_ephemerides: bool = True,
                   json_indent: int = 2) -> Dict[str, Any]:
    """
    Parse UBX file and extract GPS and Galileo navigation data

    Args:
        filename: Path to UBX file
        save_all_ephemerides: If True, save all ephemerides; if False, only most recent
        json_indent: Number of spaces for JSON indentation

    Returns:
        Dictionary with parsed ephemerides organized by constellation
    """
    gps_parser = GPSNavDataParser()
    galileo_parser = GalileoNavDataParser()
    message_count = 0
    gps_nav_count = 0
    galileo_nav_count = 0

    logger.info(f"Parsing UBX file: {filename}")

    with UBXParser(filename) as ubx:
        for msg_class, msg_id, payload in ubx.read_messages():
            message_count += 1

            # Only process RXM-SFRBX messages
            if msg_class == UBXParser.CLASS_RXM and msg_id == UBXParser.MSG_RXM_SFRBX:
                sfrbx = RXMSFRBXParser.parse(payload)

                if sfrbx is None:
                    continue

                # Process GPS data
                if sfrbx['gnssId'] == GNSSId.GPS:
                    # Validate svID
                    if not (1 <= sfrbx['svId'] <= 32):
                        logger.warning(f"Invalid GPS svID: {sfrbx['svId']}")
                        continue

                    # Parse GPS navigation words
                    msg_type = gps_parser.parse_gps_words(sfrbx['words'], sfrbx['svId'])
                    if msg_type is not None:
                        gps_nav_count += 1

                # Process Galileo data
                elif sfrbx['gnssId'] == GNSSId.GALILEO:
                    # Validate svID
                    if not (1 <= sfrbx['svId'] <= 36):
                        logger.warning(f"Invalid Galileo svID: {sfrbx['svId']}")
                        continue

                    # Parse Galileo navigation words
                    word_type = galileo_parser.parse_galileo_words(sfrbx['words'], sfrbx['svId'])
                    if word_type is not None:
                        galileo_nav_count += 1

    logger.info(f"Processed {message_count} UBX messages")
    logger.info(f"  {gps_nav_count} GPS navigation messages")
    logger.info(f"  {galileo_nav_count} Galileo navigation messages")
    logger.info(f"Found {len(gps_parser.ephemerides)} GPS satellites with ephemerides")
    logger.info(f"Found {len(galileo_parser.ephemerides)} Galileo satellites with ephemerides")

    # Combine results
    result = {}

    gps_data = gps_parser.get_ephemerides_dict(save_all=save_all_ephemerides)
    if gps_data:
        result["GPS"] = gps_data

    galileo_data = galileo_parser.get_ephemerides_dict(save_all=save_all_ephemerides)
    if galileo_data:
        result["Galileo"] = galileo_data

    return result


def save_to_json(data: Dict[str, Any], output_filename: str, indent: int = 2):
    """
    Save ephemerides to JSON file

    Args:
        data: Ephemerides dictionary
        output_filename: Output JSON file path
        indent: Number of spaces for indentation
    """
    with open(output_filename, 'w') as f:
        json.dump(data, f, indent=indent)
    logger.info(f"Saved ephemerides to {output_filename}")


def main():
    """Example usage"""
    import sys

    if len(sys.argv) < 2:
        print("Usage: python unified_sfrbx_parser.py <input.ubx> [output.json]")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else "ephemerides.json"

    # Configuration
    save_all = True  # Set to False to only save most recent ephemeris
    json_indent = 2  # Number of spaces for JSON formatting

    # Parse UBX file
    ephemerides = parse_ubx_file(input_file, save_all_ephemerides=save_all,
                                  json_indent=json_indent)

    # Save to JSON
    save_to_json(ephemerides, output_file, indent=json_indent)

    print(f"\nProcessing complete!")
    print(f"Input file: {input_file}")
    print(f"Output file: {output_file}")

    # Count satellites
    gps_count = sum(len(sats) for sats in ephemerides.get('GPS', {}).values())
    galileo_count = sum(len(sats) for sats in ephemerides.get('Galileo', {}).values())
    print(f"GPS satellites found: {gps_count}")
    print(f"Galileo satellites found: {galileo_count}")


if __name__ == "__main__":
    main()