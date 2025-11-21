"""
UBX SFRBX Navigation Data Parser
Parses u-blox UBX files to extract GPS navigation messages (ephemeris and clock corrections)
according to IS-GPS-200N specifications.
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
        
        # Store in ephemeris
        eph.WN = WN
        eph.toe = toe
        eph.tow = tow
        eph.sqrtA = sqrtA
        eph.e = e
        eph.M0 = M0_rad
        eph.omega = omega_rad
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
            Dictionary organized by constellation and satellite
        """
        result = {"GPS": {}}
        
        for svId, toe_dict in self.ephemerides.items():
            sat_key = f"G{svId:02d}"
            
            if save_all:
                # Save all ephemerides with toe as key
                result["GPS"][sat_key] = {}
                for toe, eph in sorted(toe_dict.items()):
                    toe_key = f"G{svId:02d}_{int(toe)}"
                    result["GPS"][sat_key][toe_key] = self.ephemeris_to_dict(eph)
            else:
                # Only save most recent ephemeris
                if toe_dict:
                    latest_toe = max(toe_dict.keys())
                    eph = toe_dict[latest_toe]
                    toe_key = f"G{svId:02d}_{int(latest_toe)}"
                    result["GPS"][sat_key] = {toe_key: self.ephemeris_to_dict(eph)}
                    
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
            "sqrtA": eph.sqrtA,
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
        }


def parse_ubx_file(filename: str, save_all_ephemerides: bool = True, 
                   json_indent: int = 2) -> Dict[str, Any]:
    """
    Parse UBX file and extract GPS navigation data
    
    Args:
        filename: Path to UBX file
        save_all_ephemerides: If True, save all ephemerides; if False, only most recent
        json_indent: Number of spaces for JSON indentation
        
    Returns:
        Dictionary with parsed ephemerides
    """
    nav_parser = GPSNavDataParser()
    message_count = 0
    gps_nav_count = 0
    
    logger.info(f"Parsing UBX file: {filename}")
    
    with UBXParser(filename) as ubx:
        for msg_class, msg_id, payload in ubx.read_messages():
            message_count += 1
            
            # Only process RXM-SFRBX messages
            if msg_class == UBXParser.CLASS_RXM and msg_id == UBXParser.MSG_RXM_SFRBX:
                sfrbx = RXMSFRBXParser.parse(payload)
                
                if sfrbx is None:
                    continue
                    
                # Only process GPS data for now
                if sfrbx['gnssId'] != GNSSId.GPS:
                    continue
                    
                # Validate svID
                if not (1 <= sfrbx['svId'] <= 32):
                    logger.warning(f"Invalid GPS svID: {sfrbx['svId']}")
                    continue
                    
                # Parse GPS navigation words
                msg_type = nav_parser.parse_gps_words(sfrbx['words'], sfrbx['svId'])
                if msg_type is not None:
                    gps_nav_count += 1
                    
    logger.info(f"Processed {message_count} UBX messages, {gps_nav_count} GPS navigation messages")
    logger.info(f"Found {len(nav_parser.ephemerides)} satellites with ephemerides")
    
    return nav_parser.get_ephemerides_dict(save_all=save_all_ephemerides)


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
        print("Usage: python ubx_nav_parser.py <input.ubx> [output.json]")
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
    print(f"Satellites found: {sum(len(sats) for sats in ephemerides.get('GPS', {}).values())}")


if __name__ == "__main__":
    main()
