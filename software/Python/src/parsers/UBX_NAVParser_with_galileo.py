"""
UBX Navigation Data Parser - Version 3
Parses u-blox UBX files to extract GPS and Galileo navigation messages
Supports: GPS L1 C/A and Galileo E1-B/E5b I/NAV
"""

import struct
import json
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
from enum import IntEnum
import logging

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class GNSSId(IntEnum):
    GPS = 0
    SBAS = 1
    GALILEO = 2
    BEIDOU = 3
    IMES = 4
    QZSS = 5
    GLONASS = 6


class GPSMessageType(IntEnum):
    EPHEMERIS_1 = 10
    EPHEMERIS_2 = 11
    CLOCK_DIFF_CORR = 13
    EPH_DIFF_CORR = 14
    CLOCK_IONO_GROUP = 30
    CLOCK_EOP = 32
    CLOCK_UTC = 33
    CLOCK_DIFF = 34


class GalileoWordType(IntEnum):
    EPHEMERIS_1 = 1
    EPHEMERIS_2 = 2
    EPHEMERIS_3 = 3
    EPHEMERIS_4 = 4
    ION_BGD_HEALTH = 5
    GST_UTC = 6


@dataclass
class GPSEphemeris:
    """GPS Ephemeris parameters"""
    a: float = 0.0
    e: float = 0.0
    i0: float = 0.0
    Omega0: float = 0.0
    omega: float = 0.0
    M0: float = 0.0
    Cis: float = 0.0
    Cic: float = 0.0
    Crs: float = 0.0
    Crc: float = 0.0
    Cus: float = 0.0
    Cuc: float = 0.0
    IDOT: float = 0.0
    Omega_dot: float = 0.0
    toe: float = 0.0
    toc: float = 0.0
    TOW: float = 0.0
    af0: float = 0.0
    af1: float = 0.0
    af2: float = 0.0
    WN: int = 0
    svID: int = 0
    utc_time: Optional[float] = None
    has_ephemeris_1: bool = False
    has_ephemeris_2: bool = False
    has_clock: bool = False


@dataclass
class GalileoEphemeris:
    """Galileo Ephemeris parameters"""
    a: float = 0.0  # Semi-major axis (computed from sqrtA)
    e: float = 0.0
    i0: float = 0.0
    Omega0: float = 0.0
    omega: float = 0.0
    M0: float = 0.0
    Cis: float = 0.0
    Cic: float = 0.0
    Crs: float = 0.0
    Crc: float = 0.0
    Cus: float = 0.0
    Cuc: float = 0.0
    i_dot: float = 0.0  # Note: Galileo uses i_dot instead of IDOT
    Omega_dot: float = 0.0
    Delta_n: float = 0.0
    toe: float = 0.0
    toc: float = 0.0
    TOW: float = 0.0
    af0: float = 0.0  # Note: Galileo uses af0 (called alpha0 in ICD)
    af1: float = 0.0  # Note: Galileo uses af1 (called alpha1 in ICD)
    af2: float = 0.0  # Note: Galileo uses af2 (called alpha2 in ICD)
    WN: int = 0
    svID: int = 0
    IOD_nav: int = 0
    utc_time: Optional[float] = None
    has_word_1: bool = False
    has_word_2: bool = False
    has_word_3: bool = False
    has_word_4: bool = False
    has_word_5: bool = False


@dataclass
class UTCParameters:
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
    """CRC-24Q calculator for GPS and Galileo navigation messages"""
    POLYNOMIAL = 0x1864CFB

    @staticmethod
    def calculate(data: bytes, num_bits: int) -> int:
        crc = 0
        for bit_idx in range(num_bits):
            byte_idx = bit_idx // 8
            bit_pos = 7 - (bit_idx % 8)
            if byte_idx >= len(data):
                break
            bit = (data[byte_idx] >> bit_pos) & 1
            crc ^= (bit << 23)
            if crc & 0x800000:
                crc ^= CRC24Q.POLYNOMIAL
        return crc & 0xFFFFFF


class UBXParser:
    SYNC_CHAR_1 = 0xB5
    SYNC_CHAR_2 = 0x62
    CLASS_RXM = 0x02
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
        while True:
            sync1 = self.file.read(1)
            if not sync1:
                break
            if sync1[0] != self.SYNC_CHAR_1:
                continue
            sync2 = self.file.read(1)
            if not sync2 or sync2[0] != self.SYNC_CHAR_2:
                continue
            header = self.file.read(4)
            if len(header) < 4:
                break
            msg_class, msg_id, length = struct.unpack('<BBH', header)
            payload = self.file.read(length)
            if len(payload) < length:
                break
            checksum = self.file.read(2)
            if len(checksum) < 2:
                break
            ck_a, ck_b = self.calculate_checksum(header + payload)
            if ck_a != checksum[0] or ck_b != checksum[1]:
                logger.warning(f"UBX checksum mismatch for class {msg_class:02X} id {msg_id:02X}")
                continue
            yield msg_class, msg_id, payload

    @staticmethod
    def calculate_checksum(data: bytes) -> Tuple[int, int]:
        ck_a = 0
        ck_b = 0
        for byte in data:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        return ck_a, ck_b


class RXMSFRBXParser:
    EXPECTED_GPS_WORDS = 10
    EXPECTED_GALILEO_WORDS = 8
    GPS_PREAMBLE = 0x8B

    @staticmethod
    def parse(payload: bytes) -> Optional[Dict[str, Any]]:
        if len(payload) < 8:
            return None
        gnssId = payload[0]
        svId = payload[1]
        sigId = payload[2]
        freqId = payload[3]
        numWords = payload[4]
        chn = payload[5]
        version = payload[6]
        reserved = payload[7]
        words = []
        for i in range(numWords):
            word_offset = 8 + i * 4
            if word_offset + 4 > len(payload):
                return None
            word = struct.unpack('<I', payload[word_offset:word_offset + 4])[0]
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
    A_REF = 26559710.0
    OMEGA_DOT_REF = -2.6e-9
    PI = 3.1415926535898

    def __init__(self, validate_crc: bool = True):
        self.ephemerides: Dict[int, Dict[float, GPSEphemeris]] = {}
        self.utc_params: Optional[UTCParameters] = None
        self.partial_ephemerides: Dict[int, GPSEphemeris] = {}
        self.validate_crc = validate_crc

    def parse_gps_words(self, words: List[int], svId: int) -> Optional[int]:
        if len(words) != RXMSFRBXParser.EXPECTED_GPS_WORDS:
            return None
        bit_stream = bytearray()
        for word in words:
            bit_stream.extend(struct.pack('>I', word))
        preamble = self.extract_bits(bit_stream, 1, 8)
        if preamble != RXMSFRBXParser.GPS_PREAMBLE:
            logger.warning(f"Invalid GPS preamble: 0x{preamble:02X}, expected 0x{RXMSFRBXParser.GPS_PREAMBLE:02X}")
            return None
        msg_type = self.extract_bits(bit_stream, 15, 6)
        tow_count = self.extract_bits(bit_stream, 21, 17)
        tow_seconds = tow_count * 6.0
        message_crc = self.extract_bits(bit_stream, 278, 24)
        calculated_crc = CRC24Q.calculate(bytes(bit_stream), 277)
        if self.validate_crc and message_crc != calculated_crc:
            logger.warning(f"CRC mismatch for GPS svID {svId} msg_type {msg_type}: "
                           f"got 0x{message_crc:06X}, expected 0x{calculated_crc:06X}")
            return None
        if msg_type == GPSMessageType.EPHEMERIS_1:
            self.parse_message_type_10(bit_stream, svId, tow_seconds)
        elif msg_type == GPSMessageType.EPHEMERIS_2:
            self.parse_message_type_11(bit_stream, svId, tow_seconds)
        elif msg_type == GPSMessageType.CLOCK_IONO_GROUP:
            self.parse_message_type_30(bit_stream, svId, tow_seconds)
        elif msg_type == GPSMessageType.CLOCK_UTC:
            self.parse_message_type_33(bit_stream, svId, tow_seconds)
        elif msg_type == GPSMessageType.CLOCK_EOP:
            self.parse_message_type_32(bit_stream, svId, tow_seconds)
        elif msg_type == GPSMessageType.CLOCK_DIFF:
            self.parse_message_type_34(bit_stream, svId, tow_seconds)
        elif msg_type in [GPSMessageType.CLOCK_DIFF_CORR, GPSMessageType.EPH_DIFF_CORR]:
            pass
        else:
            return None
        return msg_type

    def extract_bits(self, data: bytearray, start_bit: int, num_bits: int) -> int:
        value = 0
        for i in range(num_bits):
            bit_pos = start_bit + i - 1
            byte_idx = bit_pos // 8
            bit_idx = 7 - (bit_pos % 8)
            if byte_idx < len(data):
                bit = (data[byte_idx] >> bit_idx) & 1
                value = (value << 1) | bit
        return value

    def extract_signed_bits(self, data: bytearray, start_bit: int, num_bits: int) -> int:
        value = self.extract_bits(data, start_bit, num_bits)
        if value & (1 << (num_bits - 1)):
            value -= (1 << num_bits)
        return value

    def get_or_create_partial_ephemeris(self, svId: int) -> GPSEphemeris:
        if svId not in self.partial_ephemerides:
            self.partial_ephemerides[svId] = GPSEphemeris(svID=svId)
        return self.partial_ephemerides[svId]

    def finalize_ephemeris(self, svId: int):
        if svId not in self.partial_ephemerides:
            return
        eph = self.partial_ephemerides[svId]
        if eph.has_ephemeris_1 and eph.has_ephemeris_2 and eph.has_clock:
            if svId not in self.ephemerides:
                self.ephemerides[svId] = {}
            toe = eph.toe
            self.ephemerides[svId][toe] = eph
            logger.info(f"Complete GPS ephemeris for G{svId:02d} at toe={toe}, TOW={eph.TOW}")
            self.partial_ephemerides[svId] = GPSEphemeris(svID=svId)

    def parse_message_type_10(self, data: bytearray, svId: int, tow: float):
        eph = self.get_or_create_partial_ephemeris(svId)
        WN = self.extract_bits(data, 39, 13)
        toe = self.extract_bits(data, 71, 11) * 300
        Delta_A = self.extract_signed_bits(data, 82, 26) * 2 ** -9
        A_dot = self.extract_signed_bits(data, 108, 25) * 2 ** -21
        Delta_n0 = self.extract_signed_bits(data, 133, 17) * 2 ** -44
        M0 = self.extract_signed_bits(data, 173, 33) * 2 ** -32
        e = self.extract_bits(data, 206, 33) * 2 ** -34
        omega = self.extract_signed_bits(data, 239, 33) * 2 ** -32
        a = self.A_REF + Delta_A
        M0_rad = M0 * self.PI
        omega_rad = omega * self.PI
        eph.WN = WN
        eph.toe = toe
        eph.TOW = tow
        eph.a = a
        eph.e = e
        eph.M0 = M0_rad
        eph.omega = omega_rad
        eph.has_ephemeris_1 = True
        self.finalize_ephemeris(svId)

    def parse_message_type_11(self, data: bytearray, svId: int, tow: float):
        eph = self.get_or_create_partial_ephemeris(svId)
        toe = self.extract_bits(data, 39, 11) * 300
        Omega0 = self.extract_signed_bits(data, 50, 33) * 2 ** -32
        i0 = self.extract_signed_bits(data, 83, 33) * 2 ** -32
        Delta_Omega_dot = self.extract_signed_bits(data, 116, 17) * 2 ** -44
        IDOT = self.extract_signed_bits(data, 133, 15) * 2 ** -44
        Cis = self.extract_signed_bits(data, 148, 16) * 2 ** -30
        Cic = self.extract_signed_bits(data, 164, 16) * 2 ** -30
        Crs = self.extract_signed_bits(data, 180, 24) * 2 ** -8
        Crc = self.extract_signed_bits(data, 204, 24) * 2 ** -8
        Cus = self.extract_signed_bits(data, 228, 21) * 2 ** -30
        Cuc = self.extract_signed_bits(data, 249, 21) * 2 ** -30
        Omega_dot = self.OMEGA_DOT_REF + Delta_Omega_dot
        Omega0_rad = Omega0 * self.PI
        i0_rad = i0 * self.PI
        Omega_dot_rad = Omega_dot * self.PI
        IDOT_rad = IDOT * self.PI
        eph.toe = toe
        eph.TOW = tow
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
        eph = self.get_or_create_partial_ephemeris(svId)
        toc = self.extract_bits(data, 61, 11) * 300
        af0 = self.extract_signed_bits(data, 72, 26) * 2 ** -35
        af1 = self.extract_signed_bits(data, 98, 20) * 2 ** -48
        af2 = self.extract_signed_bits(data, 118, 10) * 2 ** -60
        eph.toc = toc
        eph.TOW = tow
        eph.af0 = af0
        eph.af1 = af1
        eph.af2 = af2
        eph.has_clock = True
        self.finalize_ephemeris(svId)

    def parse_message_type_32(self, data: bytearray, svId: int, tow: float):
        eph = self.get_or_create_partial_ephemeris(svId)
        toc = self.extract_bits(data, 61, 11) * 300
        af0 = self.extract_signed_bits(data, 72, 26) * 2 ** -35
        af1 = self.extract_signed_bits(data, 98, 20) * 2 ** -48
        af2 = self.extract_signed_bits(data, 118, 10) * 2 ** -60
        eph.toc = toc
        eph.TOW = tow
        eph.af0 = af0
        eph.af1 = af1
        eph.af2 = af2
        eph.has_clock = True
        self.finalize_ephemeris(svId)

    def parse_message_type_33(self, data: bytearray, svId: int, tow: float):
        eph = self.get_or_create_partial_ephemeris(svId)
        toc = self.extract_bits(data, 61, 11) * 300
        af0 = self.extract_signed_bits(data, 72, 26) * 2 ** -35
        af1 = self.extract_signed_bits(data, 98, 20) * 2 ** -48
        af2 = self.extract_signed_bits(data, 118, 10) * 2 ** -60
        A0 = self.extract_signed_bits(data, 128, 16) * 2 ** -35
        A1 = self.extract_signed_bits(data, 144, 13) * 2 ** -51
        A2 = self.extract_signed_bits(data, 157, 7) * 2 ** -68
        Delta_t_LS = self.extract_signed_bits(data, 164, 8)
        t_ot = self.extract_bits(data, 172, 16) * 2 ** 4
        WN_ot = self.extract_bits(data, 188, 13)
        WN_LSF = self.extract_bits(data, 201, 13)
        DN = self.extract_bits(data, 214, 4)
        Delta_t_LSF = self.extract_signed_bits(data, 218, 8)
        eph.toc = toc
        eph.TOW = tow
        eph.af0 = af0
        eph.af1 = af1
        eph.af2 = af2
        eph.has_clock = True
        self.utc_params = UTCParameters(
            A0=A0, A1=A1, A2=A2,
            Delta_t_LS=Delta_t_LS,
            t_ot=t_ot, WN_ot=WN_ot,
            WN_LSF=WN_LSF, DN=DN,
            Delta_t_LSF=Delta_t_LSF
        )
        self.finalize_ephemeris(svId)

    def parse_message_type_34(self, data: bytearray, svId: int, tow: float):
        eph = self.get_or_create_partial_ephemeris(svId)
        toc = self.extract_bits(data, 61, 11) * 300
        af0 = self.extract_signed_bits(data, 72, 26) * 2 ** -35
        af1 = self.extract_signed_bits(data, 98, 20) * 2 ** -48
        af2 = self.extract_signed_bits(data, 118, 10) * 2 ** -60
        eph.toc = toc
        eph.TOW = tow
        eph.af0 = af0
        eph.af1 = af1
        eph.af2 = af2
        eph.has_clock = True
        self.finalize_ephemeris(svId)

    def get_ephemerides_dict(self, save_all: bool = True) -> Dict[str, Any]:
        result = {}
        for svId, toe_dict in self.ephemerides.items():
            sat_key = f"G{svId:02d}"
            if save_all:
                result[sat_key] = {}
                for toe, eph in sorted(toe_dict.items()):
                    toe_key = f"G{svId:02d}_{int(toe)}"
                    result[sat_key][toe_key] = self.ephemeris_to_dict(eph)
            else:
                if toe_dict:
                    latest_toe = max(toe_dict.keys())
                    eph = toe_dict[latest_toe]
                    toe_key = f"G{svId:02d}_{int(latest_toe)}"
                    result[sat_key] = {toe_key: self.ephemeris_to_dict(eph)}
        return result

    def ephemeris_to_dict(self, eph: GPSEphemeris) -> Dict[str, Any]:
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
            "TOW": eph.TOW,
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
            "utc_time": eph.utc_time
        }


class GalileoNavDataParser:
    """Parser for Galileo I/NAV navigation data according to OS-SIS-ICD"""

    PI = 3.1415926535898

    def __init__(self, validate_crc: bool = True):
        self.ephemerides: Dict[int, Dict[float, GalileoEphemeris]] = {}
        self.utc_params: Optional[UTCParameters] = None
        self.partial_ephemerides: Dict[int, GalileoEphemeris] = {}
        self.validate_crc = validate_crc

    def parse_galileo_words(self, words: List[int], svId: int) -> Optional[int]:
        """
        Parse Galileo I/NAV page from 8 32-bit words

        Args:
            words: List of 8 32-bit words
            svId: Satellite vehicle ID

        Returns:
            Word type ID or None if invalid
        """
        if len(words) != RXMSFRBXParser.EXPECTED_GALILEO_WORDS:
            logger.debug(
                f"Galileo svID {svId}: Expected {RXMSFRBXParser.EXPECTED_GALILEO_WORDS} words, got {len(words)}")
            return None

        # Concatenate all 8 words into a 256-bit stream (240 bits used + 16 padding)
        bit_stream = bytearray()
        for word in words:
            bit_stream.extend(struct.pack('>I', word))

        logger.debug(f"Galileo svID {svId}: Processing 8 words, first word: 0x{words[0]:08X}")

        # Extract Even page part (120 bits): bit 1-120
        # Bit 1: Even/Odd (should be 0 for even)
        even_odd_1 = self.extract_bits(bit_stream, 1, 1)
        # Bit 2: Page Type
        page_type_1 = self.extract_bits(bit_stream, 2, 1)
        # Bits 3-114: Data (1/2) - 112 bits
        data_part1 = self.extract_bits_as_bytearray(bit_stream, 3, 112)
        # Bits 115-120: Tail (6 bits) - not used

        # Extract Odd page part (120 bits): bits 121-240
        # Bit 121: Even/Odd (should be 1 for odd)
        even_odd_2 = self.extract_bits(bit_stream, 121, 1)
        # Bit 122: Page Type
        page_type_2 = self.extract_bits(bit_stream, 122, 1)
        # Bits 123-138: Data (2/2) - 16 bits
        data_part2 = self.extract_bits_as_bytearray(bit_stream, 123, 16)
        # Bits 139-178: OSNMA (40 bits) - E1-B only
        # Bits 179-200: SAR (22 bits) - E1-B only
        # Bits 201-202: Spare (2 bits)
        # Bits 203-226: CRC (24 bits)
        crc_received = self.extract_bits(bit_stream, 203, 24)
        # Bits 227-234: SSP (8 bits)
        # Bits 235-240: Tail (6 bits) - not used

        logger.debug(
            f"Galileo svID {svId}: even/odd bits: {even_odd_1}/{even_odd_2}, page_type: {page_type_1}/{page_type_2}, CRC: 0x{crc_received:06X}")

        # Validate page structure
        if even_odd_1 != 0 or even_odd_2 != 1:
            logger.warning(f"Invalid Galileo page structure for svID {svId}: "
                           f"even/odd bits are {even_odd_1}/{even_odd_2}, expected 0/1")
            return None

        # Combine data parts to form 128-bit I/NAV word
        inav_word = bytearray()
        # Append 112 bits from data_part1
        inav_word.extend(data_part1)
        # Append 16 bits from data_part2
        inav_word.extend(data_part2)

        # Calculate CRC on the I/NAV page
        # Assume E1-B format: CRC covers Even/Odd + Page Type + Data(1/2) + Data(2/2) + OSNMA + SAR + Spare
        # Total: 1 + 1 + 112 + 1 + 1 + 16 + 40 + 22 + 2 = 196 bits
        if self.validate_crc:
            crc_data = bytearray()
            # Even/Odd bit 1
            self.append_bits_to_bytearray(crc_data, even_odd_1, 1)
            # Page Type bit 1
            self.append_bits_to_bytearray(crc_data, page_type_1, 1)
            # Data part 1 (112 bits)
            crc_data.extend(data_part1)
            # Even/Odd bit 2
            self.append_bits_to_bytearray(crc_data, even_odd_2, 1)
            # Page Type bit 2
            self.append_bits_to_bytearray(crc_data, page_type_2, 1)
            # Data part 2 (16 bits)
            crc_data.extend(data_part2)
            # Include OSNMA (40 bits)
            osnma_bits = self.extract_bits(bit_stream, 139, 40)
            self.append_bits_to_bytearray(crc_data, osnma_bits, 40)
            # Include SAR (22 bits)
            sar_bits = self.extract_bits(bit_stream, 179, 22)
            self.append_bits_to_bytearray(crc_data, sar_bits, 22)
            # Include Spare (2 bits)
            spare_bits = self.extract_bits(bit_stream, 201, 2)
            self.append_bits_to_bytearray(crc_data, spare_bits, 2)
            # Total: 196 bits
            calculated_crc = CRC24Q.calculate(bytes(crc_data), 196)

            if self.validate_crc and crc_received != calculated_crc:
                logger.warning(f"CRC mismatch for Galileo svID {svId}: "
                               f"got 0x{crc_received:06X}, expected 0x{calculated_crc:06X}")
                return None

        # Extract word type from first 6 bits of I/NAV word
        word_type = self.extract_bits_from_bytearray(inav_word, 1, 6)

        logger.debug(f"Galileo svID {svId}: Word type = {word_type}")

        # Parse based on word type
        if word_type == GalileoWordType.EPHEMERIS_1:
            logger.debug(f"Galileo svID {svId}: Parsing Word Type 1 (Ephemeris 1/4)")
            self.parse_word_type_1(inav_word, svId)
        elif word_type == GalileoWordType.EPHEMERIS_2:
            logger.debug(f"Galileo svID {svId}: Parsing Word Type 2 (Ephemeris 2/4)")
            self.parse_word_type_2(inav_word, svId)
        elif word_type == GalileoWordType.EPHEMERIS_3:
            logger.debug(f"Galileo svID {svId}: Parsing Word Type 3 (Ephemeris 3/4)")
            self.parse_word_type_3(inav_word, svId)
        elif word_type == GalileoWordType.EPHEMERIS_4:
            logger.debug(f"Galileo svID {svId}: Parsing Word Type 4 (Ephemeris 4/4)")
            self.parse_word_type_4(inav_word, svId)
        elif word_type == GalileoWordType.ION_BGD_HEALTH:
            logger.debug(f"Galileo svID {svId}: Parsing Word Type 5 (ION/BGD/Health)")
            self.parse_word_type_5(inav_word, svId)
        elif word_type == GalileoWordType.GST_UTC:
            logger.debug(f"Galileo svID {svId}: Parsing Word Type 6 (GST-UTC)")
            self.parse_word_type_6(inav_word, svId)
        else:
            # Other word types (7-10, 16) not needed for ephemeris
            logger.debug(f"Galileo svID {svId}: Skipping Word Type {word_type} (not needed for ephemeris)")
            return None

        return word_type

    def extract_bits(self, data: bytearray, start_bit: int, num_bits: int) -> int:
        """Extract bits from byte array (1-indexed, MSB first)"""
        value = 0
        for i in range(num_bits):
            bit_pos = start_bit + i - 1
            byte_idx = bit_pos // 8
            bit_idx = 7 - (bit_pos % 8)
            if byte_idx < len(data):
                bit = (data[byte_idx] >> bit_idx) & 1
                value = (value << 1) | bit
        return value

    def extract_bits_as_bytearray(self, data: bytearray, start_bit: int, num_bits: int) -> bytearray:
        """Extract bits and return as bytearray"""
        result = bytearray()
        current_byte = 0
        bits_in_byte = 0

        for i in range(num_bits):
            bit_pos = start_bit + i - 1
            byte_idx = bit_pos // 8
            bit_idx = 7 - (bit_pos % 8)

            if byte_idx < len(data):
                bit = (data[byte_idx] >> bit_idx) & 1
                current_byte = (current_byte << 1) | bit
                bits_in_byte += 1

                if bits_in_byte == 8:
                    result.append(current_byte)
                    current_byte = 0
                    bits_in_byte = 0

        # Add remaining bits if any
        if bits_in_byte > 0:
            current_byte <<= (8 - bits_in_byte)
            result.append(current_byte)

        return result

    def extract_bits_from_bytearray(self, data: bytearray, start_bit: int, num_bits: int) -> int:
        """Extract bits from bytearray (1-indexed, MSB first)"""
        value = 0
        for i in range(num_bits):
            bit_pos = start_bit + i - 1
            byte_idx = bit_pos // 8
            bit_idx = 7 - (bit_pos % 8)
            if byte_idx < len(data):
                bit = (data[byte_idx] >> bit_idx) & 1
                value = (value << 1) | bit
        return value

    def extract_signed_bits_from_bytearray(self, data: bytearray, start_bit: int, num_bits: int) -> int:
        """Extract signed integer from bytearray (two's complement)"""
        value = self.extract_bits_from_bytearray(data, start_bit, num_bits)
        if value & (1 << (num_bits - 1)):
            value -= (1 << num_bits)
        return value

    def append_bits_to_bytearray(self, byte_array: bytearray, value: int, num_bits: int):
        """Append bits to a bytearray for CRC calculation"""
        for i in range(num_bits - 1, -1, -1):
            bit = (value >> i) & 1
            byte_idx = len(byte_array) - 1 if len(byte_array) > 0 else -1

            # Check if we need to start a new byte
            if byte_idx < 0 or (len(byte_array) * 8) % 8 == 0:
                byte_array.append(0)
                byte_idx = len(byte_array) - 1

            # Determine bit position in current byte
            bits_used = (len(byte_array) * 8 - 8) % 8 if len(byte_array) > 1 else 0
            bit_pos = 7 - bits_used
            byte_array[byte_idx] |= (bit << bit_pos)

    def get_or_create_partial_ephemeris(self, svId: int) -> GalileoEphemeris:
        """Get or create partial ephemeris for a satellite"""
        if svId not in self.partial_ephemerides:
            self.partial_ephemerides[svId] = GalileoEphemeris(svID=svId)
        return self.partial_ephemerides[svId]

    def finalize_ephemeris(self, svId: int):
        """Check if ephemeris is complete and finalize it"""
        if svId not in self.partial_ephemerides:
            return

        eph = self.partial_ephemerides[svId]

        # Check if we have all required word types
        if (eph.has_word_1 and eph.has_word_2 and eph.has_word_3 and
                eph.has_word_4 and eph.has_word_5):
            # Complete ephemeris - save it
            if svId not in self.ephemerides:
                self.ephemerides[svId] = {}

            toe = eph.toe
            self.ephemerides[svId][toe] = eph
            logger.info(
                f"Complete Galileo ephemeris for E{svId:02d} at toe={toe}, TOW={eph.TOW}, IOD_nav={eph.IOD_nav}")

            # Create new partial ephemeris for next set
            self.partial_ephemerides[svId] = GalileoEphemeris(svID=svId)

    def parse_word_type_1(self, data: bytearray, svId: int):
        """Parse Word Type 1 - Ephemeris (1/4)"""
        eph = self.get_or_create_partial_ephemeris(svId)

        # Type=1 (6 bits) already extracted
        # IOD_nav (10 bits) - bits 7-16
        IOD_nav = self.extract_bits_from_bytearray(data, 7, 10)
        # toe (14 bits) - bits 17-30
        toe = self.extract_bits_from_bytearray(data, 17, 14) * 60  # Scale by 60
        # M0 (32 bits, signed) - bits 31-62
        M0 = self.extract_signed_bits_from_bytearray(data, 31, 32) * 2 ** -31
        # e (32 bits, unsigned) - bits 63-94
        e = self.extract_bits_from_bytearray(data, 63, 32) * 2 ** -33
        # sqrtA (32 bits, unsigned) - bits 95-126
        sqrtA = self.extract_bits_from_bytearray(data, 95, 32) * 2 ** -19
        # Reserved (2 bits) - bits 127-128

        # Compute semi-major axis
        a = sqrtA * sqrtA

        # Convert semi-circles to radians
        M0_rad = M0 * self.PI

        # Store in ephemeris
        eph.IOD_nav = IOD_nav
        eph.toe = toe
        eph.M0 = M0_rad
        eph.e = e
        eph.a = a
        eph.has_word_1 = True

        self.finalize_ephemeris(svId)

    def parse_word_type_2(self, data: bytearray, svId: int):
        """Parse Word Type 2 - Ephemeris (2/4)"""
        eph = self.get_or_create_partial_ephemeris(svId)

        # Type=2 (6 bits) already extracted
        # IOD_nav (10 bits) - bits 7-16
        IOD_nav = self.extract_bits_from_bytearray(data, 7, 10)
        # Omega0 (32 bits, signed) - bits 17-48
        Omega0 = self.extract_signed_bits_from_bytearray(data, 17, 32) * 2 ** -31
        # i0 (32 bits, signed) - bits 49-80
        i0 = self.extract_signed_bits_from_bytearray(data, 49, 32) * 2 ** -31
        # omega (32 bits, signed) - bits 81-112
        omega = self.extract_signed_bits_from_bytearray(data, 81, 32) * 2 ** -31
        # i_dot (14 bits, signed) - bits 113-126
        i_dot = self.extract_signed_bits_from_bytearray(data, 113, 14) * 2 ** -43
        # Reserved (2 bits) - bits 127-128

        # Convert semi-circles to radians
        Omega0_rad = Omega0 * self.PI
        i0_rad = i0 * self.PI
        omega_rad = omega * self.PI
        i_dot_rad = i_dot * self.PI

        # Store in ephemeris
        eph.IOD_nav = IOD_nav
        eph.Omega0 = Omega0_rad
        eph.i0 = i0_rad
        eph.omega = omega_rad
        eph.i_dot = i_dot_rad
        eph.has_word_2 = True

        self.finalize_ephemeris(svId)

    def parse_word_type_3(self, data: bytearray, svId: int):
        """Parse Word Type 3 - Ephemeris (3/4) and SISA"""
        eph = self.get_or_create_partial_ephemeris(svId)

        # Type=3 (6 bits) already extracted
        # IOD_nav (10 bits) - bits 7-16
        IOD_nav = self.extract_bits_from_bytearray(data, 7, 10)
        # Omega_dot (24 bits, signed) - bits 17-40
        Omega_dot = self.extract_signed_bits_from_bytearray(data, 17, 24) * 2 ** -43
        # Delta_n (16 bits, signed) - bits 41-56
        Delta_n = self.extract_signed_bits_from_bytearray(data, 41, 16) * 2 ** -43
        # Cuc (16 bits, signed) - bits 57-72
        Cuc = self.extract_signed_bits_from_bytearray(data, 57, 16) * 2 ** -29
        # Cus (16 bits, signed) - bits 73-88
        Cus = self.extract_signed_bits_from_bytearray(data, 73, 16) * 2 ** -29
        # Crc (16 bits, signed) - bits 89-104
        Crc = self.extract_signed_bits_from_bytearray(data, 89, 16) * 2 ** -5
        # Crs (16 bits, signed) - bits 105-120
        Crs = self.extract_signed_bits_from_bytearray(data, 105, 16) * 2 ** -5
        # SISA(E1,E5b) (8 bits) - bits 121-128

        # Convert semi-circles/s to radians/s
        Omega_dot_rad = Omega_dot * self.PI
        Delta_n_rad = Delta_n * self.PI

        # Store in ephemeris
        eph.IOD_nav = IOD_nav
        eph.Omega_dot = Omega_dot_rad
        eph.Delta_n = Delta_n_rad
        eph.Cuc = Cuc
        eph.Cus = Cus
        eph.Crc = Crc
        eph.Crs = Crs
        eph.has_word_3 = True

        self.finalize_ephemeris(svId)

    def parse_word_type_4(self, data: bytearray, svId: int):
        """Parse Word Type 4 - SVID, Ephemeris (4/4), and Clock Correction Parameters"""
        eph = self.get_or_create_partial_ephemeris(svId)

        # Type=4 (6 bits) already extracted
        # IOD_nav (10 bits) - bits 7-16
        IOD_nav = self.extract_bits_from_bytearray(data, 7, 10)
        # SVID (6 bits) - bits 17-22
        SVID = self.extract_bits_from_bytearray(data, 17, 6)
        # Cic (16 bits, signed) - bits 23-38
        Cic = self.extract_signed_bits_from_bytearray(data, 23, 16) * 2 ** -29
        # Cis (16 bits, signed) - bits 39-54
        Cis = self.extract_signed_bits_from_bytearray(data, 39, 16) * 2 ** -29
        # toc (14 bits) - bits 55-68
        toc = self.extract_bits_from_bytearray(data, 55, 14) * 60  # Scale by 60
        # af0 (31 bits, signed) - bits 69-99 (called alpha0 in ICD)
        af0 = self.extract_signed_bits_from_bytearray(data, 69, 31) * 2 ** -34
        # af1 (21 bits, signed) - bits 100-120 (called alpha1 in ICD)
        af1 = self.extract_signed_bits_from_bytearray(data, 100, 21) * 2 ** -46
        # af2 (6 bits, signed) - bits 121-126 (called alpha2 in ICD)
        af2 = self.extract_signed_bits_from_bytearray(data, 121, 6) * 2 ** -59
        # Spare (2 bits) - bits 127-128

        # Store in ephemeris
        eph.IOD_nav = IOD_nav
        eph.svID = SVID  # Update svID from the data
        eph.Cic = Cic
        eph.Cis = Cis
        eph.toc = toc
        eph.af0 = af0
        eph.af1 = af1
        eph.af2 = af2
        eph.has_word_4 = True

        self.finalize_ephemeris(svId)

    def parse_word_type_5(self, data: bytearray, svId: int):
        """Parse Word Type 5 - Ionospheric Correction, BGD, Signal Health, and Data Validity Status and GST"""
        eph = self.get_or_create_partial_ephemeris(svId)

        # Type=5 (6 bits) already extracted
        # ai0 (11 bits) - bits 7-17
        # ai1 (11 bits) - bits 18-28
        # ai2 (14 bits) - bits 29-42
        # Region flags (5 bits) - bits 43-47
        # BGD(E1,E5a) (10 bits) - bits 48-57
        # BGD(E1,E5b) (10 bits) - bits 58-67
        # E5b_HS (2 bits) - bits 68-69
        # E1B_HS (2 bits) - bits 70-71
        # E5b_DVS (1 bit) - bit 72
        # E1B_DVS (1 bit) - bit 73
        # WN (12 bits) - bits 74-85
        WN = self.extract_bits_from_bytearray(data, 74, 12)
        # TOW (20 bits) - bits 86-105
        TOW = self.extract_bits_from_bytearray(data, 86, 20)
        # Spare (23 bits) - bits 106-128

        # Store in ephemeris
        eph.WN = WN
        eph.TOW = float(TOW)
        eph.has_word_5 = True

        self.finalize_ephemeris(svId)

    def parse_word_type_6(self, data: bytearray, svId: int):
        """Parse Word Type 6 - GST-UTC Conversion Parameters"""
        # Type=6 (6 bits) already extracted
        # A0 (32 bits, signed) - bits 7-38
        A0 = self.extract_signed_bits_from_bytearray(data, 7, 32) * 2 ** -30
        # A1 (24 bits, signed) - bits 39-62
        A1 = self.extract_signed_bits_from_bytearray(data, 39, 24) * 2 ** -50
        # Delta_t_LS (8 bits, signed) - bits 63-70
        Delta_t_LS = self.extract_signed_bits_from_bytearray(data, 63, 8)
        # t_ot (8 bits) - bits 71-78
        t_ot = self.extract_bits_from_bytearray(data, 71, 8) * 3600  # Scale by 3600
        # WN_0t (8 bits) - bits 79-86
        WN_ot = self.extract_bits_from_bytearray(data, 79, 8)
        # WN_LSF (8 bits) - bits 87-94
        WN_LSF = self.extract_bits_from_bytearray(data, 87, 8)
        # DN (3 bits) - bits 95-97
        DN = self.extract_bits_from_bytearray(data, 95, 3)
        # Delta_t_LSF (8 bits, signed) - bits 98-105
        Delta_t_LSF = self.extract_signed_bits_from_bytearray(data, 98, 8)
        # TOW (20 bits) - bits 106-125
        TOW = self.extract_bits_from_bytearray(data, 106, 20)
        # Spare (3 bits) - bits 126-128

        # Store UTC parameters
        self.utc_params = UTCParameters(
            A0=A0, A1=A1, A2=0.0,  # Galileo doesn't have A2
            Delta_t_LS=Delta_t_LS,
            t_ot=t_ot, WN_ot=WN_ot,
            WN_LSF=WN_LSF, DN=DN,
            Delta_t_LSF=Delta_t_LSF
        )

    def get_ephemerides_dict(self, save_all: bool = True) -> Dict[str, Any]:
        """Get ephemerides as dictionary for JSON export"""
        result = {}
        for svId, toe_dict in self.ephemerides.items():
            sat_key = f"E{svId:02d}"
            if save_all:
                result[sat_key] = {}
                for toe, eph in sorted(toe_dict.items()):
                    toe_key = f"E{svId:02d}_{int(toe)}"
                    result[sat_key][toe_key] = self.ephemeris_to_dict(eph)
            else:
                if toe_dict:
                    latest_toe = max(toe_dict.keys())
                    eph = toe_dict[latest_toe]
                    toe_key = f"E{svId:02d}_{int(latest_toe)}"
                    result[sat_key] = {toe_key: self.ephemeris_to_dict(eph)}
        return result

    def ephemeris_to_dict(self, eph: GalileoEphemeris) -> Dict[str, Any]:
        """Convert Galileo ephemeris to dictionary with sorted keys"""
        return {
            "Cic": eph.Cic,
            "Cis": eph.Cis,
            "Crc": eph.Crc,
            "Crs": eph.Crs,
            "Cuc": eph.Cuc,
            "Cus": eph.Cus,
            "Delta_n": eph.Delta_n,
            "IOD_nav": eph.IOD_nav,
            "M0": eph.M0,
            "Omega0": eph.Omega0,
            "Omega_dot": eph.Omega_dot,
            "TOW": eph.TOW,
            "WN": eph.WN,
            "a": eph.a,
            "af0": eph.af0,
            "af1": eph.af1,
            "af2": eph.af2,
            "e": eph.e,
            "i0": eph.i0,
            "i_dot": eph.i_dot,
            "omega": eph.omega,
            "svID": eph.svID,
            "toc": eph.toc,
            "toe": eph.toe,
            "utc_time": eph.utc_time
        }


def parse_ubx_file(filename: str, save_all_ephemerides: bool = True,
                   json_indent: int = 2, validate_crc: bool = True) -> Dict[str, Any]:
    """
    Parse UBX file and extract GPS and Galileo navigation data
    """
    gps_parser = GPSNavDataParser(validate_crc=validate_crc)
    galileo_parser = GalileoNavDataParser(validate_crc=validate_crc)

    message_count = 0
    gps_nav_count = 0
    galileo_nav_count = 0

    logger.info(f"Parsing UBX file: {filename}")

    with UBXParser(filename) as ubx:
        for msg_class, msg_id, payload in ubx.read_messages():
            message_count += 1

            if msg_class == UBXParser.CLASS_RXM and msg_id == UBXParser.MSG_RXM_SFRBX:
                sfrbx = RXMSFRBXParser.parse(payload)

                if sfrbx is None:
                    continue

                logger.debug(
                    f"SFRBX: gnssId={sfrbx['gnssId']}, svId={sfrbx['svId']}, sigId={sfrbx['sigId']}, numWords={sfrbx['numWords']}")

                # Process GPS data
                if sfrbx['gnssId'] == GNSSId.GPS:
                    if not (1 <= sfrbx['svId'] <= 32):
                        logger.warning(f"Invalid GPS svID: {sfrbx['svId']}")
                        continue
                    msg_type = gps_parser.parse_gps_words(sfrbx['words'], sfrbx['svId'])
                    if msg_type is not None:
                        gps_nav_count += 1

                # Process Galileo data
                elif sfrbx['gnssId'] == GNSSId.GALILEO:
                    logger.info(f"Found Galileo message: svID={sfrbx['svId']}, numWords={sfrbx['numWords']}")
                    if not (1 <= sfrbx['svId'] <= 36):
                        logger.warning(f"Invalid Galileo svID: {sfrbx['svId']}")
                        continue
                    word_type = galileo_parser.parse_galileo_words(
                        sfrbx['words'], sfrbx['svId']
                    )
                    if word_type is not None:
                        galileo_nav_count += 1

    logger.info(f"Processed {message_count} UBX messages")
    logger.info(f"GPS: {gps_nav_count} navigation messages, "
                f"{len(gps_parser.ephemerides)} satellites with ephemerides")
    logger.info(f"Galileo: {galileo_nav_count} navigation messages, "
                f"{len(galileo_parser.ephemerides)} satellites with ephemerides")

    # Combine results
    result = {}

    # Add GPS ephemerides
    gps_data = gps_parser.get_ephemerides_dict(save_all=save_all_ephemerides)
    if gps_data:
        result["GPS"] = gps_data

    # Add Galileo ephemerides
    galileo_data = galileo_parser.get_ephemerides_dict(save_all=save_all_ephemerides)
    if galileo_data:
        result["Galileo"] = galileo_data

    return result


def save_to_json(data: Dict[str, Any], output_filename: str, indent: int = 2):
    """Save ephemerides to JSON file"""
    with open(output_filename, 'w') as f:
        json.dump(data, f, indent=indent)
    logger.info(f"Saved ephemerides to {output_filename}")


def main():
    """Example usage"""
    import sys

    if len(sys.argv) < 2:
        print("Usage: python UBX_NAVParser_with_galileo.py <input.ubx> [output.json]")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else "ephemerides.json"

    # Configuration
    save_all = True
    json_indent = 2
    validate_crc = False

    # Parse UBX file
    ephemerides = parse_ubx_file(input_file, save_all_ephemerides=save_all,
                                 json_indent=json_indent, validate_crc=validate_crc)

    # Save to JSON
    save_to_json(ephemerides, output_file, indent=json_indent)

    print(f"\nProcessing complete!")
    print(f"Input file: {input_file}")
    print(f"Output file: {output_file}")

    # Count satellites
    gps_count = sum(len(sats) for sats in ephemerides.get('GPS', {}).values())
    galileo_count = sum(len(sats) for sats in ephemerides.get('Galileo', {}).values())
    print(f"GPS satellites: {gps_count}")
    print(f"Galileo satellites: {galileo_count}")


if __name__ == "__main__":
    main()