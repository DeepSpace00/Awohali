import struct
import json
from typing import Dict, Optional, List, Tuple
from dataclasses import dataclass, field
from datetime import datetime, timedelta


@dataclass
class GPSEphemeris:
    """GPS Ephemeris parameters according to IS-GPS-200N"""
    # Time parameters
    toe: Optional[float] = None  # Time of Ephemeris
    toc: Optional[float] = None  # Clock correction time
    week: Optional[int] = None  # GPS week number
    time_utc: Optional[str] = None  # UTC time string (ISO 8601 format)

    # Clock correction parameters
    af0: Optional[float] = None  # Clock bias
    af1: Optional[float] = None  # Clock drift
    af2: Optional[float] = None  # Clock drift rate

    # Orbit parameters
    M0: Optional[float] = None  # Mean anomaly at reference time
    delta_n: Optional[float] = None  # Mean motion difference
    e: Optional[float] = None  # Eccentricity
    sqrt_A: Optional[float] = None  # Square root of semi-major axis

    # Perturbations
    Cic: Optional[float] = None  # Amplitude of cosine harmonic correction to angle of inclination
    Cis: Optional[float] = None  # Amplitude of sine harmonic correction to angle of inclination
    Crc: Optional[float] = None  # Amplitude of cosine harmonic correction to orbit radius
    Crs: Optional[float] = None  # Amplitude of sine harmonic correction to orbit radius
    Cuc: Optional[float] = None  # Amplitude of cosine harmonic correction to argument of latitude
    Cus: Optional[float] = None  # Amplitude of sine harmonic correction to argument of latitude

    # Orbital elements
    omega0: Optional[float] = None  # Longitude of ascending node
    omega: Optional[float] = None  # Argument of perigee
    i0: Optional[float] = None  # Inclination angle at reference time
    OMEGA_DOT: Optional[float] = None  # Rate of right ascension
    IDOT: Optional[float] = None  # Rate of inclination angle

    # Additional parameters
    ura: Optional[float] = None  # User Range Accuracy
    health: Optional[int] = None
    tgd: Optional[float] = None  # Group delay
    iodc: Optional[int] = None  # Issue of Data, Clock
    iode: Optional[int] = None  # Issue of Data, Ephemeris

    # Track subframe collection
    subframes: Dict[int, List[int]] = field(default_factory=dict)

    def is_complete(self) -> bool:
        """Check if we have all three subframes needed for ephemeris"""
        return len(self.subframes) >= 3 and all(i in self.subframes for i in [1, 2, 3])

    def to_dict(self) -> Dict:
        """Convert to dictionary, excluding internal tracking fields"""
        result = {}
        for key, value in self.__dict__.items():
            if key != 'subframes' and value is not None:
                result[key] = value
        # Return sorted dictionary
        return dict(sorted(result.items()))


@dataclass
class GalileoEphemeris:
    """Galileo Ephemeris parameters according to Galileo OS-SIS-ICD"""
    # Time parameters
    toe: Optional[float] = None
    toc: Optional[float] = None
    week: Optional[int] = None
    time_utc: Optional[str] = None  # UTC time string (ISO 8601 format)

    # Clock correction parameters
    af0: Optional[float] = None
    af1: Optional[float] = None
    af2: Optional[float] = None

    # Orbit parameters
    M0: Optional[float] = None
    delta_n: Optional[float] = None
    e: Optional[float] = None
    sqrt_A: Optional[float] = None

    # Perturbations
    Cic: Optional[float] = None
    Cis: Optional[float] = None
    Crc: Optional[float] = None
    Crs: Optional[float] = None
    Cuc: Optional[float] = None
    Cus: Optional[float] = None

    # Orbital elements
    omega0: Optional[float] = None
    omega: Optional[float] = None
    i0: Optional[float] = None
    OMEGA_DOT: Optional[float] = None
    IDOT: Optional[float] = None

    # Additional Galileo-specific parameters
    sisa: Optional[float] = None  # Signal In Space Accuracy
    health_e5a: Optional[int] = None
    health_e5b: Optional[int] = None
    bgd_e5a: Optional[float] = None  # Broadcast Group Delay E5a
    bgd_e5b: Optional[float] = None  # Broadcast Group Delay E5b
    iodnav: Optional[int] = None  # Issue of Data of Navigation

    # Track word type collection
    word_types: Dict[int, List[int]] = field(default_factory=dict)

    def is_complete(self) -> bool:
        """Check if we have required word types for ephemeris"""
        # Need at least word types 1, 2, 3, 4 for complete ephemeris
        required = {1, 2, 3, 4}
        return required.issubset(self.word_types.keys())

    def to_dict(self) -> Dict:
        """Convert to dictionary, excluding internal tracking fields"""
        result = {}
        for key, value in self.__dict__.items():
            if key != 'word_types' and value is not None:
                result[key] = value
        # Return sorted dictionary
        return dict(sorted(result.items()))


class UBXParser:
    """Parser for UBX format files"""

    UBX_SYNC1 = 0xB5
    UBX_SYNC2 = 0x62
    UBX_RXM_SFRBX = (0x02, 0x13)  # Class and ID for RXM-SFRBX
    UBX_RXM_RAWX = (0x02, 0x15)  # Class and ID for RXM-RAWX
    UBX_NAV_TIMEUTC = (0x01, 0x21)  # Class and ID for NAV-TIMEUTC

    # GPS epoch: January 6, 1980 00:00:00 UTC
    GPS_EPOCH = datetime(1980, 1, 6, 0, 0, 0)

    def __init__(self):
        self.gps_ephemeris: Dict[int, GPSEphemeris] = {}
        self.galileo_ephemeris: Dict[int, GalileoEphemeris] = {}
        self.latest_utc_time: Optional[datetime] = None
        self.latest_rcv_tow: Optional[float] = None
        self.latest_week: Optional[int] = None

    def parse_file(self, filepath: str) -> Tuple[Dict, Dict]:
        """Parse UBX file and return ephemeris data"""
        with open(filepath, 'rb') as f:
            data = f.read()

        idx = 0
        while idx < len(data) - 8:
            # Look for UBX sync bytes
            if data[idx] == self.UBX_SYNC1 and data[idx + 1] == self.UBX_SYNC2:
                msg_class = data[idx + 2]
                msg_id = data[idx + 3]
                length = struct.unpack('<H', data[idx + 4:idx + 6])[0]

                # Check if we have enough data
                if idx + 8 + length > len(data):
                    break

                payload = data[idx + 6:idx + 6 + length]

                # Process RXM-SFRBX messages
                if (msg_class, msg_id) == self.UBX_RXM_SFRBX:
                    self._process_sfrbx(payload)
                # Process RXM-RAWX messages for receiver time
                elif (msg_class, msg_id) == self.UBX_RXM_RAWX:
                    self._process_rawx(payload)
                # Process NAV-TIMEUTC messages for UTC time
                elif (msg_class, msg_id) == self.UBX_NAV_TIMEUTC:
                    self._process_timeutc(payload)

                idx += 8 + length
            else:
                idx += 1

        return self._get_gps_dict(), self._get_galileo_dict()

    def _process_sfrbx(self, payload: bytes):
        """Process UBX-RXM-SFRBX message"""
        if len(payload) < 8:
            return

        gnss_id = payload[0]
        sv_id = payload[1]
        num_words = payload[4]

        # Extract words (each word is 4 bytes)
        words = []
        for i in range(num_words):
            offset = 8 + i * 4
            if offset + 4 <= len(payload):
                word = struct.unpack('<I', payload[offset:offset + 4])[0]
                words.append(word)

        # GPS (gnssId = 0)
        if gnss_id == 0:
            self._process_gps_subframe(sv_id, words)
        # Galileo (gnssId = 2)
        elif gnss_id == 2:
            self._process_galileo_words(sv_id, words)

    def _process_rawx(self, payload: bytes):
        """Process UBX-RXM-RAWX message to extract receiver time"""
        if len(payload) < 16:
            return

        # RXM-RAWX structure:
        # Bytes 0-7: rcvTow (double, seconds)
        # Bytes 8-9: week (u2)
        # Bytes 10: leapS (i1)
        # Bytes 11: numMeas (u1)
        # ... rest is measurement data

        rcv_tow = struct.unpack('<d', payload[0:8])[0]
        week = struct.unpack('<H', payload[8:10])[0]
        leap_s = struct.unpack('b', payload[10:11])[0]

        # Store the latest receiver time
        self.latest_rcv_tow = rcv_tow
        self.latest_week = week

        # Calculate UTC time from GPS time
        if week is not None and rcv_tow is not None and leap_s is not None:
            try:
                # Handle GPS week rollover
                full_week = week
                if full_week < 1024:
                    full_week += 2048

                # Calculate GPS time
                gps_time = self.GPS_EPOCH + timedelta(weeks=full_week, seconds=rcv_tow)

                # Convert to UTC using leap seconds from message
                utc_time = gps_time - timedelta(seconds=leap_s)
                self.latest_utc_time = utc_time

            except Exception:
                pass

    def _process_timeutc(self, payload: bytes):
        """Process UBX-NAV-TIMEUTC message to extract UTC time"""
        if len(payload) < 20:
            return

        # NAV-TIMEUTC structure:
        # Bytes 0-3: iTOW (u4, ms)
        # Bytes 4-7: tAcc (u4, ns)
        # Bytes 8-11: nano (i4, ns)
        # Bytes 12-13: year (u2)
        # Byte 14: month (u1)
        # Byte 15: day (u1)
        # Byte 16: hour (u1)
        # Byte 17: min (u1)
        # Byte 18: sec (u1)
        # Byte 19: valid (X1, bitfield)

        year = struct.unpack('<H', payload[12:14])[0]
        month = payload[14]
        day = payload[15]
        hour = payload[16]
        minute = payload[17]
        second = payload[18]
        valid = payload[19]

        # Check if UTC time is valid (bit 2 of valid field)
        if valid & 0x04:
            try:
                # Get nanoseconds for sub-second precision
                nano = struct.unpack('<i', payload[8:12])[0]

                utc_time = datetime(year, month, day, hour, minute, second)
                # Add nanoseconds as microseconds (truncate to microsecond precision)
                utc_time += timedelta(microseconds=nano // 1000)

                self.latest_utc_time = utc_time

            except Exception:
                pass

    def _process_gps_subframe(self, sv_id: int, words: List[int]):
        """Process GPS subframe data"""
        if len(words) < 10:
            return

        # Extract subframe ID from word 2 (bits 49-51 of the subframe)
        # Word 2 is words[1] after TLM
        word2 = words[1]
        subframe_id = (word2 >> 8) & 0x7

        if subframe_id not in [1, 2, 3]:
            return

        if sv_id not in self.gps_ephemeris:
            self.gps_ephemeris[sv_id] = GPSEphemeris()

        eph = self.gps_ephemeris[sv_id]
        # Store the raw words for this subframe
        eph.subframes[subframe_id] = words.copy()

        # Parse subframes when available
        if subframe_id == 1:
            self._parse_gps_subframe1(sv_id, words)
        elif subframe_id == 2:
            self._parse_gps_subframe2(sv_id, words)
        elif subframe_id == 3:
            self._parse_gps_subframe3(sv_id, words)

    def _extract_bits(self, words: List[int], start_bit: int, num_bits: int) -> int:
        """Extract bits from GPS words (30-bit words, MSB first in transmission)"""
        # u-blox provides words as 32-bit values with parity removed
        # Data is in upper 24 bits for most words
        value = 0
        for i in range(num_bits):
            bit_pos = start_bit + i
            word_idx = bit_pos // 24
            bit_in_word = bit_pos % 24

            if word_idx >= len(words):
                break

            # Extract bit (bit 0 is MSB of the 24-bit data)
            bit = (words[word_idx] >> (23 - bit_in_word)) & 1
            value = (value << 1) | bit

        return value

    def _twos_complement(self, value: int, bits: int) -> int:
        """Convert unsigned value to signed using two's complement"""
        if value >= (1 << (bits - 1)):
            return value - (1 << bits)
        return value

    def _parse_gps_subframe1(self, sv_id: int, words: List[int]):
        """Parse GPS Subframe 1 - Clock correction and health"""
        eph = self.gps_ephemeris[sv_id]

        # Skip TLM and HOW (words 0, 1), start from word 2
        # Week number: word 2, bits 60-69 (10 bits)
        week = self._extract_bits(words[2:], 20, 10)
        eph.week = week

        # URA: word 2, bits 72-75
        ura_index = self._extract_bits(words[2:], 8, 4)
        eph.ura = ura_index

        # Health: word 2, bits 76-81
        eph.health = self._extract_bits(words[2:], 2, 6)

        # IODC: word 2 bits 82-83, word 7 bits 210-217
        iodc_msb = self._extract_bits(words[2:], 0, 2)
        iodc_lsb = self._extract_bits(words[7:], 18, 8)
        eph.iodc = (iodc_msb << 8) | iodc_lsb

        # toc: word 7, bits 218-233 (16 bits)
        toc = self._extract_bits(words[7:], 10, 16)
        eph.toc = toc * 16.0

        # af2: word 8, bits 240-247 (8 bits, signed)
        af2 = self._twos_complement(self._extract_bits(words[8:], 16, 8), 8)
        eph.af2 = af2 * (2 ** -55)

        # af1: word 8, bits 248-263 (16 bits, signed)
        af1 = self._twos_complement(self._extract_bits(words[8:], 0, 16), 16)
        eph.af1 = af1 * (2 ** -43)

        # af0: word 9, bits 270-291 (22 bits, signed)
        af0 = self._twos_complement(self._extract_bits(words[9:], 2, 22), 22)
        eph.af0 = af0 * (2 ** -31)

        # Tgd: word 6, bits 196-203 (8 bits, signed)
        tgd = self._twos_complement(self._extract_bits(words[6:], 4, 8), 8)
        eph.tgd = tgd * (2 ** -31)

    def _parse_gps_subframe2(self, sv_id: int, words: List[int]):
        """Parse GPS Subframe 2 - Ephemeris data"""
        eph = self.gps_ephemeris[sv_id]

        # IODE: word 2, bits 60-67
        eph.iode = self._extract_bits(words[2:], 20, 8)

        # Crs: word 2, bits 68-83 (16 bits, signed)
        Crs = self._twos_complement(self._extract_bits(words[2:], 4, 16), 16)
        eph.Crs = Crs * (2 ** -5)

        # Delta n: word 3, bits 90-105 (16 bits, signed)
        delta_n = self._twos_complement(self._extract_bits(words[3:], 18, 16), 16)
        eph.delta_n = delta_n * (2 ** -43) * 3.14159265359

        # M0: word 3 bits 106-113, word 4 bits 120-143 (32 bits, signed)
        M0_msb = self._extract_bits(words[3:], 2, 8)
        M0_lsb = self._extract_bits(words[4:], 0, 24)
        M0 = self._twos_complement((M0_msb << 24) | M0_lsb, 32)
        eph.M0 = M0 * (2 ** -31) * 3.14159265359

        # Cuc: word 5, bits 150-165 (16 bits, signed)
        Cuc = self._twos_complement(self._extract_bits(words[5:], 18, 16), 16)
        eph.Cuc = Cuc * (2 ** -29)

        # e: word 5 bits 166-173, word 6 bits 180-203 (32 bits, unsigned)
        e_msb = self._extract_bits(words[5:], 2, 8)
        e_lsb = self._extract_bits(words[6:], 0, 24)
        e = (e_msb << 24) | e_lsb
        eph.e = e * (2 ** -33)

        # Cus: word 7, bits 210-225 (16 bits, signed)
        Cus = self._twos_complement(self._extract_bits(words[7:], 18, 16), 16)
        eph.Cus = Cus * (2 ** -29)

        # sqrt_A: word 7 bits 226-233, word 8 bits 240-263 (32 bits, unsigned)
        sqrtA_msb = self._extract_bits(words[7:], 2, 8)
        sqrtA_lsb = self._extract_bits(words[8:], 0, 24)
        sqrt_A = (sqrtA_msb << 24) | sqrtA_lsb
        eph.sqrt_A = sqrt_A * (2 ** -19)

        # toe: word 9, bits 270-285 (16 bits, unsigned)
        toe = self._extract_bits(words[9:], 2, 16)
        eph.toe = toe * 16.0

    def _parse_gps_subframe3(self, sv_id: int, words: List[int]):
        """Parse GPS Subframe 3 - Ephemeris data"""
        eph = self.gps_ephemeris[sv_id]

        # Cic: word 2, bits 60-75 (16 bits, signed)
        Cic = self._twos_complement(self._extract_bits(words[2:], 20, 16), 16)
        eph.Cic = Cic * (2 ** -29)

        # OMEGA0: word 2 bits 76-83, word 3 bits 90-113 (32 bits, signed)
        omega0_msb = self._extract_bits(words[2:], 4, 8)
        omega0_lsb = self._extract_bits(words[3:], 2, 24)
        omega0 = self._twos_complement((omega0_msb << 24) | omega0_lsb, 32)
        eph.Omega0 = omega0 * (2 ** -31) * 3.14159265359

        # Cis: word 4, bits 120-135 (16 bits, signed)
        Cis = self._twos_complement(self._extract_bits(words[4:], 8, 16), 16)
        eph.Cis = Cis * (2 ** -29)

        # i0: word 4 bits 136-143, word 5 bits 150-173 (32 bits, signed)
        i0_msb = self._extract_bits(words[4:], 0, 8)
        i0_lsb = self._extract_bits(words[5:], 2, 24)
        i0 = self._twos_complement((i0_msb << 24) | i0_lsb, 32)
        eph.i0 = i0 * (2 ** -31) * 3.14159265359

        # Crc: word 6, bits 180-195 (16 bits, signed)
        Crc = self._twos_complement(self._extract_bits(words[6:], 8, 16), 16)
        eph.Crc = Crc * (2 ** -5)

        # omega: word 6 bits 196-203, word 7 bits 210-233 (32 bits, signed)
        omega_msb = self._extract_bits(words[6:], 0, 8)
        omega_lsb = self._extract_bits(words[7:], 2, 24)
        omega = self._twos_complement((omega_msb << 24) | omega_lsb, 32)
        eph.omega = omega * (2 ** -31) * 3.14159265359

        # OMEGA_DOT: word 8, bits 240-263 (24 bits, signed)
        OMEGA_DOT = self._twos_complement(self._extract_bits(words[8:], 0, 24), 24)
        eph.Omega_dot = OMEGA_DOT * (2 ** -43) * 3.14159265359

        # IDOT: word 9, bits 278-291 (14 bits, signed)
        IDOT = self._twos_complement(self._extract_bits(words[9:], 10, 14), 14)
        eph.IDOT = IDOT * (2 ** -43) * 3.14159265359

    def _process_galileo_words(self, sv_id: int, words: List[int]):
        """Process Galileo I/NAV words"""
        if len(words) < 8:
            return

        if sv_id not in self.galileo_ephemeris:
            self.galileo_ephemeris[sv_id] = GalileoEphemeris()

        # Galileo I/NAV page is 244 bits (including sync pattern)
        # Word type is in the first word
        # Extract word type from bits 0-5 (6 bits)
        word_type = (words[0] >> 26) & 0x3F

        eph = self.galileo_ephemeris[sv_id]
        eph.word_types[word_type] = words.copy()

        # Parse based on word type
        if word_type == 1:
            self._parse_galileo_word1(sv_id, words)
        elif word_type == 2:
            self._parse_galileo_word2(sv_id, words)
        elif word_type == 3:
            self._parse_galileo_word3(sv_id, words)
        elif word_type == 4:
            self._parse_galileo_word4(sv_id, words)
        elif word_type == 5:
            self._parse_galileo_word5(sv_id, words)

    def _extract_galileo_bits(self, words: List[int], start_bit: int, num_bits: int) -> int:
        """Extract bits from Galileo words"""
        value = 0
        for i in range(num_bits):
            bit_pos = start_bit + i
            word_idx = bit_pos // 32
            bit_in_word = bit_pos % 32

            if word_idx >= len(words):
                break

            bit = (words[word_idx] >> (31 - bit_in_word)) & 1
            value = (value << 1) | bit

        return value

    def _parse_galileo_word1(self, sv_id: int, words: List[int]):
        """Parse Galileo Word Type 1 - IODnav, toe, M0, e, sqrt_A"""
        eph = self.galileo_ephemeris[sv_id]

        # IODnav: bits 6-15 (10 bits)
        eph.iodnav = self._extract_galileo_bits(words, 6, 10)

        # toe: bits 16-29 (14 bits, unsigned)
        toe = self._extract_galileo_bits(words, 16, 14)
        eph.toe = toe * 60.0

        # M0: bits 30-61 (32 bits, signed)
        M0 = self._twos_complement(self._extract_galileo_bits(words, 30, 32), 32)
        eph.M0 = M0 * (2 ** -31) * 3.14159265359

        # e: bits 62-93 (32 bits, unsigned)
        e = self._extract_galileo_bits(words, 62, 32)
        eph.e = e * (2 ** -33)

        # sqrt_A: bits 94-125 (32 bits, unsigned)
        sqrt_A = self._extract_galileo_bits(words, 94, 32)
        eph.sqrt_A = sqrt_A * (2 ** -19)

    def _parse_galileo_word2(self, sv_id: int, words: List[int]):
        """Parse Galileo Word Type 2 - IODnav, OMEGA0, i0, omega, IDOT"""
        eph = self.galileo_ephemeris[sv_id]

        # IODnav: bits 6-15
        iodnav = self._extract_galileo_bits(words, 6, 10)

        # OMEGA0: bits 16-47 (32 bits, signed)
        omega0 = self._twos_complement(self._extract_galileo_bits(words, 16, 32), 32)
        eph.Omega0 = omega0 * (2 ** -31) * 3.14159265359

        # i0: bits 48-79 (32 bits, signed)
        i0 = self._twos_complement(self._extract_galileo_bits(words, 48, 32), 32)
        eph.i0 = i0 * (2 ** -31) * 3.14159265359

        # omega: bits 80-111 (32 bits, signed)
        omega = self._twos_complement(self._extract_galileo_bits(words, 80, 32), 32)
        eph.omega = omega * (2 ** -31) * 3.14159265359

        # IDOT: bits 112-125 (14 bits, signed)
        IDOT = self._twos_complement(self._extract_galileo_bits(words, 112, 14), 14)
        eph.IDOT = IDOT * (2 ** -43) * 3.14159265359

    def _parse_galileo_word3(self, sv_id: int, words: List[int]):
        """Parse Galileo Word Type 3 - IODnav, OMEGA_DOT, delta_n, Cuc, Cus, Crc, Crs"""
        eph = self.galileo_ephemeris[sv_id]

        # IODnav: bits 6-15
        iodnav = self._extract_galileo_bits(words, 6, 10)

        # OMEGA_DOT: bits 16-39 (24 bits, signed)
        OMEGA_DOT = self._twos_complement(self._extract_galileo_bits(words, 16, 24), 24)
        eph.Omega_dot = OMEGA_DOT * (2 ** -43) * 3.14159265359

        # delta_n: bits 40-55 (16 bits, signed)
        delta_n = self._twos_complement(self._extract_galileo_bits(words, 40, 16), 16)
        eph.delta_n = delta_n * (2 ** -43) * 3.14159265359

        # Cuc: bits 56-71 (16 bits, signed)
        Cuc = self._twos_complement(self._extract_galileo_bits(words, 56, 16), 16)
        eph.Cuc = Cuc * (2 ** -29)

        # Cus: bits 72-87 (16 bits, signed)
        Cus = self._twos_complement(self._extract_galileo_bits(words, 72, 16), 16)
        eph.Cus = Cus * (2 ** -29)

        # Crc: bits 88-103 (16 bits, signed)
        Crc = self._twos_complement(self._extract_galileo_bits(words, 88, 16), 16)
        eph.Crc = Crc * (2 ** -5)

        # Crs: bits 104-119 (16 bits, signed)
        Crs = self._twos_complement(self._extract_galileo_bits(words, 104, 16), 16)
        eph.Crs = Crs * (2 ** -5)

    def _parse_galileo_word4(self, sv_id: int, words: List[int]):
        """Parse Galileo Word Type 4 - IODnav, Cic, Cis, toc, af0, af1, af2"""
        eph = self.galileo_ephemeris[sv_id]

        # IODnav: bits 6-15
        iodnav = self._extract_galileo_bits(words, 6, 10)

        # Cic: bits 16-31 (16 bits, signed)
        Cic = self._twos_complement(self._extract_galileo_bits(words, 16, 16), 16)
        eph.Cic = Cic * (2 ** -29)

        # Cis: bits 32-47 (16 bits, signed)
        Cis = self._twos_complement(self._extract_galileo_bits(words, 32, 16), 16)
        eph.Cis = Cis * (2 ** -29)

        # toc: bits 48-61 (14 bits, unsigned)
        toc = self._extract_galileo_bits(words, 48, 14)
        eph.toc = toc * 60.0

        # af0: bits 62-92 (31 bits, signed)
        af0 = self._twos_complement(self._extract_galileo_bits(words, 62, 31), 31)
        eph.af0 = af0 * (2 ** -34)

        # af1: bits 93-113 (21 bits, signed)
        af1 = self._twos_complement(self._extract_galileo_bits(words, 93, 21), 21)
        eph.af1 = af1 * (2 ** -46)

        # af2: bits 114-119 (6 bits, signed)
        af2 = self._twos_complement(self._extract_galileo_bits(words, 114, 6), 6)
        eph.af2 = af2 * (2 ** -59)

    def _parse_galileo_word5(self, sv_id: int, words: List[int]):
        """Parse Galileo Word Type 5 - BGD, health, week, etc."""
        eph = self.galileo_ephemeris[sv_id]

        # BGD E5a/E1: bits 47-56 (10 bits, signed)
        bgd_e5a = self._twos_complement(self._extract_galileo_bits(words, 47, 10), 10)
        eph.bgd_e5a = bgd_e5a * (2 ** -32)

        # BGD E5b/E1: bits 57-66 (10 bits, signed)
        bgd_e5b = self._twos_complement(self._extract_galileo_bits(words, 57, 10), 10)
        eph.bgd_e5b = bgd_e5b * (2 ** -32)

        # E5b health: bits 67-68 (2 bits)
        eph.health_e5b = self._extract_galileo_bits(words, 67, 2)

        # E5a health: bits 69-70 (2 bits)
        eph.health_e5a = self._extract_galileo_bits(words, 69, 2)

        # Week number: bits 73-84 (12 bits)
        week = self._extract_galileo_bits(words, 73, 12)
        eph.week = week

        # SISA: bits 85-92 (8 bits)
        sisa_index = self._extract_galileo_bits(words, 85, 8)
        eph.sisa = sisa_index

    def _get_gps_dict(self) -> Dict:
        """Get GPS ephemeris as dictionary with satellite PRN keys"""
        result = {}
        for sv_id, eph in self.gps_ephemeris.items():
            if eph.is_complete():
                # Set UTC time from latest receiver time
                if self.latest_utc_time:
                    eph.time_utc = self.latest_utc_time.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"
                # GPS satellite numbering: G01, G02, etc.
                sat_key = f"G{sv_id:02d}"
                result[sat_key] = eph.to_dict()
        # Return sorted by satellite number
        return dict(sorted(result.items()))

    def _get_galileo_dict(self) -> Dict:
        """Get Galileo ephemeris as dictionary with satellite PRN keys"""
        result = {}
        for sv_id, eph in self.galileo_ephemeris.items():
            if eph.is_complete():
                # Set UTC time from latest receiver time
                if self.latest_utc_time:
                    eph.time_utc = self.latest_utc_time.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"
                # Galileo satellite numbering: E01, E02, etc.
                sat_key = f"E{sv_id:02d}"
                result[sat_key] = eph.to_dict()
        # Return sorted by satellite number
        return dict(sorted(result.items()))

    def _calculate_gps_utc_time(self, eph: GPSEphemeris):
        """Calculate UTC time from GPS ephemeris data"""
        if eph.week is None or eph.toe is None:
            return

        try:
            # Handle GPS week rollover (10-bit week number, 1024-week cycle)
            # Current GPS week as of code creation is ~2290 (cycle 2, started April 7, 2019)
            # Assume we're in cycle 2 (weeks 1024-2047) or later
            full_week = eph.week
            if full_week < 1024:
                # Assume cycle 2 (starting April 7, 2019)
                full_week += 2048
            elif full_week < 2048:
                # Could be cycle 2, keep as is
                pass

            # Calculate GPS time
            gps_time = self.GPS_EPOCH + timedelta(weeks=full_week, seconds=eph.toe)

            # Convert to UTC by subtracting leap seconds
            utc_time = gps_time - timedelta(seconds=self.LEAP_SECONDS)

            # Format as ISO 8601 string
            eph.time_utc = utc_time.strftime("%Y-%m-%dT%H:%M:%S.%fZ")[:-4] + "Z"

        except Exception as e:
            # If calculation fails, leave time_utc as None
            pass

    def _calculate_galileo_utc_time(self, eph: GalileoEphemeris):
        """Calculate UTC time from Galileo ephemeris data"""
        if eph.week is None or eph.toe is None:
            return

        try:
            # Galileo week number is 12 bits (0-4095)
            # Calculate Galileo System Time (GST)
            gst_time = self.GALILEO_EPOCH + timedelta(weeks=eph.week, seconds=eph.toe)

            # Convert to UTC by subtracting leap seconds
            # Note: Galileo and GPS have the same relationship to UTC
            utc_time = gst_time - timedelta(seconds=self.LEAP_SECONDS)

            # Format as ISO 8601 string
            eph.time_utc = utc_time.strftime("%Y-%m-%dT%H:%M:%S.%fZ")[:-4] + "Z"

        except Exception as e:
            # If calculation fails, leave time_utc as None
            pass


def parse_ubx_file(input_filepath: str, output_filepath: str = None) -> Dict:
    """
    Parse a UBX file and extract GPS and Galileo ephemeris data.

    Args:
        input_filepath: Path to the input .ubx file
        output_filepath: Optional path to output JSON file. If None, returns dict only.

    Returns:
        Dictionary containing GPS and Galileo ephemeris data
    """
    parser = UBXParser()
    gps_data, galileo_data = parser.parse_file(input_filepath)

    result = {
        "GPS": gps_data,
        "Galileo": galileo_data
    }

    if output_filepath:
        with open(output_filepath, 'w') as f:
            json.dump(result, f, indent=4, sort_keys=True)
        print(f"Ephemeris data written to {output_filepath}")
        print(f"GPS satellites: {len(gps_data)}")
        print(f"Galileo satellites: {len(galileo_data)}")

    return result


def main():
    """Command-line interface for the parser"""
    import sys

    if len(sys.argv) < 2:
        print("Usage: python ubx_parser.py <input.ubx> [output.json]")
        print("  input.ubx  - Input UBX file to parse")
        print("  output.json - Optional output JSON file (default: ephemeris_data.json)")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else "ephemeris_data.json"

    try:
        parse_ubx_file(input_file, output_file)
    except FileNotFoundError:
        print(f"Error: File '{input_file}' not found")
        sys.exit(1)
    except Exception as e:
        print(f"Error parsing file: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()