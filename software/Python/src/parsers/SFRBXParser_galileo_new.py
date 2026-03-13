"""
Galileo I/NAV Parser for u-blox UBX-RXM-SFRBX Messages

Handles u-blox proprietary formatting of Galileo I/NAV:
  1. Word-level byte reversal (little-endian to MSB-first bit stream)
  2. Non-byte-aligned bit extraction starting at bit offset 2
  3. Word type dispatch and ephemeris assembly

Compatible with numWords=8 (single even page) and numWords=16 (even+odd pair).
Based on Galileo OS-SIS-ICD v2.0.
"""

import struct
import json
import logging
from pathlib import Path
from typing import Dict, List, Optional
import math

logger = logging.getLogger(__name__)


class GalileoINavParser:

    CRC24Q_POLY = 0x1864CFB

    SCALE_FACTORS = {
        'toe':       60,
        'M0':        2**-31 * math.pi,
        'e':         2**-33,
        'sqrtA':     2**-19,
        'OMEGA0':    2**-31 * math.pi,
        'i0':        2**-31 * math.pi,
        'omega':     2**-31 * math.pi,
        'iDot':      2**-43 * math.pi,
        'OMEGAdot':  2**-43 * math.pi,
        'deltan':    2**-43 * math.pi,
        'Cuc':       2**-29,
        'Cus':       2**-29,
        'Crc':       2**-5,
        'Crs':       2**-5,
        'Cic':       2**-29,
        'Cis':       2**-29,
        'af0':       2**-34,
        'af1':       2**-46,
        'af2':       2**-59,
        'BGD_E1E5a': 2**-32,
        'BGD_E1E5b': 2**-32,
    }

    def __init__(self):
        self.ephemerides = {}

    @staticmethod
    def getbitu(buff: bytes, pos: int, length: int) -> int:
        bits = 0
        for i in range(pos, pos + length):
            bits = (bits << 1) + ((buff[i // 8] >> (7 - i % 8)) & 1)
        return bits

    @staticmethod
    def getbits(buff: bytes, pos: int, length: int) -> int:
        bits = GalileoINavParser.getbitu(buff, pos, length)
        if bits & (1 << (length - 1)):
            bits -= (1 << length)
        return bits

    def extract_inav_from_sfrbx(self, msg: bytes) -> Optional[bytes]:
        """
        Extract 30-byte I/NAV page from a UBX-RXM-SFRBX message.

        Accepts numWords=8 (single even page) or numWords=16 (even+odd pair).
        For numWords=16, only the even page (first 8 words) is used.
        """
        num_words = msg[10]

        if num_words not in (8, 16):
            logger.debug("Skipping Galileo message with unexpected numWords=%d", num_words)
            return None

        data_offset = 14
        expected_len = data_offset + (num_words * 4) + 2
        if len(msg) < expected_len:
            logger.debug("Message too short: %d bytes, expected %d", len(msg), expected_len)
            return None

        # Byte-reverse each 32-bit word (little-endian to MSB-first bit stream).
        # Only use the first 8 words regardless of numWords.
        payload = bytearray()
        for i in range(8):
            word_offset = data_offset + i * 4
            payload.extend([
                msg[word_offset + 3],
                msg[word_offset + 2],
                msg[word_offset + 1],
                msg[word_offset + 0],
            ])

        # Extract 30 bytes starting at bit offset 2
        inav = bytearray()
        for i in range(30):
            inav.append(self.getbitu(payload, 2 + i * 8, 8))

        return bytes(inav)

    def parse_word_type_1(self, inav: bytes, svid: int) -> Dict:
        """Word Type 1: Ephemeris 1/4 — toe, M0, e, sqrtA"""
        SF = self.SCALE_FACTORS
        return {
            'word_type': 1,
            'svid':      svid,
            'IODnav':    self.getbitu(inav,  6, 10),
            'toe':       self.getbitu(inav, 16, 14) * SF['toe'],
            'M0':        self.getbits(inav, 30, 32) * SF['M0'],
            'e':         self.getbitu(inav, 62, 32) * SF['e'],
            'sqrtA':     self.getbitu(inav, 94, 32) * SF['sqrtA'],
        }

    def parse_word_type_2(self, inav: bytes, svid: int) -> Dict:
        """Word Type 2: Ephemeris 2/4 — OMEGA0, i0, omega, iDot"""
        SF = self.SCALE_FACTORS
        return {
            'word_type': 2,
            'svid':      svid,
            'IODnav':    self.getbitu(inav,   6, 10),
            'OMEGA0':    self.getbits(inav,  16, 32) * SF['OMEGA0'],
            'i0':        self.getbits(inav,  48, 32) * SF['i0'],
            'omega':     self.getbits(inav,  80, 32) * SF['omega'],
            'iDot':      self.getbits(inav, 112, 14) * SF['iDot'],
        }

    def parse_word_type_3(self, inav: bytes, svid: int) -> Dict:
        """Word Type 3: Ephemeris 3/4 — OMEGAdot, deltan, harmonics, SISA"""
        SF = self.SCALE_FACTORS
        return {
            'word_type':  3,
            'svid':       svid,
            'IODnav':     self.getbitu(inav,   6, 10),
            'OMEGAdot':   self.getbits(inav,  16, 24) * SF['OMEGAdot'],
            'deltan':     self.getbits(inav,  40, 16) * SF['deltan'],
            'Cuc':        self.getbits(inav,  56, 16) * SF['Cuc'],
            'Cus':        self.getbits(inav,  72, 16) * SF['Cus'],
            'Crc':        self.getbits(inav,  88, 16) * SF['Crc'],
            'Crs':        self.getbits(inav, 104, 16) * SF['Crs'],
            'SISA':       self.getbitu(inav, 120,  8),
        }

    def parse_word_type_4(self, inav: bytes, svid: int) -> Dict:
        """Word Type 4: Ephemeris 4/4 + clock — Cic, Cis, toc, af0/1/2"""
        SF = self.SCALE_FACTORS
        return {
            'word_type':  4,
            'svid':       svid,
            'IODnav':     self.getbitu(inav,  6, 10),
            'svid_field': self.getbitu(inav, 16,  6),
            'Cic':        self.getbits(inav, 22, 16) * SF['Cic'],
            'Cis':        self.getbits(inav, 38, 16) * SF['Cis'],
            'toc':        self.getbitu(inav, 54, 14) * 60,
            'af0':        self.getbits(inav, 68, 31) * SF['af0'],
            'af1':        self.getbits(inav, 99, 21) * SF['af1'],
            'af2':        self.getbits(inav,120,  6) * SF['af2'],
        }

    def parse_word_type_5(self, inav: bytes, svid: int) -> Dict:
        """Word Type 5: Ionospheric, BGD, signal health, GST"""
        SF = self.SCALE_FACTORS
        return {
            'word_type':    5,
            'svid':         svid,
            'ai0':          self.getbitu(inav,  6, 11) * 2**-2,
            'ai1':          self.getbits(inav, 17, 11) * 2**-8,
            'ai2':          self.getbits(inav, 28, 14) * 2**-15,
            'region_flags': self.getbitu(inav, 42,  5),
            'BGD_E1E5a':    self.getbits(inav, 47, 10) * SF['BGD_E1E5a'],
            'BGD_E1E5b':    self.getbits(inav, 57, 10) * SF['BGD_E1E5b'],
            'E5b_HS':       self.getbitu(inav, 67,  2),
            'E1B_HS':       self.getbitu(inav, 69,  2),
            'E5b_DVS':      self.getbitu(inav, 71,  1),
            'E1B_DVS':      self.getbitu(inav, 72,  1),
            'WN':           self.getbitu(inav, 73, 12),
            'TOW':          self.getbitu(inav, 85, 20),
        }

    def parse_inav_message(self, inav: bytes, svid: int) -> Optional[Dict]:
        """Dispatch I/NAV page to the appropriate word type parser."""
        word_type = self.getbitu(inav, 0, 6)
        parsers = {
            1: self.parse_word_type_1,
            2: self.parse_word_type_2,
            3: self.parse_word_type_3,
            4: self.parse_word_type_4,
            5: self.parse_word_type_5,
        }
        parser = parsers.get(word_type)
        if parser is None:
            return None
        return parser(inav, svid)

    def assemble_ephemeris(self, word_data: List[Dict]) -> Optional[Dict]:
        """
        Assemble a complete ephemeris from a list of parsed word type dicts.
        Groups by IODnav; returns the most recent complete set (types 1-4).
        """
        by_iodnav: Dict[int, Dict[int, Dict]] = {}
        for data in word_data:
            if 'IODnav' in data:
                iodnav = data['IODnav']
                by_iodnav.setdefault(iodnav, {})[data['word_type']] = data

        for iodnav in sorted(by_iodnav, reverse=True):
            w = by_iodnav[iodnav]
            if not all(wt in w for wt in [1, 2, 3, 4]):
                continue

            eph = {
                'constellation': 'Galileo',
                'svid':     w[1]['svid'],
                'IODnav':   iodnav,
                'toe':      w[1]['toe'],
                'toc':      w[4]['toc'],
                'M0':       w[1]['M0'],
                'e':        w[1]['e'],
                'sqrtA':    w[1]['sqrtA'],
                'OMEGA0':   w[2]['OMEGA0'],
                'i0':       w[2]['i0'],
                'omega':    w[2]['omega'],
                'iDot':     w[2]['iDot'],
                'OMEGAdot': w[3]['OMEGAdot'],
                'deltan':   w[3]['deltan'],
                'Cuc':      w[3]['Cuc'],
                'Cus':      w[3]['Cus'],
                'Crc':      w[3]['Crc'],
                'Crs':      w[3]['Crs'],
                'Cic':      w[4]['Cic'],
                'Cis':      w[4]['Cis'],
                'af0':      w[4]['af0'],
                'af1':      w[4]['af1'],
                'af2':      w[4]['af2'],
                'SISA':     w[3]['SISA'],
            }

            if 5 in w:
                eph.update({
                    'BGD_E1E5a': w[5]['BGD_E1E5a'],
                    'BGD_E1E5b': w[5]['BGD_E1E5b'],
                    'E5b_HS':    w[5]['E5b_HS'],
                    'E1B_HS':    w[5]['E1B_HS'],
                    'WN':        w[5]['WN'],
                    'TOW':       w[5]['TOW'],
                })

            return eph

        return None


def parse_ubx_file(filepath: str) -> Dict:
    """Parse a UBX binary file and extract all Galileo ephemerides."""
    parser = GalileoINavParser()
    all_word_data: Dict[int, List[Dict]] = {}

    data = Path(filepath).read_bytes()
    i = 0
    while i < len(data) - 8:
        if data[i] != 0xB5 or data[i + 1] != 0x62:
            i += 1
            continue

        msg_class = data[i + 2]
        msg_id    = data[i + 3]
        length    = struct.unpack_from('<H', data, i + 4)[0]
        msg_end   = i + 6 + length + 2

        if msg_end > len(data):
            break

        if msg_class == 0x02 and msg_id == 0x13:  # RXM-SFRBX
            gnss_id   = data[i + 6]
            sv_id     = data[i + 7]
            num_words = data[i + 10]

            if gnss_id == 2 and num_words in (8, 16):
                msg = data[i:msg_end]
                inav = parser.extract_inav_from_sfrbx(msg)
                if inav:
                    word_data = parser.parse_inav_message(inav, sv_id)
                    if word_data:
                        logger.debug("E%02d WT%d IODnav=%s",
                                     sv_id, word_data['word_type'],
                                     word_data.get('IODnav', '-'))
                        all_word_data.setdefault(sv_id, []).append(word_data)

        i = msg_end

    ephemerides = {}
    for sv_id, word_list in all_word_data.items():
        eph = parser.assemble_ephemeris(word_list)
        if eph:
            key = f"E{sv_id:02d}"
            ephemerides[key] = eph
            logger.info("Assembled ephemeris for %s (IODnav=%d)", key, eph['IODnav'])

    logger.info("Found %d Galileo satellite(s)", len(ephemerides))
    return {'Galileo': ephemerides}


def main():
    import sys
    import argparse

    logging.basicConfig(level=logging.INFO, format='%(levelname)s %(message)s')

    ap = argparse.ArgumentParser(description='Parse Galileo I/NAV from UBX file')
    ap.add_argument('ubx_file', help='Input UBX binary file')
    ap.add_argument('-o', '--output', help='Output JSON file (default: <input>_galileo.json)')
    ap.add_argument('-v', '--verbose', action='store_true', help='Enable debug logging')
    args = ap.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    if not Path(args.ubx_file).exists():
        print(f"Error: {args.ubx_file} not found")
        sys.exit(1)

    results = parse_ubx_file(args.ubx_file)

    out = args.output or Path(args.ubx_file).stem + '_galileo.json'
    with open(out, 'w') as f:
        json.dump(results, f, indent=2)

    n = len(results.get('Galileo', {}))
    print(f"Found {n} Galileo satellite(s) — saved to {out}")


if __name__ == '__main__':
    main()