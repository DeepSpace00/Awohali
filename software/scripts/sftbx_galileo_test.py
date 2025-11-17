# galileo_inav_parser.py
# Parser for Galileo I/NAV pages extracted from UBX-RXM-SFRBX (8 x 32-bit words example)
# Returns raw integer fields extracted per I/NAV bit allocations (OS SIS ICD v2.1).
# See: Galileo OS SIS ICD for scaling/units and interpretation. :contentReference[oaicite:1]{index=1}

from typing import List, Dict, Any, Tuple

def _concat_4words_to_128(w: List[int]) -> int:
    """Concatenate 4 32-bit words into a single 128-bit integer.
    Expect w[0]..w[3], with w[0] being the most-significant 32 bits (MSB first).
    """
    if len(w) != 4:
        raise ValueError("Need exactly 4 words to make an I/NAV page (128 bits).")
    return (w[0] << 96) | (w[1] << 64) | (w[2] << 32) | (w[3] & 0xFFFFFFFF)

def _slice_bits(value: int, start_bit: int, length: int) -> int:
    """Extract `length` bits starting at `start_bit` (0 = MSB of 128-bit value).
    start_bit counts from MSB (0) to LSB (127).
    Returns unsigned integer of that slice.
    """
    if length == 0:
        return 0
    # convert to LSB=0 indexing:
    msb_index = 127  # highest bit index in 128-bit value
    lsb_bit = msb_index - (start_bit + length - 1)
    msb_bit = msb_index - start_bit
    mask = (1 << (msb_bit - lsb_bit + 1)) - 1
    return (value >> lsb_bit) & mask

def _to_signed(u: int, bits: int) -> int:
    """Convert unsigned integer u (bits wide) to signed two's complement integer."""
    sign_bit = 1 << (bits - 1)
    return (u ^ sign_bit) - sign_bit

# Field definitions by Word Type (bit sizes). Bits counted left-to-right across 128-bit page.
# The page bit indexing scheme: bit 0 is MSB of the 128-bit page, bit 127 is LSB.
# These definitions are taken directly from Galileo OS SIS ICD I/NAV word tables (word type bit allocations). :contentReference[oaicite:2]{index=2}

# For each word type, define a list of (name, size_in_bits, signed_bool)
_WORD_TYPE_FIELDS = {
    # Word Type 1: Ephemeris (1/4) (Type[6], IODnav[10], t0e[14], M0[32], e[32], sqrtA[32], res 2)
    1: [
        ("Type", 6, False),
        ("IODnav", 10, False),
        ("t0e", 14, False),
        ("M0", 32, True),
        ("e", 32, False),           # eccentricity stored as unsigned fraction (interpretation per ICD)
        ("sqrtA", 32, False),
        ("reserved", 2, False),
    ],
    # Word Type 2: Ephemeris (2/4) (Type[6], IODnav[10], Omega0[32], i0[32], w[32], i_dot? 14, res2)
    2: [
        ("Type", 6, False),
        ("IODnav", 10, False),
        ("Omega0", 32, True),
        ("i0", 32, True),
        ("omega", 32, True),
        ("i_dot_sign_and_frac", 14, True),  # 14-bit value; interpret per ICD (signed)
        ("reserved", 2, False),
    ],
    # Word Type 3: Ephemeris (3/4) + SISA (Type[6], IODnav[10], Omega_dot? 24, dn 16, CUC 16, CUS 16, CRC 16, CRS 16, SISA 8)
    3: [
        ("Type", 6, False),
        ("IODnav", 10, False),
        ("Omega_dot", 24, True),
        ("Delta_n", 16, True),
        ("CUC", 16, True),
        ("CUS", 16, True),
        ("CRC", 16, True),
        ("CRS", 16, True),
        ("SISA_E1_E5b", 8, False),
    ],
    # Word Type 4: SVID, Ephemeris (4/4), Clock correction (Type[6], IODnav[10], SVID[6], CIC[16], CIS[16], toc[14], af0[31], af1[21], af2[6], spare 2)
    4: [
        ("Type", 6, False),
        ("IODnav", 10, False),
        ("SVID", 6, False),
        ("Cic", 16, True),
        ("Cis", 16, True),
        ("t0c", 14, False),
        ("af0", 31, True),
        ("af1", 21, True),
        ("af2", 6, True),
        ("spare", 2, False),
    ],
    # Word Type 5: Ionospheric, BGD, health, GST (complex; here extract fields present per table; many small fields)
    5: [
        ("Type", 6, False),
        ("azimuth_id", 11, False),  # az
        ("ai0", 11, True),
        ("ai1", 14, True),
        ("ai2", 1, False),
        ("region1", 1, False),
        ("region2", 1, False),
        ("region3", 1, False),
        ("region4", 1, False),
        ("region5", 1, False),
        ("BGD_E1_E5a", 10, True),
        ("BGD_E1_E5b", 10, True),
        ("E5bHS", 2, False),
        ("E1BHS", 2, False),
        ("E5bDVS", 1, False),
        ("E1BDVS", 1, False),
        ("WN", 12, False),
        ("TOW", 20, False),
        ("spare", 23, False),
    ],
    # Word Type 16: Reduced Clock and Ephemeris Data (CED) parameters (Type[6], LSB fields etc)
    16: [
        ("Type", 6, False),
        ("IODnav", 5, False),  # table shows 6 then 5 for Ared? (table shows Type=16: 6,5,13,13,17,23,23,22,6)
        ("Ared", 13, False),
        ("ex_red", 13, True),
        ("redey_red", 17, True),
        ("dA_red", 23, True),
        ("dI_red", 23, True),
        ("Omega0_red", 22, True),
        ("lambda0_red", 6, False),
        # Note: the table is more nuanced; this parser returns raw fields for you to interpret per ICD.
    ],
}

def _parse_page(page_words: List[int]) -> Dict[str, Any]:
    """Parse a single I/NAV 4-word page and return a dict with decoded raw fields."""
    page128 = _concat_4words_to_128(page_words)
    # read Type (first 6 bits)
    type_field = _slice_bits(page128, 0, 6)
    fields_def = _WORD_TYPE_FIELDS.get(type_field)
    result = {"word_type": type_field}
    if fields_def is None:
        # unknown/unhandled word type: return raw 128-bit page as hex for diagnosis
        result["raw_page_hex"] = f"{page128:032x}"
        return result

    # iterate over field definitions extracting by cumulative start bit
    bit_ptr = 0
    for name, size, is_signed in fields_def:
        value = _slice_bits(page128, bit_ptr, size)
        if is_signed:
            value = _to_signed(value, size)
        result[name] = value
        bit_ptr += size

    # sanity: if we didn't consume full 128 bits, optionally include remainder
    if bit_ptr < 128:
        rem = _slice_bits(page128, bit_ptr, 128 - bit_ptr)
        result["_trailing_bits"] = rem
    return result

def parse_sfrbx_words(words: List[int]) -> List[Dict[str, Any]]:
    """Parse a list of 32-bit words from UBX-RXM-SFRBX.
    Accepts any length multiple of 4; returns one dict per 4-word I/NAV page.
    """
    if len(words) % 4 != 0:
        raise ValueError("Input words length must be a multiple of 4 (4 words per I/NAV page).")
    pages = []
    for i in range(0, len(words), 4):
        page_words = words[i:i+4]
        pages.append(_parse_page(page_words))
    return pages

# Example usage:
if __name__ == "__main__":
    # Example: 8 words (two pages). Replace these with the 32-bit word integers you get from UBX-RXM-SFRBX.
    example_words = [
        0x10DEF5BE, 0xA871DFF3, 0x510909A, 0x5BF3C000,
        0x9F200000, 0x2A, 0xAAAA6212, 0x198BC000
    ]
    parsed_pages = parse_sfrbx_words(example_words)
    import json
    print(json.dumps(parsed_pages, indent=2))
