from parsers.SFRBXParser_galileo import parse_ubx_file
import json

results = parse_ubx_file('../data/ubx_data/2026-01-15/2026-1-15_61651_serial-COM4/2026-1-15_61651_serial-COM4.ubx')
ephemerides = results['Galileo']   # dict keyed by 'E01', 'E02', etc.

print(ephemerides)

# for sv, eph in ephemerides.items():
#     print(sv, eph['toe'], eph['sqrtA'])

with open("ephemeris_new.json", 'w') as f:
    json.dump(results, f, indent=2)