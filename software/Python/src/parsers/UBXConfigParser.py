#!/usr/bin/env python3
"""
UBX Config Parser
Parses u-blox .ucf configuration files with complete key ID mappings.
Supports all 653 configuration parameters from u-blox X20 HPG 2.02.

Requires comprehensive_key_mappings.py in the same directory.
"""

import json
import csv
import sys
from pathlib import Path
from typing import Dict, List, Any, Optional

# Import comprehensive key mappings
from .UBXConfigKeyMapping import KEY_MAPPING


class UBXConfigParser:
    """Parser for u-blox UBX configuration files with comprehensive key mapping support."""

    # Layer definitions
    LAYER_NAMES = {
        0: 'RAM',
        1: 'BBR',
        2: 'Flash',
        7: 'Default'
    }

    # Value type descriptions
    VALUE_TYPE_DESC = {
        'L': 'Logical (Boolean)',
        'U1': 'Unsigned 8-bit',
        'U2': 'Unsigned 16-bit',
        'U4': 'Unsigned 32-bit',
        'I1': 'Signed 8-bit',
        'I2': 'Signed 16-bit',
        'I4': 'Signed 32-bit',
        'E1': 'Enumeration (8-bit)',
        'R8': 'Real (64-bit float)',
        'X8': 'Hex (64-bit)'
    }

    def __init__(self, custom_mappings: Optional[Dict[str, str]] = None):
        """
        Initialize the parser.

        Args:
            custom_mappings: Optional additional key mappings to merge with built-in mappings
        """
        self.config_items = []

        # Use comprehensive mappings
        self.key_mapping = KEY_MAPPING.copy()

        # Add custom mappings if provided
        if custom_mappings:
            self.key_mapping.update(custom_mappings)

    def parse_file(self, filepath: str) -> List[Dict[str, Any]]:
        """
        Parse a .ucf configuration file.

        Args:
            filepath: Path to the .ucf file

        Returns:
            List of configuration items with parsed data
        """
        with open(filepath, 'r') as f:
            data = json.load(f)

        if data.get('type') != 'CFG-ITEM':
            raise ValueError(f"Expected type 'CFG-ITEM', got '{data.get('type')}'")

        items = data.get('items', [])

        for item in items:
            parsed_item = self._parse_item(item)
            self.config_items.append(parsed_item)

        return self.config_items

    def _parse_item(self, item: Dict[str, Any]) -> Dict[str, Any]:
        """
        Parse a single configuration item.

        Args:
            item: Raw item dictionary from JSON

        Returns:
            Parsed item with parameter name and formatted values
        """
        key_id = item['key']
        value = item['value']
        layer = item['layer']
        value_type = item['valueType']
        action = item.get('action', 'Set')

        # Look up parameter name
        param_name = self.key_mapping.get(key_id, f"UNKNOWN_{key_id}")

        # Get layer name
        layer_name = self.LAYER_NAMES.get(layer, f"Unknown_{layer}")

        # Format value based on type
        formatted_value = self._format_value(value, value_type)

        return {
            'key_id': key_id,
            'parameter': param_name,
            'value': value,
            'formatted_value': formatted_value,
            'layer': layer,
            'layer_name': layer_name,
            'value_type': value_type,
            'value_type_desc': self.VALUE_TYPE_DESC.get(value_type, 'Unknown'),
            'action': action
        }

    def _format_value(self, value: Any, value_type: str) -> str:
        """
        Format value based on its type.

        Args:
            value: Raw value
            value_type: Type indicator (L, U1, U2, U4, I1, I2, I4, E1, R8, X8)

        Returns:
            Formatted value string
        """
        if value_type == 'L':
            # Logical/Boolean
            return 'true' if value else 'false'
        elif value_type in ['U1', 'U2', 'U4']:
            # Unsigned integers
            return str(value)
        elif value_type in ['I1', 'I2', 'I4']:
            # Signed integers
            return str(value)
        elif value_type == 'E1':
            # Enumeration (1 byte)
            return str(value)
        elif value_type == 'R8':
            # Real (8 bytes / double)
            return str(value)
        elif value_type == 'X8':
            # 8-byte hex value
            if isinstance(value, int):
                return f"0x{value:016x}"
            return str(value)
        else:
            return str(value)

    def to_csv(self, output_path: str, include_details: bool = False):
        """
        Export parsed configuration to CSV file.

        Args:
            output_path: Path for output CSV file
            include_details: Include additional columns (value type description, etc.)
        """
        if not self.config_items:
            print("Warning: No configuration items to export")
            return

        fieldnames = ['Parameter', 'Value', 'Layer', 'Key ID', 'Value Type', 'Action']
        if include_details:
            fieldnames.insert(5, 'Type Description')

        with open(output_path, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            writer.writeheader()
            for item in self.config_items:
                row = {
                    'Parameter': item['parameter'],
                    'Value': item['formatted_value'],
                    'Layer': item['layer_name'],
                    'Key ID': item['key_id'],
                    'Value Type': item['value_type'],
                    'Action': item['action']
                }
                if include_details:
                    row['Type Description'] = item['value_type_desc']
                writer.writerow(row)

        print(f"Exported {len(self.config_items)} configuration items to {output_path}")

    def print_summary(self, show_unknown: bool = True):
        """
        Print a summary of parsed configuration items.

        Args:
            show_unknown: Whether to show items with unknown key IDs
        """
        if not self.config_items:
            print("No configuration items parsed")
            return

        print(f"\n{'=' * 80}")
        print(f"Configuration Summary: {len(self.config_items)} items")
        print(f"{'=' * 80}\n")

        # Count unknown keys
        unknown_count = sum(1 for item in self.config_items
                            if item['parameter'].startswith('UNKNOWN_'))
        if unknown_count > 0:
            print(f"⚠ Warning: {unknown_count} items with unknown key IDs")
            if not show_unknown:
                print("  (Use --show-unknown to display them)\n")

        # Group by layer
        by_layer = {}
        for item in self.config_items:
            if not show_unknown and item['parameter'].startswith('UNKNOWN_'):
                continue
            layer = item['layer_name']
            if layer not in by_layer:
                by_layer[layer] = []
            by_layer[layer].append(item)

        for layer, items in sorted(by_layer.items()):
            print(f"\n{layer} Layer ({len(items)} items):")
            print(f"{'-' * 80}")
            for item in items:
                # Color-code unknown items
                param = item['parameter']
                if param.startswith('UNKNOWN_'):
                    param = f"⚠ {param}"
                print(f"  {param:50} = {item['formatted_value']}")

    def get_statistics(self) -> Dict[str, Any]:
        """Get statistics about the parsed configuration."""
        stats = {
            'total_items': len(self.config_items),
            'by_layer': {},
            'by_value_type': {},
            'unknown_keys': 0,
            'unique_parameters': set()
        }

        for item in self.config_items:
            # Layer counts
            layer = item['layer_name']
            stats['by_layer'][layer] = stats['by_layer'].get(layer, 0) + 1

            # Value type counts
            vtype = item['value_type']
            stats['by_value_type'][vtype] = stats['by_value_type'].get(vtype, 0) + 1

            # Unknown keys
            if item['parameter'].startswith('UNKNOWN_'):
                stats['unknown_keys'] += 1

            # Unique parameters
            stats['unique_parameters'].add(item['parameter'])

        stats['unique_parameters'] = len(stats['unique_parameters'])
        return stats


def main():
    """Main entry point for command-line usage."""
    import argparse

    parser = argparse.ArgumentParser(
        description='Parse u-blox UBX configuration (.ucf) files',
        epilog='Example: %(prog)s config.ucf -o output.csv'
    )
    parser.add_argument('input', help='Input .ucf file')
    parser.add_argument('-o', '--output', help='Output CSV file (auto-generated if not specified)')
    parser.add_argument('--details', action='store_true',
                        help='Include detailed columns in CSV output')
    parser.add_argument('--stats', action='store_true',
                        help='Show statistics about the configuration')
    parser.add_argument('--show-unknown', action='store_true',
                        help='Show items with unknown key IDs')

    args = parser.parse_args()

    if not Path(args.input).exists():
        print(f"Error: File not found: {args.input}")
        sys.exit(1)

    # Parse the configuration file
    ubx_parser = UBXConfigParser()

    try:
        ubx_parser.parse_file(args.input)
        ubx_parser.print_summary(show_unknown=args.show_unknown)

        # Show statistics if requested
        if args.stats:
            stats = ubx_parser.get_statistics()
            print(f"\n{'=' * 80}")
            print("Statistics")
            print(f"{'=' * 80}")
            print(f"Total items: {stats['total_items']}")
            print(f"Unique parameters: {stats['unique_parameters']}")
            print(f"Unknown keys: {stats['unknown_keys']}")
            print("\nBy Layer:")
            for layer, count in sorted(stats['by_layer'].items()):
                print(f"  {layer:10} {count:4} items")
            print("\nBy Value Type:")
            for vtype, count in sorted(stats['by_value_type'].items()):
                print(f"  {vtype:4} {count:4} items")

        # Export to CSV
        output_file = args.output
        if not output_file:
            input_path = Path(args.input)
            output_file = input_path.stem + '_parsed.csv'

        ubx_parser.to_csv(output_file, include_details=args.details)

    except Exception as e:
        print(f"Error parsing file: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()