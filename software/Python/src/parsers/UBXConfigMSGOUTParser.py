#!/usr/bin/env python3
"""
UBX MSGOUT Config Parser
Specialized parser for u-blox .ucf configuration files that extracts only
CFG-MSGOUT (message output) settings that are enabled.

Shows which messages are being output on each interface and at what rate.
"""

import json
import csv
import sys
from pathlib import Path
from typing import Dict, List, Any, Optional
import re

# Import comprehensive key mappings
from .UBXConfigKeyMapping import KEY_MAPPING


class UBXMsgoutParser:
    """Parser for u-blox message output configuration."""

    # Layer definitions
    LAYER_NAMES = {
        0: 'RAM',
        1: 'BBR',
        2: 'Flash',
        7: 'Default'
    }

    # Interface extraction from parameter name
    INTERFACE_PATTERN = r'_(I2C|SPI|UART1|UART2|USB)$'

    # Message type extraction
    MESSAGE_PATTERN = r'CFG-MSGOUT-([A-Z0-9_]+)_'

    def __init__(self):
        """Initialize the parser."""
        self.msgout_items = []
        self.key_mapping = KEY_MAPPING.copy()

    def parse_file(self, filepath: str) -> List[Dict[str, Any]]:
        """
        Parse a .ucf configuration file and extract MSGOUT settings.

        Args:
            filepath: Path to the .ucf file

        Returns:
            List of enabled MSGOUT configuration items
        """
        with open(filepath, 'r') as f:
            data = json.load(f)

        if data.get('type') != 'CFG-ITEM':
            raise ValueError(f"Expected type 'CFG-ITEM', got '{data.get('type')}'")

        items = data.get('items', [])

        for item in items:
            parsed_item = self._parse_item(item)
            # Only include MSGOUT items that are enabled (value > 0)
            if parsed_item and parsed_item['rate'] > 0:
                self.msgout_items.append(parsed_item)

        return self.msgout_items

    def _parse_item(self, item: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Parse a single configuration item.

        Args:
            item: Raw item dictionary from JSON

        Returns:
            Parsed item if it's a MSGOUT parameter, None otherwise
        """
        key_id = item['key']
        value = item['value']
        layer = item['layer']

        # Look up parameter name
        param_name = self.key_mapping.get(key_id)

        # Skip if not a MSGOUT parameter
        if not param_name or not param_name.startswith('CFG-MSGOUT-'):
            return None

        # Extract message type and interface
        message_type = self._extract_message_type(param_name)
        interface = self._extract_interface(param_name)

        # Get layer name
        layer_name = self.LAYER_NAMES.get(layer, f"Unknown_{layer}")

        return {
            'key_id': key_id,
            'parameter': param_name,
            'message_type': message_type,
            'interface': interface,
            'rate': value,
            'layer': layer,
            'layer_name': layer_name
        }

    def _extract_message_type(self, param_name: str) -> str:
        """
        Extract message type from parameter name.

        Example: CFG-MSGOUT-NMEA_ID_GGA_UART1 -> NMEA_ID_GGA
        """
        match = re.search(self.MESSAGE_PATTERN, param_name)
        if match:
            msg_part = match.group(1)
            # Remove trailing interface if present
            msg_part = re.sub(self.INTERFACE_PATTERN, '', msg_part)
            return msg_part
        return param_name.replace('CFG-MSGOUT-', '')

    def _extract_interface(self, param_name: str) -> str:
        """
        Extract interface from parameter name.

        Example: CFG-MSGOUT-NMEA_ID_GGA_UART1 -> UART1
        """
        match = re.search(self.INTERFACE_PATTERN, param_name)
        if match:
            return match.group(1)
        return 'Unknown'

    def to_csv(self, output_path: str, group_by: str = 'interface'):
        """
        Export enabled MSGOUT configuration to CSV file.

        Args:
            output_path: Path for output CSV file
            group_by: How to organize output - 'interface', 'message', or 'none'
        """
        if not self.msgout_items:
            print("Warning: No enabled MSGOUT items found")
            return

        # Sort based on grouping preference
        if group_by == 'interface':
            sorted_items = sorted(self.msgout_items,
                                  key=lambda x: (x['interface'], x['message_type']))
        elif group_by == 'message':
            sorted_items = sorted(self.msgout_items,
                                  key=lambda x: (x['message_type'], x['interface']))
        else:
            sorted_items = self.msgout_items

        with open(output_path, 'w', newline='') as csvfile:
            fieldnames = ['Message Type', 'Interface', 'Output Rate', 'Layer']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            writer.writeheader()
            for item in sorted_items:
                writer.writerow({
                    'Message Type': item['message_type'],
                    'Interface': item['interface'],
                    'Output Rate': item['rate'],
                    'Layer': item['layer_name']
                })

        print(f"Exported {len(self.msgout_items)} enabled message outputs to {output_path}")

    def print_summary(self, group_by: str = 'interface'):
        """
        Print a summary of enabled MSGOUT items.

        Args:
            group_by: How to organize output - 'interface' or 'message'
        """
        if not self.msgout_items:
            print("No enabled message outputs found")
            return

        print(f"\n{'=' * 80}")
        print(f"Enabled Message Outputs: {len(self.msgout_items)} items")
        print(f"{'=' * 80}\n")

        if group_by == 'interface':
            self._print_by_interface()
        elif group_by == 'message':
            self._print_by_message()
        else:
            # Simple list
            for item in self.msgout_items:
                print(f"  {item['message_type']:40} {item['interface']:10} Rate: {item['rate']}")

    def _print_by_interface(self):
        """Print summary grouped by interface."""
        by_interface = {}
        for item in self.msgout_items:
            interface = item['interface']
            if interface not in by_interface:
                by_interface[interface] = []
            by_interface[interface].append(item)

        for interface, items in sorted(by_interface.items()):
            print(f"\n{interface} ({len(items)} messages):")
            print(f"{'-' * 80}")
            for item in sorted(items, key=lambda x: x['message_type']):
                rate_str = f"every {item['rate']}" if item['rate'] > 1 else "every epoch"
                print(f"  {item['message_type']:45} {rate_str:15} ({item['layer_name']})")

    def _print_by_message(self):
        """Print summary grouped by message type."""
        by_message = {}
        for item in self.msgout_items:
            message = item['message_type']
            if message not in by_message:
                by_message[message] = []
            by_message[message].append(item)

        for message, items in sorted(by_message.items()):
            interfaces = ', '.join(f"{i['interface']}:{i['rate']}"
                                   for i in sorted(items, key=lambda x: x['interface']))
            print(f"  {message:50} {interfaces}")

    def get_statistics(self) -> Dict[str, Any]:
        """Get statistics about enabled message outputs."""
        stats = {
            'total_enabled': len(self.msgout_items),
            'by_interface': {},
            'by_message_type': {},
            'by_layer': {},
            'unique_messages': set(),
            'unique_interfaces': set()
        }

        for item in self.msgout_items:
            # Interface counts
            interface = item['interface']
            stats['by_interface'][interface] = stats['by_interface'].get(interface, 0) + 1
            stats['unique_interfaces'].add(interface)

            # Message type counts
            msg = item['message_type']
            stats['by_message_type'][msg] = stats['by_message_type'].get(msg, 0) + 1
            stats['unique_messages'].add(msg)

            # Layer counts
            layer = item['layer_name']
            stats['by_layer'][layer] = stats['by_layer'].get(layer, 0) + 1

        stats['unique_messages'] = len(stats['unique_messages'])
        stats['unique_interfaces'] = len(stats['unique_interfaces'])
        return stats


def main():
    """Main entry point for command-line usage."""
    import argparse

    parser = argparse.ArgumentParser(
        description='Parse u-blox UBX configuration files and extract enabled message outputs',
        epilog='Example: %(prog)s config.ucf -o msgout.csv'
    )
    parser.add_argument('input', help='Input .ucf file')
    parser.add_argument('-o', '--output', help='Output CSV file (auto-generated if not specified)')
    parser.add_argument('-g', '--group-by', choices=['interface', 'message', 'none'],
                        default='interface',
                        help='How to group output (default: interface)')
    parser.add_argument('--stats', action='store_true',
                        help='Show statistics about message outputs')

    args = parser.parse_args()

    if not Path(args.input).exists():
        print(f"Error: File not found: {args.input}")
        sys.exit(1)

    # Parse the configuration file
    msgout_parser = UBXMsgoutParser()

    try:
        msgout_parser.parse_file(args.input)
        msgout_parser.print_summary(group_by=args.group_by)

        # Show statistics if requested
        if args.stats:
            stats = msgout_parser.get_statistics()
            print(f"\n{'=' * 80}")
            print("Statistics")
            print(f"{'=' * 80}")
            print(f"Total enabled messages: {stats['total_enabled']}")
            print(f"Unique message types: {stats['unique_messages']}")
            print(f"Active interfaces: {stats['unique_interfaces']}")
            print("\nBy Interface:")
            for interface, count in sorted(stats['by_interface'].items()):
                print(f"  {interface:10} {count:4} messages")
            print("\nBy Layer:")
            for layer, count in sorted(stats['by_layer'].items()):
                print(f"  {layer:10} {count:4} messages")

        # Export to CSV
        output_file = args.output
        if not output_file:
            input_path = Path(args.input)
            output_file = input_path.stem + '_msgout.csv'

        msgout_parser.to_csv(output_file, group_by=args.group_by)

    except Exception as e:
        print(f"Error parsing file: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()