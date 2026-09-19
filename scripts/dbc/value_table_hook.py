#!/usr/bin/env python3
"""
dbc_value_table_updater.py

Parses a YAML file defining Value Tables (VAL_DEF_) and their signal mappings,
then updates the target DBC files accordingly.

Compatible with CANdb++ DBC format.

Usage:
    python dbc_value_table_updater.py --yaml value_tables.yaml --dbc-dir ./dbcs
    python dbc_value_table_updater.py --yaml value_tables.yaml --dbc-dir ./dbcs --dry-run
    python dbc_value_table_updater.py --yaml value_tables.yaml --dbc-dir ./dbcs --backup

YAML format:
    value_tables:
      OnOff:
        0: "Off"
        1: "On"

    signal_mappings:
      - dbc: "engine.dbc"
        message: "EngineStatus"
        signal: "EngineState"
        value_table: "OnOff"
"""

import argparse
import logging
import re
import shutil
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import yaml

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

logging.basicConfig(
    level=logging.INFO,
    format="%(levelname)-8s %(message)s",
)
log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# YAML loading
# ---------------------------------------------------------------------------


def load_yaml(yaml_path: Path) -> tuple[dict, list[dict]]:
    """Load and validate the YAML config. Returns (value_tables, signal_mappings)."""
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    if "value_tables" not in data:
        raise ValueError("YAML missing required key: 'value_tables'")
    if "signal_mappings" not in data:
        raise ValueError("YAML missing required key: 'signal_mappings'")

    # Normalise value table keys to int
    value_tables = {}
    for table_name, entries in data["value_tables"].items():
        value_tables[table_name] = {int(k): str(v) for k, v in entries.items()}

    # Validate signal mappings
    mappings = []
    for i, m in enumerate(data["signal_mappings"]):
        for required in ("dbc", "message", "signal", "value_table"):
            if required not in m:
                raise ValueError(
                    f"signal_mappings[{i}] missing required key: '{required}'"
                )
        if m["value_table"] not in value_tables:
            raise ValueError(
                f"signal_mappings[{i}]: value_table '{m['value_table']}' "
                f"not defined in value_tables"
            )
        mappings.append(m)

    log.info(
        "Loaded %d value table(s) and %d signal mapping(s) from %s",
        len(value_tables),
        len(mappings),
        yaml_path,
    )
    return value_tables, mappings


# ---------------------------------------------------------------------------
# DBC parsing helpers
# ---------------------------------------------------------------------------

# Matches:  VAL_DEF_ TableName 0 "Off" 1 "On" ;
_VAL_DEF_RE = re.compile(
    r'^VAL_TABLE_\s+(\w+)\s+((?:\d+\s+"[^"]*"\s*)+);',
    re.MULTILINE,
)

# Matches:  VAL_ <msg_id> <signal_name> 0 "Off" 1 "On" ;
_VAL_RE = re.compile(
    r'^VAL_\s+(\d+)\s+(\w+)\s+((?:\d+\s+"[^"]*"\s*)+);',
    re.MULTILINE,
)

# Matches a MESSAGE block:  BO_ <id> <name> : <len> <transmitter>
_MSG_RE = re.compile(
    r"^BO_\s+(\d+)\s+(\w+)\s*:\s*\d+\s+\w+",
    re.MULTILINE,
)


def parse_val_entries(raw: str) -> dict[int, str]:
    """Parse  '0 "Off" 1 "On" '  into  {0: "Off", 1: "On"}."""
    return {int(m.group(1)): m.group(2) for m in re.finditer(r'(\d+)\s+"([^"]*)"', raw)}


def format_val_entries(entries: dict[int, str]) -> str:
    """Format  {0: "Off", 1: "On"}  →  '0 "Off" 1 "On"'."""
    return " ".join(f'{k} "{v}"' for k, v in sorted(entries.items()))


def build_val_def_line(table_name: str, entries: dict[int, str]) -> str:
    return f"VAL_TABLE_ {table_name} {format_val_entries(entries)} ;\n"


def build_val_line(msg_id: int, signal_name: str, entries: dict[int, str]) -> str:
    return f"VAL_ {msg_id} {signal_name} {format_val_entries(entries)} ;\n"


def get_message_id(dbc_text: str, message_name: str) -> int | None:
    """Find the CAN message ID for a given message name."""
    for m in _MSG_RE.finditer(dbc_text):
        if m.group(2) == message_name:
            return int(m.group(1))
    return None


def signal_exists_in_message(
    dbc_text: str, message_name: str, signal_name: str
) -> bool:
    """Return True if <signal_name> appears inside the BO_ block for <message_name>."""
    # Find the start of the message block
    msg_match = re.search(
        rf"^BO_\s+\d+\s+{re.escape(message_name)}\s*:",
        dbc_text,
        re.MULTILINE,
    )
    if not msg_match:
        return False

    # Slice from message start to the next BO_ block (or end of file)
    block_start = msg_match.start()
    next_msg = re.search(r"^BO_\s+", dbc_text[block_start + 1 :], re.MULTILINE)
    block_end = block_start + 1 + next_msg.start() if next_msg else len(dbc_text)
    block_text = dbc_text[block_start:block_end]

    # Look for  SG_ <signal_name>  inside the block
    return bool(re.search(rf"\bSG_\s+{re.escape(signal_name)}\b", block_text))


# ---------------------------------------------------------------------------
# DBC update logic
# ---------------------------------------------------------------------------


def update_val_defs(dbc_text: str, value_tables: dict[str, dict[int, str]]) -> str:
    """
    Ensure every value table in `value_tables` has an up-to-date VAL_DEF_ entry
    in the DBC text.  Existing entries are replaced; new ones are appended.
    """
    existing: dict[str, re.Match] = {
        m.group(1): m for m in _VAL_DEF_RE.finditer(dbc_text)
    }

    for table_name, entries in value_tables.items():
        new_line = build_val_def_line(table_name, entries)

        if table_name in existing:
            old_match = existing[table_name]
            old_line = old_match.group(0)

            if old_line.strip() == new_line.strip():
                log.debug("  VAL_DEF_ %-20s unchanged", table_name)
                continue

            log.info("  VAL_DEF_ %-20s updated", table_name)
            dbc_text = dbc_text.replace(old_line, new_line.rstrip("\n"), 1)
        else:
            log.info("  VAL_DEF_ %-20s added", table_name)
            dbc_text = _append_or_insert_val_def(dbc_text, new_line)

    return dbc_text


def _append_or_insert_val_def(dbc_text: str, new_line: str) -> str:
    # Prefer: after last existing VAL_TABLE_
    last_vt = None
    for m in _VAL_DEF_RE.finditer(dbc_text):
        last_vt = m
    if last_vt:
        insert_pos = last_vt.end()
        return dbc_text[:insert_pos] + "\n" + new_line + dbc_text[insert_pos:]

    # Fallback: after last BA_DEF_DEF_ line (safe CANdb++ position)
    last_badef = None
    for m in re.finditer(r"^BA_DEF_DEF_\s+.*?;\s*$", dbc_text, re.MULTILINE):
        last_badef = m
    if last_badef:
        insert_pos = last_badef.end()
        return dbc_text[:insert_pos] + "\n\n" + new_line + dbc_text[insert_pos:]

    # Last resort: end of file
    return dbc_text.rstrip() + "\n\n" + new_line


def update_val_assignments(
    dbc_text: str,
    msg_id: int,
    signal_name: str,
    entries: dict[int, str],
    message_name: str,
) -> str:
    """
    Ensure a VAL_ line exists for the given (msg_id, signal_name) pair.
    Existing entries are replaced; new ones are appended before the final newline.
    """
    new_line = build_val_line(msg_id, signal_name, entries)

    # Find existing VAL_ for this exact (msg_id, signal_name)
    existing = None
    for m in _VAL_RE.finditer(dbc_text):
        if int(m.group(1)) == msg_id and m.group(2) == signal_name:
            existing = m
            break

    if existing:
        old_line = existing.group(0)
        if old_line.strip() == new_line.strip():
            log.debug("    VAL_ %s.%s unchanged", message_name, signal_name)
            return dbc_text
        log.info("    VAL_ %s.%-20s updated", message_name, signal_name)
        return dbc_text.replace(old_line, new_line.rstrip("\n"), 1)

    log.info("    VAL_ %s.%-20s added", message_name, signal_name)
    return dbc_text.rstrip() + "\n" + new_line


# ---------------------------------------------------------------------------
# Per-file processing
# ---------------------------------------------------------------------------


def process_dbc(
    dbc_path: Path,
    value_tables: dict[str, dict[int, str]],
    mappings: list[dict],
    dry_run: bool,
    backup: bool,
) -> bool:
    """
    Process a single DBC file.  Returns True if the file was (or would be) changed.
    """
    log.info("Processing %s", dbc_path.name)

    original_text = dbc_path.read_text(encoding="utf-8", errors="replace")
    dbc_text = original_text

    # 1. Determine which value tables are actually referenced in this DBC
    tables_needed: set[str] = set()
    for m in mappings:
        if m["dbc"] == dbc_path.name:
            tables_needed.add(m["value_table"])

    if not tables_needed:
        log.info("  No mappings reference this file — skipping")
        return False

    # 2. Update / insert VAL_DEF_ for all needed tables
    tables_subset = {k: v for k, v in value_tables.items() if k in tables_needed}
    dbc_text = update_val_defs(dbc_text, tables_subset)

    # 3. Update / insert VAL_ signal assignments
    errors: list[str] = []
    for m in mappings:
        if m["dbc"] != dbc_path.name:
            continue

        msg_name = m["message"]
        sig_name = m["signal"]
        table_name = m["value_table"]

        msg_id = get_message_id(dbc_text, msg_name)
        if msg_id is None:
            errors.append(f"  ERROR: message '{msg_name}' not found in {dbc_path.name}")
            continue

        if not signal_exists_in_message(dbc_text, msg_name, sig_name):
            errors.append(
                f"  ERROR: signal '{sig_name}' not found in message "
                f"'{msg_name}' in {dbc_path.name}"
            )
            continue

        dbc_text = update_val_assignments(
            dbc_text, msg_id, sig_name, value_tables[table_name], msg_name
        )

    for err in errors:
        log.error(err)

    # 4. Write result
    changed = dbc_text != original_text
    if not changed:
        log.info("  No changes needed")
        return False

    if dry_run:
        log.info("  [DRY RUN] Would update %s", dbc_path)
        return True

    if backup:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_path = dbc_path.with_suffix(f".{ts}.bak")
        shutil.copy2(dbc_path, backup_path)
        log.info("  Backup written to %s", backup_path.name)

    dbc_path.write_text(dbc_text, encoding="utf-8")
    log.info("  Saved %s", dbc_path)
    return True


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Update DBC Value Tables from a YAML definition file."
    )
    parser.add_argument(
        "--yaml",
        required=True,
        type=Path,
        help="Path to the YAML config file",
    )
    parser.add_argument(
        "--dbc-dir",
        required=True,
        type=Path,
        help="Directory containing the DBC files to update",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Parse and validate without writing any changes",
    )
    parser.add_argument(
        "--backup",
        action="store_true",
        help="Write a timestamped .bak file before modifying each DBC",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable debug-level output",
    )
    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Validate paths
    if not args.yaml.is_file():
        log.error("YAML file not found: %s", args.yaml)
        sys.exit(1)
    if not args.dbc_dir.is_dir():
        log.error("DBC directory not found: %s", args.dbc_dir)
        sys.exit(1)

    # Load YAML
    try:
        value_tables, mappings = load_yaml(args.yaml)
    except (ValueError, KeyError) as e:
        log.error("YAML error: %s", e)
        sys.exit(1)

    # Discover which DBC files are referenced in the YAML
    referenced_dbcs: set[str] = {m["dbc"] for m in mappings}

    # Group mappings per DBC for reporting
    mappings_by_dbc: dict[str, list] = defaultdict(list)
    for m in mappings:
        mappings_by_dbc[m["dbc"]].append(m)

    # Process each DBC
    updated: list[str] = []
    missing: list[str] = []

    for dbc_name in sorted(referenced_dbcs):
        dbc_path = args.dbc_dir / dbc_name
        if not dbc_path.is_file():
            missing.append(dbc_name)
            log.warning("DBC not found, skipping: %s", dbc_path)
            continue

        changed = process_dbc(
            dbc_path,
            value_tables,
            mappings,
            dry_run=args.dry_run,
            backup=args.backup,
        )
        if changed:
            updated.append(dbc_name)

    # Summary
    print("\n" + "=" * 50)
    print("Summary")
    print("=" * 50)
    print(f"  Value tables defined : {len(value_tables)}")
    print(f"  Signal mappings      : {len(mappings)}")
    print(f"  DBC files referenced : {len(referenced_dbcs)}")
    print(f"  DBC files missing    : {len(missing)}")
    action = "Would update" if args.dry_run else "Updated"
    print(f"  {action}             : {len(updated)}")
    if updated:
        for name in updated:
            print(f"    • {name}")
    if missing:
        print("  Missing:")
        for name in missing:
            print(f"    ✗ {name}")
    if args.dry_run:
        print("\n  [DRY RUN] No files were modified.")
    print("=" * 50)


if __name__ == "__main__":
    main()
