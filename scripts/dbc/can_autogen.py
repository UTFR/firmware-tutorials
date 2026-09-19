#!/usr/bin/env python3
"""
Generate .dbc, .h, and .c files from YAML CAN database definitions.

Scans YAML_DIR for *.yaml files, skips types.yaml, and for each remaining
file produces three output artefacts in OUTPUT_DIR:

    <name>.dbc   - CAN database (cantools / CANdb++ compatible)
    <name>.h     - C header:  structs, enums, pack/unpack prototypes
    <name>.c     - C source:  pack/unpack implementations

types.yaml defines shared enums; those enums are emitted once into
common.h and #include-d by every per-database header.

Usage:
    python can_codegen.py --yaml-dir path/to/yamls --output-dir generated/
    python can_codegen.py --yaml-dir . --output-dir out --nodes RC ACM
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Iterator, Optional

import yaml
import cantools
from cantools.database import Database
from scripts.scripts import write_file_and_format
from scripts.dbc.message import (
    CodeGenMessage,
    build_yaml_lookup,
    generate_common_choice_enums,
    generate_h_includes,
    generate_c_includes,
    generate_message_id_enums,
    generate_choice_enums,
    generate_structs,
    generate_definitions,
    generate_helpers,
    generate_dlcs,
    generate_message_cycle_time_defines,
    DBCData,
    DBCMessage,
    DBCSignal,
    ValueTable,
    TypeData,
    SignalMapping,
    TYamlLookup,
)
from scripts.dbc.util import camel_to_snake_case


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open() as fh:
        return yaml.safe_load(fh) or {}


def iter_messages(yaml_data: DBCData) -> Iterator[tuple[str, DBCMessage]]:
    for entry in yaml_data.messages:
        if entry:
            yield from entry.items()


def iter_signals(msg_props: DBCMessage) -> Iterator[tuple[str, DBCSignal]]:
    for entry in msg_props.signals:
        if entry:
            yield from entry.items()


_NS_KEYWORDS = [
    "NS_DESC_",
    "CM_",
    "BA_DEF_",
    "BA_",
    "VAL_",
    "CAT_DEF_",
    "CAT_",
    "FILTER",
    "BA_DEF_DEF_",
    "EV_DATA_",
    "ENVVAR_DATA_",
    "SGTYPE_",
    "SGTYPE_VAL_",
    "BA_DEF_SGTYPE_",
    "BA_SGTYPE_",
    "SIG_TYPE_REF_",
    "VAL_TABLE_",
    "SIG_GROUP_",
    "SIG_VALTYPE_",
    "SIGTYPE_VALTYPE_",
    "BO_TX_BU_",
    "BA_DEF_REL_",
    "BA_REL_",
    "BA_DEF_DEF_REL_",
    "BU_SG_REL_",
    "BU_EV_REL_",
    "BU_BO_REL_",
    "SG_MUL_VAL_",
]

_BA_DEF_LINES = [
    'BA_DEF_ BU_  "ECU" STRING ;',
    'BA_DEF_ BO_  "VFrameFormat" ENUM  "StandardCAN","ExtendedCAN";',
    'BA_DEF_  "BusType" STRING ;',
    'BA_DEF_  "MultiplexExtEnabled" ENUM  "No","Yes";',
    'BA_DEF_DEF_  "ECU" "";',
    'BA_DEF_DEF_  "VFrameFormat" "StandardCAN";',
    'BA_DEF_DEF_  "BusType" "CAN";',
    'BA_DEF_DEF_  "MultiplexExtEnabled" "No";',
]


def _num(val: Any) -> str:
    return str(val)


def yaml_to_dbc(
    yaml_data: DBCData, types_data: TypeData, value_tables: dict[str, ValueTable]
) -> str:
    lines: list[str] = []

    lines += ['VERSION ""', "", "NS_ :"]
    lines += [f"\t{kw}" for kw in _NS_KEYWORDS]
    lines += ["", "BS_:", ""]

    nodes = yaml_data.nodes
    lines.append(f'BU_: {" ".join(nodes)}')
    lines.append("")

    val_entries: list[tuple[int, str, ValueTable]] = []
    extended_ids: list[int] = []
    logged_ids: list[int] = []

    for msg_name, mp in iter_messages(yaml_data):
        raw_id: int = mp.id
        dlc: int = mp.dlc
        sender: str = mp.sender
        extended: bool = mp.extended
        dbc_id = (raw_id | 0x80000000) if extended else raw_id

        if extended:
            extended_ids.append(dbc_id)

        lines.append(f"BO_ {dbc_id} {msg_name}: {dlc} {sender}")

        for sig_name, sp in iter_signals(mp):
            endian = 1 if sp.endianness == "little" else 0
            sign = "-" if sp.signed else "+"
            rx = sp.receivers or ["Vector__XXX"]
            rx_str = ",".join(rx)

            lines.append(
                f" SG_ {sig_name} : {sp.start_bit}|{sp.length}@{endian}{sign}"
                f" ({_num(sp.scale)},{_num(sp.offset)})"
                f" [{_num(sp.min)}|{_num(sp.max)}]"
                f' "{sp.unit}"  {rx_str}'
            )

            enum_ref: Optional[str] = sp.enum
            if enum_ref and enum_ref in types_data.enums:
                val_entries.append((dbc_id, sig_name, types_data.enums[enum_ref]))

        lines.append("")

    lines += _BA_DEF_LINES
    lines.append("")

    for node in nodes:
        lines.append(f'BA_ "ECU" BU_ {node} "{node}";')

    for dbc_id in extended_ids:
        lines.append(f'BA_ "VFrameFormat" BO_ {dbc_id} 1;')
    for dbc_id in logged_ids:
        lines.append(f'BA_ "Logged" BO_ {dbc_id} 1;')

    if nodes or extended_ids or logged_ids:
        lines.append("")

    for dbc_id, sig_name, enum_dict in val_entries:
        pairs = " ".join(
            f'{k} "{v}"'
            for k, v in sorted(
                ((int(k), v) for k, v in enum_dict.items()),
                reverse=True,
            )
        )
        lines.append(f"VAL_ {dbc_id} {sig_name} {pairs} ;")

    if len(value_tables) > 0:
        lines.append("")

    for enum_name, value_table in value_tables.items():
        pairs = " ".join(
            f'{k} "{v}"'
            for k, v in sorted(
                ((int(k), v) for k, v in value_table.items()),
                reverse=True,
            )
        )
        lines.append(f"VAL_TABLE_ {enum_name} {pairs} ;")

    lines.append("")

    return "\n".join(lines)


def build_combined_yaml_data(
    dbc_data: dict[str, DBCData],
    type_data: TypeData,
) -> tuple[dict[str, ValueTable], list[SignalMapping]]:
    value_tables: dict[str, ValueTable] = {}
    for name, entries in type_data.enums.items():
        value_tables[name] = {int(k): str(v) for k, v in entries.items()}

    signal_mappings: list[SignalMapping] = []
    for filename, yaml_data in dbc_data.items():
        dbc_name = Path(filename).stem + ".dbc"
        for msg_name, mp in iter_messages(yaml_data):
            for sig_name, sp in iter_signals(mp):
                enum_ref = sp.enum
                if enum_ref and enum_ref in value_tables:
                    signal_mappings.append(
                        SignalMapping(
                            dbc_name,
                            msg_name,
                            sig_name,
                            enum_ref,
                        )
                    )

    return (value_tables, signal_mappings)


_AUTOGEN_NOTICE = "/* Auto-generated — do not edit. */"


def _join(*parts: str) -> str:
    return "\n\n".join(p for p in parts if p and p.strip())


def generate_h_and_c(
    db_name: str,
    cg_messages: list[CodeGenMessage],
    node_names: list[str] | None,
    yaml_lookup: TYamlLookup,
    skip_enums: set[str],
    has_common_h: bool,
    floating_point_numbers: bool,
    use_float: bool,
    use_round: bool,
) -> tuple[str, str]:
    h_name = f"{camel_to_snake_case(db_name)}.h"

    definitions, protos, helper_kinds = generate_definitions(
        db_name,
        cg_messages,
        floating_point_numbers=floating_point_numbers,
        use_float=use_float,
        node_names=node_names,
        use_round=use_round,
    )

    h_includes = generate_h_includes()
    if has_common_h:
        h_includes += '\n#include "common.h"'

    h_content = _join(
        _AUTOGEN_NOTICE,
        f"#ifndef UTFR_CAN_TYPES_{db_name.upper()}_H",
        f"#define UTFR_CAN_TYPES_{db_name.upper()}_H",
        h_includes,
        generate_message_id_enums(db_name, cg_messages),
        generate_dlcs(db_name, cg_messages),
        generate_message_cycle_time_defines(db_name, cg_messages, node_names),
        generate_choice_enums(
            cg_messages,
            node_names,
            yaml_lookup=yaml_lookup,
            skip_enums=skip_enums,
        ),
        generate_structs(db_name, cg_messages, bit_fields=False, node_names=node_names),
        protos,
        "#endif",
    )

    c_content = _join(
        _AUTOGEN_NOTICE,
        generate_c_includes(h_name),
        generate_helpers(helper_kinds),
        definitions,
    )

    return h_content, c_content


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--yaml-dir",
        default=".",
        metavar="DIR",
        help="Directory containing *.yaml files (default: current directory)",
    )
    p.add_argument(
        "--output-dir",
        default="generated",
        metavar="DIR",
        help="Write generated files here (default: ./generated)",
    )
    p.add_argument(
        "--nodes",
        nargs="*",
        metavar="NODE",
        help="Restrict pack/unpack generation to these ECU nodes (default: all)",
    )
    p.add_argument(
        "--no-floating-point",
        dest="floating_point",
        action="store_false",
        default=True,
        help="Omit floating-point encode/decode helpers",
    )
    p.add_argument(
        "--use-float",
        action="store_true",
        default=True,
        help="Use float instead of double in encode/decode helpers",
    )
    p.add_argument(
        "--use-round",
        dest="use_round",
        action="store_true",
        default=False,
        help="Use round() calls in encode helpers",
    )
    p.add_argument(
        "--dbc-output-dir",
        default=None,
        metavar="DIR",
        help="Write .dbc files here instead of --output-dir",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    yaml_dir = Path(args.yaml_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    dbc_output_dir = Path(args.dbc_output_dir) if args.dbc_output_dir else output_dir
    dbc_output_dir.mkdir(parents=True, exist_ok=True)

    types_yaml = yaml_dir / "types.yaml"
    if types_yaml.exists():
        types_data_raw = load_yaml(types_yaml)
        print(f"Loaded shared enums from {types_yaml}")
    else:
        print(f"Warning: {types_yaml} not found — no shared enums available.")
        types_data_raw = {}

    type_data = TypeData(**types_data_raw)

    db_yaml_paths = sorted(p for p in yaml_dir.glob("*.yaml") if p.name != "types.yaml")
    if not db_yaml_paths:
        print(f"No database YAML files found in {yaml_dir}.", file=sys.stderr)
        sys.exit(1)

    print(
        f"Found {len(db_yaml_paths)} database YAML(s): "
        f"{', '.join(p.name for p in db_yaml_paths)}"
    )

    dbc_yamls: dict[str, Any] = {p.name: load_yaml(p) for p in db_yaml_paths}
    dbc_data: dict[str, DBCData] = {
        name: DBCData(**val) for (name, val) in dbc_yamls.items()
    }

    value_tables, signal_mappings = build_combined_yaml_data(dbc_data, type_data)

    common_h_code, skip_enums = generate_common_choice_enums(
        value_tables, signal_mappings
    )
    has_common_h = bool(common_h_code)
    h_always = """
        typedef enum {
            CAN_BAUDRATE_1MBPS,
            CAN_BAUDRATE_500KBPS,
            CAN_BAUDRATE_250KBPS,
            CAN_BAUDRATE_125KBPS
        } can_baudrate_t;
    """

    if has_common_h:
        common_h_path = output_dir / "common.h"
        write_file_and_format(
            common_h_path,
            _join(
                _AUTOGEN_NOTICE,
                "#ifndef UTFR_CAN_TYPES_COMMON_H",
                "#define UTFR_CAN_TYPES_COMMON_H",
                generate_h_includes(),
                h_always,
                common_h_code,
                "#endif",
            ),
        )
        print(f"  Wrote {common_h_path}")

    errors: list[str] = []

    for yaml_path in db_yaml_paths:
        stem = yaml_path.stem
        db_name = stem
        yaml_data = dbc_data[yaml_path.name]

        print(f"\nProcessing {yaml_path.name}")

        dbc_str = yaml_to_dbc(yaml_data, type_data, value_tables)
        dbc_path = dbc_output_dir / f"{stem}.dbc"
        dbc_path.write_text(dbc_str)
        print(f"  Wrote {dbc_path}")

        try:
            db: Database = cantools.database.load_string(dbc_str, database_format="dbc")
            assert isinstance(db, cantools.database.can.database.Database)
        except Exception as exc:
            msg = f"  ERROR: cantools failed to parse generated DBC for {stem}: {exc}"
            print(msg, file=sys.stderr)
            errors.append(msg)
            continue

        cg_messages = [CodeGenMessage(msg) for msg in db.messages]

        yaml_lookup = build_yaml_lookup(value_tables, signal_mappings, f"{stem}.dbc")

        try:
            h_content, c_content = generate_h_and_c(
                db_name=db_name,
                cg_messages=cg_messages,
                node_names=args.nodes or None,
                yaml_lookup=yaml_lookup,
                skip_enums=skip_enums,
                has_common_h=has_common_h,
                floating_point_numbers=args.floating_point,
                use_float=args.use_float,
                use_round=args.use_round,
            )
        except Exception as exc:
            msg = f"  ERROR: code generation failed for {stem}: {exc}"
            print(msg, file=sys.stderr)
            errors.append(msg)
            continue

        h_path = output_dir / f"{stem}.h"
        c_path = output_dir / f"{stem}.c"
        write_file_and_format(h_path, h_content)
        write_file_and_format(c_path, c_content)
        print(f"  Wrote {h_path}")
        print(f"  Wrote {c_path}")

    print(f"\nDone. Output in {output_dir}/")
    if errors:
        print(f"\n{len(errors)} error(s) encountered:", file=sys.stderr)
        for e in errors:
            print(e, file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
