from collections.abc import Iterator
from cantools.database.can import Signal, Message
from scripts.dbc.util import camel_to_snake_case, get, canonical
from typing import cast, Optional, Literal
from pydantic import BaseModel
import warnings
from scripts.dbc.fmt import (
    ENUM_FMT,
    STRUCT_FMT,
    SIGNAL_MEMBER_FMT,
    SIGNAL_DEFINITION_ENCODE_PROTO_FMT,
    SIGNAL_DEFINITION_ENCODE_FMT,
    SIGNAL_DEFINITION_DECODE_PROTO_FMT,
    SIGNAL_DEFINITION_DECODE_FMT,
    SIGNAL_DEFINITION_IS_IN_RANGE_PROTO_FMT,
    SIGNAL_DEFINITION_IS_IN_RANGE_FMT,
    INIT_SIGNAL_BODY_TEMPLATE_FMT,
    DEFINITION_PACK_PROTO_FMT,
    DEFINITION_PACK_FMT,
    DEFINITION_UNPACK_PROTO_FMT,
    DEFINITION_UNPACK_FMT,
    EMPTY_DEFINITION_PROTO_FMT,
    EMPTY_DEFINITION_FMT,
    SIGN_EXTENSION_FMT,
    PACK_HELPER_LEFT_SHIFT_FMT,
    PACK_HELPER_RIGHT_SHIFT_FMT,
    UNPACK_HELPER_LEFT_SHIFT_FMT,
    UNPACK_HELPER_RIGHT_SHIFT_FMT,
    MESSAGE_DLC_DEFINE_FMT,
)

THelperKind = tuple[str, int]
TYamlLookup = dict[tuple[str, str], str]


type ValueTable = dict[int, str]


class TypeData(BaseModel):
    enums: dict[str, ValueTable]


type DBCEndianness = Literal["little", "big"]


class SignalMapping:
    dbc: str
    message: str
    signal: str
    value_table: str

    def __init__(self, dbc: str, message: str, signal: str, value_table: str):
        self.dbc = dbc
        self.message = message
        self.signal = signal
        self.value_table = value_table


class DBCSignal(BaseModel):
    enum: Optional[str] = None
    start_bit: int
    length: int
    endianness: DBCEndianness = "little"
    signed: bool = False
    scale: float = 1
    offset: float = 0
    min: float
    max: float
    unit: str = ""
    receivers: list[str] = ["Vector__XXX"]


class DBCMessage(BaseModel):
    id: int
    extended: bool = False
    dlc: int
    sender: str = "Vector__XXX"
    signals: list[dict[str, DBCSignal]]


class DBCData(BaseModel):
    nodes: list[str]
    messages: list[dict[str, DBCMessage]]


class CodeGenSignal:
    def __init__(self, signal: Signal) -> None:
        self.signal: Signal = signal
        self.snake_name = camel_to_snake_case(signal.name)

    @property
    def unit(self) -> str:
        return get(self.signal.unit, "-")

    @property
    def type_length(self) -> int:
        if self.signal.length <= 8:
            return 8
        elif self.signal.length <= 16:
            return 16
        elif self.signal.length <= 32:
            return 32
        else:
            return 64

    @property
    def type_name(self) -> str:
        if self.signal.choices:
            pass
        if self.signal.conversion.is_float:
            if self.signal.length == 32:
                type_name = "float"
            else:
                type_name = "double"
        else:
            type_name = f"int{self.type_length}_t"

            if not self.signal.is_signed:
                type_name = "u" + type_name

        return type_name

    @property
    def type_suffix(self) -> str:
        try:
            return {
                "uint8_t": "U",
                "uint16_t": "U",
                "uint32_t": "U",
                "int64_t": "LL",
                "uint64_t": "ULL",
                "float": "F",
            }[self.type_name]
        except KeyError:
            return ""

    @property
    def conversion_type_suffix(self) -> str:
        try:
            return {8: "U", 16: "U", 32: "U", 64: "ULL"}[self.type_length]
        except KeyError:
            return ""

    @property
    def unique_choices(self) -> dict[int, str]:
        """Make duplicated choice names unique by first appending its value
        and then underscores until unique.

        """
        if self.signal.choices is None:
            return {}

        items = {
            value: camel_to_snake_case(str(name)).upper()
            for value, name in self.signal.choices.items()
        }
        names = list(items.values())
        duplicated_names = [name for name in set(names) if names.count(name) > 1]
        unique_choices = {
            value: name for value, name in items.items() if names.count(name) == 1
        }

        for value, name in items.items():
            if name in duplicated_names:
                name += canonical(f"_{value}")

                while name in unique_choices.values():
                    name += "_"

                unique_choices[value] = name

        return unique_choices

    @property
    def minimum_ctype_value(self) -> int | None:
        if self.type_name == "int8_t":
            return -(2**7)
        elif self.type_name == "int16_t":
            return -(2**15)
        elif self.type_name == "int32_t":
            return -(2**31)
        elif self.type_name == "int64_t":
            return -(2**63)
        elif self.type_name.startswith("u"):
            return 0
        else:
            return None

    @property
    def maximum_ctype_value(self) -> int | None:
        if self.type_name == "int8_t":
            return 2**7 - 1
        elif self.type_name == "int16_t":
            return 2**15 - 1
        elif self.type_name == "int32_t":
            return 2**31 - 1
        elif self.type_name == "int64_t":
            return 2**63 - 1
        elif self.type_name == "uint8_t":
            return 2**8 - 1
        elif self.type_name == "uint16_t":
            return 2**16 - 1
        elif self.type_name == "uint32_t":
            return 2**32 - 1
        elif self.type_name == "uint64_t":
            return 2**64 - 1
        else:
            return None

    @property
    def minimum_can_raw_value(self) -> int | None:
        if self.signal.conversion.is_float:
            return None
        elif self.signal.is_signed:
            return cast("int", -(2 ** (self.signal.length - 1)))
        else:
            return 0

    @property
    def maximum_can_raw_value(self) -> int | None:
        if self.signal.conversion.is_float:
            return None
        elif self.signal.is_signed:
            return cast("int", (2 ** (self.signal.length - 1)) - 1)
        else:
            return cast("int", (2**self.signal.length) - 1)

    def segments(self, invert_shift: bool) -> Iterator[tuple[int, int, str, int]]:
        index, pos = divmod(self.signal.start, 8)
        left = self.signal.length

        while left > 0:
            if self.signal.byte_order == "big_endian":
                if left >= (pos + 1):
                    length = pos + 1
                    pos = 7
                    shift = -(left - length)
                    mask = (1 << length) - 1
                else:
                    length = left
                    shift = pos - length + 1
                    mask = (1 << length) - 1
                    mask <<= pos - length + 1
            else:
                shift = (left - self.signal.length) + pos

                if left >= (8 - pos):
                    length = 8 - pos
                    mask = (1 << length) - 1
                    mask <<= pos
                    pos = 0
                else:
                    length = left
                    mask = (1 << length) - 1
                    mask <<= pos

            if invert_shift:
                if shift < 0:
                    shift = -shift
                    shift_direction = "left"
                else:
                    shift_direction = "right"
            else:
                if shift < 0:
                    shift = -shift
                    shift_direction = "right"
                else:
                    shift_direction = "left"

            yield index, shift, shift_direction, mask

            left -= length
            index += 1


class CodeGenMessage:
    def __init__(self, message: Message) -> None:
        self.message = message
        self.snake_name = camel_to_snake_case(message.name)
        self.cg_signals = [CodeGenSignal(signal) for signal in message.signals]

    def get_signal_by_name(self, name: str) -> CodeGenSignal:
        for cg_signal in self.cg_signals:
            if cg_signal.signal.name == name:
                return cg_signal
        raise KeyError(f"Signal {name} not found.")


def build_yaml_lookup(
    value_tables: dict[str, ValueTable],
    signal_mappings: list[SignalMapping],
    dbc_filename: str,
) -> TYamlLookup:
    """Build a (message_name, signal_name) -> table_name lookup for one DBC file."""
    lookup: TYamlLookup = {}
    for mapping in signal_mappings:
        if mapping.dbc == dbc_filename:
            key = (mapping.message, mapping.signal)
            lookup[key] = mapping.value_table
    return lookup


def generate_common_choice_enums(
    value_tables: dict[str, ValueTable], signal_mappings: list[SignalMapping]
) -> tuple[str, set[str]]:
    """Generate enum typedefs for value tables referenced by more than one DBC file.

    These shared types belong in common.h so they are defined exactly once.

    Returns:
        (generated_code, shared_enum_snake_names) where shared_enum_snake_names
        is the set of snake_case enum names that were emitted, so per-DBC
        generators can skip them via the skip_enums parameter.
    """
    from collections import defaultdict

    table_to_dbcs: dict[str, set[str]] = defaultdict(set)
    for mapping in signal_mappings:
        table_to_dbcs[mapping.value_table].add(mapping.dbc)

    shared_table_names = sorted(
        table_name for table_name, dbcs in table_to_dbcs.items() if len(dbcs) > 1
    )

    shared_enum_names: set[str] = set()
    enums: list[str] = []

    for table_name in shared_table_names:
        entries = value_tables.get(table_name, {})
        enum_name = camel_to_snake_case(table_name)
        shared_enum_names.add(enum_name)

        members = "".join(
            f"\t{enum_name.upper()}_{camel_to_snake_case(label).upper()} = {value},\n"
            for value, label in sorted(entries.items())
        )
        enums.append(ENUM_FMT.format(members=members, enum_name=enum_name))

    return "\n\n".join(enums), shared_enum_names


def is_sender(cg_message: CodeGenMessage, node_names: list[str] | None) -> bool:
    return node_names is None or any(
        [node_name in cg_message.message.senders] for node_name in node_names
    )


def is_receiver(cg_signal: CodeGenSignal, node_names: list[str] | None) -> bool:
    return node_names is None or any(
        [node_name in cg_signal.signal.receivers] for node_name in node_names
    )


def is_sender_or_receiver(
    cg_message: CodeGenMessage, node_names: list[str] | None
) -> bool:
    if is_sender(cg_message, node_names):
        return True
    return any(
        is_receiver(cg_signal, node_names) for cg_signal in cg_message.cg_signals
    )


def generate_h_includes() -> str:
    includes = [
        "#include <stdint.h>",
        "#include <stdlib.h>",
        "#include <stdbool.h>",
        "#include <string.h>",
    ]
    return "\n".join(includes)


def generate_c_includes(h_name: str) -> str:
    includes = [
        f'#include "{h_name}"',
        "#include <math.h>",
    ]
    return "\n".join(includes)


def generate_message_id_enums(db_name: str, cg_messages: list[CodeGenMessage]):
    db_snake = camel_to_snake_case(db_name)
    enum_name = f"{db_snake}_msg_id"
    members = "".join(
        [
            f"\t{db_snake.upper()}_MSG_ID_{msg.snake_name.upper()} = 0x{msg.message.frame_id:X},\n"
            for msg in cg_messages
        ]
        + [f"{db_snake.upper()}_MSG_ID_FORCE_SIZE_U32_ = 0x7FFFFFFF"]
    )
    return ENUM_FMT.format(members=members, enum_name=enum_name)


def generate_message_cycle_time_defines(
    db_name: str, cg_messages: list[CodeGenMessage], node_names: list[str] | None
):
    db_snake = camel_to_snake_case(db_name)

    return "\n".join(
        [
            f"#define {db_snake.upper()}_MSG_{msg.snake_name.upper()}_CYCLE_TIME_MS {msg.message.cycle_time}"
            for msg in cg_messages
            if msg.message.cycle_time is not None
            and is_sender_or_receiver(msg, node_names)
        ]
    )


def format_choice_enum_members(
    cg_message: CodeGenMessage, cg_signal: CodeGenSignal
) -> str:
    members: list[str] = []

    for value, name in sorted(cg_signal.unique_choices.items()):
        fmt = "\t{signal_name}_{name} = {value},\n"
        members.append(
            fmt.format(
                message_name=cg_message.snake_name.upper(),
                signal_name=cg_signal.snake_name.upper(),
                name=str(name),
                value=value,
            )
        )

    return "".join(members)


def generate_choice_enums(
    cg_messages: list[CodeGenMessage],
    node_names: list[str] | None,
    yaml_lookup: "TYamlLookup | None" = None,
    skip_enums: "set[str] | None" = None,
) -> str:
    """Generate choice enums for all signals in cg_messages.

    If yaml_lookup is provided, signals that appear in the YAML get the
    table name from the YAML as their enum name — this is the dedup key
    that replaces the old file-based registry.  Two signals in the same
    DBC that share a table name will produce only one enum typedef.

    Signals not found in the YAML fall back to the signal's snake_name
    (original behaviour), deduplicated within this invocation only.

    skip_enums: set of snake_case enum names already emitted into common.h;
        any enum whose resolved name appears here is silently omitted so the
        type is not defined twice.
    """
    if yaml_lookup is None:
        yaml_lookup = {}
    if skip_enums is None:
        skip_enums = set()

    choice_enums: list[str] = []
    enums_emitted: set[str] = set()

    for msg in cg_messages:
        for cg_signal in msg.cg_signals:
            choices = cg_signal.signal.conversion.choices
            if choices is None:
                continue
            if not is_sender(msg, node_names) and not is_receiver(
                cg_signal, node_names
            ):
                continue

            # Resolve enum name: YAML table name takes priority over signal name.
            yaml_name = yaml_lookup.get((msg.message.name, cg_signal.signal.name))
            if yaml_name is not None:
                enum_name = camel_to_snake_case(yaml_name)
            else:
                base_name = cg_signal.snake_name
                enum_name = base_name
                suffix = 1
                while enum_name in enums_emitted:
                    enum_name = f"{base_name}_{suffix}"
                    suffix += 1

            # Skip enums already written to common.h.
            if enum_name in skip_enums:
                continue

            if enum_name in enums_emitted:
                continue

            enums_emitted.add(enum_name)
            members = format_choice_enum_members(msg, cg_signal)
            choice_enums.append(ENUM_FMT.format(members=members, enum_name=enum_name))

    return "\n\n".join(choice_enums)


def format_comment(comment: str | None) -> str:
    if comment:
        return (
            "\n".join(["     * " + line.rstrip() for line in comment.splitlines()])
            + "\n     *\n"
        )
    else:
        return ""


def format_range(cg_signal: CodeGenSignal) -> str:
    minimum = cg_signal.signal.minimum
    maximum = cg_signal.signal.maximum

    def phys_to_raw(x: int | float) -> int | float:
        raw_val = cg_signal.signal.scaled_to_raw(x)
        if cg_signal.signal.is_float:
            return float(raw_val)
        return round(raw_val)

    if minimum is not None and maximum is not None:
        return (
            f"{phys_to_raw(minimum)}.."
            f"{phys_to_raw(maximum)} "
            f"({round(minimum, 5)}..{round(maximum, 5)} {cg_signal.unit})"
        )
    elif minimum is not None:
        return f"{phys_to_raw(minimum)}.. ({round(minimum, 5)}.. {cg_signal.unit})"
    elif maximum is not None:
        return f"..{phys_to_raw(maximum)} (..{round(maximum, 5)} {cg_signal.unit})"
    else:
        return "-"


def generate_signal(cg_signal: CodeGenSignal, bit_fields: bool) -> str:
    comment = format_comment(cg_signal.signal.comment)
    range_ = format_range(cg_signal)
    scale = get(cg_signal.signal.conversion.scale, "-")
    offset = get(cg_signal.signal.conversion.offset, "-")

    if cg_signal.signal.conversion.is_float or not bit_fields:
        length = ""
    else:
        length = f" : {cg_signal.signal.length}"

    member = SIGNAL_MEMBER_FMT.format(
        comment=comment,
        range=range_,
        scale=scale,
        offset=offset,
        type_name=cg_signal.type_name,
        name=cg_signal.snake_name,
        length=length,
    )

    return member


def generate_struct(
    cg_message: CodeGenMessage, bit_fields: bool
) -> tuple[str, list[str]]:
    members: list[str] = []

    for cg_signal in cg_message.cg_signals:
        members.append(generate_signal(cg_signal, bit_fields))

    if not members:
        members = [
            "\t/**\n" "\t* Dummy signal in empty message.\n" "\t*/\n" "\tuint8_t dummy;"
        ]

    if cg_message.message.comment is None:
        comment = ""
    else:
        comment = f" * {cg_message.message.comment}\n *\n"

    return comment, members


def generate_structs(
    database_name: str,
    cg_messages: list[CodeGenMessage],
    bit_fields: bool,
    node_names: list[str] | None,
) -> str:
    structs: list[str] = []

    for cg_message in cg_messages:
        if is_sender_or_receiver(cg_message, node_names):
            comment, members = generate_struct(cg_message, bit_fields)
            structs.append(
                STRUCT_FMT.format(
                    comment=comment,
                    database_message_name=cg_message.message.name,
                    message_name=cg_message.snake_name,
                    database_name=database_name,
                    members="\n\n".join(members),
                )
            )

    return "\n".join(structs)


def get_floating_point_type(use_float: bool) -> str:
    return "float" if use_float else "double"


def generate_is_in_range(cg_signal: CodeGenSignal) -> str:
    """Generate range checks for all signals in given message."""
    minimum = cg_signal.signal.minimum
    maximum = cg_signal.signal.maximum

    if minimum is not None:
        minimum = cg_signal.signal.scaled_to_raw(minimum)

    if maximum is not None:
        maximum = cg_signal.signal.scaled_to_raw(maximum)

    if minimum is None and cg_signal.minimum_can_raw_value is not None:
        if cg_signal.minimum_ctype_value is None:
            minimum = cg_signal.minimum_can_raw_value
        elif cg_signal.minimum_can_raw_value > cg_signal.minimum_ctype_value:
            minimum = cg_signal.minimum_can_raw_value

    if maximum is None and cg_signal.maximum_can_raw_value is not None:
        if cg_signal.maximum_ctype_value is None:
            maximum = cg_signal.maximum_can_raw_value
        elif cg_signal.maximum_can_raw_value < cg_signal.maximum_ctype_value:
            maximum = cg_signal.maximum_can_raw_value

    suffix = cg_signal.type_suffix
    check: list[str] = []

    if minimum is not None:
        if not cg_signal.signal.conversion.is_float:
            minimum = round(minimum)
        else:
            minimum = float(minimum)

        minimum_ctype_value = cg_signal.minimum_ctype_value

        if (minimum_ctype_value is None) or (minimum > minimum_ctype_value):
            check.append(f"(value >= {minimum}{suffix})")

    if maximum is not None:
        if not cg_signal.signal.conversion.is_float:
            maximum = round(maximum)
        else:
            maximum = float(maximum)

        maximum_ctype_value = cg_signal.maximum_ctype_value

        if (maximum_ctype_value is None) or (maximum < maximum_ctype_value):
            check.append(f"(value <= {maximum}{suffix})")

    if not check:
        check = ["true"]
    elif len(check) == 1:
        check = [check[0][1:-1]]

    return " && ".join(check)


def generate_helpers_kind(
    kinds: set[THelperKind], left_format: str, right_format: str
) -> list[str]:
    formats = {"left": left_format, "right": right_format}
    helpers: list[str] = []

    for shift_direction, length in sorted(kinds):
        var_type = f"uint{length}_t"
        helper = formats[shift_direction].format(length=length, var_type=var_type)
        helpers.append(helper)

    return helpers


def generate_helpers(kinds: tuple[set[THelperKind], set[THelperKind]]) -> str:
    pack_helpers = generate_helpers_kind(
        kinds[0], PACK_HELPER_LEFT_SHIFT_FMT, PACK_HELPER_RIGHT_SHIFT_FMT
    )
    unpack_helpers = generate_helpers_kind(
        kinds[1], UNPACK_HELPER_LEFT_SHIFT_FMT, UNPACK_HELPER_RIGHT_SHIFT_FMT
    )
    helpers = pack_helpers + unpack_helpers

    if helpers:
        helpers.append("")

    return "\n".join(helpers)


def strip_blank_lines(lines: list[str]) -> list[str]:
    try:
        while lines[0] == "":
            lines = lines[1:]

        while lines[-1] == "":
            lines = lines[:-1]
    except IndexError:
        pass

    return lines


def format_pack_code_mux(
    cg_message: CodeGenMessage,
    mux: dict[str, dict[int, list[str]]],
    body_lines_per_index: list[str],
    variable_lines: list[str],
    helper_kinds: set[THelperKind],
) -> list[str]:
    signal_name, multiplexed_signals = next(iter(mux.items()))
    format_pack_code_signal(
        cg_message, signal_name, body_lines_per_index, variable_lines, helper_kinds
    )
    multiplexed_signals_per_id = sorted(multiplexed_signals.items())
    signal_name = camel_to_snake_case(signal_name)

    lines = ["", f"switch (src_p->{signal_name}) {{"]

    for multiplexer_id, signals_of_multiplexer_id in multiplexed_signals_per_id:
        body_lines = format_pack_code_level(
            cg_message, signals_of_multiplexer_id, variable_lines, helper_kinds
        )
        lines.append("")
        lines.append(f"case {multiplexer_id}:")

        if body_lines:
            lines.extend(body_lines[1:-1])

        lines.append("    break;")

    lines.extend(["", "default:", "    break;", "}"])

    return [("    " + line).rstrip() for line in lines]


def format_pack_code_signal(
    cg_message: CodeGenMessage,
    signal_name: str,
    body_lines: list[str],
    variable_lines: list[str],
    helper_kinds: set[THelperKind],
) -> None:
    cg_signal = cg_message.get_signal_by_name(signal_name)

    if cg_signal.signal.conversion.is_float or cg_signal.signal.is_signed:
        variable = f"    uint{cg_signal.type_length}_t {cg_signal.snake_name};"

        if cg_signal.signal.conversion.is_float:
            conversion = f"    memcpy(&{cg_signal.snake_name}, &src_p->{cg_signal.snake_name}, sizeof({cg_signal.snake_name}));"
        else:
            conversion = f"    {cg_signal.snake_name} = (uint{cg_signal.type_length}_t)src_p->{cg_signal.snake_name};"

        variable_lines.append(variable)
        body_lines.append(conversion)

    for index, shift, shift_direction, mask in cg_signal.segments(invert_shift=False):
        if cg_signal.signal.conversion.is_float or cg_signal.signal.is_signed:
            fmt = "    dst_p[{}] |= pack_{}_shift_u{}({}, {}U, 0x{:02X}U);"
        else:
            fmt = "    dst_p[{}] |= pack_{}_shift_u{}(src_p->{}, {}U, 0x{:02X}U);"

        line = fmt.format(
            index,
            shift_direction,
            cg_signal.type_length,
            cg_signal.snake_name,
            shift,
            mask,
        )
        body_lines.append(line)
        helper_kinds.add((shift_direction, cg_signal.type_length))


def format_pack_code_level(
    cg_message: CodeGenMessage,
    signal_names: list[str] | list[dict[str, dict[int, list[str]]]],
    variable_lines: list[str],
    helper_kinds: set[THelperKind],
) -> list[str]:
    """Format one pack level in a signal tree."""

    body_lines: list[str] = []
    muxes_lines: list[str] = []

    for signal_name in signal_names:
        if isinstance(signal_name, dict):
            mux_lines = format_pack_code_mux(
                cg_message, signal_name, body_lines, variable_lines, helper_kinds
            )
            muxes_lines += mux_lines
        else:
            format_pack_code_signal(
                cg_message, signal_name, body_lines, variable_lines, helper_kinds
            )

    body_lines = body_lines + muxes_lines

    if body_lines:
        body_lines = ["", *body_lines, ""]

    return body_lines


def format_pack_code(
    cg_message: CodeGenMessage, helper_kinds: set[THelperKind]
) -> tuple[str, str]:
    variable_lines: list[str] = []
    body_lines = format_pack_code_level(
        cg_message, cg_message.message.signal_tree, variable_lines, helper_kinds
    )

    if variable_lines:
        variable_lines = [*sorted(set(variable_lines)), "", ""]

    return "\n".join(variable_lines), "\n".join(body_lines)


def format_unpack_code_mux(
    cg_message: CodeGenMessage,
    mux: dict[str, dict[int, list[str]]],
    body_lines_per_index: list[str],
    variable_lines: list[str],
    helper_kinds: set[THelperKind],
    node_names: list[str] | None,
) -> list[str]:
    signal_name, multiplexed_signals = next(iter(mux.items()))
    format_unpack_code_signal(
        cg_message, signal_name, body_lines_per_index, variable_lines, helper_kinds
    )
    multiplexed_signals_per_id = sorted(multiplexed_signals.items())
    signal_name = camel_to_snake_case(signal_name)

    lines = [f"switch (dst_p->{signal_name}) {{"]

    for multiplexer_id, signals_of_multiplexer_id in multiplexed_signals_per_id:
        body_lines = format_unpack_code_level(
            cg_message,
            signals_of_multiplexer_id,
            variable_lines,
            helper_kinds,
            node_names,
        )
        lines.append("")
        lines.append(f"case {multiplexer_id}:")
        lines.extend(strip_blank_lines(body_lines))
        lines.append("    break;")

    lines.extend(["", "default:", "    break;", "}"])

    return [("    " + line).rstrip() for line in lines]


def format_unpack_code_signal(
    cg_message: CodeGenMessage,
    signal_name: str,
    body_lines: list[str],
    variable_lines: list[str],
    helper_kinds: set[THelperKind],
) -> None:
    cg_signal = cg_message.get_signal_by_name(signal_name)
    conversion_type_name = f"uint{cg_signal.type_length}_t"

    if cg_signal.signal.conversion.is_float or cg_signal.signal.is_signed:
        variable = f"    {conversion_type_name} {cg_signal.snake_name};"
        variable_lines.append(variable)

    segments = cg_signal.segments(invert_shift=True)

    for i, (index, shift, shift_direction, mask) in enumerate(segments):
        if cg_signal.signal.conversion.is_float or cg_signal.signal.is_signed:
            fmt = "    {} {} unpack_{}_shift_u{}(src_p[{}], {}U, 0x{:02X}U);"
        else:
            fmt = "    dst_p->{} {} unpack_{}_shift_u{}(src_p[{}], {}U, 0x{:02X}U);"

        line = fmt.format(
            cg_signal.snake_name,
            "=" if i == 0 else "|=",
            shift_direction,
            cg_signal.type_length,
            index,
            shift,
            mask,
        )
        body_lines.append(line)
        helper_kinds.add((shift_direction, cg_signal.type_length))

    if cg_signal.signal.conversion.is_float:
        conversion = f"    memcpy(&dst_p->{cg_signal.snake_name}, &{cg_signal.snake_name}, sizeof(dst_p->{cg_signal.snake_name}));"
        body_lines.append(conversion)
    elif cg_signal.signal.is_signed:
        mask = (1 << (cg_signal.type_length - cg_signal.signal.length)) - 1

        if mask != 0:
            mask <<= cg_signal.signal.length
            formatted = SIGN_EXTENSION_FMT.format(
                name=cg_signal.snake_name,
                shift=cg_signal.signal.length - 1,
                mask=mask,
                suffix=cg_signal.conversion_type_suffix,
            )
            body_lines.extend(formatted.splitlines())

        conversion = f"    dst_p->{cg_signal.snake_name} = (int{cg_signal.type_length}_t){cg_signal.snake_name};"
        body_lines.append(conversion)


def format_unpack_code_level(
    cg_message: CodeGenMessage,
    signal_names: list[str] | list[dict[str, dict[int, list[str]]]],
    variable_lines: list[str],
    helper_kinds: set[THelperKind],
    node_names: list[str] | None,
) -> list[str]:
    """Format one unpack level in a signal tree."""

    body_lines: list[str] = []
    muxes_lines: list[str] = []

    for signal_name in signal_names:
        if isinstance(signal_name, dict):
            mux_lines = format_unpack_code_mux(
                cg_message,
                signal_name,
                body_lines,
                variable_lines,
                helper_kinds,
                node_names,
            )

            if muxes_lines:
                muxes_lines.append("")

            muxes_lines += mux_lines
        else:
            if not is_receiver(cg_message.get_signal_by_name(signal_name), node_names):
                continue

            format_unpack_code_signal(
                cg_message, signal_name, body_lines, variable_lines, helper_kinds
            )

    if body_lines:
        if body_lines[-1] != "":
            body_lines.append("")

    if muxes_lines:
        muxes_lines.append("")

    body_lines = body_lines + muxes_lines

    if body_lines:
        body_lines = ["", *body_lines]

    return body_lines


def format_unpack_code(
    cg_message: CodeGenMessage,
    helper_kinds: set[THelperKind],
    node_names: list[str] | None,
) -> tuple[str, str]:
    variable_lines: list[str] = []
    body_lines = format_unpack_code_level(
        cg_message,
        cg_message.message.signal_tree,
        variable_lines,
        helper_kinds,
        node_names,
    )

    if variable_lines:
        variable_lines = [*sorted(set(variable_lines)), "", ""]

    return "\n".join(variable_lines), "\n".join(body_lines)


def generate_encode_decode(
    cg_signal: CodeGenSignal, use_float: bool, use_round: bool
) -> tuple[str, str]:
    floating_point_type = get_floating_point_type(use_float)

    scale = cg_signal.signal.scale
    offset = cg_signal.signal.offset

    scale_literal = (
        f"{scale}{'.0' if isinstance(scale, int) else ''}{'F' if use_float else ''}"
    )
    offset_literal = (
        f"{offset}{'.0' if isinstance(offset, int) else ''}{'F' if use_float else ''}"
    )

    if offset == 0 and scale == 1:
        encoding = "value"
        decoding = f"({floating_point_type})value"
    elif offset != 0 and scale != 1:
        encoding = f"(value - {offset_literal}) / {scale_literal}"
        decoding = (
            f"(({floating_point_type})value * {scale_literal}) + {offset_literal}"
        )
    elif offset != 0:
        encoding = f"value - {offset_literal}"
        decoding = f"({floating_point_type})value + {offset_literal}"
    else:
        encoding = f"value / {scale_literal}"
        decoding = f"({floating_point_type})value * {scale_literal}"

    if not cg_signal.signal.is_float and use_round:
        encoding = f'round{"f" if use_float else ""}({encoding})'

    return encoding, decoding


def generate_dlcs(database_name: str, cg_messages: list[CodeGenMessage]):
    defines: list[str] = []
    for cg_message in cg_messages:
        define = MESSAGE_DLC_DEFINE_FMT.format(
            db_name=database_name.upper(),
            msg_name=cg_message.snake_name.upper(),
            dlc=cg_message.message.length,
        )
        defines.append(define)
    return "\n".join(defines)


def generate_definitions(
    database_name: str,
    cg_messages: list[CodeGenMessage],
    floating_point_numbers: bool,
    use_float: bool,
    node_names: list[str] | None,
    use_round: bool,
) -> tuple[str, str, tuple[set[THelperKind], set[THelperKind]]]:
    definitions: list[str] = []
    definition_protos: list[str] = []
    pack_helper_kinds: set[THelperKind] = set()
    unpack_helper_kinds: set[THelperKind] = set()

    for cg_message in cg_messages:
        signal_definitions: list[str] = []
        signal_protos: list[str] = []
        sender = is_sender(cg_message, node_names)
        receiver = node_names is None or len(node_names) == 0
        signals_init_body = ""

        for cg_signal in cg_message.cg_signals:
            if use_float and cg_signal.type_name == "double":
                warnings.warn(
                    f"User selected `--use-float`, but database contains "
                    f"signal with data type `double`: "
                    f'"{cg_message.message.name}::{cg_signal.signal.name}"',
                    stacklevel=2,
                )
                _use_float = False
            else:
                _use_float = use_float

            encode, decode = generate_encode_decode(cg_signal, _use_float, use_round)
            check = generate_is_in_range(cg_signal)

            if is_receiver(cg_signal, node_names):
                receiver = True

            if check == "true":
                unused = "    (void)value;\n\n"
            else:
                unused = ""

            signal_definition = ""
            signal_proto = ""

            if floating_point_numbers:
                if sender:
                    signal_definition += SIGNAL_DEFINITION_ENCODE_FMT.format(
                        database_name=database_name,
                        message_name=cg_message.snake_name,
                        signal_name=cg_signal.snake_name,
                        type_name=cg_signal.type_name,
                        encode=encode,
                        floating_point_type=get_floating_point_type(_use_float),
                    )
                    signal_proto += SIGNAL_DEFINITION_ENCODE_PROTO_FMT.format(
                        database_name=database_name,
                        message_name=cg_message.snake_name,
                        signal_name=cg_signal.snake_name,
                        type_name=cg_signal.type_name,
                        floating_point_type=get_floating_point_type(_use_float),
                    )
                if node_names is None or is_receiver(cg_signal, node_names):
                    signal_definition += SIGNAL_DEFINITION_DECODE_FMT.format(
                        database_name=database_name,
                        message_name=cg_message.snake_name,
                        signal_name=cg_signal.snake_name,
                        type_name=cg_signal.type_name,
                        decode=decode,
                        floating_point_type=get_floating_point_type(_use_float),
                    )
                    signal_proto += SIGNAL_DEFINITION_DECODE_PROTO_FMT.format(
                        database_name=database_name,
                        message_name=cg_message.snake_name,
                        signal_name=cg_signal.snake_name,
                        type_name=cg_signal.type_name,
                        floating_point_type=get_floating_point_type(_use_float),
                    )

            if sender or is_receiver(cg_signal, node_names):
                signal_definition += SIGNAL_DEFINITION_IS_IN_RANGE_FMT.format(
                    database_name=database_name,
                    message_name=cg_message.snake_name,
                    signal_name=cg_signal.snake_name,
                    type_name=cg_signal.type_name,
                    unused=unused,
                    check=check,
                )
                signal_proto += SIGNAL_DEFINITION_IS_IN_RANGE_PROTO_FMT.format(
                    database_name=database_name,
                    message_name=cg_message.snake_name,
                    signal_name=cg_signal.snake_name,
                    type_name=cg_signal.type_name,
                    unused=unused,
                    check=check,
                )

                signal_definitions.append(signal_definition)
                signal_protos.append(signal_proto)

            if cg_signal.signal.initial:
                signals_init_body += INIT_SIGNAL_BODY_TEMPLATE_FMT.format(
                    signal_initial=cg_signal.signal.raw_initial,
                    signal_name=cg_signal.snake_name,
                )

        if cg_message.message.length > 0:
            pack_variables, pack_body = format_pack_code(cg_message, pack_helper_kinds)
            unpack_variables, unpack_body = format_unpack_code(
                cg_message, unpack_helper_kinds, node_names
            )
            pack_unused = ""
            unpack_unused = ""

            if not pack_body:
                pack_unused += "    (void)src_p;\n\n"

            if not unpack_body:
                unpack_unused += "    (void)dst_p;\n"
                unpack_unused += "    (void)src_p;\n\n"

            definition = ""
            definition_proto = ""
            if sender:
                definition += DEFINITION_PACK_FMT.format(
                    database_name=database_name,
                    database_message_name=cg_message.message.name,
                    message_name=cg_message.snake_name,
                    message_length=cg_message.message.length,
                    pack_unused=pack_unused,
                    pack_variables=pack_variables,
                    pack_body=pack_body,
                )
                definition_proto += DEFINITION_PACK_PROTO_FMT.format(
                    database_name=database_name,
                    message_name=cg_message.snake_name,
                )
            if receiver:
                definition += DEFINITION_UNPACK_FMT.format(
                    database_name=database_name,
                    database_message_name=cg_message.message.name,
                    message_name=cg_message.snake_name,
                    message_length=cg_message.message.length,
                    unpack_unused=unpack_unused,
                    unpack_variables=unpack_variables,
                    unpack_body=unpack_body,
                )
                definition_proto += DEFINITION_UNPACK_PROTO_FMT.format(
                    database_name=database_name,
                    message_name=cg_message.snake_name,
                )

        else:
            definition = EMPTY_DEFINITION_FMT.format(
                database_name=database_name, message_name=cg_message.snake_name
            )
            definition_proto = EMPTY_DEFINITION_PROTO_FMT.format(
                database_name=database_name, message_name=cg_message.snake_name
            )

        if signal_definitions:
            definition += "\n" + "\n".join(signal_definitions)
        if signal_protos:
            definition_protos.append("\n" + "\n".join(signal_protos))

        if definition:
            definitions.append(definition)
        if definition_proto:
            definition_protos.append(definition_proto)

    return (
        "\n".join(definitions),
        "\n".join(definition_protos),
        (pack_helper_kinds, unpack_helper_kinds),
    )
