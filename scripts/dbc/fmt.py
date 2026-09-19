ENUM_FMT = """\
typedef enum {{
{members} \
}} {enum_name}_t;
"""

STRUCT_FMT = """\
/**
 * Signals in message {database_message_name}.
 *
{comment}\
 * All signal values are as on the CAN bus.
 */
typedef struct {{
{members}
}} {database_name}_{message_name}_t;
"""


SIGNAL_MEMBER_FMT = """\
    /**
{comment}\
     * Range: {range}
     * Scale: {scale}
     * Offset: {offset}
     */
    {type_name} {name}{length};\
"""


SIGNAL_DEFINITION_ENCODE_PROTO_FMT = """{type_name} {database_name}_{message_name}_{signal_name}_encode({floating_point_type} value);"""

SIGNAL_DEFINITION_ENCODE_FMT = """\
{type_name} {database_name}_{message_name}_{signal_name}_encode({floating_point_type} value)
{{
    return ({type_name})({encode});
}}

"""

SIGNAL_DEFINITION_DECODE_PROTO_FMT = """{floating_point_type} {database_name}_{message_name}_{signal_name}_decode({type_name} value);"""

SIGNAL_DEFINITION_DECODE_FMT = """\
{floating_point_type} {database_name}_{message_name}_{signal_name}_decode({type_name} value)
{{
    return ({decode});
}}

"""

SIGNAL_DEFINITION_IS_IN_RANGE_PROTO_FMT = """bool {database_name}_{message_name}_{signal_name}_is_in_range({type_name} value);"""

SIGNAL_DEFINITION_IS_IN_RANGE_FMT = """\
bool {database_name}_{message_name}_{signal_name}_is_in_range({type_name} value)
{{
{unused}\
    return ({check});
}}
"""

INIT_SIGNAL_BODY_TEMPLATE_FMT = """\
    msg_p->{signal_name} = {signal_initial};
"""

DEFINITION_PACK_PROTO_FMT = "int {database_name}_{message_name}_pack(uint8_t *dst_p, const {database_name}_{message_name}_t *src_p, size_t size);"

DEFINITION_PACK_FMT = """\
int {database_name}_{message_name}_pack(
    uint8_t *dst_p,
    const {database_name}_{message_name}_t *src_p,
    size_t size)
{{
{pack_unused}\
{pack_variables}\
    if (size < {message_length}U) {{
        return -1;
    }}

    memset(&dst_p[0], 0, {message_length});
{pack_body}
    return ({message_length});
}}

"""

DEFINITION_UNPACK_PROTO_FMT = "int {database_name}_{message_name}_unpack({database_name}_{message_name}_t *dst_p, const uint8_t *src_p, size_t size);"

DEFINITION_UNPACK_FMT = """\
int {database_name}_{message_name}_unpack(
    {database_name}_{message_name}_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{{
{unpack_unused}\
{unpack_variables}\
    if (size < {message_length}U) {{
        return -1;
    }}
{unpack_body}
    return (0);
}}

"""

EMPTY_DEFINITION_PROTO_FMT = "int {database_name}_{message_name}_pack(uint8_t *dst_p, const {database_name}_{message_name}_t *src_p, size_t size);"

EMPTY_DEFINITION_FMT = """\
int {database_name}_{message_name}_pack(
    uint8_t *dst_p,
    const {database_name}_{message_name}_t *src_p,
    size_t size)
{{
    (void)dst_p;
    (void)src_p;
    (void)size;

    return (0);
}}
"""

SIGN_EXTENSION_FMT = """
    if (({name} & (1{suffix} << {shift})) != 0{suffix}) {{
        {name} |= 0x{mask:x}{suffix};
    }}

"""

PACK_HELPER_LEFT_SHIFT_FMT = """\
static inline uint8_t pack_left_shift_u{length}(
    {var_type} value,
    uint8_t shift,
    uint8_t mask)
{{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}}
"""

PACK_HELPER_RIGHT_SHIFT_FMT = """\
static inline uint8_t pack_right_shift_u{length}(
    {var_type} value,
    uint8_t shift,
    uint8_t mask)
{{
    return (uint8_t)((uint8_t)(value >> shift) & mask);
}}
"""

UNPACK_HELPER_LEFT_SHIFT_FMT = """\
static inline {var_type} unpack_left_shift_u{length}(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{{
    return ({var_type})(({var_type})(value & mask) << shift);
}}
"""

UNPACK_HELPER_RIGHT_SHIFT_FMT = """\
static inline {var_type} unpack_right_shift_u{length}(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{{
    return ({var_type})(({var_type})(value & mask) >> shift);
}}
"""

MESSAGE_DLC_DEFINE_FMT = """\
#define {db_name}_{msg_name}_DLC {dlc}
"""
