import re
from typing import TypeVar

_T1 = TypeVar("_T1")
_T2 = TypeVar("_T2")


def canonical(ident: str) -> str:
    """Replace anything but 'a-z', 'A-Z' and '0-9' with '_'."""

    return re.sub(r"[^a-zA-Z0-9]", "_", ident)


def camel_to_snake_case(ident: str) -> str:
    ident = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", ident)
    ident = re.sub(r"(_+)", "_", ident)
    ident = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", ident).lower()
    ident = canonical(ident)

    return ident


def get(value: _T1 | None, default: _T2) -> _T1 | _T2:
    if value is None:
        return default
    return value
