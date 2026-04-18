#!/usr/bin/env python3
#
# (C) DS prototyp 2024
# Inject binary data as a C initializer into an existing C/C++ source file.
#
# Example: resource_inject src/resources.c bin/textures.raw -n texture_data -t 16le -c 8

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path


TYPE_INFO = {
    "8": (1, None, "uint8_t"),
    "16le": (2, "little", "uint16_t"),
    "16be": (2, "big", "uint16_t"),
    "32le": (4, "little", "uint32_t"),
    "32be": (4, "big", "uint32_t"),
    "64le": (8, "little", "uint64_t"),
    "64be": (8, "big", "uint64_t"),
}


@dataclass
class VarEntry:
    name: str
    tail: str
    injected: bool = False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert a binary file into a C initializer and inject it into a target source file."
    )
    parser.add_argument("target", help="Target C/C++ source file to patch")
    parser.add_argument("sources", nargs="*", help="Binary input file(s)")
    parser.add_argument(
        "-n",
        "--name",
        default="*",
        help="Symbol name or template. '*' expands to sanitized filename and '#' to zero-based source index",
    )
    parser.add_argument(
        "-t",
        "--type",
        default="8",
        choices=tuple(TYPE_INFO.keys()),
        help="Element type/endianness. Default: 8",
    )
    parser.add_argument(
        "-c",
        "--count",
        type=int,
        help="Values per line. Use 0 for a single line. Default: 16 for 8/16, 8 for 32, 4 for 64",
    )
    parser.add_argument(
        "-o",
        "--output",
        help="Write patched output to a separate file instead of editing the target in place",
    )
    parser.add_argument(
        "--no-bak",
        action="store_true",
        help="Do not keep a .bak backup when editing the target in place",
    )
    parser.add_argument(
        "--symbols",
        action="store_true",
        help="List injectable symbols found in the target file and exit",
    )
    return parser.parse_args()


def sanitize_name(path: Path) -> str:
    name = re.sub(r"[^0-9A-Za-z_]", "_", path.name)
    name = re.sub(r"_+", "_", name).strip("_")
    if not name:
        name = "data"
    if name[0].isdigit():
        name = "_" + name
    return name


def resolve_name_template(template: str, source_path: Path, index: int) -> str:
    default_name = sanitize_name(source_path)
    if "*" in template or "#" in template:
        return template.replace("*", default_name).replace("#", str(index))
    return template


def default_count(type_name: str) -> int:
    width, _, _ = TYPE_INFO[type_name]
    if width <= 2:
        return 16
    if width == 4:
        return 8
    return 4


def read_values(path: Path, type_name: str) -> list[int]:
    width, byteorder, _ = TYPE_INFO[type_name]
    blob = path.read_bytes()

    if len(blob) % width != 0:
        raise ValueError(
            f"{path} size {len(blob)} is not divisible by {width} for type {type_name}"
        )

    if width == 1:
        return list(blob)

    return [
        int.from_bytes(blob[i : i + width], byteorder=byteorder, signed=False)
        for i in range(0, len(blob), width)
    ]


def format_value(value: int, width: int) -> str:
    digits = width * 2
    return f"0x{value:0{digits}X}"


def build_source(name: str, values: list[int], type_name: str, values_per_line: int) -> str:
    width, _, c_type = TYPE_INFO[type_name]
    count = len(values)
    if values_per_line == 0:
        rendered = ", ".join(format_value(value, width) for value in values)
        return f"{c_type} {name}[{count}] = {{ {rendered} }}\n"

    lines = [f"{c_type} {name}[{count}] = {{\n"]

    if values:
        for i in range(0, count, values_per_line):
            chunk = values[i : i + values_per_line]
            rendered_values = []
            for j, value in enumerate(chunk):
                is_last = (i + j) == (count - 1)
                suffix = "" if is_last else ","
                rendered_values.append(f"{format_value(value, width)}{suffix}")
            rendered = " ".join(rendered_values)
            lines.append(f"\t{rendered}\n")

    # Do not include the trailing semicolon here. The inject-style parser
    # flushes the declaration block at the closing brace and preserves the
    # target file's original ';' separately.
    lines.append("}\n")
    return "".join(lines)


def extract_var_name_and_after(text: str, eq_pos: int) -> tuple[str | None, str]:
    if eq_pos <= 0 or eq_pos > len(text):
        return None, ""

    i = eq_pos - 1
    while i >= 0 and text[i].isspace():
        i -= 1

    if i >= 0 and text[i] == "]":
        bracket_depth = 1
        i -= 1
        while i >= 0 and bracket_depth > 0:
            if text[i] == "]":
                bracket_depth += 1
            elif text[i] == "[":
                bracket_depth -= 1
            i -= 1
        while i >= 0 and text[i].isspace():
            i -= 1

    end = i
    while i >= 0 and (text[i].isalnum() or text[i] == "_"):
        i -= 1

    start = i + 1
    if start > end:
        return None, ""

    name = text[start : end + 1]
    return name, text[end + 1 :]


def parse_block(text: str, eq_pos: int, stage2: bool, entries: list[VarEntry]) -> str:
    if eq_pos < 0:
        return text if stage2 else ""

    name, after = extract_var_name_and_after(text, eq_pos)
    if not name:
        return text if stage2 else ""

    if not stage2:
        for entry in entries:
            if entry.name == name:
                raise ValueError(f"duplicate source symbol: {name}")
        entries.append(VarEntry(name=name, tail=after))
        print(f"[FOUND] {name}")
        return ""

    for entry in entries:
        if entry.name == name:
            if entry.injected:
                raise ValueError(f"duplicate target symbol: {name}")
            entry.injected = True
            print(f"[INJECTED] {name}")
            prefix_len = len(text) - len(after)
            return text[:prefix_len] + entry.tail

    return text


def scan_stream(text: str, stage2: bool, entries: list[VarEntry]) -> str:
    prev = ""
    depth = 0
    in_string = False
    in_char = False
    escape = False
    in_sl_comment = False
    in_ml_comment = False
    in_preproc = False
    start_of_line = True
    eq_pos = -1
    buf: list[str] = []
    output: list[str] = []

    def flush_block() -> None:
        nonlocal buf, eq_pos
        block = "".join(buf)
        if stage2:
            output.append(parse_block(block, eq_pos, True, entries))
        else:
            parse_block(block, eq_pos, False, entries)
        buf = []
        eq_pos = -1

    for ch in text:
        if ch == "\r":
            continue

        buf.append(ch)

        if ch == "\n" and prev != "\\":
            start_of_line = True
            in_sl_comment = False
            in_preproc = False
            prev = ch
            continue
        elif start_of_line and not ch.isspace():
            start_of_line = False

        if in_preproc:
            prev = ch
            continue

        if in_sl_comment or in_ml_comment:
            if in_ml_comment and prev == "*" and ch == "/":
                in_ml_comment = False
            prev = ch
            continue

        if start_of_line and ch == "#":
            in_preproc = True

        if not in_string and not in_char:
            if prev == "/" and ch == "/":
                in_sl_comment = True
                prev = ch
                continue
            if prev == "/" and ch == "*":
                in_ml_comment = True
                prev = ch
                continue

        if in_string or in_char:
            if escape:
                escape = False
            elif ch == "\\":
                escape = True
            elif (in_string and ch == '"') or (in_char and ch == "'"):
                in_string = False
                in_char = False
            prev = ch
            continue
        else:
            if ch == '"':
                in_string = True
                prev = ch
                continue
            if ch == "'":
                in_char = True
                prev = ch
                continue

        if ch == "=":
            if depth == 0 and eq_pos < 0:
                eq_pos = len(buf) - 1
        elif ch in "{([":
            depth += 1
        elif ch in "})]":
            if depth > 0:
                depth -= 1
            else:
                raise ValueError("code structure")
            if ch != "}":
                prev = ch
                continue
            if depth == 0:
                flush_block()
        elif ch == ";" and depth == 0:
            flush_block()

        prev = ch

    tail = "".join(buf)
    if stage2:
        output.append(tail)

    return "".join(output)


def list_symbols(text: str) -> list[str]:
    symbols: list[str] = []
    seen: set[str] = set()
    prev = ""
    depth = 0
    in_string = False
    in_char = False
    escape = False
    in_sl_comment = False
    in_ml_comment = False
    in_preproc = False
    start_of_line = True
    eq_pos = -1
    buf: list[str] = []

    def flush_block() -> None:
        nonlocal buf, eq_pos
        block = "".join(buf)
        if eq_pos >= 0:
            name, _after = extract_var_name_and_after(block, eq_pos)
            if name and name not in seen:
                seen.add(name)
                symbols.append(name)
        buf = []
        eq_pos = -1

    for ch in text:
        if ch == "\r":
            continue

        buf.append(ch)

        if ch == "\n" and prev != "\\":
            start_of_line = True
            in_sl_comment = False
            in_preproc = False
            prev = ch
            continue
        elif start_of_line and not ch.isspace():
            start_of_line = False

        if in_preproc:
            prev = ch
            continue

        if in_sl_comment or in_ml_comment:
            if in_ml_comment and prev == "*" and ch == "/":
                in_ml_comment = False
            prev = ch
            continue

        if start_of_line and ch == "#":
            in_preproc = True

        if not in_string and not in_char:
            if prev == "/" and ch == "/":
                in_sl_comment = True
                prev = ch
                continue
            if prev == "/" and ch == "*":
                in_ml_comment = True
                prev = ch
                continue

        if in_string or in_char:
            if escape:
                escape = False
            elif ch == "\\":
                escape = True
            elif (in_string and ch == '"') or (in_char and ch == "'"):
                in_string = False
                in_char = False
            prev = ch
            continue
        else:
            if ch == '"':
                in_string = True
                prev = ch
                continue
            if ch == "'":
                in_char = True
                prev = ch
                continue

        if ch == "=":
            if depth == 0 and eq_pos < 0:
                eq_pos = len(buf) - 1
        elif ch in "{([":
            depth += 1
        elif ch in "})]":
            if depth > 0:
                depth -= 1
            else:
                raise ValueError("code structure")
            if ch != "}":
                prev = ch
                continue
            if depth == 0:
                flush_block()
        elif ch == ";" and depth == 0:
            flush_block()

        prev = ch

    return symbols


def detect_newline(data: bytes) -> str:
    crlf = data.count(b"\r\n")
    lf = data.count(b"\n") - crlf
    cr = data.count(b"\r") - crlf

    if crlf >= lf and crlf >= cr and crlf > 0:
        return "\r\n"
    if lf >= cr and lf > 0:
        return "\n"
    if cr > 0:
        return "\r"
    return "\n"


def resolve_target_relative(base_target: Path, candidate: str) -> Path:
    path = Path(candidate)
    if path.is_absolute():
        return path
    if path.exists():
        return path
    target_relative = base_target.parent / path
    if target_relative.exists():
        return target_relative
    return target_relative


def write_result(
    target: Path, patched_text: str, output_path: str | None, keep_backup: bool
) -> None:
    if output_path:
        resolve_target_relative(target, output_path).write_text(
            patched_text, encoding="utf-8", newline="\n"
        )
        return

    tmp_path = target.with_name(target.name + ".tmp")
    bak_path = target.with_name(target.name + ".bak")

    tmp_path.write_text(patched_text, encoding="utf-8", newline="\n")
    if keep_backup:
        bak_path.unlink(missing_ok=True)
        target.replace(bak_path)
        try:
            tmp_path.replace(target)
        except Exception:
            bak_path.replace(target)
            raise
        print(f"Backed up original as {bak_path}")
    else:
        tmp_path.replace(target)


def main() -> int:
    args = parse_args()

    if args.count is not None and args.count < 0:
        print("count must be >= 0", file=sys.stderr)
        return 1

    target_path = Path(args.target)
    values_per_line = args.count if args.count is not None else default_count(args.type)

    if args.symbols:
        if args.sources:
            print("--symbols does not take source files", file=sys.stderr)
            return 1
    else:
        if not args.sources:
            print("must provide at least one source file", file=sys.stderr)
            return 1
        if args.name and len(args.sources) != 1 and "*" not in args.name and "#" not in args.name:
            print("--name must contain '*' or '#' when used with multiple source files", file=sys.stderr)
            return 1

    try:
        target_bytes = target_path.read_bytes()
        target_newline = detect_newline(target_bytes)
        target_text = target_bytes.decode("utf-8").replace("\r\n", "\n").replace("\r", "\n")
    except Exception as exc:
        print(exc, file=sys.stderr)
        return 1

    if args.symbols:
        try:
            for symbol in list_symbols(target_text):
                print(symbol)
        except Exception as exc:
            print(exc, file=sys.stderr)
            return 1
        return 0

    entries: list[VarEntry] = []

    try:
        for index, source_arg in enumerate(args.sources):
            source_path = resolve_target_relative(target_path, source_arg)
            name = resolve_name_template(args.name, source_path, index)
            values = read_values(source_path, args.type)
            generated_source = build_source(name, values, args.type, values_per_line)
            scan_stream(generated_source, False, entries)
        patched_text = scan_stream(target_text, True, entries).replace("\n", target_newline)
    except Exception as exc:
        print(exc, file=sys.stderr)
        return 1

    missing = [entry.name for entry in entries if not entry.injected]
    if missing:
        for missing_name in missing:
            print(f"[FAILED] {missing_name}")
        return 1

    try:
        write_result(target_path, patched_text, args.output, keep_backup=not args.no_bak)
    except Exception as exc:
        print(exc, file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
