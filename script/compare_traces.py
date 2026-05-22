#!/usr/bin/env python3
"""Compare NoCDAS packet traces while ignoring cycle-only differences.

The RTL router is not expected to match the C++ behavioral router cycle by
cycle.  This checker therefore ignores global_pid and path_times by default,
but still checks routing paths, packet types, compute operation, and cNoC
result/hash fields.
"""

from __future__ import annotations

import argparse
import collections
import csv
import sys
from pathlib import Path


max_csv_field_size = sys.maxsize
while True:
    try:
        csv.field_size_limit(max_csv_field_size)
        break
    except OverflowError:
        max_csv_field_size //= 10


DEFAULT_COMPARE_FIELDS = [
    "msg_type",
    "signal_id",
    "source_id",
    "destination",
    "path_nodes",
    "compute_op",
    "cnoc_in_hash",
    "cnoc_out_hash",
    "cnoc_rm",
    "cnoc_rs",
    "cnoc_sample",
]

CNOC_VALUE_FIELDS = {
    "cnoc_in_hash",
    "cnoc_out_hash",
    "cnoc_rm",
    "cnoc_rs",
    "cnoc_sample",
}

PATH_COMPARE_FIELDS = [
    "msg_type",
    "signal_id",
    "source_id",
    "destination",
    "path_nodes",
    "compute_op",
]

VALUE_COMPARE_FIELDS = [
    "msg_type",
    "signal_id",
    "source_id",
    "destination",
    "compute_op",
    "cnoc_in_hash",
    "cnoc_out_hash",
    "cnoc_rm",
    "cnoc_rs",
    "cnoc_sample",
]

LEGACY_SHORT_ROW_FIELDS = [
    "global_pid",
    "msg_type",
    "signal_id",
    "path_nodes",
    "path_times",
    "compute_op",
    "cnoc_in_hash",
    "cnoc_out_hash",
    "cnoc_rm",
    "cnoc_rs",
]

QUANT_TRACE_MARKER = "# cnoc_quant_golden:"
VALID_MSG_TYPES = {"0", "1", "2", "3", "4", "5"}


def normalize_trace_row(path: Path, line: str, fields, values):
    """Normalize older short trace rows to the current header schema.

    Some regular NoCDAS trace rows are still emitted in the pre-source/dest
    shape even when the file header advertises the newer cNoC fields.  Treat
    that as a textual compatibility issue, not as a functional trace mismatch:
    source/destination are left empty and cnoc_sample is marked absent.
    """
    if len(values) == len(fields):
        return dict(zip(fields, values))

    if (
        len(values) == len(LEGACY_SHORT_ROW_FIELDS)
        and "source_id" in fields
        and "destination" in fields
        and "cnoc_sample" in fields
    ):
        legacy_row = dict(zip(LEGACY_SHORT_ROW_FIELDS, values))
        return {
            field: legacy_row.get(field, "-" if field.startswith("cnoc_") else "")
            for field in fields
        }

    raise ValueError(
        f"{path}: row has {len(values)} values but header has {len(fields)} fields: {line}"
    )


def parse_trace(path: Path):
    fields = None
    rows = []
    has_quant_marker = False

    with path.open("r", encoding="utf-8") as trace_file:
        for raw_line in trace_file:
            line = raw_line.strip()
            if not line:
                continue
            if line.startswith(QUANT_TRACE_MARKER):
                has_quant_marker = True
                continue
            if line.startswith("# fields:"):
                fields = [field.strip() for field in line.split(":", 1)[1].split(",")]
                continue
            if line.startswith("#"):
                continue
            if fields is None:
                raise ValueError(f"{path}: missing '# fields:' header before data rows")
            values = next(csv.reader([line]))
            rows.append(normalize_trace_row(path, line, fields, values))

    if fields is None:
        raise ValueError(f"{path}: trace header not found")
    return fields, rows, has_quant_marker


def validate_compare_fields(path: Path, fields, compare_fields):
    missing = [field for field in compare_fields if field not in fields]
    if missing:
        raise ValueError(
            f"{path}: compare field(s) missing from trace header: {', '.join(missing)}"
        )


def validate_msg_types(path: Path, rows):
    invalid_counts = collections.Counter(
        row.get("msg_type", "") for row in rows
        if row.get("msg_type", "") not in VALID_MSG_TYPES
    )
    if invalid_counts:
        raise ValueError(
            f"{path}: invalid msg_type values found: {dict(sorted(invalid_counts.items()))}"
        )


def should_ignore_attention_value(row, field, quant_cnoc, strict_attention):
    return (
        quant_cnoc
        and not strict_attention
        and field in CNOC_VALUE_FIELDS
        and row.get("msg_type") == "5"
        and row.get("compute_op") == "23"
    )


def row_key(row, fields, quant_cnoc=False, strict_attention=False):
    return tuple(
        "*"
        if should_ignore_attention_value(row, field, quant_cnoc, strict_attention)
        else row.get(field, "")
        for field in fields
    )


def first_positional_mismatch(left_rows, right_rows, fields, quant_cnoc=False, strict_attention=False):
    limit = min(len(left_rows), len(right_rows))
    for idx in range(limit):
        if row_key(left_rows[idx], fields, quant_cnoc, strict_attention) != row_key(
            right_rows[idx], fields, quant_cnoc, strict_attention
        ):
            return idx, left_rows[idx], right_rows[idx]
    if len(left_rows) != len(right_rows):
        return limit, left_rows[limit] if limit < len(left_rows) else None, right_rows[limit] if limit < len(right_rows) else None
    return None


def counter_for(rows, fields, quant_cnoc=False, strict_attention=False):
    return collections.Counter(
        row_key(row, fields, quant_cnoc, strict_attention) for row in rows
    )


def diff_count(left_counter, right_counter):
    return sum((left_counter - right_counter).values())


def print_row(prefix, row, fields):
    if row is None:
        print(f"{prefix}: <missing>")
        return
    selected = ", ".join(f"{field}={row.get(field, '')}" for field in fields)
    print(f"{prefix}: {selected}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare NoCDAS trace logs")
    parser.add_argument("golden", type=Path, help="Golden C++ trace")
    parser.add_argument("candidate", type=Path, help="RTL/candidate trace")
    parser.add_argument(
        "--fields",
        default=",".join(DEFAULT_COMPARE_FIELDS),
        help="Comma-separated fields to compare as functional identity",
    )
    parser.add_argument(
        "--show-examples",
        type=int,
        default=5,
        help="Number of multiset diff examples to print",
    )
    parser.add_argument(
        "--ignore-cnoc-values",
        action="store_true",
        help="Ignore cNoC numerical fields; useful before RTL quantization is finalized",
    )
    parser.add_argument(
        "--quant-cnoc",
        action="store_true",
        help="Compare quantized cNoC integer traces; type5 Attention numeric fields are ignored unless --strict-attention is also set",
    )
    parser.add_argument(
        "--strict-attention",
        action="store_true",
        help="When used with --quant-cnoc, include type5 Attention numeric fields in comparison",
    )
    args = parser.parse_args()

    compare_fields = [field.strip() for field in args.fields.split(",") if field.strip()]
    if args.ignore_cnoc_values:
        compare_fields = [field for field in compare_fields if field not in CNOC_VALUE_FIELDS]
    golden_fields, golden_rows, golden_has_quant_marker = parse_trace(args.golden)
    candidate_fields, candidate_rows, candidate_has_quant_marker = parse_trace(args.candidate)
    validate_compare_fields(args.golden, golden_fields, compare_fields)
    validate_compare_fields(args.candidate, candidate_fields, compare_fields)
    validate_msg_types(args.golden, golden_rows)
    validate_msg_types(args.candidate, candidate_rows)
    if args.quant_cnoc and (not golden_has_quant_marker or not candidate_has_quant_marker):
        missing = []
        if not golden_has_quant_marker:
            missing.append(str(args.golden))
        if not candidate_has_quant_marker:
            missing.append(str(args.candidate))
        raise ValueError(
            "--quant-cnoc requires quantized trace marker in both traces; missing in: "
            + ", ".join(missing)
        )

    golden_type_counts = collections.Counter(row.get("msg_type", "") for row in golden_rows)
    candidate_type_counts = collections.Counter(row.get("msg_type", "") for row in candidate_rows)
    golden_opcode_counts = collections.Counter(
        row.get("compute_op", "") for row in golden_rows if row.get("msg_type") in {"4", "5"}
    )
    candidate_opcode_counts = collections.Counter(
        row.get("compute_op", "") for row in candidate_rows if row.get("msg_type") in {"4", "5"}
    )

    golden_counter = counter_for(golden_rows, compare_fields, args.quant_cnoc, args.strict_attention)
    candidate_counter = counter_for(candidate_rows, compare_fields, args.quant_cnoc, args.strict_attention)

    cnoc_rows_golden = [row for row in golden_rows if row.get("msg_type") in {"4", "5"}]
    cnoc_rows_candidate = [row for row in candidate_rows if row.get("msg_type") in {"4", "5"}]
    regular_rows_golden = [row for row in golden_rows if row.get("msg_type") in {"0", "1", "2", "3"}]
    regular_rows_candidate = [row for row in candidate_rows if row.get("msg_type") in {"0", "1", "2", "3"}]
    cnoc_counter_golden = counter_for(cnoc_rows_golden, compare_fields, args.quant_cnoc, args.strict_attention)
    cnoc_counter_candidate = counter_for(cnoc_rows_candidate, compare_fields, args.quant_cnoc, args.strict_attention)
    path_fields = [field for field in PATH_COMPARE_FIELDS if field in compare_fields]
    value_fields = [field for field in VALUE_COMPARE_FIELDS if field in compare_fields]
    path_counter_golden = counter_for(golden_rows, path_fields, args.quant_cnoc, args.strict_attention)
    path_counter_candidate = counter_for(candidate_rows, path_fields, args.quant_cnoc, args.strict_attention)
    value_counter_golden = counter_for(cnoc_rows_golden, value_fields, args.quant_cnoc, args.strict_attention)
    value_counter_candidate = counter_for(cnoc_rows_candidate, value_fields, args.quant_cnoc, args.strict_attention)

    print(f"rows: golden={len(golden_rows)} candidate={len(candidate_rows)}")
    print(
        "row_groups: "
        f"regular_golden={len(regular_rows_golden)} regular_candidate={len(regular_rows_candidate)} "
        f"cnoc_golden={len(cnoc_rows_golden)} cnoc_candidate={len(cnoc_rows_candidate)}"
    )
    print(f"type_counts_golden: {dict(sorted(golden_type_counts.items()))}")
    print(f"type_counts_candidate: {dict(sorted(candidate_type_counts.items()))}")
    print(f"cnoc_opcode_counts_golden: {dict(sorted(golden_opcode_counts.items()))}")
    print(f"cnoc_opcode_counts_candidate: {dict(sorted(candidate_opcode_counts.items()))}")

    positional = first_positional_mismatch(
        golden_rows, candidate_rows, compare_fields, args.quant_cnoc, args.strict_attention
    )
    if positional is None:
        print("first_positional_mismatch: none")
    else:
        idx, golden_row, candidate_row = positional
        print(f"first_positional_mismatch: row={idx + 1}")
        print_row("  golden", golden_row, compare_fields)
        print_row("  candidate", candidate_row, compare_fields)

    cnoc_positional = first_positional_mismatch(
        cnoc_rows_golden, cnoc_rows_candidate, compare_fields, args.quant_cnoc, args.strict_attention
    )
    if cnoc_positional is None:
        print("first_cnoc_positional_mismatch: none")
    else:
        idx, golden_row, candidate_row = cnoc_positional
        print(f"first_cnoc_positional_mismatch: cnoc_row={idx + 1}")
        print_row("  golden", golden_row, compare_fields)
        print_row("  candidate", candidate_row, compare_fields)

    missing = golden_counter - candidate_counter
    extra = candidate_counter - golden_counter
    cnoc_missing = cnoc_counter_golden - cnoc_counter_candidate
    cnoc_extra = cnoc_counter_candidate - cnoc_counter_golden
    path_missing = path_counter_golden - path_counter_candidate
    path_extra = path_counter_candidate - path_counter_golden
    value_missing = value_counter_golden - value_counter_candidate
    value_extra = value_counter_candidate - value_counter_golden

    print(f"multiset_diff_golden_minus_candidate: {sum(missing.values())}")
    print(f"multiset_diff_candidate_minus_golden: {sum(extra.values())}")
    print(f"cnoc_diff_golden_minus_candidate: {sum(cnoc_missing.values())}")
    print(f"cnoc_diff_candidate_minus_golden: {sum(cnoc_extra.values())}")
    print(f"path_diff_golden_minus_candidate: {sum(path_missing.values())}")
    print(f"path_diff_candidate_minus_golden: {sum(path_extra.values())}")
    print(f"value_diff_golden_minus_candidate: {sum(value_missing.values())}")
    print(f"value_diff_candidate_minus_golden: {sum(value_extra.values())}")

    if len(golden_rows) != len(candidate_rows) or golden_type_counts != candidate_type_counts:
        print("failure_category: packet_count_or_type_count")
    elif path_missing or path_extra:
        print("failure_category: path_or_completion")
    elif value_missing or value_extra:
        print("failure_category: cnoc_value_or_attention_state")
    elif missing or extra or cnoc_missing or cnoc_extra:
        print("failure_category: mixed_identity")
    else:
        print("failure_category: none")

    for name, diff in [
        ("golden_minus_candidate", missing),
        ("candidate_minus_golden", extra),
        ("cnoc_golden_minus_candidate", cnoc_missing),
        ("cnoc_candidate_minus_golden", cnoc_extra),
        ("path_golden_minus_candidate", path_missing),
        ("path_candidate_minus_golden", path_extra),
        ("value_golden_minus_candidate", value_missing),
        ("value_candidate_minus_golden", value_extra),
    ]:
        if not diff:
            continue
        print(f"{name}_examples:")
        for key, count in diff.most_common(args.show_examples):
            print(f"  count={count} key={key}")

    if (
        len(golden_rows) != len(candidate_rows)
        or golden_type_counts != candidate_type_counts
        or missing
        or extra
        or cnoc_missing
        or cnoc_extra
        or path_missing
        or path_extra
        or value_missing
        or value_extra
    ):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
