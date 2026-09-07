import argparse
import sys
import yaml
import json
import os.path

IGNORE_LIST = [
    "Geometry",
    "Core",
    "pose3.hpp",
    "Eigen",
    "is not used directly",
    "TEST_F",
    "MOCK_METHOD",
    "TEST",
]


def get_line_number_by_offset(filepath, offset):
    count = 0
    with open(filepath, "r") as f:
        tmp_offset = offset

        while tmp_offset > 0:
            tmp_offset -= len(f.readline())
            count += 1
    return count


def get_line_by_offset(filepath, offset):
    with open(filepath, "r") as f:
        f.seek(offset)
        line = f.readline()
    return line


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("report", default=True, help="Path to clang-tidy report")
    parser.add_argument("config", default=True, help="Path to clang-tidy report")
    args = parser.parse_args()

    exit_code = 0

    relevant_checks = []
    with open(args.config, "r") as file:
        config = yaml.safe_load(file)

    if not os.path.isfile(args.report) or os.stat(args.report).st_size == 0:
        sys.exit(0)

    with open(args.report, "r") as file:
        report = yaml.safe_load(file)

    all_checks = config["Checks"].split(", ")
    for check in all_checks:
        if check[0] != "-" and check[0] != "#":
            relevant_checks.append(check)

    for line in report["Diagnostics"]:
        if any([check in line["DiagnosticName"] for check in relevant_checks]) and not any(
            [ignore in line["DiagnosticMessage"]["Message"] for ignore in IGNORE_LIST]
        ):

            path = line["DiagnosticMessage"]["FilePath"]
            offset = int(line["DiagnosticMessage"]["FileOffset"])
            line_number = get_line_number_by_offset(path, offset)
            text = get_line_by_offset(path, offset)

            line["DiagnosticMessage"]["FilePath"] = (
                line["DiagnosticMessage"]["FilePath"] + ":" + str(line_number)
            )
            line["Line"] = text
            line["LineNumber"] = line_number

            print(json.dumps(line, indent=4))
            exit_code = 1

    sys.exit(exit_code)
