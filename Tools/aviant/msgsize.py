import sys
import os
import re

if len(sys.argv) < 3 or "-h" in sys.argv:
    print(f"Usage: {sys.argv[0]} <path/to/MsgDef.msg> <hz>")
    sys.exit(1)

def check_file(filepath: str) -> int:
    bytes_per_msg = 0
    with open(filepath, "r") as fp:
        for line in fp.readlines():
            if line == "\n" or line.startswith("#"):
                continue
            field_name = line.split(" ")[0]
            size = size_bytes(field_name)
            bytes_per_msg += size
    return bytes_per_msg

def size_bytes(typename: str):
    arrlen = 1
    if (match := re.fullmatch(r"[a-zA-Z_]+(\d+)(?:\[(\d+)\])?", typename)) is not None:
        size_bits = int(match.group(1))
        if (arrlen_str := match.group(2)) is not None:
            arrlen = int(arrlen_str)
        assert size_bits % 8 == 0
        return int(arrlen * size_bits / 8)

    _sizes = {
            "bool": 1,
            }

    if typename not in _sizes:
        _dir = os.path.dirname(sys.argv[1])
        _files = os.listdir(_dir)
        if f"{typename}.msg" in _files:
            return check_file(os.path.join(_dir, f"{typename}.msg"))
        raise NotImplementedError(f"Size of {typename} not known, add it to _sizes")
    return _sizes[typename]


def fmtsize(n: int) -> str:
    if n < 1_000:
        return f"{n}\tB"
    if n < 1_000_000:
        return f"{n/1000}\tKB"
    if n < 1_000_000_000:
        return f"{n/1_000_000}\tMB"

    return f"{n/1_000_000_000}\tGB"

if __name__ == "__main__":
    msgfile = sys.argv[1]

    bytes_per_msg = check_file(msgfile)

    hz = int(sys.argv[2])
    bytes_per_second = bytes_per_msg * hz
    bytes_per_minute = bytes_per_second * 60
    bytes_per_hour = bytes_per_minute * 60

    print(f"One msg:\t{fmtsize(bytes_per_msg)}")
    print(f"One minute:\t{fmtsize(bytes_per_minute)}")
    print(f"One hour:\t{fmtsize(bytes_per_hour)}")
