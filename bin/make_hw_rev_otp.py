#!/usr/bin/env python3
"""Program the DPECM hardware revision into the STM32F7 OTP memory over SWD.

Typical use (one command per board):
  bin/make_hw_rev_otp.py BREMEN-REV-C          program next free block, verify, lock
  bin/make_hw_rev_otp.py BREMEN-REV-C --openblt 1.16.0   same, with another OpenBLT version
  bin/make_hw_rev_otp.py --status              show the current OTP state only
  bin/make_hw_rev_otp.py BREMEN-REV-C --dry-run   show what would be written
  bin/make_hw_rev_otp.py --lock-block 0        only lock block 0 (e.g. an invalid one)

Steps of a programming run:
  1. Read the whole OTP area (0x1FF0F000, 0x420 bytes) via STM32_Programmer_CLI.
  2. Pick the first erased block after the last programmed one (the firmware uses
     the last valid block, so a new entry always wins over older ones). Nothing is
     written if the block the firmware uses already holds the same revision,
     OpenBLT version and manufacturer.
  3. Ask for confirmation (skip with --yes): OTP writes are permanent.
  4. Write the 64-byte block, read back, validate exactly like the firmware.
  5. Lock the block (skip with --no-lock): the 32-byte lock chunk at 0x1FF0F400
     is rewritten from the read-back with 0x00 at this block. CubeProgrammer only
     writes the OTP area in 32-byte aligned chunks, so a single lock byte cannot
     be written on its own.

Block layout, must match DpecmOtpBlock in board_config.cpp:
  bytes  0..3   magic "DPHW"
  byte   4      layout version (1)
  bytes  5..20  hardware revision, printable ASCII, padded with 0x00 (max 16 chars)
  bytes 21..23  OpenBLT version MAJOR, MINOR, PATCH (uint8 each)
  bytes 24..27  manufacturer, 4 ASCII characters (always "WEYT")
  bytes 28..31  first programming date, Unix time (uint32, UTC), little-endian
  bytes 32..59  reserved (0x00)
  bytes 60..63  CRC32 (zlib) over bytes 0..59, little-endian

Bytes 21..31 were reserved (0x00) in older blocks; the layout version stays 1, so
older firmware still accepts new blocks. All-zero fields mean "not recorded".

The CLI location defaults to the macOS install path; override with --cli or the
STM32_PROGRAMMER_CLI environment variable. The connection defaults to
'port=SWD mode=UR' (connect under reset, requires NRST wired to the ST-LINK);
pass --connect "port=SWD" for adapters without NRST.
"""

import argparse
import datetime
import os
import struct
import subprocess
import sys
import tempfile
import time
import zlib

DEFAULT_CLI = ("/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/"
               "STM32CubeProgrammer.app/Contents/Resources/bin/STM32_Programmer_CLI")

OTP_BASE = 0x1FF0F000
OTP_SIZE = 0x420
OTP_LOCK_OFFSET = 0x400
OTP_LOCK_BASE = OTP_BASE + OTP_LOCK_OFFSET
LOCK_CHUNK_SIZE = 32  # programming granularity of the OTP area in STM32CubeProgrammer
BLOCK_SIZE = 64
BLOCK_COUNT = 16
REV_SIZE = 16
MAGIC = b"DPHW"
LAYOUT_VERSION = 1
OPENBLT_OFFSET = 21
MANUFACTURER_OFFSET = 24
MANUFACTURER_SIZE = 4
DATE_OFFSET = 28
MANUFACTURER = "WEYT"
DEFAULT_OPENBLT = "1.16.0"


# --- Block format ---------------------------------------------------------------

def build_block(revision: str, openblt: tuple, manufacturer: str, programmed_at: int) -> bytes:
    body = MAGIC + bytes([LAYOUT_VERSION]) + revision.encode("ascii").ljust(REV_SIZE, b"\x00")
    body += bytes(openblt)
    body += manufacturer.encode("ascii").ljust(MANUFACTURER_SIZE, b"\x00")
    body += struct.pack("<I", programmed_at)
    body = body.ljust(BLOCK_SIZE - 4, b"\x00")
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


def parse_openblt_version(text: str):
    """Return (major, minor, patch) for "MAJOR.MINOR.PATCH" with each part 0..255, else None."""
    parts = text.split(".")
    if len(parts) != 3 or not all(p.isascii() and p.isdigit() and int(p) <= 255 for p in parts):
        return None
    return tuple(int(p) for p in parts)


def block_identity(block: bytes) -> bytes:
    """Revision, OpenBLT version and manufacturer; equal identities need no new block."""
    return block[5:DATE_OFFSET]


def describe_extras(block: bytes) -> str:
    """OpenBLT version, manufacturer and date of a valid block; fields still 0x00 are omitted."""
    parts = []
    openblt = block[OPENBLT_OFFSET:OPENBLT_OFFSET + 3]
    if any(openblt):
        parts.append("OpenBLT {}.{}.{}".format(*openblt))
    manufacturer = block[MANUFACTURER_OFFSET:MANUFACTURER_OFFSET + MANUFACTURER_SIZE].rstrip(b"\x00")
    if manufacturer:
        parts.append("".join(chr(c) if 0x20 <= c <= 0x7E else "." for c in manufacturer))
    programmed_at = struct.unpack("<I", block[DATE_OFFSET:DATE_OFFSET + 4])[0]
    if programmed_at:
        date = datetime.datetime.fromtimestamp(programmed_at, datetime.timezone.utc)
        parts.append(date.strftime("%Y-%m-%d %H:%M:%S UTC"))
    return ", ".join(parts)


def check_block(block: bytes):
    """Return (revision, None) if the block is valid (same rules as the firmware), else (None, reason)."""
    if block[0:4] != MAGIC:
        return None, f"magic is {block[0:4].hex(' ')}, expected {MAGIC.hex(' ')} ('DPHW')"
    if block[4] != LAYOUT_VERSION:
        return None, f"layout version is {block[4]}, expected {LAYOUT_VERSION}"
    stored = struct.unpack("<I", block[BLOCK_SIZE - 4:])[0]
    computed = zlib.crc32(block[:BLOCK_SIZE - 4]) & 0xFFFFFFFF
    if stored != computed:
        return None, f"CRC mismatch: stored 0x{stored:08X}, computed 0x{computed:08X}"
    text = block[5:5 + REV_SIZE].rstrip(b"\x00")
    if not text:
        return None, "revision text is empty"
    if b"\x00" in text or not all(0x20 <= c <= 0x7E for c in text):
        return None, f"revision text is not printable ASCII followed by 0x00 padding: {text!r}"
    return text.decode("ascii"), None


def parse_block(block: bytes):
    """Return the revision if the block is valid (same rules as the firmware), else None."""
    return check_block(block)[0]


def hexdump(data: bytes, address: int, indent: str = "       "):
    for off in range(0, len(data), 16):
        row = data[off:off + 16]
        text = "".join(chr(c) if 0x20 <= c <= 0x7E else "." for c in row)
        print(f"{indent}0x{address + off:08X}  {row.hex(' '):47}  {text}")


def is_valid_revision(rev: str) -> bool:
    return 1 <= len(rev) <= REV_SIZE and all(0x20 <= ord(c) <= 0x7E for c in rev)


# --- OTP state ------------------------------------------------------------------

class OtpState:
    def __init__(self, dump: bytes):
        if len(dump) != OTP_SIZE:
            raise RuntimeError(f"OTP dump has {len(dump)} bytes, expected {OTP_SIZE}")
        self.dump = dump
        self.blocks = [dump[i * BLOCK_SIZE:(i + 1) * BLOCK_SIZE] for i in range(BLOCK_COUNT)]
        self.lock_chunk = dump[OTP_LOCK_OFFSET:OTP_LOCK_OFFSET + LOCK_CHUNK_SIZE]

    def erased(self, i: int) -> bool:
        return self.blocks[i] == b"\xff" * BLOCK_SIZE

    def locked(self, i: int) -> bool:
        return self.lock_chunk[i] == 0x00

    def revision(self, i: int):
        return parse_block(self.blocks[i])

    def effective(self):
        """(block index, revision) of the last valid block, as the firmware sees it."""
        for i in reversed(range(BLOCK_COUNT)):
            rev = self.revision(i)
            if rev is not None:
                return i, rev
        return None, None

    def next_free_block(self):
        last_used = max((i for i in range(BLOCK_COUNT) if not self.erased(i)), default=-1)
        for i in range(last_used + 1, BLOCK_COUNT):
            if not self.locked(i):
                return i
        return None

    def print(self):
        print("OTP block  address     lock    content")
        for i in range(BLOCK_COUNT):
            reason = None
            if self.erased(i):
                content = "erased"
            else:
                rev, reason = check_block(self.blocks[i])
                if rev is not None:
                    extras = describe_extras(self.blocks[i])
                    content = f"valid '{rev}'" + (f", {extras}" if extras else "")
                else:
                    content = f"INVALID: {reason}"
            lock = "locked" if self.locked(i) else "-"
            print(f"  {i:2d}       0x{OTP_BASE + i * BLOCK_SIZE:08X}  {lock:6}  {content}")
            if reason is not None:
                hexdump(self.blocks[i], OTP_BASE + i * BLOCK_SIZE)
        idx, rev = self.effective()
        if idx is None:
            any_programmed = any(not self.erased(i) for i in range(BLOCK_COUNT))
            print(f"Firmware shows: {'INVALID' if any_programmed else 'UNPROGRAMMED'}")
        else:
            print(f"Firmware shows: '{rev}' (block {idx})")


# --- STM32CubeProgrammer --------------------------------------------------------

class Programmer:
    def __init__(self, cli: str, connect: str):
        self.cli = cli
        self.connect = connect.split()

    # Failures while connecting: the CLI did not touch the target, so a retry is always safe.
    # Typical right after a write with a plain 'port=SWD' connect, when the target was reset and
    # bootloader/firmware are starting. The default connect under reset (mode=UR) avoids this;
    # the retry stays as a fallback.
    CONNECT_ERRORS = ("DEV_TARGET_NOT_HALTED", "DEV_TARGET_RESET_ERR", "Unable to read device id",
                      "No STM32 target found", "DEV_CONNECT_ERR")

    def run(self, *args: str, attempts: int = 3):
        cmd = [self.cli, "-c", *self.connect, *args]
        for attempt in range(1, attempts + 1):
            result = subprocess.run(cmd, capture_output=True, text=True)
            output = result.stdout + result.stderr
            if result.returncode == 0 and "Error:" not in output:
                return output
            connect_failed = any(e in output for e in self.CONNECT_ERRORS)
            if not connect_failed or attempt == attempts:
                raise RuntimeError(f"command failed: {' '.join(cmd)}\n{output}")
            print(f"Connection to target failed (attempt {attempt}/{attempts}), retrying...")
            time.sleep(2)

    def read_otp(self, workdir: str) -> OtpState:
        path = os.path.join(workdir, "otp_dump.bin")
        self.run("-u", hex(OTP_BASE), str(OTP_SIZE), path)
        with open(path, "rb") as f:
            return OtpState(f.read())

    def write(self, workdir: str, name: str, data: bytes, address: int):
        path = os.path.join(workdir, name)
        with open(path, "wb") as f:
            f.write(data)
        self.run("-w", path, hex(address), "-v")


# --- Main -----------------------------------------------------------------------

def confirm(prompt: str) -> bool:
    try:
        return input(f"{prompt} Type YES to continue: ").strip() == "YES"
    except EOFError:
        return False


def write_lock(prog: Programmer, workdir: str, state: OtpState, block_idx: int) -> OtpState:
    chunk = bytearray(state.lock_chunk)
    chunk[block_idx] = 0x00
    expected = bytes(chunk)
    write_error = None
    try:
        prog.write(workdir, "lock_chunk.bin", expected, OTP_LOCK_BASE)
    except RuntimeError as e:
        # CubeProgrammer reports "failed to download Sector[0]" when the lock chunk already
        # contains a 0x00 (an earlier lock), although the new lock byte is programmed.
        # The read-back below decides.
        write_error = e
    state = prog.read_otp(workdir)
    if state.lock_chunk != expected:
        detail = f"\n{write_error}" if write_error else ""
        raise RuntimeError(f"lock chunk reads {state.lock_chunk.hex(' ')}, expected {expected.hex(' ')}{detail}")
    if write_error:
        print("Note: STM32_Programmer_CLI reported an error, but the read-back shows the lock was written correctly.")
    return state


def lock_only(args, prog: Programmer, workdir: str) -> int:
    block_idx = args.lock_block
    state = prog.read_otp(workdir)
    state.print()
    print()

    if state.locked(block_idx):
        print(f"Nothing to do: block {block_idx} is already locked.")
        return 0
    print(f"Plan: lock block {block_idx} (lock byte 0x{OTP_LOCK_BASE + block_idx:08X}).")
    if state.erased(block_idx):
        print(f"WARNING: block {block_idx} is erased; locking makes it permanently unusable.")

    if args.dry_run:
        print("Dry run: nothing written.")
        return 0
    if not args.yes and not confirm("OTP writes cannot be undone."):
        print("Aborted, nothing written.")
        return 1

    state = write_lock(prog, workdir, state, block_idx)
    print(f"Block {block_idx} locked.")
    print()
    state.print()
    return 0


def program(args, prog: Programmer, workdir: str) -> int:
    rev = args.revision
    openblt = parse_openblt_version(args.openblt)
    # The date of this programming run; Unix time is UTC by definition.
    block = build_block(rev, openblt, MANUFACTURER, int(time.time()))
    what = f"'{rev}', OpenBLT {args.openblt}, {MANUFACTURER}"
    state = prog.read_otp(workdir)
    state.print()
    print()

    eff_idx, _ = state.effective()
    if eff_idx is not None and block_identity(state.blocks[eff_idx]) == block_identity(block):
        if state.locked(eff_idx) or args.no_lock:
            print(f"Nothing to do: block {eff_idx} already holds {what}"
                  f"{' and is locked' if state.locked(eff_idx) else ''}.")
            return 0
        block_idx = eff_idx
        write_data = False
        print(f"Block {eff_idx} already holds {what} but is not locked; only locking it.")
    else:
        block_idx = state.next_free_block()
        if block_idx is None:
            print("error: no free OTP block left after the last programmed one", file=sys.stderr)
            return 1
        write_data = True
        print(f"Plan: write {what} to block {block_idx} at 0x{OTP_BASE + block_idx * BLOCK_SIZE:08X}"
              f"{'' if args.no_lock else ', then lock it'}.")

    if args.dry_run:
        print("Dry run: nothing written.")
        return 0
    if not args.yes and not confirm("OTP writes cannot be undone."):
        print("Aborted, nothing written.")
        return 1

    if write_data:
        prog.write(workdir, "rev_block.bin", block, OTP_BASE + block_idx * BLOCK_SIZE)
        state = prog.read_otp(workdir)
        if state.blocks[block_idx] != block or state.effective() != (block_idx, rev):
            print(f"error: read-back of block {block_idx} does not match; NOT locking", file=sys.stderr)
            state.print()
            return 1
        print(f"Block {block_idx} written and verified.")

    if not args.no_lock:
        state = write_lock(prog, workdir, state, block_idx)
        print(f"Block {block_idx} locked.")

    print()
    state.print()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("revision", nargs="?", help="hardware revision, 1..16 printable ASCII characters")
    parser.add_argument("--openblt", default=DEFAULT_OPENBLT, metavar="MAJOR.MINOR.PATCH",
                        help=f"OpenBLT version to record, each part 0..255 (default: {DEFAULT_OPENBLT})")
    parser.add_argument("--status", action="store_true", help="only read and show the OTP state")
    parser.add_argument("--dry-run", action="store_true", help="read and show the plan, write nothing")
    parser.add_argument("--no-lock", action="store_true", help="write and verify, but do not lock the block")
    parser.add_argument("--lock-block", type=int, metavar="N", help="only lock OTP block N (0..15), write no revision")
    parser.add_argument("--yes", action="store_true", help="do not ask for confirmation")
    parser.add_argument("--cli", default=os.environ.get("STM32_PROGRAMMER_CLI", DEFAULT_CLI),
                        help="path to STM32_Programmer_CLI")
    parser.add_argument("--connect", default="port=SWD mode=UR",
                        help="connection parameters for -c (default: 'port=SWD mode=UR', connect under reset, "
                             "needs NRST wired; use 'port=SWD' without NRST)")
    args = parser.parse_args()

    if args.lock_block is not None:
        if args.revision is not None or args.status or args.no_lock:
            parser.error("--lock-block cannot be combined with a revision, --status or --no-lock")
        if not 0 <= args.lock_block < BLOCK_COUNT:
            parser.error(f"--lock-block must be 0..{BLOCK_COUNT - 1}")
    elif not args.status:
        if args.revision is None:
            parser.error("revision is required unless --status or --lock-block is given")
        if not is_valid_revision(args.revision):
            parser.error(f"revision must be 1..{REV_SIZE} printable ASCII characters")
        if parse_openblt_version(args.openblt) is None:
            parser.error("--openblt must look like MAJOR.MINOR.PATCH with each part 0..255, e.g. 1.16.0")
    if not os.path.isfile(args.cli):
        print(f"error: STM32_Programmer_CLI not found at {args.cli}", file=sys.stderr)
        return 1

    prog = Programmer(args.cli, args.connect)
    try:
        with tempfile.TemporaryDirectory() as workdir:
            if args.status:
                prog.read_otp(workdir).print()
                return 0
            if args.lock_block is not None:
                return lock_only(args, prog, workdir)
            return program(args, prog, workdir)
    except RuntimeError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
