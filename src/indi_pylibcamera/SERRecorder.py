"""
SER file format recorder for INDI video streaming.

SER is a standard video format used in astronomy for recording planetary, lunar,
and solar videos. See: http://www.grischa-hahn.homepage.t-online.de/astro/ser/

File structure:
  - 178-byte header
  - Image frames (contiguous raw pixel data)
  - Optional trailer with per-frame timestamps (8 bytes each, little-endian uint64)
"""

import struct
import os
import datetime
import time
import logging

logger = logging.getLogger(__name__)

# SER color IDs
SER_MONO = 0
SER_BAYER_RGGB = 8
SER_BAYER_GRBG = 9
SER_BAYER_GBRG = 10
SER_BAYER_BGGR = 11
SER_BAYER_CYYM = 16
SER_BAYER_YCMY = 17
SER_BAYER_YMCY = 18
SER_BAYER_MYYC = 19
SER_RGB = 100
SER_BGR = 101

# SER header format (178 bytes total)
SER_HEADER_FORMAT = '<14sIIIIIIIIQ40s40sQ'
SER_HEADER_SIZE = 178


class SERRecorder:
    """Records video frames to SER file format.

    Usage:
        recorder = SERRecorder()
        recorder.open("output.ser", width=640, height=480, color_id=SER_MONO, bit_depth=8)
        recorder.add_frame(frame_bytes, timestamp_ns=time.time_ns())
        recorder.close()
    """

    def __init__(self):
        self._file = None
        self._frame_count = 0
        self._timestamps = []
        self._width = 0
        self._height = 0
        self._color_id = SER_MONO
        self._bit_depth = 8
        self._bytes_per_pixel = 1
        self._filepath = None

    @property
    def is_open(self):
        return self._file is not None

    @property
    def frame_count(self):
        return self._frame_count

    def open(self, filepath, width, height, color_id=SER_MONO, bit_depth=8, observer="", instrument=""):
        """Open a new SER file for writing.

        Args:
            filepath: Path to the SER file to create.
            width: Frame width in pixels.
            height: Frame height in pixels.
            color_id: SER color ID constant.
            bit_depth: Bits per pixel per channel (8 or 16).
            observer: Observer name (max 40 chars).
            instrument: Instrument/telescope name (max 40 chars).
        """
        if self._file is not None:
            self.close()

        self._filepath = filepath
        self._width = width
        self._height = height
        self._color_id = color_id
        self._bit_depth = bit_depth
        self._frame_count = 0
        self._timestamps = []

        # Calculate bytes per pixel
        bytes_per_channel = 2 if bit_depth > 8 else 1
        if color_id >= SER_RGB:
            channels = 3
        else:
            channels = 1
        self._bytes_per_pixel = bytes_per_channel * channels

        # Create directory if needed
        os.makedirs(os.path.dirname(filepath) or ".", exist_ok=True)

        self._file = open(filepath, 'wb')

        # Write placeholder header (will be updated on close with final frame count)
        self._write_header(observer=observer, instrument=instrument)
        logger.info(f"SER recording started: {filepath} ({width}x{height}, color_id={color_id}, {bit_depth}bit)")

    def _write_header(self, observer="", instrument=""):
        """Write SER file header."""
        # DateTime and DateTimeUTC as SER timestamp (100ns ticks since 1 Jan 0001)
        now = datetime.datetime.utcnow()
        # SER epoch: 1 January 0001
        ser_epoch = datetime.datetime(1, 1, 1)
        delta = now - ser_epoch
        ticks = int(delta.total_seconds() * 1e7)

        # Write header field by field (178 bytes total)
        self._file.seek(0)
        self._file.write(b'LUCAM-RECORDER')  # 14 bytes - FileID
        self._file.write(struct.pack('<I', 0))  # 4 bytes - LuID
        self._file.write(struct.pack('<I', self._color_id))  # 4 bytes - ColorID
        self._file.write(struct.pack('<I', 0))  # 4 bytes - LittleEndian (0 = big endian)
        self._file.write(struct.pack('<I', self._width))  # 4 bytes - ImageWidth
        self._file.write(struct.pack('<I', self._height))  # 4 bytes - ImageHeight
        self._file.write(struct.pack('<I', self._bit_depth))  # 4 bytes - PixelDepthPerPlane
        self._file.write(struct.pack('<I', self._frame_count))  # 4 bytes - FrameCount
        # Observer (40 bytes, null-padded)
        obs_bytes = observer.encode('ascii', errors='replace')[:40]
        self._file.write(obs_bytes.ljust(40, b'\x00'))
        # Instrument (40 bytes, null-padded)
        inst_bytes = instrument.encode('ascii', errors='replace')[:40]
        self._file.write(inst_bytes.ljust(40, b'\x00'))
        # Telescope (40 bytes, null-padded) — not always in the spec but some implementations include it
        # Actually standard SER has: DateTime (8 bytes), DateTimeUTC (8 bytes)
        self._file.write(struct.pack('<Q', ticks))  # 8 bytes - DateTime (local)
        self._file.write(struct.pack('<Q', ticks))  # 8 bytes - DateTimeUTC
        # Total so far: 14+4+4+4+4+4+4+4+40+40+8+8 = 138
        # SER spec says header is 178 bytes. The remaining 40 bytes is the Telescope field.
        self._file.write(b'\x00' * 40)  # 40 bytes - Telescope (padding)
        # Total: 178 bytes

    def add_frame(self, frame_data, timestamp_ns=None):
        """Add a raw frame to the SER file.

        Args:
            frame_data: Raw pixel bytes for one frame. Must be exactly
                        width * height * bytes_per_pixel bytes.
            timestamp_ns: Timestamp in nanoseconds since Unix epoch.
                         If None, current time is used.
        """
        if self._file is None:
            raise RuntimeError("SER file not open")

        expected_size = self._width * self._height * self._bytes_per_pixel
        if len(frame_data) != expected_size:
            logger.warning(
                f"SER frame size mismatch: expected {expected_size}, got {len(frame_data)}. "
                f"Padding/truncating."
            )
            if len(frame_data) < expected_size:
                frame_data = frame_data + b'\x00' * (expected_size - len(frame_data))
            else:
                frame_data = frame_data[:expected_size]

        self._file.write(frame_data)
        self._frame_count += 1

        if timestamp_ns is None:
            timestamp_ns = time.time_ns()
        # Convert Unix nanoseconds to SER ticks (100ns since 1 Jan 0001)
        # Unix epoch is 1 Jan 1970. Days from 0001-01-01 to 1970-01-01 = 719162
        unix_to_ser_offset = 719162 * 24 * 3600 * 10_000_000  # in 100ns ticks
        ser_ticks = timestamp_ns // 100 + unix_to_ser_offset
        self._timestamps.append(ser_ticks)

    def close(self):
        """Close the SER file, writing the final frame count and timestamp trailer."""
        if self._file is None:
            return

        # Write timestamp trailer
        for ts in self._timestamps:
            self._file.write(struct.pack('<Q', ts))

        # Update frame count in header (offset 38: after FileID(14)+LuID(4)+ColorID(4)+LE(4)+W(4)+H(4)+Depth(4))
        self._file.seek(38)
        self._file.write(struct.pack('<I', self._frame_count))

        self._file.close()
        self._file = None
        logger.info(f"SER recording finished: {self._filepath} ({self._frame_count} frames)")
        self._filepath = None
