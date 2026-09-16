"""MicroPython driver for a 256x64, 4-bit grayscale SSD1322 OLED over SPI."""

from micropython import const
import framebuf
import time


_CMD_LOCK = const(0xFD)
_DISPLAY_OFF = const(0xAE)
_DISPLAY_ON = const(0xAF)
_SET_COLUMN = const(0x15)
_SET_ROW = const(0x75)
_WRITE_RAM = const(0x5C)

ROTATION_0 = const(0)
ROTATION_180 = const(180)

_REMAP_ROTATION_0 = const(0x06)
_REMAP_ROTATION_180 = const(0x14)


class SSD1322_SPI(framebuf.FrameBuffer):
    """SSD1322 256x64 panel in 4-wire SPI mode.

    The SSD1322's RAM is wider than this panel.  The 3.12 inch 256x64 module
    maps its visible area to controller columns 0x1C through 0x5B.
    """

    color_on = const(0x0F)

    def __init__(
        self,
        width,
        height,
        spi,
        dc,
        res,
        cs,
        column_offset=0x1C,
        rotation=ROTATION_0,
    ):
        if width % 4:
            raise ValueError("SSD1322 width must be divisible by 4")

        self._rotation_remap(rotation)

        self.width = width
        self.height = height
        self.spi = spi
        self.dc = dc
        self.res = res
        self.cs = cs
        self.column_offset = column_offset
        self.rotation = rotation
        self.buffer = bytearray(width * height // 2)

        # Two 4-bit grayscale pixels are stored in each byte.
        super().__init__(self.buffer, width, height, framebuf.GS4_HMSB)

        self.dc.init(self.dc.OUT, value=0)
        self.res.init(self.res.OUT, value=1)
        self.cs.init(self.cs.OUT, value=1)
        self.reset()
        self.init_display()

    def reset(self):
        self.res(1)
        time.sleep_ms(1)
        self.res(0)
        time.sleep_ms(10)
        self.res(1)
        time.sleep_ms(100)

    def write_cmd(self, command, *data):
        self.cs(0)
        self.dc(0)
        self.spi.write(bytearray([command]))
        if data:
            self.dc(1)
            self.spi.write(bytearray(data))
        self.cs(1)

    def write_data(self, data):
        self.cs(0)
        self.dc(1)
        self.spi.write(data)
        self.cs(1)

    def init_display(self):
        # Settings are for the 3.12 inch, 256x64, dual-COM SSD1322 module.
        self.write_cmd(_CMD_LOCK, 0x12)
        self.write_cmd(_DISPLAY_OFF)
        self.write_cmd(0xB3, 0x91)  # display clock
        self.write_cmd(0xCA, 0x3F)  # 1/64 multiplex ratio
        self.write_cmd(0xA2, 0x00)  # display offset
        self.write_cmd(0xA1, 0x00)  # display start line
        # The physical module is upright with the 0x06 remap. 0x14 is the
        # opposite 180-degree orientation; neither changes framebuffer geometry.
        self.write_cmd(0xA0, self._rotation_remap(self.rotation), 0x11)
        self.write_cmd(0xAB, 0x01)  # internal VDD regulator
        self.write_cmd(0xB4, 0xA0, 0xFD)  # external VSL
        self.write_cmd(0xC1, 0x9F)  # contrast current
        self.write_cmd(0xC7, 0x0F)  # master contrast
        self.write_cmd(0xB1, 0xE2)  # phase length
        self.write_cmd(0xD1, 0x82)  # display enhancement B
        self.write_cmd(0xBB, 0x1F)  # pre-charge voltage
        self.write_cmd(0xB6, 0x08)  # second pre-charge period
        self.write_cmd(0xBE, 0x07)  # VCOMH voltage
        self.write_cmd(0xA4)        # display follows RAM
        self.write_cmd(0xA6)        # normal (non-inverted) display
        self.write_cmd(0xA9)        # exit partial display mode
        self.fill(0)
        self.show()
        self.write_cmd(_DISPLAY_ON)

    def show(self):
        # Each controller column spans four visible pixels (two data bytes).
        column_end = self.column_offset + self.width // 4 - 1
        self.write_cmd(_SET_COLUMN, self.column_offset, column_end)
        self.write_cmd(_SET_ROW, 0, self.height - 1)
        self.write_cmd(_WRITE_RAM)
        self.write_data(self.buffer)

    def poweroff(self):
        self.write_cmd(_DISPLAY_OFF)

    def poweron(self):
        self.write_cmd(_DISPLAY_ON)

    def contrast(self, value):
        self.write_cmd(0xC7, value & 0x0F)

    def set_rotation(self, rotation):
        """Apply one of the two supported physical panel orientations."""
        remap = self._rotation_remap(rotation)
        self.rotation = rotation
        self.write_cmd(0xA0, remap, 0x11)

    @staticmethod
    def _rotation_remap(rotation):
        if rotation == ROTATION_0:
            return _REMAP_ROTATION_0
        if rotation == ROTATION_180:
            return _REMAP_ROTATION_180
        raise ValueError("rotation must be 0 or 180")
