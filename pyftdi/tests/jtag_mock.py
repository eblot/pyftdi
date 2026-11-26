"""Exercise JTAG commands without a physical USB device."""

# SPDX-License-Identifier: BSD-3-Clause
# pylint: disable=protected-access
from unittest import TestCase, main
from unittest.mock import PropertyMock, patch

from pyftdi.bits import BitSequence
from pyftdi.ftdi import Ftdi
from pyftdi.jtag import JtagController, JtagEngine, JtagError


class SingleBitShiftTestCase(TestCase):
    """Verify the engine and command buffer with simulated USB transfers."""
    def test_shift_and_update_single_bit(self):
        """Transfer the final bit through TMS for every input/output value."""
        for sent in (0, 1):
            for received in (0, 1):
                with self.subTest(sent=sent, received=received):
                    engine = JtagEngine()
                    controller = engine._ctrl
                    with (
                        patch.object(
                            Ftdi,
                            "is_connected",
                            new_callable=PropertyMock,
                            return_value=True,
                        ),
                        patch.object(controller.ftdi, "write_data") as write,
                        patch.object(
                            controller.ftdi,
                            "read_data_bytes",
                            return_value=bytes([received << 6]),
                        ) as read,
                    ):
                        engine.change_state("shift_dr")
                        controller.sync()
                        write.reset_mock()
                        result = engine.shift_and_update_register(
                            BitSequence(sent, length=1)
                        )
                        self.assertEqual(len(result), 1)
                        self.assertEqual(int(result), received)
                        self.assertEqual(str(engine._sm.state()), "update_dr")
                        write.assert_called_once_with(
                            bytearray(
                                [Ftdi.RW_BITS_TMS_PVE_NVE, 1, 3 | (sent << 7)]
                            )
                        )
                        read.assert_called_once_with(1, 4)

    def test_single_bit_without_deferred_transition(self):
        """Keep ordinary single-bit shifts unchanged."""
        controller = JtagController()
        self.assertEqual(
            controller.write_with_read(BitSequence(1, length=1)), 1
        )
        self.assertEqual(
            controller._write_buff, bytearray([Ftdi.RW_BITS_PVE_NVE_LSB, 0, 1])
        )

    def test_byte_and_deferred_bit(self):
        """Preserve the byte-transfer path before the final TMS bit."""
        engine = JtagEngine()
        controller = engine._ctrl
        with (
            patch.object(
                Ftdi,
                "is_connected",
                new_callable=PropertyMock,
                return_value=True,
            ),
            patch.object(controller.ftdi, "write_data") as write,
            patch.object(
                controller.ftdi,
                "read_data_bytes",
                side_effect=[b"\xa5", b"\x40"],
            ),
        ):
            engine.change_state("shift_dr")
            controller.sync()
            write.reset_mock()
            result = engine.shift_and_update_register(
                BitSequence(0x1A5, length=9)
            )
            self.assertEqual(len(result), 9)
            self.assertEqual(int(result), 0x1A5)
            write.assert_called_once_with(
                bytearray(
                    [
                        Ftdi.RW_BYTES_PVE_NVE_LSB,
                        0,
                        0,
                        0xA5,
                        Ftdi.RW_BITS_TMS_PVE_NVE,
                        1,
                        0x83,
                    ]
                )
            )

    def test_empty_sequence_still_rejected(self):
        """Reject empty input before trying to defer its last bit."""
        for use_last in (False, True):
            with self.subTest(use_last=use_last):
                with self.assertRaisesRegex(JtagError, "Nothing to shift"):
                    JtagController().write_with_read(
                        BitSequence(), use_last=use_last
                    )


if __name__ == "__main__":
    main()
