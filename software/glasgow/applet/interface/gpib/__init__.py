import logging
import argparse
import math
import asyncio
from amaranth import *
from amaranth.lib import io
from amaranth.lib.cdc import FFSynchronizer

from ... import *

"""
This applet acts as a GPIB listener, allowing it to receive data from
GPIB devices where no addressing is used. For example, a TDS
oscilloscope has a "talk only" more - when configured to use GPIB, it
will write a screenshot over GPIB.

In this mode, only four control/handshaking lines are required, in
addition to the 8 data lines.

"""

class GPIBBus(Elaboratable):
    def __init__(self, ports):
        self.ports = ports

        self.dio  = Signal(8) # Data Lines
        self.eoi  = Signal()  # End or Identify
        self.dav  = Signal()  # Data Valid
        self.nrfd = Signal()  # Not Ready For Data
        self.ndac = Signal()  # Not Data Accepted

    def elaborate(self, platform):
        m = Module()

        m.submodules.dio_buffer  = dio_buffer  = io.Buffer("i", self.ports.dio)
        m.submodules.dav_buffer  = dav_buffer  = io.Buffer("i", self.ports.dav)
        m.submodules.eoi_buffer  = eoi_buffer  = io.Buffer("i", self.ports.eoi)
        m.submodules.nrfd_buffer = nrfd_buffer = io.Buffer("o", self.ports.nrfd)
        m.submodules.ndac_buffer = ndac_buffer = io.Buffer("o", self.ports.ndac)

        m.d.comb += [
            nrfd_buffer.oe.eq(1),
            nrfd_buffer.o.eq(self.nrfd),
            ndac_buffer.oe.eq(1),
            ndac_buffer.o.eq(self.ndac),
            self.dio.eq(dio_buffer.i),
            self.eoi.eq(eoi_buffer.i),
            self.dav.eq(dav_buffer.i),
        ]

        return m

class GPIB(Elaboratable):
    def __init__(self, ports):
        self.bus = GPIBBus(ports)

        self.rx_data     = Signal(8)
        self.rx_data_rdy = Signal(1)
        self.rx_data_ack = Signal(1)

        self.eoi         = Signal(1)

    def elaborate(self, platform):
        m = Module()

        m.submodules.bus = self.bus

        m.d.sync += self.eoi.eq(self.bus.eoi)

        with m.FSM():
            with m.State("Idle"):
                m.d.sync += [
                    self.bus.ndac.eq(0),
                    self.bus.nrfd.eq(1),
                ]
                with m.If(~self.bus.dav):
                    m.next = "Read data"

            with m.State("Read data"):
                m.d.sync += [
                    self.rx_data.eq(~self.bus.dio),
                    self.bus.ndac.eq(1),
                    self.rx_data_rdy.eq(1),
                ]
                m.next = "Wait for acknowledgement"

            with m.State("Wait for acknowledgement"):
                with m.If(self.rx_data_ack):
                    m.d.sync += [
                        self.rx_data_rdy.eq(0),
                        self.bus.nrfd.eq(0),
                        self.bus.ndac.eq(0),
                    ]
                    m.next = "Wait for DAV Unasserted"

            with m.State("Wait for DAV Unasserted"):
                with m.If(self.bus.dav):
                    m.next = "Idle"

        return m

class GPIBSubtarget(Elaboratable):

    def __init__(self, ports, in_fifo, eoi):
        self.ports    = ports
        self.in_fifo  = in_fifo
        self.eoi      = eoi

        self.gpib = GPIB(ports)

    def elaborate(self, platform):
        m = Module()

        m.submodules.gpib = gpib = self.gpib

        m.d.comb += [
            self.in_fifo.w_data.eq(gpib.rx_data),
            self.in_fifo.w_en.eq(gpib.rx_data_rdy),
            gpib.rx_data_ack.eq(self.in_fifo.w_rdy),
            self.eoi.eq(gpib.eoi),
        ]

        return m


class GPIBApplet(GlasgowApplet):
    logger = logging.getLogger(__name__)
    help = "receive data from gpib"
    description = """
    Receive 'talk only' data from equipment, not bidirectional.
    """
    required_revision = "C0"

    @classmethod
    def add_build_arguments(cls, parser, access):
        super().add_build_arguments(parser, access)

        access.add_pin_set_argument(parser, "dio", width=range(1, 8), default=(0,1,2,3,15,14,13,12))
        access.add_pin_argument(parser, "eoi",  default=4)
        access.add_pin_argument(parser, "dav",  default=5)
        access.add_pin_argument(parser, "nrfd", default=6)
        access.add_pin_argument(parser, "ndac", default=7)

    def build(self, target, args):
        eoi, self.__addr_eoi = target.registers.add_ro(1)

        self.mux_interface = iface = target.multiplexer.claim_interface(self, args)
        subtarget = iface.add_subtarget(GPIBSubtarget(
            ports=iface.get_port_group(
                dio  = args.pin_set_dio,
                eoi  = args.pin_eoi,
                dav  = args.pin_dav,
                nrfd = args.pin_nrfd,
                ndac = args.pin_ndac,
            ),
            in_fifo=iface.get_in_fifo(),
            eoi=eoi
        ))

        self._sample_freq = target.sys_clk_freq

    @classmethod
    def add_run_arguments(cls, parser, access):
        super().add_run_arguments(parser, access)

    async def run(self, device, args):
        pull_high = set(args.pin_set_dio).union({
            args.pin_dav, args.pin_eoi
        })

        iface = await device.demultiplexer.claim_interface(self, self.mux_interface, args,
                                                           pull_high=pull_high)
        return iface

    @classmethod
    def add_interact_arguments(cls, parser):
        pass

    async def interact(self, device, args, gpib):
        eoi = True
        while eoi:
            read_data = await gpib.read()
            print(read_data.tobytes().decode('ascii'), end='')
            eoi = await device.read_register(self.__addr_eoi)

    @classmethod
    def tests(cls):
        pass
