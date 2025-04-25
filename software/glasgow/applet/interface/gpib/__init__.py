import logging
import argparse
import math
from amaranth import *
from amaranth.lib import io
from amaranth.lib.cdc import FFSynchronizer

from ....gateware.analyzer import *
from ... import *

"""
GPIB / IEEE-488 is a 16 line bus, with a single controller (in this case, the
controller will be the Glasgow. The bus can be in one of two modes, depending
on the ATN line (active low). When ATN is low, all other devices on the bus
must listen to the controller. When high, only the addressed device needs to
listen.

The sixteen lines can be broken into three groups. These are the data lines (x8),
the bus management lines (x5) and the handshake lines (x3).

  *** DATA LINES  ***
  DIOx   - There are eight data I/O lines.

  *** BUS MANAGEMENT LINES ***
  ATN    - Attention
           This dictates whether we are in command or data mode.
  EOI    - End-or-Identify
           Any device on the bus can use this to signal the end of binary data, or
           to delimit textual data.
  IFC    - Interface Clear
           Allows the controller (us) to instruct all devices on the bus to reset
           their bus function to the initial state.
  SRQ    - Service Request
           All devices, aside from the controller, can use this line to indicate to
           the controller that something has finished, or that an error has occured.
           When this line is pulled low by a device, the controller should poll to
           find out which device is asking for service and what they want.
  REN    - Remote Enable

  *** HANDSHAKE LINES ***
  DAV    - Data Valid
           A device pulls this line high when it is sending data.
  NRFD   - Not Ready for Data
           A device pulls this line height when data hasn't been fully received yet.
  NDAC   - Not Data Accepted
           A device is not ready to receive data yet.

Whether the ports are inputs or outputs is dictated by whether the device is
listening or talking.

+--------++------+-----+-----+------+------+--*--+--*--+--*--+--*--+
| Action || DIOx | EOI | DAV | NRFD | NDAC | IFC | SRQ | ATN | REN |
+--------++------+-----+-----+------+------+-----+-----+-----+-----+
| Talk   || OUT  | OUT | OUT | IN   | IN   | OUT | IN  | OUT | OUT |
| Listen || IN   | IN  | IN  | OUT  | OUT  | OUT | IN  | OUT | OUT |
+--------++------+-----+-----+------+------+-----+-----+-----+-----+
"""

class GPIBBus(Elaboratable):
    def __init__(self, ports):
        self.ports = ports

        # High for Talk, Low for Listen
        self.activity = Signal()

        self.dio  = Signal(8)
        self.eoi  = Signal() # End or Identify
        self.dav  = Signal() # Data Valid
        self.nrfd = Signal() # Not Ready For Data
        self.ndac = Signal() # Not Data Accepted
        self.ifc  = Signal() # Interface Clear
        self.srq  = Signal() # Service Request
        self.atn  = Signal() # Attention!
        self.ren  = Signal() # Remote Enable

    def elaborate(self, platform):
        m = Module()

        m.submodules.dio_buffer  = dio_buffer  = io.FFBuffer("io", self.ports.dio)
        m.submodules.dav_buffer  = dav_buffer  = io.FFBuffer("io", self.ports.dav)
        m.submodules.eoi_buffer  = eoi_buffer  = io.FFBuffer("io", self.ports.eoi)
        m.submodules.nrfd_buffer = nrfd_buffer = io.FFBuffer("io", self.ports.nrfd)
        m.submodules.ndac_buffer = ndac_buffer = io.FFBuffer("io", self.ports.ndac)

        m.submodules.ifc_buffer  = ifc_buffer  = io.Buffer("o", self.ports.ifc)
        m.submodules.atn_buffer  = atn_buffer  = io.Buffer("o", self.ports.atn)
        m.submodules.ren_buffer  = ren_buffer  = io.Buffer("o", self.ports.ren)
        m.submodules.srq_buffer  = srq_buffer  = io.Buffer("i", self.ports.srq)

        m.d.sync += [
            ifc_buffer.o.eq(self.ifc),
            atn_buffer.o.eq(self.atn),
            ren_buffer.o.eq(self.ren),
            self.srq.eq(srq_buffer.i),
        ]

        # Talk
        with m.If(self.activity):
            m.d.sync += [
                dio_buffer.o.eq(self.dio),
                eoi_buffer.o.eq(self.eoi),
                dav_buffer.o.eq(self.dav),
                self.nrfd.eq(nrfd_buffer.i),
                self.ndac.eq(ndac_buffer.i),
            ]

        # Listen
        with m.Else():
            m.d.sync += [
                self.dio.eq(dio_buffer.i),
                self.eoi.eq(eoi_buffer.i),
                self.dav.eq(dav_buffer.i),
                nrfd_buffer.o.eq(self.nrfd),
                ndac_buffer.o.eq(self.ndac),
            ]

        return m

class GPIB(Elaboratable):


    def __init__(self, ports):
        self.bus = GPIBBus(ports)

        self.mode    = Signal(1) # Command (Low) or Data (High)
        self.rx_data = Signal(8)
        self.tx_data = Signal(8)

    def elaborate(self, platform):
        delay_cycles = math.ceil(1e10 * platform.default_clk_frequency)
        timer = Signal(range(delay_cycles))

        # If there's data waiting to be sent
        # Feel like I need a separate flag for this, so we can still send null characters....
        with m.If(self.tx_data.i):
            m.d.sync += self.bus.activity.eq(1)

            with m.FSM():
                with m.State("Idle"):
                    with m.If(self.tx_data.i):
                        m.next("Start")

                with m.State("Start"):
                    m.d.sync += self.bus.dav.eq(0)
                    m.next("Check NRFD and NDAC are low")

                with m.State("Check NRFD and NDAC are low"):
                    # NRFD is active low, NDAC is active high
                    with m.If(self.bus.nrfd & ~self.bus.ndac):
                        m.next("Set mode")
                    with m.Else():
                        m.next("Error")

                with m.State("Set mode"):
                    m.d.sync += self.bus.atn.eq(self.mode)
                    m.next("Set DIO lines")

                with m.State("Set DIO lines"):
                    m.d.sync += [
                        self.bus.dio.eq(self.tx_data),
                        m.d.sync += timer.eq(delay_cycles),
                    ]
                    m.next("Wait for lines to settle")

                with m.State("Wait for lines to settle"):
                    with m.If(timer == 0):
                        m.next("Assert DAV")
                    with m.Else():
                        m.d.sync += timer.eq(timer - 1)

                with m.State("Assert DAV"):
                    m.d.sync += self.bus.dav.eq(1)
                    with m.If(~data.bus.ndac):
                        m.State("Unassert DAV")

                with m.State("Unassert DAV"):
                    m.d.sync += [
                        self.bus.dav.eq(0),
                        self.bus.dio.eq(0),
                    ]
                    m.next("Idle")

                with m.State("Error"):
                    pass

        with m.Else():
            m.d.sync += self.bus.activity.eq(0) # Listen
            with m.FSM():
                with m.State("Idle"):
                    # Tell the bus we're ready to receive
                    m.d.sync += self.bus.nrfd.eq(0)
                    m.d.sync += self.bus.ndac.eq(1)

                    # with a timeout...
                    m.d.sync += timer.eq(delay_cycles)

                    m.next("Check DAV")

                with m.State("Check DAV"):
                    with m.If(timer == 0):
                        m.d.sync += timer.eq(timer - 1)
                    with m.Else():
                        with m.If(self.bus.dav):
                            m.next("Assert NRFD")

                with m.State("Assert NRFD"):
                    m.d.sync += self.bus.nrfd.eq(1)
                    m.next("Read data")

                with m.State("Read data"):
                    # Do something with what's on DIOx
                    m.next("Unassert NDAC")

                with m.State("Unassert NDAC"):
                    m.d.sync += self.bus.ndac.eq(0)
                    m.next("Wait for DAV to be unasserted")

                with m.State("Wait for DAV to be unasserted"):
                    with m.If(~self.bus.dav):
                        m.next("Idle")


class GPIBSubtarget(Elaboratable):

    def __init__(self, ports, out_fifo, in_fifo):
        self.ports    = ports
        self.out_fifo = out_fifo
        self.in_fifi  = in_fifo

        self.gpib = GPIB(ports)

    def elaborate(self, platform):
        m = Module()


        m.d.sync +=

        return m


class GPIBInterface:
    def __init__(self, interface, event_sources):
        self.lower   = interface
        self.decoder = TraceDecoder(event_sources)

    async def read(self):
        self.decoder.process(await self.lower.read())
        return self.decoder.flush()


class GPIBApplet(GlasgowApplet):
    logger = logging.getLogger(__name__)
    help = "talks to a gpib device"
    description = """
    Talk to a GPIB device, e.g. for doing oscilloscope acquisitions
    """
    required_revision = "C0" # iCE40UP5K isn't quite fast enoughs

    @classmethod
    def add_build_arguments(cls, parser, access):
        super().add_build_arguments(parser, access)

        access.add_pin_set_argument(parser, "i", width=range(1, 17), default=1)
        parser.add_argument(
            "--pin-names", metavar="NAMES", default=None,
            help="optional comma separated list of pin names")

    def build(self, target, args):
        self.mux_interface = iface = target.multiplexer.claim_interface(self, args)
        subtarget = iface.add_subtarget(AnalyzerSubtarget(
            ports=iface.get_port_group(i = args.pin_set_i),
            in_fifo=iface.get_in_fifo(),
            trigger_pattern=args.trigger,
        ))

        self._sample_freq = target.sys_clk_freq
        self._event_sources = subtarget.analyzer.event_sources

    @classmethod
    def add_run_arguments(cls, parser, access):
        super().add_run_arguments(parser, access)

        g_pulls = parser.add_mutually_exclusive_group()
        g_pulls.add_argument(
            "--pull-ups", default=False, action="store_true",
            help="enable pull-ups on all pins")
        g_pulls.add_argument(
            "--pull-downs", default=False, action="store_true",
            help="enable pull-downs on all pins")

    async def run(self, device, args):
        pull_low  = set()
        pull_high = set()
        if args.pull_ups:
            pull_high = set(args.pin_set_i)
        if args.pull_downs:
            pull_low = set(args.pin_set_i)
        iface = await device.demultiplexer.claim_interface(self, self.mux_interface, args,
                                                           pull_low=pull_low, pull_high=pull_high)
        return AnalyzerInterface(iface, self._event_sources)

    @classmethod
    def add_interact_arguments(cls, parser):
        parser.add_argument(
            "file", metavar="VCD-FILE", type=argparse.FileType("w"),
            help="write VCD waveforms to VCD-FILE")

    async def interact(self, device, args, iface):
        vcd_writer = VCDWriter(args.file, timescale="1 ns", check_values=False)
        signals = []

        names = []
        if args.pin_names:
            names = args.pin_names.split(",")
            assert len(names) == self._event_sources[0].width
        else:
            names = [ f"pin[{index}]" for index in range(self._event_sources[0].width) ]

        for index in range(self._event_sources[0].width):
            signals.append(vcd_writer.register_var(scope="glasgow", name=names[index],
                var_type="wire", size=1, init=0))

        try:
            overrun = False
            timestamp = 0
            trigger_offset = 0
            while not overrun:
                for cycle, events in await iface.read():
                    timestamp = cycle * 1_000_000_000 // self._sample_freq

                    if events == "overrun":
                        self.logger.error("FIFO overrun, shutting down")
                        for signal in signals:
                            vcd_writer.change(signal, timestamp - trigger_offset, "x")
                        overrun = True
                        break

                    if "triggered" in events:
                        trigger_offset = timestamp
                        self.logger.info(f"Triggered after {int(timestamp/1e3)}us")

                    if "pin" in events: # could be also "throttle"
                        value = events["pin"]
                        for bit, signal in enumerate(signals):
                            vcd_writer.change(signal, timestamp - trigger_offset, (value >> bit) & 1)

        finally:
            vcd_writer.close(timestamp)

    @classmethod
    def tests(cls):
        from . import test
        return test.AnalyzerAppletTestCase
