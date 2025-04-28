import logging
import argparse
import math
import asyncio
from amaranth import *
from amaranth.lib import io
from amaranth.lib.cdc import FFSynchronizer

from ... import *

"""
GPIB / IEEE-488 is a 16 line bus, with a single controller (in this case, the
controller will be the Glasgow). The bus can be in one of two modes, depending
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
           Allows the controller to instruct all devices on the bus to reset their
           bus function to the initial state.
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

All lines use active low signalling. It probably would have made sense to invert the pins.
"""

class GPIBBus(Elaboratable):
    def __init__(self, ports):
        self.ports = ports

        self.dio_i  = Signal(8) # Data Lines            Listen
        self.dio_o  = Signal(8) #                       Talk
        self.eoi_i  = Signal()  # End or Identify       Listen
        self.eoi_o  = Signal()  #                       Talk
        self.dav_i  = Signal()  # Data Valid            Listen
        self.dav_o  = Signal()  #                       Talk
        self.nrfd_i = Signal()  # Not Ready For Data    Talk
        self.nrfd_o = Signal()  #                       Listen
        self.ndac_i = Signal()  # Not Data Accepted     Talk
        self.ndac_o = Signal()  #                       Listen
        self.srq_i  = Signal()  # Service Request       Any
        self.ifc_o  = Signal()  # Interface Clear       Any
        self.atn_o  = Signal()  # Attention             Any
        self.ren_o  = Signal()  # Remote Enable         Any

        self.direction = Signal()  # Talkng (HIGH) or Listening (LOW)

    def elaborate(self, platform):
        m = Module()

        m.submodules.dio_buffer  = dio_buffer  = io.Buffer("io", self.ports.dio)
        m.submodules.eoi_buffer  = eoi_buffer  = io.Buffer("io", self.ports.eoi)
        m.submodules.dav_buffer  = dav_buffer  = io.Buffer("io", self.ports.dav)
        m.submodules.nrfd_buffer = nrfd_buffer = io.Buffer("io", self.ports.nrfd)
        m.submodules.ndac_buffer = ndac_buffer = io.Buffer("io", self.ports.ndac)
        m.submodules.srq_buffer  = srq_buffer  = io.Buffer("i", self.ports.srq)
        m.submodules.ifc_buffer  = ifc_buffer  = io.Buffer("o", self.ports.ifc)
        m.submodules.atn_buffer  = atn_buffer  = io.Buffer("o", self.ports.atn)
        m.submodules.ren_buffer  = ren_buffer  = io.Buffer("o", self.ports.ren)

        # To make the rest of this easier to follow, we can break out
        # the direction into two signals.
        talking   = Signal()
        listening = Signal()
        m.d.sync += [
            talking.eq(self.direction),
            listening.eq(~self.direction),
        ]

        # and use that to determine if an output should be enabled.
        # srq is input only, so it does not get an oe signal.
        m.d.comb += [
            dio_buffer.oe.eq(talking),
            eoi_buffer.oe.eq(talking),
            dav_buffer.oe.eq(talking),
            nrfd_buffer.oe.eq(listening),
            ndac_buffer.oe.eq(listening),
            ifc_buffer.oe.eq(talking),
            atn_buffer.oe.eq(listening),
            ren_buffer.oe.eq(listening),
        ]

        # Some lines never change from the controller's perspective.
        m.d.sync += [
            self.srq_i.eq(srq_buffer.i),
            ifc_buffer.o.eq(self.ifc_o),
            atn_buffer.o.eq(self.atn_o),
            ren_buffer.o.eq(self.ren_o),
        ]

        with m.If(listening):
            m.d.sync += [
                self.dio_i.eq(dio_buffer.i),
                self.eoi_i.eq(eoi_buffer.i),
                self.dav_i.eq(dav_buffer.i),
                nrfd_buffer.o.eq(self.nrfd_o),
                ndac_buffer.o.eq(self.ndac_o),
            ]

        with m.If(talking):
            m.d.comb += [
                dio_buffer.o.eq(self.dio_o),
                eoi_buffer.o.eq(self.eoi_o),
                dav_buffer.o.eq(self.dav_o),
                self.nrfd_i.eq(nrfd_buffer.i),
                self.ndac_i.eq(ndac_buffer.i),
            ]

        return m

class GPIB(Elaboratable):
    def __init__(self, ports):
        self.bus = GPIBBus(ports)

        self.rx_data     = Signal(8)
        self.rx_data_rdy = Signal(1)
        self.rx_data_ack = Signal(1)

        self.tx_data     = Signal(8)
        self.tx_data_rdy = Signal(1)
        self.tx_data_ack = Signal(1)

        self.direction   = Signal(1) # Talk = HIGH,  Listen = LOW

        self.eoi         = Signal(1)
        self.atn         = Signal(1)

    def elaborate(self, platform):
        m = Module()

        m.submodules.bus = self.bus

        # Some signals need to be accessible as registers.
        # EOI       - Allows the interact know when it should stop reading
        # Direction - Determines whether we are listening or talking.
        #             The state of pull up resistors is handled by interact.
        # ATN       - When active, puts the GPIB into Command mode.
        m.d.sync += [
            self.eoi.eq(self.bus.eoi_i),
            self.bus.direction.eq(self.direction),
            self.bus.atn_o.eq(self.atn),
        ]

        with m.FSM():
            # Determine whether we're in talk or listen mode
            with m.State("Direction"):
                with m.If(self.direction):
                    m.next = "Talk: Begin"
                with m.Else():
                    m.next = "Listen: Begin"

            # Listen!
            with m.State("Listen: Begin"):
                m.d.sync += [
                    self.bus.ndac_o.eq(0),
                    self.bus.nrfd_o.eq(1),
                    self.bus.ifc_o.eq(1),
                    self.bus.atn_o.eq(1),
                    self.bus.ren_o.eq(1),
                ]
                with m.If(~self.bus.dav_i):
                    m.next = "Listen: Read data"

            with m.State("Listen: Read data"):
                m.d.sync += [
                    self.rx_data.eq(~self.bus.dio_i),
                    self.bus.ndac_o.eq(1),
                    self.rx_data_rdy.eq(1),
                ]
                m.next = "Listen: Wait for acknowledgement"

            with m.State("Listen: Wait for acknowledgement"):
                with m.If(self.rx_data_ack):
                    m.d.sync += [
                        self.rx_data_rdy.eq(0),
                        self.bus.nrfd_o.eq(0),
                        self.bus.ndac_o.eq(0),
                    ]
                    m.next = "Listen: Wait for DAV Unasserted"

            with m.State("Listen: Wait for DAV Unasserted"):
                with m.If(self.bus.dav_i):
                    m.next = "Direction"

            # Talk!
            with m.State("Talk: Begin"):
                m.d.sync += [
                    self.bus.dav_o.eq(1),
                    self.tx_data_rdy.eq(0)
                ]
                with m.If(self.tx_data_ack):
                    m.next = "Talk: Send data"

            with m.State("Talk: Send data"):
                m.d.sync += [
                    self.bus.dav_o.eq(1),
                ]
                with m.If(~self.bus.ndac_i & self.bus.nrfd_i):
                    m.d.sync += [
                        self.bus.dio_o.eq(~self.tx_data),
                    ]
                    m.next = "Talk: Await acknowledge"

            with m.State("Talk: Await acknowledge"):
                with m.If(self.tx_data_ack):
                    m.d.sync += [
                        self.tx_data_rdy.eq(1),
                        self.bus.dav_o.eq(0),
                    ]
                with m.If(self.bus.ndac_i):
                    m.next = "Direction"

        return m

class GPIBSubtarget(Elaboratable):

    def __init__(self, ports, in_fifo, out_fifo, eoi, direction):
        self.ports     = ports
        self.in_fifo   = in_fifo
        self.out_fifo  = out_fifo
        self.eoi       = eoi
        self.direction = direction

        self.gpib = GPIB(ports)

    def elaborate(self, platform):
        m = Module()

        m.submodules.gpib = gpib = self.gpib

        m.d.comb += [
            self.in_fifo.w_data.eq(gpib.rx_data),
            self.in_fifo.w_en.eq(gpib.rx_data_rdy),
            gpib.rx_data_ack.eq(self.in_fifo.w_rdy),
        ]

        m.d.comb += [
            gpib.tx_data.eq(self.out_fifo.r_data),
            gpib.tx_data_ack.eq(self.out_fifo.r_rdy),
            self.out_fifo.r_en.eq(gpib.tx_data_rdy),
        ]

        m.d.comb += [
            self.eoi.eq(gpib.eoi),
            gpib.direction.eq(self.direction),
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
        access.add_pin_argument(parser, "srq",  default=9)
        access.add_pin_argument(parser, "ifc",  default=10)
        access.add_pin_argument(parser, "atn",  default=8)
        access.add_pin_argument(parser, "ren",  default=11)

    def build(self, target, args):
        eoi,       self.__addr_eoi       = target.registers.add_ro(1)
        direction, self.__addr_direction = target.registers.add_rw(1)

        self.mux_interface = iface = target.multiplexer.claim_interface(self, args)
        subtarget = iface.add_subtarget(GPIBSubtarget(
            ports=iface.get_port_group(
                dio  = args.pin_set_dio,
                eoi  = args.pin_eoi,
                dav  = args.pin_dav,
                nrfd = args.pin_nrfd,
                ndac = args.pin_ndac,
                srq  = args.pin_srq,
                ifc  = args.pin_ifc,
                atn  = args.pin_atn,
                ren  = args.pin_ren,
            ),
            in_fifo=iface.get_in_fifo(),
            out_fifo=iface.get_out_fifo(),
            eoi=eoi, direction=direction
        ))

        self._sample_freq = target.sys_clk_freq

    @classmethod
    def add_run_arguments(cls, parser, access):
        super().add_run_arguments(parser, access)

    async def run(self, device, args):
        self.listen_pull_high = default_pull_high = set(args.pin_set_dio).union({
            args.pin_eoi, args.pin_dav, args.pin_ifc, args.pin_atn, args.pin_ren
        })
        self.talk_pull_high   = {
            args.pin_nrfd, args.pin_ndac, args.pin_srq, args.pin_ifc, args.pin_atn, args.pin_ren
        }
        iface = await device.demultiplexer.claim_interface(self, self.mux_interface, args,
                                                           pull_high=default_pull_high)
        return iface

    @classmethod
    def add_interact_arguments(cls, parser):
        pass

    async def talk(self, device, args, gpib, data):
        await device.set_pulls(args.port_spec, high={pin.number for pin in self.talk_pull_high})
        await device.write_register(self.__addr_direction, 1)

    async def listen(self, device, args, gpib):
        listen_pull_high = set(args.pin_set_dio).union({
            args.pin_eoi, args.pin_dav, args.pin_ifc, args.pin_atn, args.pin_ren
        })
        await device.set_pulls(args.port_spec, high={pin.number for pin in self.listen_pull_high})
        await device.write_register(self.__addr_direction, 0)
        return (await gpib.read()).tobytes().decode('ascii')

    async def interact(self, device, args, gpib):
        while True:
            print(await self.listen(device, args, gpib), end='')

        # await device.write_register(self.__addr_direction, 0)
        # eoi = True
        # while True:
        #     read_data = await gpib.read()
        #     print(read_data.tobytes().decode('ascii'), end='')
        #     eoi = await device.read_register(self.__addr_eoi)

    @classmethod
    def tests(cls):
        pass
