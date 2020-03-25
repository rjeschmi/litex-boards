#!/usr/bin/env python3

# Support for the Pano Logic Zero Client G2

import argparse

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex_boards.platforms import pano_logic_g2

from litex.soc.cores.clock import *

from litex.soc.interconnect import wishbone
from litex.soc.integration.soc_core import *
from litex.soc.integration.soc_sdram import *
from litex.soc.integration.builder import *

from litex.soc.integration.soc_core import soc_core_argdict, soc_core_args

from litedram.modules import MT47H32M16
from litedram.phy import s6ddrphy
from litedram.core import ControllerSettings

from liteeth.phy.gmii import LiteEthPHYGMII
from liteeth.core import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone

#from gateware import info
#from gateware import cas

#from targets.utils import csr_map_update


class _CRG(Module):
    def __init__(self, platform):
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_sdram_half = ClockDomain()
        self.clock_domains.cd_sdram_full_wr = ClockDomain()
        self.clock_domains.cd_sdram_full_rd = ClockDomain()

        self.reset = Signal()

        f0 = int(125e6)

        clk125 = platform.request(platform.default_clk_name)
        #clk125a = Signal()

        #self.specials += Instance("IBUFG", i_I=clk125, o_O=clk125a)

        clk125b = Signal()

        self.specials += Instance(
            "BUFIO2", p_DIVIDE=1,
            p_DIVIDE_BYPASS="TRUE", p_I_INVERT="FALSE",
            i_I=clk125, o_DIVCLK=clk125b)

        unbuf_sdram_full = Signal()
        unbuf_sdram_half_a = Signal()
        unbuf_sdram_half_b = Signal()
        unbuf_encoder = Signal()
        unbuf_sys = Signal()
        unbuf_unused = Signal()

        # PLL signals
        pll_lckd = Signal()
        pll_fb = Signal()
        self.specials.pll = Instance(
            "PLL_ADV",
            name="crg_pll_adv",
            p_SIM_DEVICE="SPARTAN6", p_BANDWIDTH="OPTIMIZED", p_COMPENSATION="INTERNAL",
            p_REF_JITTER=.01,
            i_DADDR=0, i_DCLK=0, i_DEN=0, i_DI=0, i_DWE=0, i_RST=0, i_REL=0,
            p_DIVCLK_DIVIDE=1,
            # Input Clocks (125MHz)
            i_CLKIN1=clk125b,
            p_CLKIN1_PERIOD=platform.default_clk_period,
            i_CLKIN2=0,
            p_CLKIN2_PERIOD=0.,
            i_CLKINSEL=1,
            # Feedback
            # (1000MHz) vco
            i_CLKFBIN=pll_fb, o_CLKFBOUT=pll_fb, o_LOCKED=pll_lckd,
            p_CLK_FEEDBACK="CLKFBOUT",
            p_CLKFBOUT_MULT=8, p_CLKFBOUT_PHASE=0.,
            # (200MHz) sdram wr rd
            o_CLKOUT0=unbuf_sdram_full, p_CLKOUT0_DUTY_CYCLE=.5,
            p_CLKOUT0_PHASE=0., p_CLKOUT0_DIVIDE=5,
            # (100MHz) unused
            o_CLKOUT1=unbuf_encoder, p_CLKOUT1_DUTY_CYCLE=.5,
            p_CLKOUT1_PHASE=0., p_CLKOUT1_DIVIDE=10,
            # (100MHz) sdram_half - sdram dqs adr ctrl
            o_CLKOUT2=unbuf_sdram_half_a, p_CLKOUT2_DUTY_CYCLE=.5,
            p_CLKOUT2_PHASE=270., p_CLKOUT2_DIVIDE=10,
            # (100MHz) off-chip ddr
            o_CLKOUT3=unbuf_sdram_half_b, p_CLKOUT3_DUTY_CYCLE=.5,
            p_CLKOUT3_PHASE=250., p_CLKOUT3_DIVIDE=10,
            # (100MHz) unused
            o_CLKOUT4=unbuf_unused, p_CLKOUT4_DUTY_CYCLE=.5,
            p_CLKOUT4_PHASE=0., p_CLKOUT4_DIVIDE=10,
            # ( 50MHz) sysclk
            o_CLKOUT5=unbuf_sys, p_CLKOUT5_DUTY_CYCLE=.5,
            p_CLKOUT5_PHASE=0., p_CLKOUT5_DIVIDE=20,
        )

        # power on reset?
        reset = ~platform.request("cpu_reset") | self.reset
        self.clock_domains.cd_por = ClockDomain()
        por = Signal(max=1 << 11, reset=(1 << 11) - 1)
        self.sync.por += If(por != 0, por.eq(por - 1))
        self.specials += AsyncResetSynchronizer(self.cd_por, reset)

        # System clock - 50MHz
        self.specials += Instance("BUFG", name="sys_bufg", i_I=unbuf_sys, o_O=self.cd_sys.clk)
        self.comb += self.cd_por.clk.eq(self.cd_sys.clk)
        self.specials += AsyncResetSynchronizer(self.cd_sys, ~pll_lckd | (por > 0))

        # SDRAM clocks
        # ------------------------------------------------------------------------------
        self.clk4x_wr_strb = Signal()
        self.clk4x_rd_strb = Signal()

        # sdram_full
        self.specials += Instance("BUFPLL", name="sdram_full_bufpll",
                                  p_DIVIDE=4,
                                  i_PLLIN=unbuf_sdram_full, i_GCLK=self.cd_sys.clk,
                                  i_LOCKED=pll_lckd,
                                  o_IOCLK=self.cd_sdram_full_wr.clk,
                                  o_SERDESSTROBE=self.clk4x_wr_strb)
        self.comb += [
            self.cd_sdram_full_rd.clk.eq(self.cd_sdram_full_wr.clk),
            self.clk4x_rd_strb.eq(self.clk4x_wr_strb),
        ]
        # sdram_half
        self.specials += Instance("BUFG", name="sdram_half_a_bufpll", i_I=unbuf_sdram_half_a, o_O=self.cd_sdram_half.clk)
        clk_sdram_half_shifted = Signal()
        self.specials += Instance("BUFG", name="sdram_half_b_bufpll", i_I=unbuf_sdram_half_b, o_O=clk_sdram_half_shifted)

        output_clk = Signal()
        clk = platform.request("ddram_clock_b")
        self.specials += Instance("ODDR2", p_DDR_ALIGNMENT="NONE",
                                  p_INIT=0, p_SRTYPE="SYNC",
                                  i_D0=1, i_D1=0, i_S=0, i_R=0, i_CE=1,
                                  i_C0=clk_sdram_half_shifted,
                                  i_C1=~clk_sdram_half_shifted,
                                  o_Q=output_clk)
        self.specials += Instance("OBUFDS", i_I=output_clk, o_O=clk.p, o_OB=clk.n)

class BaseSoC(SoCCore):

    def __init__(self, **kwargs):
        if 'integrated_rom_size' not in kwargs:
            kwargs['integrated_rom_size']=0x8000
        if 'integrated_sram_size' not in kwargs:
            kwargs['integrated_sram_size']=0x8000

        platform = pano_logic_g2.Platform()
        clk_freq = int(125e6)
        SoCCore.__init__(self, platform, clk_freq, **kwargs)

        self.submodules.crg = _CRG(platform)

        # sdram
        sdram_module = MT47H32M16(self.clk_freq, "1:2")
        self.submodules.ddrphy = s6ddrphy.S6HalfRateDDRPHY(
            platform.request("ddram_b"),
            sdram_module.memtype,
            rd_bitslip=0,
            wr_bitslip=4,
            dqs_ddr_alignment="C0")
        self.add_csr("ddrphy")
        controller_settings = ControllerSettings(with_bandwidth=True)
        self.add_sdram ("sdram",
                phy =       self.ddrphy,
                module = MT47H32M16(self.clk_freq, "1:2"),
                origin = self.mem_map["main_ram"],
                size = kwargs.get("max_sdram_size", 0x40000000),
                l2_cache_size = kwargs.get("l2_size", 8192),
                l2_cache_min_data_width = kwargs.get("min_l2_data_width", 128),
                l2_cache_reverse = True
                )
        self.submodules.ethphy = LiteEthPHYGMII(
            self.platform.request("eth_clocks"),
            self.platform.request("eth"))
        self.add_csr("ethphy")
        self.add_etherbone(phy=self.ethphy)


def main():
    parser = argparse.ArgumentParser("LiteX SoC on PanoLogic")
    parser.add_argument("--with-etherbone", action="store_true", help="enable Etherbone support")
    builder_args(parser)
    soc_sdram_args(parser)
    args = parser.parse_args()
    
    soc = BaseSoC()

    builder = Builder(soc, **builder_argdict(args))
    builder.build()


if __name__ == "__main__":
    main()
