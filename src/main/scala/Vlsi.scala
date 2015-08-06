// See LICENSE for license details.

package rocketchip

import Chisel._
import junctions._
import uncore._

class MemDessert extends Module {
  val io = new MemDesserIO(params(HTIFWidth))
  val x = Module(new MemDesser(params(HTIFWidth)))
  io.narrow <> x.io.narrow
  io.wide <> x.io.wide
}

object VLSIUtils {
  def doOuterMemorySystemSerdes(
      llcs: Seq[NASTIMasterIO],
      mems: Seq[NASTIMasterIO],
      backup: MemSerializedIO,
      en: Bool,
      nMemChannels: Int,
      htifWidth: Int,
      blockOffsetBits: Int) {

    val arb = Module(new NASTIArbiter(nMemChannels))
    val conv = Module(new MemIONASTISlaveIOConverter(blockOffsetBits))
    val mem_serdes = Module(new MemSerdes(htifWidth))

    conv.io.nasti <> arb.io.slave
    mem_serdes.io.wide <> conv.io.mem
    backup <> mem_serdes.io.narrow

    llcs zip mems zip arb.io.master foreach { case ((llc, mem), wide) =>
      llc.ar.ready := Mux(en, wide.ar.ready, mem.ar.ready)
      mem.ar.valid := llc.ar.valid && !en
      mem.ar.bits := llc.ar.bits
      wide.ar.valid := llc.ar.valid && en
      wide.ar.bits := llc.ar.bits

      llc.aw.ready := Mux(en, wide.aw.ready, mem.aw.ready)
      mem.aw.valid := llc.aw.valid && !en
      mem.aw.bits := llc.aw.bits
      wide.aw.valid := llc.aw.valid && en
      wide.aw.bits := llc.aw.bits

      llc.w.ready := Mux(en, wide.w.ready, mem.w.ready)
      mem.w.valid := llc.w.valid && !en
      mem.w.bits := llc.w.bits
      wide.w.valid := llc.w.valid && en
      wide.w.bits := llc.w.bits

      llc.b.valid := Mux(en, wide.b.valid, mem.b.valid)
      llc.b.bits := Mux(en, wide.b.bits, mem.b.bits)
      mem.b.ready  := llc.b.ready && !en
      wide.b.ready := llc.b.ready && en

      llc.r.valid := Mux(en, wide.r.valid, mem.r.valid)
      llc.r.bits := Mux(en, wide.r.bits, mem.r.bits)
      mem.r.ready  := llc.r.ready && !en
      wide.r.ready := llc.r.ready && en
    }
  }

  def padOutHTIFWithDividedClock(
      htif: HTIFModuleIO,
      child: MemSerializedIO, 
      parent: MemBackupCtrlIO,
      host: HostIO,
      htifW: Int) {
    val hio = Module((new SlowIO(512)) { Bits(width = htifW+1) })
    hio.io.set_divisor.valid := htif.scr.wen && (htif.scr.waddr === UInt(63))
    hio.io.set_divisor.bits := htif.scr.wdata
    htif.scr.rdata(63) := hio.io.divisor

    hio.io.out_fast.valid := htif.host.out.valid || child.req.valid
    hio.io.out_fast.bits := Cat(htif.host.out.valid, Mux(htif.host.out.valid, htif.host.out.bits, child.req.bits))
    htif.host.out.ready := hio.io.out_fast.ready
    child.req.ready := hio.io.out_fast.ready && !htif.host.out.valid
    host.out.valid := hio.io.out_slow.valid && hio.io.out_slow.bits(htifW)
    host.out.bits := hio.io.out_slow.bits
    parent.out_valid := hio.io.out_slow.valid && !hio.io.out_slow.bits(htifW)
    hio.io.out_slow.ready := Mux(hio.io.out_slow.bits(htifW), host.out.ready, parent.out_ready)

    val mem_backup_resp_valid = parent.en && parent.in_valid
    hio.io.in_slow.valid := mem_backup_resp_valid || host.in.valid
    hio.io.in_slow.bits := Cat(mem_backup_resp_valid, host.in.bits)
    host.in.ready := hio.io.in_slow.ready
    child.resp.valid := hio.io.in_fast.valid && hio.io.in_fast.bits(htifW)
    child.resp.bits := hio.io.in_fast.bits
    htif.host.in.valid := hio.io.in_fast.valid && !hio.io.in_fast.bits(htifW)
    htif.host.in.bits := hio.io.in_fast.bits
    hio.io.in_fast.ready := Mux(hio.io.in_fast.bits(htifW), Bool(true), htif.host.in.ready)
    host.clk := hio.io.clk_slow
    host.clk_edge := Reg(next=host.clk && !Reg(next=host.clk))
  }
}
