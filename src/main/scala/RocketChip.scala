// See LICENSE for license details.

package rocketchip

import Chisel._
import junctions._
import uncore._
import rocket._
import rocket.Util._

/** Top-level parameters of RocketChip, values set in e.g. PublicConfigs.scala */

/** Number of tiles */
case object NTiles extends Field[Int]
/** Number of memory channels */
case object NMemoryChannels extends Field[Int]
/** Number of banks per memory channel */
case object NBanksPerMemoryChannel extends Field[Int]
/** Least significant bit of address used for bank partitioning */
case object BankIdLSB extends Field[Int]
/** Number of outstanding memory requests */
case object NOutstandingMemReqsPerChannel extends Field[Int]
/** Whether to use the slow backup memory port [VLSI] */
case object UseBackupMemoryPort extends Field[Boolean]
/** Function for building some kind of coherence manager agent */
case object BuildL2CoherenceManager extends Field[() => CoherenceAgent]
/** Function for building some kind of tile connected to a reset signal */
case object BuildTiles extends Field[Seq[(Bool) => Tile]]

/** Utility trait for quick access to some relevant parameters */
trait TopLevelParameters extends UsesParameters {
  val htifW = params(HTIFWidth)
  val nTiles = params(NTiles)
  val nMemChannels = params(NMemoryChannels)
  val nBanksPerMemChannel = params(NBanksPerMemoryChannel)
  val nBanks = nMemChannels*nBanksPerMemChannel
  val lsb = params(BankIdLSB)
  val nMemReqs = params(NOutstandingMemReqsPerChannel)
  val mifAddrBits = params(MIFAddrBits)
  val mifDataBeats = params(MIFDataBeats)
  require(lsb + log2Up(nBanks) < mifAddrBits)
}

class MemBackupCtrlIO extends Bundle {
  val en = Bool(INPUT)
  val in_valid = Bool(INPUT)
  val out_ready = Bool(INPUT)
  val out_valid = Bool(OUTPUT)
}

/** Top-level io for the chip */
class BasicTopIO extends Bundle {
  val host = new HostIO
  val mem_backup_ctrl = new MemBackupCtrlIO
}

class TopIO extends BasicTopIO {
  val mem = new MemIO
}

class MultiChannelTopIO extends BasicTopIO with TopLevelParameters {
  val mem = Vec.fill(nMemChannels){ new NASTIMasterIO }
}

/** Top-level module for the chip */
//TODO: Remove this wrapper once multichannel DRAM controller is provided
class Top extends Module with TopLevelParameters {
  val io = new TopIO
  if(!params(UseZscale)) {
    val temp = Module(new MultiChannelTop)
    val arb = Module(new NASTIArbiter(nMemChannels))
    val conv = Module(new MemIONASTISlaveIOConverter(params(CacheBlockOffsetBits)))
    arb.io.master <> temp.io.mem
    conv.io.nasti <> arb.io.slave
    io.mem <> conv.io.mem
    io.mem_backup_ctrl <> temp.io.mem_backup_ctrl
    io.host <> temp.io.host
  } else {
    val temp = Module(new ZscaleTop)
    io.host <> temp.io.host
  }
}

class MultiChannelTop extends Module with TopLevelParameters {
  val io = new MultiChannelTopIO

  // Build an Uncore and a set of Tiles
  val uncore = Module(new Uncore, {case TLId => "L1ToL2"})
  val tileList = uncore.io.htif zip params(BuildTiles) map { case(hl, bt) => bt(hl.reset) }

  // Connect each tile to the HTIF
  uncore.io.htif.zip(tileList).zipWithIndex.foreach {
    case ((hl, tile), i) =>
      tile.io.host.id := UInt(i)
      tile.io.host.reset := Reg(next=Reg(next=hl.reset))
      tile.io.host.pcr_req <> Queue(hl.pcr_req)
      hl.pcr_rep <> Queue(tile.io.host.pcr_rep)
      hl.ipi_req <> Queue(tile.io.host.ipi_req)
      tile.io.host.ipi_rep <> Queue(hl.ipi_rep)
      hl.debug_stats_pcr := tile.io.host.debug_stats_pcr
  }

  // Connect the uncore to the tile memory ports, HostIO and MemIO
  uncore.io.tiles_cached <> tileList.map(_.io.cached)
  uncore.io.tiles_uncached <> tileList.map(_.io.uncached)
  io.host <> uncore.io.host
  io.mem <> uncore.io.mem
  if(params(UseBackupMemoryPort)) { io.mem_backup_ctrl <> uncore.io.mem_backup_ctrl }
}

/** Wrapper around everything that isn't a Tile.
  *
  * Usually this is clocked and/or place-and-routed separately from the Tiles.
  * Contains the Host-Target InterFace module (HTIF).
  */
class Uncore extends Module with TopLevelParameters {
  val io = new Bundle {
    val host = new HostIO
    val mem = Vec.fill(nMemChannels){ new NASTIMasterIO }
    val tiles_cached = Vec.fill(nTiles){new ClientTileLinkIO}.flip
    val tiles_uncached = Vec.fill(nTiles){new ClientUncachedTileLinkIO}.flip
    val htif = Vec.fill(nTiles){new HTIFIO}.flip
    val mem_backup_ctrl = new MemBackupCtrlIO
  }

  val htif = Module(new HTIF(CSRs.mreset)) // One HTIF module per chip
  val outmemsys = Module(new OuterMemorySystem) // NoC, LLC and SerDes
  outmemsys.io.incoherent := htif.io.cpu.map(_.reset)
  outmemsys.io.htif_uncached <> htif.io.mem
  outmemsys.io.tiles_uncached <> io.tiles_uncached
  outmemsys.io.tiles_cached <> io.tiles_cached

  // Wire the htif to the memory port(s) and host interface
  io.host.debug_stats_pcr := htif.io.host.debug_stats_pcr
  htif.io.cpu <> io.htif
  io.mem <> outmemsys.io.mem
  if(params(UseBackupMemoryPort)) {
    outmemsys.io.mem_backup_en := io.mem_backup_ctrl.en
    VLSIUtils.padOutHTIFWithDividedClock(htif.io, outmemsys.io.mem_backup, io.mem_backup_ctrl, io.host, htifW)
  } else {
    htif.io.host.out <> io.host.out
    htif.io.host.in <> io.host.in
  }
}

/** The whole outer memory hierarchy, including a NoC, some kind of coherence
  * manager agent, and a converter from TileLink to MemIO.
  */ 
class OuterMemorySystem extends Module with TopLevelParameters {
  val io = new Bundle {
    val tiles_cached = Vec.fill(nTiles){new ClientTileLinkIO}.flip
    val tiles_uncached = Vec.fill(nTiles){new ClientUncachedTileLinkIO}.flip
    val htif_uncached = (new ClientUncachedTileLinkIO).flip
    val incoherent = Vec.fill(nTiles){Bool()}.asInput
    val mem = Vec.fill(nMemChannels){ new NASTIMasterIO }
    val mem_backup = new MemSerializedIO(htifW)
    val mem_backup_en = Bool(INPUT)
  }

  // Create a simple L1toL2 NoC between the tiles+htif and the banks of outer memory
  // Cached ports are first in client list, making sharerToClientId just an indentity function
  // addrToBank is sed to hash physical addresses (of cache blocks) to banks (and thereby memory channels)
  val ordered_clients = (io.tiles_cached ++ (io.tiles_uncached :+ io.htif_uncached).map(TileLinkIOWrapper(_))) 
  def sharerToClientId(sharerId: UInt) = sharerId
  def addrToBank(addr: Bits): UInt = if(nBanks > 1) addr(lsb + log2Up(nBanks) - 1, lsb) else UInt(0)
  val preBuffering = TileLinkDepths(2,2,2,2,2)
  val postBuffering = TileLinkDepths(0,0,1,0,0) //TODO: had EOS24 crit path on inner.release
  val l1tol2net = Module(
    if(nBanks == 1) new RocketChipTileLinkArbiter(sharerToClientId, preBuffering, postBuffering)
    else new RocketChipTileLinkCrossbar(addrToBank, sharerToClientId, preBuffering, postBuffering))

  // Create point(s) of coherence serialization
  val nManagers = nMemChannels * nBanksPerMemChannel
  val managerEndpoints = List.fill(nManagers) { params(BuildL2CoherenceManager)()}
  managerEndpoints.foreach { _.incoherent := io.incoherent }

  // Wire the tiles and htif to the TileLink client ports of the L1toL2 network,
  // and coherence manager(s) to the other side
  l1tol2net.io.clients <> ordered_clients
  l1tol2net.io.managers <> managerEndpoints.map(_.innerTL)

  // Create a converter between TileLinkIO and MemIO for each channel
  val outerTLParams = params.alterPartial({ case TLId => "L2ToMC" })
  val backendBuffering = TileLinkDepths(0,0,0,0,0)

  val interconnect = params(BuildNASTI)()

  for ((bank, i) <- managerEndpoints.zipWithIndex) {
    val conv = Module(new NASTIMasterIOTileLinkIOConverter)(outerTLParams)
    conv.io.tl <> bank.outerTL
    interconnect.io.masters(i) <> conv.io.nasti
  }

  val mem_channels = interconnect.io.slaves.take(nMemChannels)

  for (s <- interconnect.io.slaves.drop(nMemChannels)) {
    s.ar.ready := Bool(false)
    s.aw.ready := Bool(false)
    s.w.ready  := Bool(false)
    s.b.valid  := Bool(false)
    s.r.valid  := Bool(false)
  }

  // Create a SerDes for backup memory port
  if(params(UseBackupMemoryPort)) {
    VLSIUtils.doOuterMemorySystemSerdes(
      mem_channels, io.mem, io.mem_backup, io.mem_backup_en,
      nMemChannels, params(HTIFWidth), params(CacheBlockOffsetBits))
  } else { io.mem <> mem_channels }
}
