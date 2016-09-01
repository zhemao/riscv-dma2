package dma

import Chisel._
import groundtest._
import rocket.{TLBPTWIO, HasCoreParameters}
import uncore.tilelink._
import uncore.agents.CacheBlockBytes
import uncore.util._
import uncore.constants._
import junctions.{Timer, PAddrBits}
import rocket._
import cde.{Parameters, Field}

case class DmaTestParameters(
  val src_start: Int,
  val dst_start: Int,
  val segment_size: Int,
  val nsegments: Int,
  val src_stride: Int,
  val dst_stride: Int)

case object DmaTestKey extends Field[DmaTestParameters]

class VirtualMemAdapter(implicit val p: Parameters) extends Module
    with HasCoreParameters
    with HasTileLinkParameters {
  val io = new Bundle {
    val vmem = new ClientUncachedTileLinkIO().flip
    val pmem = new ClientUncachedTileLinkIO
    val ptw  = new TLBPTWIO
  }

  val tlBlockOffset = tlBeatAddrBits + tlByteAddrBits

  val cur_vpn = Reg(UInt(width = vpnBits))
  val cur_ppn = Reg(UInt(width = ppnBits))
  val trans_valid = Reg(init = Bool(false))
  val trans_inflight = Reg(init = Bool(false))

  val req_vaddr = io.vmem.acquire.bits.full_addr()
  val req_vpn = req_vaddr(paddrBits - 1, pgIdxBits)
  val req_idx = req_vaddr(pgIdxBits - 1, 0)
  val req_paddr = Cat(cur_ppn, req_idx)
  val req_block = req_paddr(paddrBits - 1, tlBlockOffset)

  val req_ok = trans_valid && cur_vpn === req_vpn

  io.pmem.acquire.valid := io.vmem.acquire.valid && req_ok
  io.vmem.acquire.ready := io.pmem.acquire.ready && req_ok
  io.pmem.acquire.bits := io.vmem.acquire.bits
  io.pmem.acquire.bits.addr_block := req_block
  io.vmem.grant <> io.pmem.grant

  io.ptw.req.valid := io.vmem.acquire.valid && !req_ok && !trans_inflight
  io.ptw.req.bits.prv := UInt(0)
  io.ptw.req.bits.pum := Bool(false)
  io.ptw.req.bits.mxr := Bool(false)
  io.ptw.req.bits.store := Bool(true)
  io.ptw.req.bits.fetch := Bool(false)
  io.ptw.req.bits.addr := req_vpn

  when (io.ptw.req.fire()) {
    trans_valid := Bool(false)
    trans_inflight := Bool(true)
    cur_vpn := req_vpn
  }

  when (io.ptw.resp.valid) {
    trans_valid := Bool(true)
    trans_inflight := Bool(false)
    cur_ppn := io.ptw.resp.bits.pte.ppn
  }

  assert(!io.ptw.resp.valid || io.ptw.resp.bits.pte.leaf(),
         "page table lookup is invalid")
}

class DmaTestDriver(implicit val p: Parameters)
    extends Module with HasTileLinkParameters {

  val io = new Bundle {
    val mem = new HellaCacheIO
    val dma = new ClientDmaIO
    val busy = Bool(INPUT)
    val finished = Bool(OUTPUT)
  }

  val testParams = p(DmaTestKey)
  require(Seq(
      testParams.src_start,
      testParams.dst_start,
      testParams.segment_size,
      testParams.src_stride,
      testParams.dst_stride).map(_ % 4 == 0).reduce(_ && _),
    "DmaTest parameters must have 4 byte alignment")

  val (word_cnt, word_flip) = Counter(io.mem.resp.valid, testParams.segment_size / 4)
  val (segment_cnt, segment_flip) = Counter(word_flip, testParams.nsegments)
  val segment_base = Reg(UInt(width = p(PAddrBits)))
  val test_data = Wire(UInt(width = 32))
  test_data := Cat(segment_cnt, word_cnt)

  val (s_idle :: s_put_req :: s_put_resp ::
       s_dma_req :: s_dma_resp :: s_wait ::
       s_get_req :: s_get_resp :: s_done :: Nil) = Enum(Bits(), 9)
  val state = Reg(init = s_idle)

  io.mem.req.valid := state.isOneOf(s_put_req, s_get_req)
  io.mem.req.bits.addr := segment_base + Cat(word_cnt, UInt(0, 2))
  io.mem.req.bits.cmd := Mux(state === s_put_req, M_XWR, M_XRD)
  io.mem.req.bits.typ := MT_WU
  io.mem.req.bits.data := test_data
  io.mem.req.bits.tag := UInt(0)
  io.mem.req.bits.phys := Bool(false)
  io.mem.invalidate_lr := Bool(false)

  io.dma.req.valid := (state === s_dma_req)
  io.dma.req.bits := ClientDmaRequest(
    cmd = DmaRequest.DMA_CMD_COPY,
    src_start = UInt(testParams.src_start),
    dst_start = UInt(testParams.dst_start),
    segment_size = UInt(testParams.segment_size),
    nsegments = UInt(testParams.nsegments),
    src_stride = UInt(testParams.src_stride),
    dst_stride = UInt(testParams.dst_stride))

  assert(!io.dma.resp.valid || io.dma.resp.bits.status === UInt(0),
    "DMA frontend returned non-zero status")

  io.finished := (state === s_done)

  when (state === s_idle) {
    segment_base := UInt(testParams.src_start)
    state := s_put_req
  }
  when (state === s_put_req && io.mem.req.ready) {
    state := s_put_resp
  }
  when (state === s_put_resp && io.mem.resp.valid) {
    when (segment_flip) {
      state := s_dma_req
    } .elsewhen (word_flip) {
      segment_base := segment_base + UInt(
        testParams.segment_size + testParams.src_stride)
      state := s_put_req
    } .otherwise { state := s_put_req }
  }
  when (io.dma.req.fire()) { state := s_dma_resp }
  when (state === s_dma_resp && io.dma.resp.valid) { state := s_wait }
  when (state === s_wait && !io.busy) {
    segment_base := UInt(testParams.dst_start)
    state := s_get_req
  }
  when (state === s_get_req && io.mem.req.ready) {
    state := s_get_resp
  }
  when (state === s_get_resp && io.mem.resp.valid) {
    when (segment_flip) {
      state := s_done
    } .elsewhen (word_flip) {
      segment_base := segment_base + UInt(
        testParams.segment_size + testParams.dst_stride)
      state := s_get_req
    } .otherwise { state := s_get_req }
  }

  assert(state =/= s_get_resp || !io.mem.resp.valid ||
         io.mem.resp.bits.data === test_data,
         "DmaTest: get data does not match")
}

class DmaTest(implicit p: Parameters) extends GroundTest()(p)
    with HasCoreParameters {
  val pageBlocks = (1 << pgIdxBits) / p(CacheBlockBytes)
  val driver = Module(new DmaTestDriver)
  val frontend = Module(new DmaFrontend)
  val backend = Module(new DmaBackend)

  require(io.ptw.size == 1)
  require(io.mem.size == 1)
  require(io.cache.size == 1)

  io.ptw.head <> frontend.io.ptw
  io.mem.head <> backend.io.mem
  io.cache.head <> driver.io.mem
  io.status.finished := driver.io.finished
  io.status.timeout.valid := Bool(false)
  io.status.error.valid := Bool(false)

  driver.io.busy := frontend.io.busy
  frontend.io.cpu <> driver.io.dma
  backend.io.dma <> frontend.io.dma
}
