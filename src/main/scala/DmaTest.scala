package dma

import chisel3._
import chisel3.util._
import groundtest._
import rocket.{TLBPTWIO, HasCoreParameters}
import uncore.tilelink._
import uncore.agents.CacheBlockBytes
import uncore.constants._
import _root_.util._
import junctions.PAddrBits
import rocket._
import cde.{Parameters, Field}

case class DmaTestParameters(
  val src_start: BigInt,
  val dst_start: BigInt,
  val segment_size: Int,
  val nsegments: Int,
  val src_stride: Int,
  val dst_stride: Int)

case object DmaTestKey extends Field[DmaTestParameters]

class DmaTestDriver(implicit val p: Parameters)
    extends Module with HasTileLinkParameters {

  val io = IO(new Bundle {
    val mem = new HellaCacheIO
    val dma = new ClientDmaIO
    val busy = Input(Bool())
    val finished = Output(Bool())
  })

  val testParams = p(DmaTestKey)
  require(Seq(
      testParams.src_start,
      testParams.dst_start).map(_ % 4 == 0).reduce(_ && _),
    "DmaTest parameters must have 4 byte alignment")
  require(Seq(
      testParams.segment_size,
      testParams.src_stride,
      testParams.dst_stride).map(_ % 4 == 0).reduce(_ && _),
    "DmaTest parameters must have 4 byte alignment")

  val (word_cnt, word_flip) = Counter(io.mem.resp.valid, testParams.segment_size / 4)
  val (segment_cnt, segment_flip) = Counter(word_flip, testParams.nsegments)
  val segment_base = Reg(UInt(p(PAddrBits).W))
  val test_data = Wire(UInt(32.W))
  test_data := Cat(segment_cnt, word_cnt)

  val (s_idle :: s_put_req :: s_put_resp ::
       s_dma_req :: s_dma_resp :: s_wait ::
       s_get_req :: s_get_resp :: s_done :: Nil) = Enum(9)
  val state = Reg(init = s_idle)

  io.mem.req.valid := state.isOneOf(s_put_req, s_get_req)
  io.mem.req.bits.addr := segment_base + Cat(word_cnt, 0.U(2.W))
  io.mem.req.bits.cmd := Mux(state === s_put_req, M_XWR, M_XRD)
  io.mem.req.bits.typ := MT_WU
  io.mem.req.bits.data := test_data
  io.mem.req.bits.tag := 0.U
  io.mem.req.bits.phys := Bool(false)
  io.mem.invalidate_lr := Bool(false)

  io.dma.req.valid := (state === s_dma_req)
  io.dma.req.bits := ClientDmaRequest(
    cmd = DmaRequest.DMA_CMD_COPY,
    src_start = testParams.src_start.U,
    dst_start = testParams.dst_start.U,
    segment_size = testParams.segment_size.U,
    nsegments = testParams.nsegments.U,
    src_stride = testParams.src_stride.U,
    dst_stride = testParams.dst_stride.U)

  assert(!io.dma.resp.valid || io.dma.resp.bits.status === 0.U,
    "DMA frontend returned non-zero status")

  io.finished := (state === s_done)

  when (state === s_idle) {
    segment_base := testParams.src_start.U
    state := s_put_req
  }
  when (state === s_put_req && io.mem.req.ready) {
    state := s_put_resp
  }
  when (state === s_put_resp && io.mem.resp.valid) {
    when (segment_flip) {
      state := s_dma_req
    } .elsewhen (word_flip) {
      segment_base := segment_base + (testParams.segment_size + testParams.src_stride).U
      state := s_put_req
    } .otherwise { state := s_put_req }
  }
  when (io.dma.req.fire()) { state := s_dma_resp }
  when (state === s_dma_resp && io.dma.resp.valid) { state := s_wait }
  when (state === s_wait && !io.busy) {
    segment_base := testParams.dst_start.U
    state := s_get_req
  }
  when (state === s_get_req && io.mem.req.ready) {
    state := s_get_resp
  }
  when (state === s_get_resp && io.mem.resp.valid) {
    when (segment_flip) {
      state := s_done
    } .elsewhen (word_flip) {
      segment_base := segment_base + (testParams.segment_size + testParams.dst_stride).U
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
  require(io.mem.size == backend.io.mem.size)
  require(io.cache.size == 1)

  io.ptw.head <> frontend.io.ptw
  io.mem <> backend.io.mem
  io.cache.head <> driver.io.mem
  io.status.finished := driver.io.finished
  io.status.timeout.valid := Bool(false)
  io.status.error.valid := Bool(false)

  driver.io.busy := frontend.io.busy
  frontend.io.cpu <> driver.io.dma
  backend.io.dma <> frontend.io.dma
  frontend.io.pause := Bool(false)

  val timer = Module(new Timer(5000, p(NDmaXacts)))
  timer.io.start.valid := backend.io.dma.req.fire()
  timer.io.start.bits := backend.io.dma.req.bits.xact_id
  timer.io.stop.valid := backend.io.dma.resp.fire()
  timer.io.stop.bits := backend.io.dma.resp.bits.xact_id

  assert(!timer.io.timeout.valid, "DMA Backend timed out")
}
