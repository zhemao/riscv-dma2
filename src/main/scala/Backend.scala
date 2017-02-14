package dma

import chisel3._
import chisel3.util._
import cde.{Parameters, Field}
import junctions._
import junctions.NastiConstants._
import _root_.util._
import uncore.tilelink._
import uncore.util._

case object NDmaTrackers extends Field[Int]
case object NDmaXacts extends Field[Int]
case object NDmaTrackerMemXacts extends Field[Int]
case object BuildDmaTracker extends Field[Parameters => DmaTracker]

trait HasDmaParameters {
  implicit val p: Parameters
  val nDmaTrackers = p(NDmaTrackers)
  val nDmaXacts = p(NDmaXacts)
  val nDmaTrackerMemXacts = p(NDmaTrackerMemXacts)
  val dmaXactIdBits = log2Up(nDmaXacts)
  val addrBits = p(PAddrBits)
  val dmaStatusBits = 3
}

abstract class DmaModule(implicit val p: Parameters) extends Module with HasDmaParameters
abstract class DmaBundle(implicit val p: Parameters) extends ParameterizedBundle()(p) with HasDmaParameters

class DmaRequest(implicit p: Parameters) extends DmaBundle()(p) {
  val xact_id = UInt(dmaXactIdBits.W)
  val cmd = UInt(DmaRequest.DMA_CMD_SZ.W)
  val source = UInt(addrBits.W)
  val dest = UInt(addrBits.W)
  val length = UInt(addrBits.W)
  val alloc = UInt(2.W)

  def isPrefetch(dummy: Int = 0): Bool =
    cmd === DmaRequest.DMA_CMD_PFR || cmd === DmaRequest.DMA_CMD_PFW
}

class DmaResponse(implicit p: Parameters) extends DmaBundle()(p) {
  val xact_id = UInt(dmaXactIdBits.W)
  val status = UInt(dmaStatusBits.W)
}

object DmaRequest {
  val DMA_CMD_SZ = 2

  val DMA_CMD_COPY = "b00".U
  val DMA_CMD_PFR  = "b10".U
  val DMA_CMD_PFW  = "b11".U

  def apply(xact_id: UInt = 0.U,
            cmd: UInt,
            source: UInt,
            dest: UInt,
            length: UInt,
            alloc: UInt = "b10".U)(implicit p: Parameters): DmaRequest = {
    val req = Wire(new DmaRequest)
    req.xact_id := xact_id
    req.cmd := cmd
    req.source := source
    req.dest := dest
    req.length := length
    req.alloc := alloc
    req
  }
}
import DmaRequest._

class DmaIO(implicit p: Parameters) extends DmaBundle()(p) {
  val req = Decoupled(new DmaRequest)
  val resp = Flipped(Decoupled(new DmaResponse))
}

class DmaTrackerIO(implicit p: Parameters) extends DmaBundle()(p) {
  val dma = Flipped(new DmaIO)
  val mem = new ClientUncachedTileLinkIO
}

abstract class DmaTracker(implicit p: Parameters)
    extends DmaModule()(p) with HasTileLinkParameters {
  val io = IO(new DmaTrackerIO)
}

class DmaBackend(implicit p: Parameters) extends DmaModule()(p) {
  val io = IO(new Bundle {
    val dma = Flipped(new DmaIO)
    val mem = Vec(nDmaTrackers, new ClientUncachedTileLinkIO)
  })

  val trackers = List.fill(nDmaTrackers) { p(BuildDmaTracker)(p) }
  val reqReadys = trackers.map(_.io.dma.req.ready).asUInt

  io.mem <> trackers.map(_.io.mem)

  if (nDmaTrackers > 1) {
    val resp_arb = Module(new RRArbiter(new DmaResponse, nDmaTrackers))
    resp_arb.io.in <> trackers.map(_.io.dma.resp)
    io.dma.resp <> resp_arb.io.out

    val selection = PriorityEncoder(reqReadys)
    trackers.zipWithIndex.foreach { case (tracker, i) =>
      tracker.io.dma.req.valid := io.dma.req.valid && selection === i.U
      tracker.io.dma.req.bits := io.dma.req.bits
    }
    io.dma.req.ready := reqReadys.orR
  } else {
    trackers.head.io.dma <> io.dma
  }
}

class BigBufferDmaTracker(implicit p: Parameters) extends DmaTracker()(p) {

  private val blockOffset = tlBeatAddrBits + tlByteAddrBits
  private val blockBytes = tlDataBeats * tlDataBytes

  val data_buffer = Reg(Vec(2 * tlDataBeats, Bits(tlDataBits.W)))
  val get_inflight = Reg(UInt((2 * tlDataBeats).W))
  val put_inflight = Reg(Bool())
  val put_half = Reg(UInt(1.W))
  val get_half = Reg(UInt(1.W))
  val prefetch_put = Reg(Bool())
  val get_done = !get_inflight.orR
  val alloc = Reg(UInt(2.W))

  val src_block = Reg(UInt(tlBlockAddrBits.W))
  val dst_block = Reg(UInt(tlBlockAddrBits.W))
  val offset    = Reg(UInt(blockOffset.W))
  val alignment = Reg(UInt(blockOffset.W))
  val shift_dir = Reg(Bool())

  val bytes_left = Reg(UInt(addrBits.W))

  val acq = io.mem.acquire.bits
  val gnt = io.mem.grant.bits

  val (s_idle :: s_get :: s_put :: s_prefetch ::
       s_wait :: s_resp :: Nil) = Enum(6)
  val state = Reg(init = s_idle)

  val (put_beat, put_done) = Counter(
    io.mem.acquire.fire() && acq.hasData(), tlDataBeats)

  val put_mask = Seq.tabulate(tlDataBytes) { i =>
    val byte_index = Cat(put_beat, i.U(tlByteAddrBits.W))
    byte_index >= offset && byte_index < bytes_left
  }.asUInt

  val prefetch_sent = io.mem.acquire.fire() && io.mem.acquire.bits.isPrefetch()
  val prefetch_busy = Reg(init = 0.U(nDmaTrackerMemXacts.W))
  val (prefetch_id, _) = Counter(prefetch_sent, nDmaTrackerMemXacts)

  val base_index = Cat(put_half, put_beat)
  val put_data = Wire(init = Bits(0, tlDataBits))
  val beat_align = alignment(blockOffset - 1, tlByteAddrBits)
  val bit_align = Cat(alignment(tlByteAddrBits - 1, 0), 0.U(3.W))
  val rev_align = tlDataBits.U - bit_align

  def getBit(value: UInt, sel: UInt): Bool =
    (value >> sel)(0)

  when (alignment === 0.U) {
    put_data := data_buffer(base_index)
  } .elsewhen (shift_dir) {
    val shift_index = base_index - beat_align
    when (bit_align === 0.U) {
      put_data := data_buffer(shift_index)
    } .otherwise {
      val upper_bits = data_buffer(shift_index)
      val lower_bits = data_buffer(shift_index - 1.U)
      val upper_shifted = upper_bits << bit_align
      val lower_shifted = lower_bits >> rev_align
      put_data := upper_shifted | lower_shifted
    }
  } .otherwise {
    val shift_index = base_index + beat_align
    when (bit_align === 0.U) {
      put_data := data_buffer(shift_index)
    } .otherwise {
      val upper_bits = data_buffer(shift_index + 1.U)
      val lower_bits = data_buffer(shift_index)
      val upper_shifted = upper_bits << rev_align
      val lower_shifted = lower_bits >> bit_align
      put_data := upper_shifted | lower_shifted
    }
  }

  val put_acquire = PutBlock(
    client_xact_id = 2.U,
    addr_block = dst_block,
    addr_beat = put_beat,
    data = put_data,
    wmask = Some(put_mask),
    alloc = alloc(1))

  val get_acquire = GetBlock(
    client_xact_id = get_half,
    addr_block = src_block,
    alloc = alloc(0))

  val prefetch_acquire = Mux(prefetch_put,
    PutPrefetch(client_xact_id = prefetch_id, addr_block = dst_block),
    GetPrefetch(client_xact_id = prefetch_id, addr_block = dst_block))

  val resp_xact_id = Reg(UInt(dmaXactIdBits.W))

  io.mem.acquire.valid := (state === s_get) ||
                          (state === s_put && get_done) ||
                          (state === s_prefetch && !prefetch_busy(prefetch_id))
  io.mem.acquire.bits := MuxLookup(
    state, prefetch_acquire, Seq(
      s_get -> get_acquire,
      s_put -> put_acquire))
  io.mem.grant.ready := true.B
  io.dma.req.ready := state === s_idle
  io.dma.resp.valid := state === s_resp
  io.dma.resp.bits.xact_id := resp_xact_id
  io.dma.resp.bits.status := 0.U

  when (io.dma.req.fire()) {
    val src_off = io.dma.req.bits.source(blockOffset - 1, 0)
    val dst_off = io.dma.req.bits.dest(blockOffset - 1, 0)
    val direction = src_off < dst_off

    resp_xact_id := io.dma.req.bits.xact_id
    src_block := io.dma.req.bits.source(addrBits - 1, blockOffset)
    dst_block := io.dma.req.bits.dest(addrBits - 1, blockOffset)
    alignment := Mux(direction, dst_off - src_off, src_off - dst_off)
    shift_dir := direction
    offset := dst_off
    bytes_left := io.dma.req.bits.length + dst_off
    get_inflight := 0.U
    put_inflight := false.B
    get_half := 0.U
    put_half := 0.U
    alloc    := io.dma.req.bits.alloc

    when (io.dma.req.bits.cmd === DMA_CMD_COPY) {
      state := s_get
    } .elsewhen (io.dma.req.bits.isPrefetch()) {
      prefetch_put := io.dma.req.bits.cmd(0)
      state := s_prefetch
    } .otherwise {
      assert(false.B, "Unknown DMA command")
    }
  }

  when (state === s_get && io.mem.acquire.ready) {
    get_inflight := get_inflight | FillInterleaved(tlDataBeats, UIntToOH(get_half))
    src_block := src_block + 1.U
    val bytes_in_buffer = blockBytes.U - alignment
    val extra_read = alignment > 0.U && !shift_dir && // dst_off < src_off
                     get_half === 0.U && // this is the first block
                     bytes_in_buffer < bytes_left // there is still more data left to fetch
    get_half := get_half + 1.U
    when (!extra_read) { state := s_put }
  }

  when (prefetch_sent) {
    prefetch_busy := prefetch_busy | UIntToOH(prefetch_id)
    when (bytes_left < blockBytes.U) {
      bytes_left := 0.U
      state := s_wait
    } .otherwise {
      bytes_left := bytes_left - blockBytes.U
      dst_block := dst_block + 1.U
    }
  }

  when (io.mem.grant.fire()) {
    when (gnt.g_type === Grant.prefetchAckType) {
      prefetch_busy := prefetch_busy & ~UIntToOH(gnt.client_xact_id)
    } .elsewhen (gnt.hasData()) {
      val write_half = gnt.client_xact_id(0)
      val write_idx = Cat(write_half, gnt.addr_beat)
      get_inflight := get_inflight & ~UIntToOH(write_idx)
      data_buffer(write_idx) := gnt.data
    } .otherwise {
      put_inflight := false.B
    }
  }

  when (put_done) { // state === s_put
    put_half := put_half + 1.U
    offset := 0.U
    when (bytes_left < blockBytes.U) {
      bytes_left := 0.U
    } .otherwise {
      bytes_left := bytes_left - blockBytes.U
    }
    put_inflight := true.B
    dst_block := dst_block + 1.U
    state := s_wait
  }

  when (state === s_wait && get_done && !put_inflight && !prefetch_busy.orR) {
    state := Mux(bytes_left === 0.U, s_resp, s_get)
  }

  when (io.dma.resp.fire()) { state := s_idle }
}

case object DmaTrackerPipelineDepth extends Field[Int]

trait PipelineUtils {
  implicit val p: Parameters
  val pipelineDepth = p(DmaTrackerPipelineDepth)
  val pipelineIdxBits = log2Up(pipelineDepth)
  val pipelineCountBits = log2Up(pipelineDepth+1)

  def incWrap(cur: UInt, inc: UInt): UInt = {
    val unwrapped = cur +& inc
    Mux(unwrapped >= pipelineDepth.U, unwrapped - pipelineDepth.U, unwrapped)
  }
}

class PipelinePacket(implicit p: Parameters)
    extends DmaBundle()(p) with HasTileLinkParameters {
  val data = UInt(tlDataBits.W)
  val bytes = UInt(log2Up(tlDataBytes).W)
}

class ReservationRequest extends Bundle {
  val multibeat = Bool()
}

class ReservationResponse(implicit val p: Parameters) extends ParameterizedBundle()(p)
    with PipelineUtils {
  val idx = UInt(pipelineIdxBits.W)
}

class ReservationData(implicit p: Parameters) extends TLBundle()(p)
    with PipelineUtils {
  val data = UInt(tlDataBits.W)
  val bytes = UInt(tlByteAddrBits.W)
  val idx = UInt(pipelineIdxBits.W)
}

class ReservationInputIO(implicit p: Parameters) extends ParameterizedBundle()(p) {
  val req = Decoupled(new ReservationRequest)
  val resp = Flipped(Decoupled(new ReservationResponse))
  val data = Decoupled(new ReservationData)
}

class ReservationOutputIO(implicit val p: Parameters) extends ParameterizedBundle()(p)
    with PipelineUtils {
  val count = UInt(pipelineCountBits.W)
  val data = Decoupled(new PipelinePacket)
}

class ReservationQueue(implicit p: Parameters) extends TLModule()(p)
    with PipelineUtils {
  val io = IO(new Bundle {
    val in = Flipped(new ReservationInputIO)
    val out = new ReservationOutputIO
  })

  val req = Queue(io.in.req, 1)

  val pkt_buffer = Mem(pipelineDepth, new PipelinePacket)
  val pkt_valid = Reg(init = 0.U(pipelineDepth.W))

  val head = Reg(init = 0.U(pipelineIdxBits.W))
  val tail = Reg(init = 0.U(pipelineIdxBits.W))
  val count = Reg(init = 0.U(pipelineCountBits.W))

  val req_count = Mux(req.bits.multibeat, tlDataBeats.U, 1.U)
  count := count + Mux(req.fire(), req_count, 0.U) - io.out.data.fire()

  val full = (count + req_count) > pipelineDepth.U

  req.ready := io.in.resp.ready && !full
  io.in.resp.valid := req.valid && !full
  io.in.resp.bits.idx := tail

  io.in.data.ready := true.B
  io.out.data.valid := (pkt_valid >> head)(0)
  io.out.data.bits := pkt_buffer(head)
  io.out.count := count // count is amount allocated, not amount stored

  when (req.fire()) {
    tail := incWrap(tail, req_count)
  }

  when (io.in.data.fire()) {
    val pkt = Wire(new PipelinePacket)
    pkt.data := io.in.data.bits.data
    pkt.bytes := io.in.data.bits.bytes
    pkt_buffer(io.in.data.bits.idx) := pkt
  }

  when (io.out.data.fire()) {
    head := incWrap(head, 1.U)
  }

  pkt_valid := (pkt_valid &
    ~Mux(io.out.data.fire(), UIntToOH(head), 0.U)) |
    Mux(io.in.data.fire(), UIntToOH(io.in.data.bits.idx), 0.U)
}

class PipelinedDmaTrackerPrefetcher(implicit p: Parameters)
    extends DmaModule()(p) with HasTileLinkParameters {
  val io = IO(new Bundle {
    val dma = Flipped(new DmaIO)
    val mem = new ClientUncachedTileLinkIO
  })

  private val blockOffset = tlBeatAddrBits + tlByteAddrBits
  private val blockBytes = tlDataBeats * tlDataBytes

  val dst_block = Reg(UInt(tlBlockAddrBits.W))
  val bytes_left = Reg(UInt(addrBits.W))

  val prefetch_put = Reg(Bool())
  val prefetch_busy = Reg(UInt(nDmaTrackerMemXacts.W), init = 0.U)
  val prefetch_id_onehot = PriorityEncoderOH(~prefetch_busy)
  val prefetch_id = OHToUInt(prefetch_id_onehot)

  val dma_req_id = Reg(io.dma.req.bits.xact_id.cloneType)

  prefetch_busy := (prefetch_busy |
    Mux(io.mem.acquire.fire(), UIntToOH(prefetch_id), 0.U)) &
    ~Mux(io.mem.grant.fire(), UIntToOH(io.mem.grant.bits.client_xact_id), 0.U)

  val s_idle :: s_prefetch :: s_resp :: Nil = Enum(3)
  val state = Reg(init = s_idle)

  io.mem.acquire.valid := (state === s_prefetch) && !prefetch_busy.andR
  io.mem.acquire.bits := Mux(prefetch_put,
    PutPrefetch(client_xact_id = prefetch_id, addr_block = dst_block),
    GetPrefetch(client_xact_id = prefetch_id, addr_block = dst_block))
  io.mem.grant.ready := prefetch_busy.orR

  io.dma.req.ready := state === s_idle
  io.dma.resp.valid := (state === s_resp) && !prefetch_busy.orR
  io.dma.resp.bits.xact_id := dma_req_id
  io.dma.resp.bits.status := 0.U

  when (io.dma.req.fire()) {
    val dst_off = io.dma.req.bits.dest(blockOffset - 1, 0)
    dst_block := io.dma.req.bits.dest(addrBits - 1, blockOffset)
    bytes_left := io.dma.req.bits.length + dst_off
    dma_req_id := io.dma.req.bits.xact_id
    state := s_prefetch
  }

  when (io.mem.acquire.fire()) {
    when (bytes_left < blockBytes.U) {
      bytes_left := 0.U
      state := s_resp
    } .otherwise {
      bytes_left := bytes_left - blockBytes.U
      dst_block := dst_block + 1.U
    }
  }

  when (io.dma.resp.fire()) { state := s_idle }
}

class PipelinedDmaTrackerReader(implicit p: Parameters)
    extends DmaModule()(p)
    with HasTileLinkParameters
    with PipelineUtils {

  val io = IO(new Bundle {
    val dma_req = Flipped(Decoupled(new DmaRequest))
    val mem = new ClientUncachedTileLinkIO
    val res = new ReservationInputIO
  })

  private val blockOffset = tlBeatAddrBits + tlByteAddrBits
  private val blockBytes = tlDataBeats * tlDataBytes

  val src_addr = Reg(UInt(addrBits.W))
  val src_block = src_addr(addrBits - 1, blockOffset)
  val src_beat = src_addr(blockOffset - 1, tlByteAddrBits)
  val src_byte_off = src_addr(tlByteAddrBits - 1, 0)
  val bytes_left = Reg(UInt(addrBits.W))

  val s_idle :: s_reserve :: s_mem_req :: Nil = Enum(3)
  val state = Reg(init = s_idle)

  val get_busy = Reg(init = 0.U(nDmaTrackerMemXacts.W))
  val byte_offset = Mem(nDmaTrackerMemXacts, UInt(tlByteAddrBits.W))
  val bytes_valid = Mem(nDmaTrackerMemXacts, UInt(tlByteAddrBits.W))
  val get_id_onehot = PriorityEncoderOH(~get_busy)
  val get_id = Reg(UInt(log2Up(nDmaTrackerMemXacts).W))
  val data_index = Reg(Vec(nDmaTrackerMemXacts, UInt(log2Up(pipelineDepth).W)))

  val alloc = Reg(Bool())
  val send_block =
    src_beat === 0.U && src_byte_off === 0.U &&
    bytes_left >= blockBytes.U

  io.dma_req.ready := (state === s_idle)

  when (io.dma_req.fire()) {
    src_addr := io.dma_req.bits.source
    bytes_left := io.dma_req.bits.length
    alloc := io.dma_req.bits.alloc(0)
    state := s_reserve
  }

  when (io.res.req.fire()) {
    get_id := OHToUInt(get_id_onehot)
    state := s_mem_req
  }

  when (io.res.resp.fire()) {
    data_index(get_id) := io.res.resp.bits.idx
  }

  when (io.mem.acquire.fire()) {
    val bytes_to_read =
      Mux(send_block, blockBytes.U, tlDataBytes.U - src_byte_off)

    src_addr := src_addr + bytes_to_read
    byte_offset(get_id) := src_byte_off
    bytes_valid(get_id) := Mux(bytes_to_read > tlDataBytes.U, 0.U,
                           Mux(bytes_to_read < bytes_left, bytes_to_read, bytes_left))

    when (bytes_left > bytes_to_read) {
      bytes_left := bytes_left - bytes_to_read
      state := s_reserve
    } .otherwise {
      bytes_left := 0.U
      state := s_idle
    }
  }

  io.res.req.valid := (state === s_reserve) && !get_busy.andR
  io.res.req.bits.multibeat := send_block

  io.res.resp.ready := (state === s_mem_req) && io.mem.acquire.ready
  io.mem.acquire.valid := (state === s_mem_req) && io.res.resp.valid
  io.mem.acquire.bits := Mux(send_block,
    GetBlock(
      client_xact_id = get_id,
      addr_block = src_block,
      alloc = alloc),
    Get(
      client_xact_id = get_id,
      addr_block = src_block,
      addr_beat = src_beat,
      alloc = alloc))

  val grant_id = io.mem.grant.bits.client_xact_id

  get_busy := (get_busy | Mux(io.res.req.fire(), get_id_onehot, 0.U)) &
                          ~Mux(io.mem.grant.fire() && io.mem.grant.bits.last(),
                            UIntToOH(grant_id), 0.U)

  when (io.res.data.fire()) {
    data_index(grant_id) := incWrap(data_index(grant_id), 1.U)
  }

  io.mem.grant.ready := io.res.data.ready
  io.res.data.valid := io.mem.grant.valid
  io.res.data.bits.idx := data_index(grant_id)
  io.res.data.bits.data := io.mem.grant.bits.data >> Cat(byte_offset(grant_id), 0.U(3.W))
  io.res.data.bits.bytes := bytes_valid(grant_id) - 1.U
}

class PipelinedDmaTrackerWriter(implicit p: Parameters)
    extends DmaModule()(p) with HasTileLinkParameters {

  val pipelineDepth = p(DmaTrackerPipelineDepth)

  val io = IO(new Bundle {
    val dma = Flipped(new DmaIO)
    val mem = new ClientUncachedTileLinkIO
    val pipe = Flipped(new ReservationOutputIO)
  })

  private val blockOffset = tlBeatAddrBits + tlByteAddrBits
  private val blockBytes = tlDataBeats * tlDataBytes

  val dst_addr = Reg(UInt(addrBits.W))
  val dst_block = dst_addr(addrBits - 1, blockOffset)
  val dst_beat = dst_addr(blockOffset - 1, tlByteAddrBits)
  val dst_byte_off = dst_addr(tlByteAddrBits - 1, 0)
  val bytes_left = Reg(UInt(addrBits.W))

  val dma_req_id = Reg(io.dma.req.bits.xact_id.cloneType)

  val s_idle :: s_mem_req :: s_resp :: Nil = Enum(3)
  val state = Reg(init = s_idle)

  val last_data = Reg(UInt(tlDataBits.W))
  val last_bytes_val = Reg(UInt((log2Up(tlDataBytes) + 1).W))

  val put_busy = Reg(UInt(nDmaTrackerMemXacts.W), init = 0.U)
  val put_id_onehot = PriorityEncoderOH(~put_busy)
  val put_id = OHToUInt(put_id_onehot)
  val put_block_id = RegEnable(put_id, io.mem.acquire.fire() && io.mem.acquire.bits.first())

  val off_size = MuxCase(log2Up(tlDataBytes).U,
    (0 until log2Up(tlDataBytes))
      .map(place => (dst_addr(place) -> place.U)))

  val data = last_data | Mux(io.pipe.data.valid,
    (io.pipe.data.bits.data << Cat(last_bytes_val, 0.U(3.W))), 0.U)
  val bytes_val = Mux(io.pipe.data.valid,
    last_bytes_val + io.pipe.data.bits.bytes + 1.U,
    last_bytes_val)

  val bytes_val_size = Log2(bytes_val)
  val size = Mux(bytes_val_size < off_size, bytes_val_size, off_size)
  val storegen = new StoreGen(size, dst_addr, data, tlDataBytes)

  val off_true_size = 1.U << off_size
  val needs_more = (bytes_val < off_true_size) && (bytes_val < bytes_left)
  val flush_buffer = (last_bytes_val >= bytes_left)

  val send_block = Reg(init = false.B)
  val alloc = Reg(Bool())
  val block_acquire = send_block && (io.pipe.count < (tlDataBeats.U - dst_beat))
  val acquire_ok = (state === s_mem_req) &&
                   (!put_busy.andR || send_block && dst_beat =/= 0.U) &&
                   !block_acquire

  io.mem.acquire.valid := acquire_ok && !needs_more && (io.pipe.data.valid || flush_buffer)
  io.mem.acquire.bits := Mux(send_block,
    PutBlock(
      client_xact_id = Mux(dst_beat === 0.U, put_id, put_block_id),
      addr_block = dst_block,
      addr_beat = dst_beat,
      data = storegen.data,
      alloc = alloc),
    Put(
      client_xact_id = put_id,
      addr_block = dst_block,
      addr_beat = dst_beat,
      data = storegen.data,
      wmask = Some(storegen.mask),
      alloc = alloc))
  io.mem.grant.ready := put_busy.orR

  io.pipe.data.ready := (acquire_ok && io.mem.acquire.ready && !flush_buffer) || needs_more

  put_busy := (put_busy |
    Mux(io.mem.acquire.fire() && io.mem.acquire.bits.first(), UIntToOH(put_id), 0.U)) &
    ~Mux(io.mem.grant.fire(), UIntToOH(io.mem.grant.bits.client_xact_id), 0.U)

  io.dma.req.ready := (state === s_idle)

  io.dma.resp.valid := (state === s_resp) && !put_busy.orR
  io.dma.resp.bits.xact_id := dma_req_id
  io.dma.resp.bits.status := 0.U

  when (io.dma.req.fire()) {
    dma_req_id := io.dma.req.bits.xact_id
    dst_addr := io.dma.req.bits.dest
    bytes_left := io.dma.req.bits.length
    last_data := 0.U
    last_bytes_val := 0.U
    alloc := io.dma.req.bits.alloc(1)
    send_block := io.dma.req.bits.dest(blockOffset - 1, 0) === 0.U &&
                  io.dma.req.bits.length >= blockBytes.U
    state := s_mem_req
  }

  when (io.pipe.data.fire() && needs_more) {
    last_data := data
    last_bytes_val := bytes_val
  }

  when (io.mem.acquire.fire()) {
    val true_size = 1.U << size
    val next_addr = dst_addr + true_size
    val next_bytes_left = bytes_left - true_size

    last_bytes_val := bytes_val - true_size
    last_data := data >> Cat(true_size, 0.U(3.W))
    bytes_left := next_bytes_left
    dst_addr := next_addr

    when (next_bytes_left === 0.U) {
      state := s_resp
    }

    when (next_addr(blockOffset - 1, 0) === 0.U) {
      send_block := next_bytes_left >= blockBytes.U
    }
  }

  when (io.dma.resp.fire()) { state := s_idle }
}

class PipelinedDmaTracker(implicit p: Parameters) extends DmaTracker()(p)
    with HasTileLinkParameters {
  val prefetch = Module(new PipelinedDmaTrackerPrefetcher)
  val reader = Module(new PipelinedDmaTrackerReader)
  val writer = Module(new PipelinedDmaTrackerWriter)
  val memPorts = Seq(prefetch.io.mem, reader.io.mem, writer.io.mem)

  val pipelineDepth = p(DmaTrackerPipelineDepth)
  require(pipelineDepth >= 1)
  // we can't have more outstanding requests than we have pipeline space
  require(nDmaTrackerMemXacts <= pipelineDepth)
  // The pipeline must at least be able to hold a full block
  require(pipelineDepth >= tlDataBeats)

  val resq = Module(new ReservationQueue)
  resq.io.in <> reader.io.res
  writer.io.pipe <> resq.io.out

  val s_idle :: s_prefetch :: s_read :: s_write :: Nil = Enum(4)
  val state = Reg(init = s_idle)

  val is_prefetch = io.dma.req.bits.isPrefetch()
  val is_copy = io.dma.req.bits.cmd === DMA_CMD_COPY

  val req = Reg(new DmaRequest)

  io.dma.req.ready := (state === s_idle)

  prefetch.io.dma.req.valid := (state === s_prefetch)
  prefetch.io.dma.req.bits := req

  reader.io.dma_req.valid := (state === s_read)
  reader.io.dma_req.bits := req

  writer.io.dma.req.valid := (state === s_write)
  writer.io.dma.req.bits := req

  when (io.dma.req.fire()) {
    req := io.dma.req.bits
    state := MuxCase(s_idle, Seq(
      is_prefetch -> s_prefetch,
      is_copy -> s_read))
  }

  when (prefetch.io.dma.req.fire()) {
    state := s_idle
  }

  when (reader.io.dma_req.fire()) {
    state := s_write
  }

  when (writer.io.dma.req.fire()) {
    state := s_idle
  }

  val memArb = Module(new ClientUncachedTileLinkIOArbiter(memPorts.size))
  memArb.io.in <> memPorts
  io.mem <> memArb.io.out

  val respArb = Module(new RRArbiter(new DmaResponse, 2))
  respArb.io.in <> Seq(prefetch.io.dma.resp, writer.io.dma.resp)
  io.dma.resp <> respArb.io.out
}
