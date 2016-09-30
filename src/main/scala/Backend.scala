package dma

import Chisel._
import cde.{Parameters, Field}
import junctions._
import junctions.NastiConstants._
import util._
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
  val dmaStatusBits = 2
}

abstract class DmaModule(implicit val p: Parameters) extends Module with HasDmaParameters
abstract class DmaBundle(implicit val p: Parameters) extends ParameterizedBundle()(p) with HasDmaParameters

class DmaRequest(implicit p: Parameters) extends DmaBundle()(p) {
  val xact_id = UInt(width = dmaXactIdBits)
  val cmd = UInt(width = DmaRequest.DMA_CMD_SZ)
  val source = UInt(width = addrBits)
  val dest = UInt(width = addrBits)
  val length = UInt(width = addrBits)
  val alloc = UInt(width = 2)

  def isPrefetch(dummy: Int = 0): Bool =
    cmd === DmaRequest.DMA_CMD_PFR || cmd === DmaRequest.DMA_CMD_PFW
}

class DmaResponse(implicit p: Parameters) extends DmaBundle()(p) {
  val xact_id = UInt(width = dmaXactIdBits)
  val status = UInt(width = dmaStatusBits)
}

object DmaRequest {
  val DMA_CMD_SZ = 2

  val DMA_CMD_COPY = UInt("b00")
  val DMA_CMD_PFR  = UInt("b10")
  val DMA_CMD_PFW  = UInt("b11")

  def apply(xact_id: UInt = UInt(0),
            cmd: UInt,
            source: UInt,
            dest: UInt,
            length: UInt,
            alloc: UInt = UInt("b10"))(implicit p: Parameters): DmaRequest = {
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
  val resp = Decoupled(new DmaResponse).flip
}

class DmaTrackerIO(implicit p: Parameters) extends DmaBundle()(p) {
  val dma = (new DmaIO).flip
  val mem = new ClientUncachedTileLinkIO
}

abstract class DmaTracker(implicit p: Parameters)
    extends DmaModule()(p) with HasTileLinkParameters {
  val io = new DmaTrackerIO
}

class DmaTrackerFile(implicit p: Parameters) extends DmaModule()(p) {
  val io = new Bundle {
    val dma = (new DmaIO).flip
    val mem = Vec(nDmaTrackers, new ClientUncachedTileLinkIO)
  }

  val trackers = List.fill(nDmaTrackers) { p(BuildDmaTracker)(p) }
  val reqReadys = trackers.map(_.io.dma.req.ready).asUInt

  io.mem <> trackers.map(_.io.mem)

  if (nDmaTrackers > 1) {
    val resp_arb = Module(new RRArbiter(new DmaResponse, nDmaTrackers))
    resp_arb.io.in <> trackers.map(_.io.dma.resp)
    io.dma.resp <> resp_arb.io.out

    val selection = PriorityEncoder(reqReadys)
    trackers.zipWithIndex.foreach { case (tracker, i) =>
      tracker.io.dma.req.valid := io.dma.req.valid && selection === UInt(i)
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

  val data_buffer = Reg(Vec(2 * tlDataBeats, Bits(width = tlDataBits)))
  val get_inflight = Reg(UInt(width = 2 * tlDataBeats))
  val put_inflight = Reg(Bool())
  val put_half = Reg(UInt(width = 1))
  val get_half = Reg(UInt(width = 1))
  val prefetch_put = Reg(Bool())
  val get_done = !get_inflight.orR
  val alloc = Reg(UInt(width = 2))

  val src_block = Reg(UInt(width = tlBlockAddrBits))
  val dst_block = Reg(UInt(width = tlBlockAddrBits))
  val offset    = Reg(UInt(width = blockOffset))
  val alignment = Reg(UInt(width = blockOffset))
  val shift_dir = Reg(Bool())

  val bytes_left = Reg(UInt(width = addrBits))

  val acq = io.mem.acquire.bits
  val gnt = io.mem.grant.bits

  val (s_idle :: s_get :: s_put :: s_prefetch ::
       s_wait :: s_resp :: Nil) = Enum(Bits(), 6)
  val state = Reg(init = s_idle)

  val (put_beat, put_done) = Counter(
    io.mem.acquire.fire() && acq.hasData(), tlDataBeats)

  val put_mask = Seq.tabulate(tlDataBytes) { i =>
    val byte_index = Cat(put_beat, UInt(i, tlByteAddrBits))
    byte_index >= offset && byte_index < bytes_left
  }.asUInt

  val prefetch_sent = io.mem.acquire.fire() && io.mem.acquire.bits.isPrefetch()
  val prefetch_busy = Reg(init = UInt(0, nDmaTrackerMemXacts))
  val (prefetch_id, _) = Counter(prefetch_sent, nDmaTrackerMemXacts)

  val base_index = Cat(put_half, put_beat)
  val put_data = Wire(init = Bits(0, tlDataBits))
  val beat_align = alignment(blockOffset - 1, tlByteAddrBits)
  val bit_align = Cat(alignment(tlByteAddrBits - 1, 0), UInt(0, 3))
  val rev_align = UInt(tlDataBits) - bit_align

  def getBit(value: UInt, sel: UInt): Bool =
    (value >> sel)(0)

  when (alignment === UInt(0)) {
    put_data := data_buffer(base_index)
  } .elsewhen (shift_dir) {
    val shift_index = base_index - beat_align
    when (bit_align === UInt(0)) {
      put_data := data_buffer(shift_index)
    } .otherwise {
      val upper_bits = data_buffer(shift_index)
      val lower_bits = data_buffer(shift_index - UInt(1))
      val upper_shifted = upper_bits << bit_align
      val lower_shifted = lower_bits >> rev_align
      put_data := upper_shifted | lower_shifted
    }
  } .otherwise {
    val shift_index = base_index + beat_align
    when (bit_align === UInt(0)) {
      put_data := data_buffer(shift_index)
    } .otherwise {
      val upper_bits = data_buffer(shift_index + UInt(1))
      val lower_bits = data_buffer(shift_index)
      val upper_shifted = upper_bits << rev_align
      val lower_shifted = lower_bits >> bit_align
      put_data := upper_shifted | lower_shifted
    }
  }

  val put_acquire = PutBlock(
    client_xact_id = UInt(2),
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

  val resp_xact_id = Reg(UInt(width = dmaXactIdBits))

  io.mem.acquire.valid := (state === s_get) ||
                          (state === s_put && get_done) ||
                          (state === s_prefetch && !prefetch_busy(prefetch_id))
  io.mem.acquire.bits := MuxLookup(
    state, prefetch_acquire, Seq(
      s_get -> get_acquire,
      s_put -> put_acquire))
  io.mem.grant.ready := Bool(true)
  io.dma.req.ready := state === s_idle
  io.dma.resp.valid := state === s_resp
  io.dma.resp.bits.xact_id := resp_xact_id
  io.dma.resp.bits.status := UInt(0)

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
    get_inflight := UInt(0)
    put_inflight := Bool(false)
    get_half := UInt(0)
    put_half := UInt(0)
    alloc    := io.dma.req.bits.alloc

    when (io.dma.req.bits.cmd === DMA_CMD_COPY) {
      state := s_get
    } .elsewhen (io.dma.req.bits.isPrefetch()) {
      prefetch_put := io.dma.req.bits.cmd(0)
      state := s_prefetch
    } .otherwise {
      assert(Bool(false), "Unknown DMA command")
    }
  }

  when (state === s_get && io.mem.acquire.ready) {
    get_inflight := get_inflight | FillInterleaved(tlDataBeats, UIntToOH(get_half))
    src_block := src_block + UInt(1)
    val bytes_in_buffer = UInt(blockBytes) - alignment
    val extra_read = alignment > UInt(0) && !shift_dir && // dst_off < src_off
                     get_half === UInt(0) && // this is the first block
                     bytes_in_buffer < bytes_left // there is still more data left to fetch
    get_half := get_half + UInt(1)
    when (!extra_read) { state := s_put }
  }

  when (prefetch_sent) {
    prefetch_busy := prefetch_busy | UIntToOH(prefetch_id)
    when (bytes_left < UInt(blockBytes)) {
      bytes_left := UInt(0)
      state := s_wait
    } .otherwise {
      bytes_left := bytes_left - UInt(blockBytes)
      dst_block := dst_block + UInt(1)
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
      put_inflight := Bool(false)
    }
  }

  when (put_done) { // state === s_put
    put_half := put_half + UInt(1)
    offset := UInt(0)
    when (bytes_left < UInt(blockBytes)) {
      bytes_left := UInt(0)
    } .otherwise {
      bytes_left := bytes_left - UInt(blockBytes)
    }
    put_inflight := Bool(true)
    dst_block := dst_block + UInt(1)
    state := s_wait
  }

  when (state === s_wait && get_done && !put_inflight && !prefetch_busy.orR) {
    state := Mux(bytes_left === UInt(0), s_resp, s_get)
  }

  when (io.dma.resp.fire()) { state := s_idle }
}

case object DmaTrackerPipelineDepth extends Field[Int]

class PipelinePacket(implicit p: Parameters)
    extends DmaBundle()(p) with HasTileLinkParameters {
  val data = UInt(width = tlDataBits)
  val bytes = UInt(width = log2Up(tlDataBytes))
}

class PipelinedDmaTrackerPrefetcher(implicit p: Parameters)
    extends DmaModule()(p) with HasTileLinkParameters {
  val io = new Bundle {
    val dma_req = Valid(new DmaRequest).flip
    val mem = new ClientUncachedTileLinkIO
    val busy = Bool(OUTPUT)
  }

  private val blockOffset = tlBeatAddrBits + tlByteAddrBits
  private val blockBytes = tlDataBeats * tlDataBytes

  val dst_block = Reg(UInt(width = tlBlockAddrBits))
  val bytes_left = Reg(UInt(width = addrBits))

  val prefetch_put = Reg(Bool())
  val prefetch_busy = Reg(UInt(width = nDmaTrackerMemXacts), init = UInt(0))
  val prefetch_id_onehot = PriorityEncoderOH(~prefetch_busy)
  val prefetch_id = OHToUInt(prefetch_id_onehot)

  prefetch_busy := (prefetch_busy |
    Mux(io.mem.acquire.fire(), UIntToOH(prefetch_id), UInt(0))) &
    ~Mux(io.mem.grant.fire(), UIntToOH(io.mem.grant.bits.client_xact_id), UInt(0))

  val s_idle :: s_prefetch :: Nil = Enum(Bits(), 2)
  val state = Reg(init = s_idle)

  io.mem.acquire.valid := (state === s_prefetch) && !prefetch_busy.andR
  io.mem.acquire.bits := Mux(prefetch_put,
    PutPrefetch(client_xact_id = prefetch_id, addr_block = dst_block),
    GetPrefetch(client_xact_id = prefetch_id, addr_block = dst_block))
  io.mem.grant.ready := prefetch_busy.orR
  io.busy := (state =/= s_idle) || prefetch_busy.orR

  when (state === s_idle && io.dma_req.valid) {
    val dst_off = io.dma_req.bits.dest(blockOffset - 1, 0)
    dst_block := io.dma_req.bits.dest(addrBits - 1, blockOffset)
    bytes_left := io.dma_req.bits.length + dst_off
    state := s_prefetch
  }

  when (io.mem.acquire.fire()) {
    when (bytes_left < UInt(blockBytes)) {
      bytes_left := UInt(0)
      state := s_idle
    } .otherwise {
      bytes_left := bytes_left - UInt(blockBytes)
      dst_block := dst_block + UInt(1)
    }
  }
}

class PipelinedDmaTrackerReader(implicit p: Parameters)
    extends DmaModule()(p) with HasTileLinkParameters {

  val pipelineDepth = p(DmaTrackerPipelineDepth)

  val io = new Bundle {
    val dma_req = Valid(new DmaRequest).flip
    val mem = new ClientUncachedTileLinkIO
    val pipe = Decoupled(new PipelinePacket)
    val pipe_cnt = UInt(INPUT, log2Up(pipelineDepth+1))
    val busy = Bool(OUTPUT)
  }

  private val blockOffset = tlBeatAddrBits + tlByteAddrBits
  private val blockBytes = tlDataBeats * tlDataBytes

  val src_addr = Reg(UInt(width = addrBits))
  val src_block = src_addr(addrBits - 1, blockOffset)
  val src_beat = src_addr(blockOffset - 1, tlByteAddrBits)
  val src_byte_off = src_addr(tlByteAddrBits - 1, 0)
  val bytes_left = Reg(UInt(width = addrBits))

  val s_idle :: s_mem_req :: Nil = Enum(Bits(), 2)
  val state = Reg(init = s_idle)

  val get_busy = Reg(UInt(width = nDmaTrackerMemXacts), init = UInt(0))
  val get_id_onehot = PriorityEncoderOH(~get_busy)
  val get_id = OHToUInt(get_id_onehot)

  val byte_offsets = Reg(Vec(nDmaTrackerMemXacts, UInt(width = tlByteAddrBits)))
  val bytes_valid = Reg(Vec(nDmaTrackerMemXacts, UInt(width = tlByteAddrBits)))

  val alloc = Reg(Bool())
  val send_block =
    if (pipelineDepth >= tlDataBeats)
      src_beat === UInt(0) && src_byte_off === UInt(0) && bytes_left >= UInt(blockBytes)
    else Bool(false)
  val beats_inflight = Reg(UInt(width = log2Up(pipelineDepth + 1)), init = UInt(0))

  beats_inflight := beats_inflight +
    Mux(io.mem.acquire.fire(),
      Mux(send_block, UInt(tlDataBeats), UInt(1)), UInt(0)) -
    Mux(io.mem.grant.fire(), UInt(1), UInt(0))

  when (state === s_idle && io.dma_req.valid) {
    src_addr := io.dma_req.bits.source
    bytes_left := io.dma_req.bits.length
    alloc := io.dma_req.bits.alloc(0)
    state := s_mem_req
  }

  when (io.mem.acquire.fire()) {
    val bytes_to_read =
      Mux(send_block, UInt(blockBytes), UInt(tlDataBytes) - src_byte_off)
    src_addr := src_addr + bytes_to_read
    byte_offsets(get_id) := src_byte_off
    bytes_valid(get_id) := Mux(bytes_to_read < bytes_left, bytes_to_read, bytes_left)

    when (bytes_left > bytes_to_read) {
      bytes_left := bytes_left - bytes_to_read
    } .otherwise {
      bytes_left := UInt(0)
      state := s_idle
    }
  }

  get_busy := (get_busy |
    Mux(io.mem.acquire.fire(), UIntToOH(get_id), UInt(0))) &
    ~Mux(io.mem.grant.fire() && io.mem.grant.bits.last(),
      UIntToOH(io.mem.grant.bits.client_xact_id), UInt(0))

  // How many reads are outstanding?
  val flow_ctrl_cnt = beats_inflight + io.pipe_cnt
  val send_cnt = Mux(send_block, UInt(tlDataBeats), UInt(1)).suggestName("send_cnt")
  val block_acquire = get_busy.andR || (send_cnt +& flow_ctrl_cnt) > UInt(pipelineDepth)

  val gnt_byte_off = byte_offsets(io.mem.grant.bits.client_xact_id)
  val gnt_bytes_valid = bytes_valid(io.mem.grant.bits.client_xact_id)

  io.mem.acquire.valid := state === s_mem_req && !block_acquire
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
  io.mem.grant.ready := io.pipe.ready
  io.pipe.valid := io.mem.grant.valid
  io.pipe.bits.data := io.mem.grant.bits.data >> Cat(gnt_byte_off, UInt(0, 3))
  io.pipe.bits.bytes := gnt_bytes_valid - UInt(1)
  io.busy := state =/= s_idle || get_busy.orR
}

class PipelinedDmaTrackerWriter(implicit p: Parameters)
    extends DmaModule()(p) with HasTileLinkParameters {
  val io = new Bundle {
    val dma_req = Valid(new DmaRequest).flip
    val mem = new ClientUncachedTileLinkIO
    val pipe = Decoupled(new PipelinePacket).flip
    val busy = Bool(OUTPUT)
  }

  private val blockOffset = tlBeatAddrBits + tlByteAddrBits
  private val blockBytes = tlDataBeats * tlDataBytes

  val dst_addr = Reg(UInt(width = addrBits))
  val dst_block = dst_addr(addrBits - 1, blockOffset)
  val dst_beat = dst_addr(blockOffset - 1, tlByteAddrBits)
  val dst_byte_off = dst_addr(tlByteAddrBits - 1, 0)
  val bytes_left = Reg(UInt(width = addrBits))

  val s_idle :: s_pipe :: s_check :: s_mem_req :: Nil = Enum(Bits(), 4)
  val state = Reg(init = s_idle)

  val data = Reg(UInt(width = 2 * tlDataBits))
  val bytes_val = Reg(UInt(width = log2Up(tlDataBytes) + 1))

  val put_busy = Reg(UInt(width = nDmaTrackerMemXacts), init = UInt(0))
  val put_id_onehot = PriorityEncoderOH(~put_busy)
  val put_id = OHToUInt(put_id_onehot)

  val off_size = MuxCase(UInt(log2Up(tlDataBytes)),
    (0 until log2Up(tlDataBytes))
      .map(place => (dst_addr(place) -> UInt(place))))
  val bytes_val_size = Log2(bytes_val)
  val size = Mux(bytes_val_size < off_size, bytes_val_size, off_size)
  val storegen = new StoreGen(size, dst_addr, data, tlDataBytes)

  val alloc = Reg(Bool())

  io.mem.acquire.valid := (state === s_mem_req) && !put_busy.andR
  io.mem.acquire.bits := Put(
    client_xact_id = put_id,
    addr_block = dst_block,
    addr_beat = dst_beat,
    data = storegen.data,
    wmask = Some(storegen.mask),
    alloc = alloc)
  io.mem.grant.ready := put_busy.orR

  put_busy := (put_busy |
    Mux(io.mem.acquire.fire(), UIntToOH(put_id), UInt(0))) &
    ~Mux(io.mem.grant.fire(), UIntToOH(io.mem.grant.bits.client_xact_id), UInt(0))

  when (state === s_idle && io.dma_req.valid) {
    dst_addr := io.dma_req.bits.dest
    bytes_left := io.dma_req.bits.length
    data := UInt(0)
    bytes_val := UInt(0)
    alloc := io.dma_req.bits.alloc(1)
    state := s_pipe
  }

  when (io.pipe.fire()) {
    data := data | (io.pipe.bits.data << Cat(bytes_val, UInt(0, 3)))
    bytes_val := bytes_val + io.pipe.bits.bytes + UInt(1)
    state := s_check
  }

  when (state === s_check) {
    val off_true_size = UInt(1) << off_size
    when (bytes_left === UInt(0)) {
      state := s_idle
    } .elsewhen (bytes_val < off_true_size && bytes_val < bytes_left) {
      state := s_pipe
    } .otherwise {
      state := s_mem_req
    }
  }

  when (io.mem.acquire.fire()) {
    val true_size = UInt(1) << size

    bytes_val := bytes_val - true_size
    bytes_left := bytes_left - true_size
    dst_addr := dst_addr + true_size
    data := data >> Cat(true_size, UInt(0, 3))
    state := s_check
  }

  io.pipe.ready := state === s_pipe
  io.busy := state =/= s_idle || put_busy.orR
}

class PipelinedDmaTracker(implicit p: Parameters) extends DmaTracker()(p) {
  val prefetch = Module(new PipelinedDmaTrackerPrefetcher)
  val reader = Module(new PipelinedDmaTrackerReader)
  val writer = Module(new PipelinedDmaTrackerWriter)
  val memPorts = Seq(prefetch.io.mem, reader.io.mem, writer.io.mem)

  val busy = prefetch.io.busy || reader.io.busy || writer.io.busy
  val xact_id = Reg(UInt(width = dmaXactIdBits))

  val pipelineDepth = p(DmaTrackerPipelineDepth)
  require(pipelineDepth >= 1)
  // we can't have more outstanding requests than we have pipeline space
  require(nDmaTrackerMemXacts <= pipelineDepth)

  if (p(DmaTrackerPipelineDepth) > 1) {
    // one pipeline register is in the writer itself
    val pipe = Module(new Queue(new PipelinePacket, pipelineDepth - 1))
    pipe.io.enq <> reader.io.pipe
    writer.io.pipe <> pipe.io.deq
    reader.io.pipe_cnt := pipe.io.count +& !writer.io.pipe.ready
  } else {
    writer.io.pipe <> reader.io.pipe
    reader.io.pipe_cnt := !writer.io.pipe.ready
  }

  val s_idle :: s_resp :: Nil = Enum(Bits(), 2)
  val state = Reg(init = s_idle)

  io.dma.req.ready := state === s_idle
  io.dma.resp.valid := state === s_resp && !busy
  io.dma.resp.bits.xact_id := xact_id
  io.dma.resp.bits.status := UInt(0)

  val is_prefetch = io.dma.req.bits.isPrefetch()
  val is_copy = io.dma.req.bits.cmd === DMA_CMD_COPY

  prefetch.io.dma_req.valid := io.dma.req.fire() && is_prefetch
  prefetch.io.dma_req.bits := io.dma.req.bits
  reader.io.dma_req.valid := io.dma.req.fire() && is_copy
  reader.io.dma_req.bits := io.dma.req.bits
  writer.io.dma_req.valid := io.dma.req.fire() && is_copy
  writer.io.dma_req.bits := io.dma.req.bits

  when (io.dma.req.fire()) {
    xact_id := io.dma.req.bits.xact_id
    state := s_resp
  }
  when (io.dma.resp.fire()) { state := s_idle }

  val arb = Module(new ClientUncachedTileLinkIOArbiter(memPorts.size))
  arb.io.in <> memPorts
  io.mem <> arb.io.out
}

class DmaBackend(implicit p: Parameters) extends DmaModule()(p) {
  val io = new Bundle {
    val dma = (new DmaIO).flip
    val mem = new ClientUncachedTileLinkIO
  }

  val memArb = Module(new ClientUncachedTileLinkIOArbiter(nDmaTrackers))
  val trackerFile = Module(new DmaTrackerFile)
  
  trackerFile.io.dma <> io.dma
  memArb.io.in <> trackerFile.io.mem
  io.mem <> memArb.io.out
}
