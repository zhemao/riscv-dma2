package dma

import Chisel._
import uncore.tilelink._
import uncore.agents._
import uncore.util._
import junctions.AddrMap
import util.ParameterizedBundle
import rocket._
import cde.Parameters
import DmaRequest._

trait HasClientDmaParameters extends HasCoreParameters with HasDmaParameters {
  val dmaAddrBits = coreMaxAddrBits
  val dmaSegmentSizeBits = coreMaxAddrBits
  val dmaSegmentBits = 24
}

abstract class ClientDmaBundle(implicit val p: Parameters)
  extends ParameterizedBundle()(p) with HasClientDmaParameters
abstract class ClientDmaModule(implicit val p: Parameters)
  extends Module with HasClientDmaParameters

class ClientDmaRequest(implicit p: Parameters) extends ClientDmaBundle()(p) {
  val cmd = UInt(width = DMA_CMD_SZ)
  val src_start  = UInt(width = dmaAddrBits)
  val dst_start  = UInt(width = dmaAddrBits)
  val src_stride = UInt(width = dmaSegmentSizeBits)
  val dst_stride = UInt(width = dmaSegmentSizeBits)
  val segment_size = UInt(width = dmaSegmentSizeBits)
  val nsegments  = UInt(width = dmaSegmentBits)
  val alloc = UInt(width = 2)

  def isPrefetch(dummy: Int = 0): Bool =
    cmd === DmaRequest.DMA_CMD_PFR || cmd === DmaRequest.DMA_CMD_PFW
}

object ClientDmaRequest {
  val DMA_CMD_RESUME = UInt("b01")

  def apply(cmd: UInt,
            src_start: UInt,
            dst_start: UInt,
            segment_size: UInt,
            nsegments: UInt = UInt(1),
            src_stride: UInt = UInt(0),
            dst_stride: UInt = UInt(0),
            alloc: UInt = UInt("b10"))
      (implicit p: Parameters) = {
    val req = Wire(new ClientDmaRequest)
    req.cmd := cmd
    req.src_start := src_start
    req.dst_start := dst_start
    req.src_stride := src_stride
    req.dst_stride := dst_stride
    req.segment_size := segment_size
    req.nsegments := nsegments
    req.alloc := alloc
    req
  }
}
import ClientDmaRequest._

object ClientDmaResponse {
  val NO_ERROR = UInt("b000")
  val PAUSED = UInt("b001")
  val SRC_PAGE_FAULT = UInt("b010")
  val DST_PAGE_FAULT = UInt("b011")
  val SRC_INVALID_REGION = UInt("b100")
  val DST_INVALID_REGION = UInt("b101")

  def apply(status: UInt = UInt(0), fault_vpn: UInt = UInt(0))
           (implicit p: Parameters) = {
    val resp = Wire(new ClientDmaResponse)
    resp.status := status
    resp.fault_vpn := fault_vpn
    resp
  }
}
import ClientDmaResponse._

class ClientDmaResponse(implicit p: Parameters)
    extends ClientDmaBundle()(p) with HasCoreParameters {
  val status = UInt(width = dmaStatusBits)
  val fault_vpn = UInt(width = vpnBits)
}

class ClientDmaIO(implicit p: Parameters) extends ParameterizedBundle()(p) {
  val req = Decoupled(new ClientDmaRequest)
  val resp = Valid(new ClientDmaResponse).flip
}

class DmaFrontend(implicit p: Parameters) extends CoreModule()(p)
    with HasClientDmaParameters with HasTileLinkParameters {
  val io = new Bundle {
    val cpu = (new ClientDmaIO).flip
    val ptw = new TLBPTWIO
    val dma = new DmaIO
    val busy = Bool(OUTPUT)
    val pause = Bool(INPUT)
  }

  val tlb = Module(new DecoupledTLB()(p.alterPartial({
    case CacheName => "L1D"
  })))
  io.ptw <> tlb.io.ptw

  private val pgSize = 1 << pgIdxBits

  val cmd = Reg(UInt(width = DMA_CMD_SZ))
  val adv_ptr = MuxLookup(cmd, UInt("b11"), Seq(
    DMA_CMD_PFR -> UInt("b10"),
    DMA_CMD_PFW -> UInt("b10")))

  val segment_size = Reg(UInt(width = dmaSegmentSizeBits))
  val bytes_left = Reg(UInt(width = dmaSegmentSizeBits))
  val segments_left = Reg(UInt(width = dmaSegmentBits))

  val src_vaddr = Reg(UInt(width = dmaAddrBits))
  val dst_vaddr = Reg(UInt(width = dmaAddrBits))
  val src_vpn = src_vaddr(dmaAddrBits - 1, pgIdxBits)
  val dst_vpn = dst_vaddr(dmaAddrBits - 1, pgIdxBits)
  val src_idx = src_vaddr(pgIdxBits - 1, 0)
  val dst_idx = dst_vaddr(pgIdxBits - 1, 0)
  val src_pglen = UInt(pgSize) - src_idx
  val dst_pglen = UInt(pgSize) - dst_idx

  val src_stride = Reg(UInt(width = dmaSegmentSizeBits))
  val dst_stride = Reg(UInt(width = dmaSegmentSizeBits))

  val src_ppn = Reg(UInt(width = ppnBits))
  val dst_ppn = Reg(UInt(width = ppnBits))

  val src_paddr = Cat(src_ppn, src_idx)
  val dst_paddr = Cat(dst_ppn, dst_idx)

  val last_src_vpn = Reg(UInt(width = vpnBits))
  val last_dst_vpn = Reg(UInt(width = vpnBits))

  val tx_len = src_pglen min dst_pglen min bytes_left

  val dma_busy = Reg(init = UInt(0, tlMaxClientXacts))
  val dma_xact_id = PriorityEncoder(~dma_busy)

  val alloc = Reg(UInt(width = 2))

  val (s_idle :: s_translate :: s_dma_req :: s_dma_update ::
       s_prepare :: s_finish :: Nil) = Enum(Bits(), 6)
  val state = Reg(init = s_idle)

  // lower bit is for src, higher bit is for dst
  val to_translate = Reg(UInt(width = 2), init = UInt(0))
  val tlb_sent = Reg(UInt(width = 2), init = ~UInt(0, 2))
  val tlb_to_send = to_translate & ~tlb_sent
  val resp_status = Reg(UInt(width = dmaStatusBits))
  val fault_vpn = Reg(UInt(width = vpnBits))
  val ptw_errors = Reg(init = UInt(0, 2))

  tlb.io.req.valid := tlb_to_send.orR
  tlb.io.req.bits.vpn := Mux(tlb_to_send(0), src_vpn, dst_vpn)
  tlb.io.req.bits.passthrough := Bool(false)
  tlb.io.req.bits.instruction := Bool(false)
  tlb.io.req.bits.store := !tlb_to_send(0)
  tlb.io.resp.ready := tlb_sent.orR

  when (tlb.io.req.fire()) {
    tlb_sent := tlb_sent | PriorityEncoderOH(tlb_to_send)
  }

  when (tlb.io.resp.fire()) {
    val recv_choice_oh = PriorityEncoderOH(to_translate)
    val recv_choice = OHToUInt(recv_choice_oh)(0)
    val page_fault = Mux(recv_choice,
      tlb.io.resp.bits.xcpt_st, tlb.io.resp.bits.xcpt_ld)
    val bad_region = Mux(recv_choice,
      alloc(1) && !tlb.io.resp.bits.cacheable,
      alloc(0) && !tlb.io.resp.bits.cacheable)

    when (page_fault || bad_region) {
      resp_status := Mux(page_fault,
        Mux(recv_choice, DST_PAGE_FAULT, SRC_PAGE_FAULT),
        Mux(recv_choice, DST_INVALID_REGION, SRC_INVALID_REGION))
      fault_vpn := Mux(recv_choice, dst_vpn, src_vpn)
      ptw_errors := ptw_errors | recv_choice_oh
    } .otherwise {
      // getting the src translation
      when (recv_choice) {
        dst_ppn := tlb.io.resp.bits.ppn
      } .otherwise {
        src_ppn := tlb.io.resp.bits.ppn
      }

      to_translate := to_translate & ~recv_choice_oh
    }
  }

  io.cpu.req.ready := state === s_idle
  io.cpu.resp.valid := state === s_finish
  io.cpu.resp.bits := ClientDmaResponse(resp_status, fault_vpn)

  io.dma.req.valid := (state === s_dma_req) && !dma_busy.andR
  io.dma.req.bits := DmaRequest(
    xact_id = dma_xact_id,
    cmd = cmd,
    source = src_paddr,
    dest = dst_paddr,
    length = tx_len,
    alloc = alloc)
  io.dma.resp.ready := Bool(true)

  when (io.cpu.req.fire()) {
    val req = io.cpu.req.bits
    when (req.cmd =/= DMA_CMD_RESUME) {
      cmd := req.cmd
      src_vaddr := req.src_start
      dst_vaddr := req.dst_start
      src_stride := req.src_stride
      dst_stride := req.dst_stride
      segment_size := req.segment_size
      segments_left := req.nsegments - UInt(1)
      bytes_left := req.segment_size
      to_translate := Mux(req.isPrefetch(), UInt("b10"), UInt("b11"))
      alloc := req.alloc
    } .otherwise {
      // On resume, retranslate any pages that had errors
      to_translate := ptw_errors
    }
    when (io.pause) {
      resp_status := PAUSED
      state := s_finish
    } .otherwise {
      tlb_sent := UInt(0)
      ptw_errors := UInt(0)
      state := s_translate
    }
  }

  when (state === s_translate && !to_translate.orR) {
    state := Mux(ptw_errors.orR, s_finish, s_dma_req)
  }

  def setBusy(set: Bool, xact_id: UInt): UInt =
    Mux(set, UIntToOH(xact_id), UInt(0))

  dma_busy := (dma_busy |
                setBusy(io.dma.req.fire(), dma_xact_id)) &
                ~setBusy(io.dma.resp.fire(), io.dma.resp.bits.xact_id)


  when (io.dma.req.fire()) {
    src_vaddr := src_vaddr + Mux(adv_ptr(0), tx_len, UInt(0))
    dst_vaddr := dst_vaddr + Mux(adv_ptr(1), tx_len, UInt(0))
    bytes_left := bytes_left - tx_len
    state := s_dma_update
  }

  when (state === s_dma_update) {
    when (bytes_left === UInt(0)) {
      when (segments_left === UInt(0)) {
        resp_status := NO_ERROR
        state := s_finish
      } .otherwise {
        last_src_vpn := src_vpn
        last_dst_vpn := dst_vpn
        src_vaddr := src_vaddr + Mux(adv_ptr(0), src_stride, UInt(0))
        dst_vaddr := dst_vaddr + Mux(adv_ptr(1), dst_stride, UInt(0))
        bytes_left := segment_size
        segments_left := segments_left - UInt(1)
        state := s_prepare
      }
    } .otherwise { state := s_prepare }
  }

  when (state === s_prepare) {
    to_translate := adv_ptr & Cat(
      dst_vpn =/= last_dst_vpn,
      src_vpn =/= last_src_vpn)
    when (io.pause) {
      resp_status := PAUSED
      state := s_finish
    } .otherwise {
      tlb_sent := UInt(0)
      state := s_translate
    }
  }

  when (state === s_finish) { state := s_idle }

  io.busy := (state =/= s_idle) || dma_busy.orR
}
