package dma

import chisel3._
import chisel3.util._
import rocket.RoCC
import uncore.tilelink._
import rocket._
import cde.{Parameters, Field}
import scala.math.max

object DmaCtrlRegNumbers {
  val SRC_STRIDE = 0
  val DST_STRIDE = 1
  val SEGMENT_SIZE = 2
  val NSEGMENTS = 3
  val ACCEL_CTRL = 4
  val RESP_STATUS = 5
  val RESP_VPN = 6
}
import DmaCtrlRegNumbers._

class DmaCtrlRegFile(implicit val p: Parameters) extends Module
    with HasClientDmaParameters with HasTileLinkParameters {

  private val nRegs = 7

  val io = IO(new Bundle {
    val wen = Input(Bool())
    val rwaddr = Input(UInt(log2Up(nRegs).W))
    val wdata = Input(UInt(dmaSegmentSizeBits.W))
    val rdata = Output(UInt(dmaSegmentSizeBits.W))
    val set = Input(Bool())
    val clear = Input(Bool())

    val src_stride = Output(UInt(dmaSegmentSizeBits.W))
    val dst_stride = Output(UInt(dmaSegmentSizeBits.W))
    val segment_size = Output(UInt(dmaSegmentSizeBits.W))
    val nsegments = Output(UInt(dmaSegmentBits.W))
    val alloc = Output(UInt(2.W))
    val pause = Output(Bool())

    val dma_resp = Flipped(Valid(new ClientDmaResponse))
    val error = Output(Bool())
  })

  val regSize = max(dmaSegmentSizeBits, vpnBits)
  val regs = Reg(Vec(nRegs, UInt(regSize.W)))

  when (reset) {
    regs(ACCEL_CTRL) := "b010".U
    regs(RESP_STATUS) := 0.U
  }

  io.src_stride := regs(SRC_STRIDE)
  io.dst_stride := regs(DST_STRIDE)
  io.segment_size := regs(SEGMENT_SIZE)
  io.nsegments := regs(NSEGMENTS)
  io.alloc := regs(ACCEL_CTRL)(1, 0)
  io.pause := regs(ACCEL_CTRL)(2)

  val wdata = MuxCase(io.wdata, Seq(
    io.set -> (regs(io.rwaddr) | io.wdata),
    io.clear -> (regs(io.rwaddr) & ~io.wdata)))

  when (io.wen) { regs(io.rwaddr) := wdata }
  when (io.dma_resp.valid) {
    regs(RESP_STATUS) := io.dma_resp.bits.status
    regs(RESP_VPN) := io.dma_resp.bits.fault_vpn
  }

  io.rdata := regs(io.rwaddr)
  io.error := regs(RESP_STATUS) =/= 0.U
}

class DmaController(implicit val p: Parameters) extends Module
    with HasClientDmaParameters {
  val io = IO(new Bundle {
    val cmd = Flipped(Decoupled(new RoCCCommand))
    val resp = Decoupled(new RoCCResponse)
    val ptw = new TLBPTWIO
    val dma = new DmaIO
    val busy = Output(Bool())
    val interrupt = Output(Bool())
  })

  val cmd = Queue(io.cmd)
  val inst = cmd.bits.inst
  val is_transfer = inst.funct < 4.U
  val is_cr_read = inst.funct === 4.U
  val is_cr_write = inst.funct >= 5.U && inst.funct <= 7.U
  val is_cr_set = inst.funct === 6.U
  val is_cr_clear = inst.funct === 7.U

  val crfile = Module(new DmaCtrlRegFile)
  val frontend = Module(new DmaFrontend)

  crfile.io.rwaddr := cmd.bits.rs1
  crfile.io.wdata := cmd.bits.rs2
  crfile.io.wen := cmd.fire() && is_cr_write
  crfile.io.set := is_cr_set
  crfile.io.clear := is_cr_clear
  crfile.io.dma_resp <> frontend.io.cpu.resp

  frontend.io.cpu.req.valid := cmd.valid && is_transfer
  frontend.io.cpu.req.bits := ClientDmaRequest(
    cmd = cmd.bits.inst.funct,
    src_start = cmd.bits.rs2,
    dst_start = cmd.bits.rs1,
    src_stride = crfile.io.src_stride,
    dst_stride = crfile.io.dst_stride,
    segment_size = crfile.io.segment_size,
    nsegments = crfile.io.nsegments,
    alloc = crfile.io.alloc)
  frontend.io.pause := crfile.io.pause

  io.ptw <> frontend.io.ptw
  io.dma <> frontend.io.dma
  io.busy := cmd.valid || frontend.io.busy
  io.interrupt := false.B

  io.resp.valid := cmd.valid && is_cr_read
  io.resp.bits.rd := inst.rd
  io.resp.bits.data := crfile.io.rdata

  cmd.ready := (is_transfer && frontend.io.cpu.req.ready) ||
               is_cr_write || // Write can always go through immediately
               (is_cr_read && io.resp.ready)
}

class CopyAccelerator(implicit p: Parameters) extends RoCC()(p) {
  val ctrl = Module(new DmaController)
  val backend = Module(new DmaBackend)

  ctrl.io.cmd <> io.cmd
  io.resp <> ctrl.io.resp
  io.ptw.head <> ctrl.io.ptw
  io.busy := ctrl.io.busy

  backend.io.dma <> ctrl.io.dma

  io.utl <> backend.io.mem
  io.autl.acquire.valid := false.B
  io.autl.grant.ready := false.B

  io.mem.req.valid := false.B
  io.mem.invalidate_lr := false.B
  io.interrupt := ctrl.io.interrupt
}
