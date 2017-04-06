package dma

import chisel3._
import rocket._
import groundtest._
import coreplex.WithL2Cache
import rocketchip._
import junctions._
import uncore.tilelink.TLKey
import uncore.agents.CacheBlockBytes
import cde.{Parameters, Config, Knob, CDEMatchError}

class WithDma extends Config(
  topDefinitions = (pname, site, here) => pname match {
    case BuildRoCC => Seq(
      RoccParameters(
        opcodes = OpcodeSet.all,
        generator = (p: Parameters) => Module(new CopyAccelerator()(p)),
        nMemChannels = site(NDmaTrackers),
        nPTWPorts = 1))
    case NDmaTrackers => 1
    case NDmaXacts => 4
    case NDmaTrackerMemXacts => {
      val innerDataBits = site(XLen)
      val innerDataBeats = (8 * site(CacheBlockBytes)) / innerDataBits
      site(DmaTrackerPipelineDepth) / innerDataBeats
    }
    // 3 clients (prefetch, put, get) per tracker
    case RoccMaxTaggedMemXacts => 3 * site(NDmaTrackerMemXacts)
    case DmaTrackerPipelineDepth => 16
    case BuildDmaTracker => (p: Parameters) => Module(new PipelinedDmaTracker()(p))
    case TLKey("DMA") => {
      val oldKey = site(TLKey("L1toL2"))
      val newBeats = oldKey.dataBits / 64
      oldKey.copy(dataBeats = newBeats)
    }
    case SimMemLatency => 20
    case _ => throw new CDEMatchError
  })

class DmaConfig extends Config(new WithDma ++ new WithL2Cache ++ new BaseConfig)

class WithDmaTest extends Config(
  (pname, site, here) => pname match {
    case GroundTestKey => Seq(GroundTestTileSettings(
      uncached = 1, cached = 1, ptw = 1,
      maxXacts = 3 * site(NDmaTrackerMemXacts) * site(NDmaTrackers)))
    case DmaTestKey => DmaTestParameters(
      src_start = 0x80000004L,
      dst_start = 0x80001000L,
      segment_size = 0x100,
      nsegments = 1,
      src_stride = 0,
      dst_stride = 0)
    case BuildGroundTest => (p: Parameters) => Module(new DmaTest()(p))
    case _ => throw new CDEMatchError
  })

class DmaTestConfig extends Config(new WithDmaTest ++ new WithDma ++ new WithL2Cache ++ new GroundTestConfig)
