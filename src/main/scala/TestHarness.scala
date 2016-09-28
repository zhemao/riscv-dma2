package dma

import Chisel._
import cde.Parameters

class TestHarness(q: Parameters) extends rocketchip.TestHarness(q)

object Generator extends util.GeneratorApp {
  val longName = names.topModuleProject + "." + names.configs
  generateFirrtl
  generateTestSuiteMakefrags // TODO: Needed only for legacy make targets
  generateParameterDump // TODO: Needed only for legacy make targets
}
