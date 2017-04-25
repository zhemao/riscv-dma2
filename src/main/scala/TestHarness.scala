package dma

import Chisel._
import config.Parameters

class TestHarness(implicit p: Parameters) extends rocketchip.TestHarness

object Generator extends util.GeneratorApp {
  val longName = names.topModuleProject + "." + names.configs
  generateFirrtl
}
