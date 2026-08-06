#include "services/diagnostics.h"

#include "config/config_registry.h"
#include "core/types.h"
#include "services/safety.h"

namespace vcm {

DiagnosticsService diagnostics;

void DiagnosticsService::tick() {
  freeHeap_ = ESP.getFreeHeap();
  minFreeHeap_ = ESP.getMinFreeHeap();

  // CPU load estimate: a fixed busy-loop probe takes longer when the
  // scheduler preempts it, giving a cheap load indication without
  // requiring FreeRTOS runtime stats to be compiled in.
  const uint32_t t0 = micros();
  volatile uint32_t spin = 0;
  for (int i = 0; i < 1000; ++i) spin += i;
  const uint32_t probeUs = micros() - t0;
  const float baselineUs = 8.0f;  // measured unloaded at 240 MHz
  const float load = (probeUs > baselineUs)
                         ? (1.0f - baselineUs / probeUs) * 100.0f
                         : 0.0f;
  cpuLoad_ += 0.3f * (load - cpuLoad_);

  // Low-heap warning fault
  if (freeHeap_ < (uint32_t)config.i(SAF_MIN_HEAP) * 1024u) {
    safety.raiseFault(FLT_LOW_HEAP);
  } else {
    safety.clearFault(FLT_LOW_HEAP);
  }
}

}  // namespace vcm
