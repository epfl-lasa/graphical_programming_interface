import {log10} from './log10'
import {ceilStep} from './ceilStep'

/**
 * @param element {HTMLElement}
 */


export function getSliderTicks (settings, minNumTicks = 4) {
  const min = parseFloat(settings.min)
  const max = parseFloat(settings.max)

  var range = max - min
  var pot = Math.floor(log10(range / minNumTicks))
  var step = Math.pow(10, pot)

  // Number of steps on the ruler (smaller than 50)
  if (range / step < 20) {
    settings.step = step / 2
  } else if (range / step < 5) {
    settings.step = step / 10
  } else {
    settings.step = step
  }
  settings.floatDigits = Math.max(pot * (-1), 0)

  if (range / step > 11) {
    // Additionally allow for 5er steps
    step = step * 5
  }

  var tick
  const tickList = []
  if (min >= 0 || max <= 0) {
    tick = ceilStep(min, step) // TODO: check if this is ideal
    while (tick <= max) {
      tickList.push(tick)
      tick = tick + step
    }
  } else {
    tickList.push(0)

    tick = step
    while (tick <= max) {
      tickList.push(tick)
      tick = tick + step
    }
    tick = (-1) * step
    while (tick >= min) {
      tickList.push(tick)
      tick = tick - step
    }
  }
  return tickList
}

export default {
  getSliderTicks
}
