/**
 * @param element {HTMLElement}
 * @return {{top: number, left: number}}
 */
export function ceilStep (x, step) {
  return Math.ceil(x * step) / step
}

export default {
  ceilStep
}
