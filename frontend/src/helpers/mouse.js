import {getOffsetRect} from './dom'

/**
 * @param event {MouseEvent}
 * @param element {HTMLElement}
 * @return {{x: number, y: number}}
 */
export function getMousePosition (element, event) {
  let mouseX
  let mouseY
  if (event.type === 'touchstart' || event.type === 'touchmove') {
    mouseX = event.touches[0].clientX
    mouseY = event.touches[0].clientY
  } else {
    mouseX = event.pageX || event.clientX + document.documentElement.scrollLeft
    mouseY = event.pageY || event.clientY + document.documentElement.scrollTop
  }

  let offset = getOffsetRect(element)
  let x = mouseX - offset.left
  let y = mouseY - offset.top

  return {
    x: x,
    y: y
  }
}

export default {
  getMousePosition
}
