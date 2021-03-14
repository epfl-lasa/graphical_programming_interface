/**
 * @param event {MouseEvent}
 * @param element {HTMLElement}
 * @return {{x: number, y: number}}
 */

// TOOD differentiate between initial setup & casual loading...
export function propertyLoader (properties) {
  // Create [new] default containers
  Object.values(properties)
    .forEach(prop => {
      if (prop.type === 'eulerPose') {
        if (!(prop.value && prop.value.frameId)) {
          prop.value = {
            frameId: 'base',
            position: {x: 0, y: 0, z: 0},
            orientation: {x: 0, y: 0, z: 0},
            settings: {
              eulerOrder: 'XYZ'
            }
          }
          // } else {
          // console.log(prop.value.frameId)
        }
      } else if (prop.type === 'vec3' || prop.type === 'pos3') {
        if (!(prop.value && prop.value.x)) {
          prop.value = {
            x: 0, y: 0, z: 0
          }
          // } else {
          // console.log(prop.value.frameId)
        }
      } else if (prop.type === 'slider') {
        if (!prop.settings) {
          console.log('Have to do new one')
          prop.settings = {
            min: 0,
            max: 10
          }
        }

        if (!prop.value) {
          if (prop.settings.min <= 0 && prop.settings.max >= 0) {
            prop.value = 0
          } else {
            prop.value = prop.settings.min
          }
        }
        // Create steps and slider on creation
        // sliderHelpers.getSliderTicks(prop.settings)
        // this.getSliderTicks(prop.settings)

        // TODO: maybe safe in json / only modify on creation
      } else if (prop.type === 'button') {
        if (!prop.value) {
          prop.value = Object.keys(prop.settings.values)[0]
        }
      }
      return prop
    })
  return properties
}

export default {
  propertyLoader
}
