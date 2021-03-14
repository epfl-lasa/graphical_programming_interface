<template>
<div class="side-menu">
  <div class="side-menu-header">
    <h1> {{ module.title }} </h1>
  </div>
  <div class="property-panel side-menu-body">
     <div class="property" v-for="(p, propKey) in properties">
      <div class="property-box" v-if="p.type==='eulerPose'">
        <h2> {{ p.label }} </h2>
        <div v-if="false">
          <label> Frame: </label>
            <!-- split-variant="outline-dark" -->
          <b-dropdown variant="dark" id="dropdown-1" v-bind:text="referenceFrames[p.value.frameId]" class="m-md-2">
            <template v-for="(frame, key) in referenceFrames">
              <b-dropdown-item v-on:click="p.value.frameId = key" href="#"> {{frame}} </b-dropdown-item>
            </template>
        </b-dropdown>
        </div>
        <div class="button-container-pose">

          <div>
            <h3> Position [mm]</h3>
            <div v-for="(dir, key) in p.value.position">
              <label>{{key}}:</label>
              <input type="number" pattern="[0-9]*" inputmode="numeric"
                     class="pose-input position"
                     v-model="p.value.position[key]" />
                     >
            </div>
          </div>

          <div>
            <h3> Orientation [deg]</h3>
            <div v-for="(dir, key) in p.value.orientation">
              <label>{{key}}:</label>
              <input type="number" pattern="[0-9]*" inputmode="numeric"
                     class="pose-input orientation"
                     v-model="p.value.orientation[key]" />

              <!-- <input type="number" pattern="[0-9]*" inputmode="numeric"> -->
            </div>
          </div>
        </div>

        <div class="reference-button-container">
          <div v-if="robotIsMoving" class="aica-button danger reference-button"
               @click="stopRobot($event)" @touchstart="stopRobot($event)"
               >
            <p> Stop Robot </p>
          </div>
          <div v-else class="aica-button" id="move-reference reference-button"
               @click="moveToPosition($event)" @touchstart="moveToPosition($event)">
            <p> Move Robot </br> to Reference </p>
          </div>

          <div v-if="!(robotIsMoving)"
               @click="setToRobotPosition($event)" @touchstart="setToRobotPosition($event)"
               class="aica-button reference-button"
               :class="{critical: awaitRobotPosition}">
            <p> Set to </br> Robot Position </p>
          </div>
        </div>

      </div>

      <div class="property-box" v-else-if="p.type==='pos3' || p.type==='vec3'">
        <h3> {{p.label}} </h3>
        <template v-for="(dir, key) in p.value">
          <label>{{key}}:</label>
          <input class="position-input" type="number" v-model="p.value[key]">
        </template>
      </div>

      <div class="property-box" v-else-if="p.type==='slider'">
        <label> {{ p.label }} </label>
        <input class="slider-input" type="number" v-model="p.value"/>

        <b-field class="slider-field">
          <b-slider rounded
                    :tooltip="false"
                    :min="p.settings.min"
                    :max="p.settings.max"
                    :step="p.settings.step"
                    v-model="p.value">
            <template v-for="val in getSliderTicks(p.settings)">
                <b-slider-tick :value="val" :key="val">
                  {{ val.toFixed(p.settings.floatDigits) }}
                  <!-- {{ val }} -->
                </b-slider-tick>
            </template>
           </b-slider>
        </b-field>
      </div>

      <div class="property-box" v-else-if="p.type==='database'">

        <ModuleDataList
          :module="module"
          :robotIsMoving="robotIsMoving"
          :multipleRecordings="true"
          :settings=p.settings
          @setRobotStateMoving="setRobotStateMoving"
          @stopRobot="$emit('stopRobot')"
          />
      </div>

      <div class="property-box" v-else-if="p.type==='button'">
        <h4 :for="p.name">{{p.label||p.name}}:</h4>
        <!-- <input type="text" v-model="p.value"> -->
        <div class="reference-button-container">
          <template v-for="(act, key) in p.settings.values">
            <div class="aica-button property-button"
                 @click="setValueButton($event, p, key)"
                 @touchstart="setValueButton($event, p, key)"
                 :class="{selected: key===p.value}"
                 >
              <!-- @click="p.value===key" -->
              <!-- TODO: check this clicking event... -->
              <p> {{act}} </p>
            </div>
          </template>
        </div>
      </div>

      <div class="property-box" v-else>
        <h4 :for="p.name">{{p.label||p.name}}:</h4>
        <input type="text" v-model="p.value">
      </div>
    </div>

     <!-- Currently no saving / canceling implemented -->
    <div v-if="false" id="save-button-container">
      <div class="aica-button reference-button"
           @click="save($event)" @touchstart="save($event)"
           >
         <p> Save </p>
      </div>
      <div class="aica-button critical reference-button"
           @click="cancel($event)" @touchstart="cancel($event)"
           >
        <p> Cancel</p>
       </div>
    </div>

  </div>
</div>
</template>


<script>
import axios from 'axios' // Needed to pass. Only temporarily?

import sliderHelpers from '../helpers/slider'
import blockloaderHelper from '../helpers/blockloader'

import ModuleDataList from './ModuleDataList'

export default {
  name: 'VueBlockProperty',
  components: {
    ModuleDataList
  },
  mounted () {
    this.loadModule()
    // No coupling at startup
    this.robotIsCoupledToPose = false
    // this.moveToButtonLabel = this.buttonLabelsList[0]
    // Update Scene [TODO: only for new ones..]
    // this.$emit('updateScene')
  },
  beforeUnmount () {
    console.log('Unmount')
    if (this.robotIsMoving) {
      this.stopRobot()
    }
  },
  props: {
    robotIsMoving: Boolean,
    property: Object,
    module: Object,
    referenceFrames: {
      type: Object,
      default () {
        return {
          'base': 'Robot Base',
          'endEffector': 'End-Effector'
        }
      }
    }
  },
  data () {
    return {
      properties: [],
      robotIsCoupledToPose: false,
      // isMovingToReference: false,
      moveToButtonLabel: '',
      awaitRobotPosition: false
    }
  },
  // computed: {
  // },
  methods: {
    // Basic math methods
    test () {
      sliderHelpers.getSliderTicks(null)
    },
    log10 (x) {
      return Math.log(x) / Math.log(10)
    },
    ceilStep (x, step) {
      return Math.ceil(x * step) / step
    },
    floorStep (x, step) {
      return Math.floor(x * step) / step
    },
    roundDec (x, dec) {
      return Math.round(x * Math.pow(10, dec)) / Math.pow(10, dec)
    },
    roundStep (x, step) {
      return Math.round(x * Math.pow(10, step)) / Math.pow(10, step)
    },
    getSliderTicks (settings, minNumTicks = 4) {
      return sliderHelpers.getSliderTicks(settings, minNumTicks)
    },
    getSliderTicksOld (settings, minNumTicks = 4) {
      const min = parseFloat(settings.min)
      const max = parseFloat(settings.max)

      var range = max - min
      var pot = Math.floor(this.log10(range / minNumTicks))
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
        tick = this.ceilStep(min, step) // TODO: check if this is ideal
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
    },
    // ------------------------------
    //    Robot Movement
    // ------------------------------
    setRobotStateMoving () {
      this.$emit('setRobotStateMoving')
    },
    stopRobot (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      // console.log('Stop Robot')
      this.$emit('stopRobot')
    },
    setValueButton (e, property, value) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      property.value = value
      // this.properties[propKey].value = toString(value)

      console.log('@Block Property: Button Value')
      console.log(this.properties['action'].value)
      console.log(this.properties)
    },
    executeModule (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      this.setRobotStateMoving()
      axios.get(this.$localIP + `/executemodule/` + this.module.id)
        .then(response => {
          console.log(response.statusText)
          // Finished robot movement - reset to not moving.
          this.$emit('stopRobot')
        })
        .catch(error => {
          console.log(error)
        })
    },
    moveToPosition (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      // Transfer an euler pose
      this.setRobotStateMoving()
      this.robotIsCoupledToPose = true
      axios.get(this.$localIP + `/movetoposition`,
                {'params': {'eulerPose': this.property.reference.value}})
        .then(response => {
          console.log(response.statusText)
          // Finished robot movement - reset to not moving.
          this.$emit('stopRobot')
        })
        .catch(error => {
          console.log(error)
        })
    },
    setToRobotPosition (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      // Set property to position
      this.awaitRobotPosition = true

      axios.get(this.$localIP + `/getrobotposition`)
        .then(response => {
          this.properties.reference.value.frameId = response.data.pose.frameId
          this.properties.reference.value.position = response.data.pose.position
          this.properties.reference.value.orientation = response.data.pose.orientation
          this.awaitRobotPosition = false
        })
        .catch(error => {
          console.log(error)
        })
    },
    // stopAndUncoupleRobot () {
    // this.stopRobot()
    // this.robotIsCoupledToPose = false
    // },
    // stopRobot is implement in App.vue (root)
    toggleMotion (eulerPose) {
      if (this.isMovingToReference === true) {
        // Stop moving robot
        console.log('Stop moving robot')
        this.isMovingToReference = false
        axios.get(this.$localIP + `stopmotion`, {'params': {}})
          .then(response => {
            console.log(response.statusText)
          })
          .catch(error => {
            console.log(error)
          })

        // Set button to 'start moving'
        this.moveToButtonLabel = this.buttonLabelsList[0]
      } else {
        // Start moving robot
        console.log('Move robot to reference.')
        this.isMovingToReference = true
        axios.get(this.$localIP + `/moveto`, {'params': {'eulerPose': eulerPose}})
          .then(response => {
            console.log(response.statusText)
          })
          .catch(error => {
            console.log(error)
          })

        // Set button to 'stop moving'
        this.moveToButtonLabel = this.buttonLabelsList[1]
      }
    },
    // Load Properties and Module
    loadProperties () {
      this.properties = this.property
      console.log('@VueBlockProperties: properties')
      console.log(this.properties)
    },
    loadModule () {
      if (this.module) {
        this.properties = this.module.values.property
      } else {
        this.properties = null
      }
      this.property = blockloaderHelper.propertyLoader(this.property)
    },
    loadModuleOld () {
      if (this.module) {
        this.properties = this.module.values.property
      } else {
        this.properties = null
      }
      // Create [new] default containers
      Object.values(this.properties)
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
    },
    save (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      console.log('@VueBlockPropertylocal props')
      console.log(this.properties)
      this.$emit('save', this.properties)
    },
    cancel (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      console.log('Cancel properties')
    }
    // check () {
      // DEBUG ONLY
      // console.log('check props')
      // console.log(this.properties)
    // },
    // debug Test () {
      // console.log('@VueBlockProperty: Debug Test')
      // console.log(this.properties)
    // }
  },
  watch: {
    module () {
      // console.log('@VueBlockProperty: load module')
      this.loadModule()
      this.robotIsCoupledToPose = false // Reset at mounted
    },
    property () {
      // console.log('@VueBlockProperty: load property')
      console.log('@VueBlockProperty: whats the difference between properties & property')
      this.loadProperties()
    },
    properties (newValue) {
      console.log('@VueBlockProperty: update of properties')
      // TODO: update reference point
    },
    robotIsMoving (newValue) {
      if (!newValue) {
        // When robot movement is stopped make sure it gets uncoupled.
        this.robotIsCoupledToPose = false
      }
      console.log('@VueBlockProperty: Robot is not Moving anymore')
    }
  }
}
</script>

<style lang="less" scoped>
@import './../assets/styles/main.less';
@import './../assets/styles/main.css';

.button-container-pose {
    display: grid;
    grid-template-columns: auto auto;
    margin-bottom: @header-height*0.5;
}

.pose-input {
    // font-size: @fontsize-medium;
    width: @sidebar-width * 0.2;
    text-align: right;
    // float: right;
    // clear: both;
}

.menu-title {
    size: @fontsize-medium;
}


#run-module {
    position: absolute;
    top: @header-height*0.3;
    right: @header-padding-sideways;
}

.aica-button {
    &.selected {
        background-color: var(--color-highlight1-dark);
    }
}

.reference-button-container{
    display: grid;
    grid-template-columns: auto auto;
}

.property-title {
    font-size: @fontsize-huge;
}
.property-panel {
    .property {
        // border-top: 1px solid @color-border;
        border-bottom: 1px solid @color-border;
        padding-top: @header-height*0.5;
        padding-bottom: $padding-top;
    }
}

#save-button-container {
    position: absolute;
    right: @sidebar-width*0.08;
    bottom: $right*1.5;
    // padding: 20px;

    display: grid;
    grid-template-columns: auto auto;
    grid-column-gap: $right;
}

.dropdown-1 {
    position: relative;
    display: inline-block;
    font-color: #FFFFFF
}

.slider-input {
    width: 50px;
}

.slider-field{
    margin-left: 5px;
    margin-right: 5px;
}

</style>
