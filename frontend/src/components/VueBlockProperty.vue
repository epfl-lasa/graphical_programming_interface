<template>
<div class="property-panel">
  <h1> {{ module.title }} </h1>
  <h5> [ Module {{ module.name }} ] </h5>
  <div class="property" v-for="p in properties">
    <div class="property-box" v-if="p.type==='eulerPose'">
      <h3> {{ p.label }} </h3>
      <div>
        <label> Frame: </label>
          <!-- split-variant="outline-dark" -->
        <b-dropdown variant="dark" id="dropdown-1" v-bind:text="referenceFrames[p.value.frameId]" class="m-md-2">
          <template v-for="(frame, key) in referenceFrames">
            <b-dropdown-item v-on:click="p.value.frameId = key" href="#"> {{frame}} </b-dropdown-item>
          </template>
      </b-dropdown>
      </div>
      <h6> Position [mm]</h6>
      <template v-for="(dir, key) in p.value.position">
        <label>{{key}}:</label>
        <input class="position-input" type="text" v-model="p.value.position[key]">
      </template>
      <br> <br>

      <h6> Orientation [deg]</h6>
      <template v-for="(dir, key) in p.value.orientation">
        <label>{{key}}:</label>
        <input class="position-input" type="text" v-model="p.value.orientation[key]">
      </template>
      <br> <br>

      <button type="button" class="btn btn-secondary" v-on:click="toggleMotion(p)">
        {{ moveToButtonLabel }}
      </button>
    </div>

    <div class="property-box" v-else-if="p.type==='pos3' || p.type==='vec3'">
      <h6> {{p.label}} </h6>
      <template v-for="(dir, key) in p.value">
        <label>{{key}}:</label>
        <input class="position-input" type="text" v-model="p.value[key]">
      </template>
    </div>

    <div class="property-box" v-else-if="p.type==='slider'">
      <!-- <h4> {{ p.label }}: {{p.value}} </h4> -->
      <label> {{ p.label }} </label>
      <input class="slider-input" type="text" v-model="p.value">

      <!-- TODO: remove after debugging -->
      <!-- <b-field> -->
      <!--   <p> Min </p> -->
      <!--   <b-slider :min="-1" :max="0" :step="0.1" v-model="p.settings.min"> </b-slider> -->
      <!-- </b-field> -->
      <!-- <b-field> -->
      <!--   <br> -->
      <!--   <p> Max </p> -->
      <!--   <b-slider :min="1" :max="100" :step="10" v-model="p.settings.max"> </b-slider> -->
      <!-- </b-field> -->

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

    <div class="property-box" v-else>
    <!-- <div class="property-box" v-else-if="p.type==='text'"> -->
      <!-- <label :for="p.name">{{p.label||p.name}}:</label> -->
      <h4 :for="p.name">{{p.label||p.name}}:</h4>
      <input type="text" v-model="p.value">
    </div>

  </div>
  <button class="save-button" @click.prevent="save">Save</button>
  <!-- <button class="save-button" @click.prevent="check">Check</button> -->
</div>
</template>

<script>
import axios from 'axios' // Needed to pass. Only temporarily?

export default {
  name: 'VueBlockProperty',
  // components: {
  // RangeSlider
  // VueSimpleRangeSlider
  // },
  // props: ['property'],
  mounted () {
    this.loadModule()
    this.moveToButtonLabel = this.buttonLabelsList[0]
  },
  props: {
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
      isMovingToReference: false,
      buttonLabelsList: ['Move Robot to Reference', 'Stop Robot'],
      moveToButtonLabel: ''
    }
  },
  // computed: {
  // },
  methods: {
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
      console.log('DEBUG: running getSliderTicks')
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
        axios.get(this.$localIP + `moveto`, {'params': {'eulerPose': eulerPose}})
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
    loadProperties () {
      this.properties = this.property
      console.log(this.properties)
    },
    loadModule () {
      if (this.module) {
        this.properties = this.module.values.property
      } else {
        this.properties = null
      }

      console.log('loadinloading')
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
            console.log('Slider start')
            console.log(prop.settings)
            if (!prop.settings) {
              console.log('Have to do new one')
              prop.settings = {
                min: 0,
                max: 10
              }
              if (!prop.value) {
                if (prop.settings.min < 0 && prop.settings.max > 0) {
                  prop.value = 0
                } else {
                  prop.value = prop.settings.min
                }
              }
            }
            // Create steps and slider on creation
            this.getSliderTicks(prop.settings)

            // TODO: maybe safe in json / only modify on creation
          }
          return prop
        })
    },
    save () {
      console.log('local props')
      console.log(this.properties)
      this.$emit('save', this.properties)
    },
    check () {
      // DEBUG ONLY
      console.log('check props')
      console.log(this.properties)
    }
  },
  watch: {
    module () {
      console.log('load module')
      this.loadModule()
    },
    property () {
      console.log('load property')
      this.loadProperties()
    }
  }
}
</script>

<style lang="less" scoped>
.property-panel {
    position: absolute;
    right: 0;
    top: 60px;

    width: 300px;
    // min-height: 200px;
    // min-height: 200px;
    box-sizing: border-box;
    padding: 8px;

    background: #87adc4;
    // border: 5px solid #000000;
    border: 2px solid	#FFFFFF;
    border-radius: 15px;

    .property {
        background:  #4c7c9a ;
        border: 2px solid	#000000;
        border-radius: 5px;

        padding: 5px;
        margin-top: 5px;
    }
}

.property-box {
    // text-color:
}

.save-button {
    margin-top: 10px;
    // left: 50%;
    // left: 100px;
    font-size: 20px;
    border-radius: 4px;
    margin-left: 40%;
    bottom: 10px;

    // margin-left: 0;
    // margin-right: 0;
    // position: absolute;
    // left: 50%;
    // -ms-transform: translateX(-50%);
    // transform: translateX(-50%)
    // -ms-transform: translate(-50%, -50%);
    // transform: translate(-50%, -50%);
}

.dropdown-1 {
    position: relative;
    display: inline-block;
    font-color: #FFFFFF
                    // background: #FFFFFF;
}

.position-input {
    width: 50px;
    margin-right: 10px;
}

.orientaion-input {
    width: 50px;
    margin-right: 10px;
}

.slider-input {
    width: 50px;
}

.slider-field{
    margin-left: 5px;
    margin-right: 5px;
}
</style>
