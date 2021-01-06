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

    <div class="property-box" v-else-if="p.type==='slider'">
      <h4> {{ p.label }} </h4>
      <label> {{p.value.range[0]}} </label>

      <label> {{p.value.range[1]}} </label>
      <template v-if="0">
        <v-container fluid>
          <v-row>
            <v-col cols="12">
              <v-slider
                :min="p.value.range[0]"
                :max="p.value.range[1]"
                value="30"
                v-model="p.value.value"
                ></v-slider>
            </v-col>
          </v-row>
        </v-container>
      </template>
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
  methods: {
    // updateFrameId (prop, key) {
    // prop.value.frameId = key
    // },
    // Why is it currently needed twice? Can this be changed...
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

      // Create [new] default containers
      Object.values(this.properties)
        .forEach(prop => {
          if (prop.type === 'eulerPose') {
            if (prop.value && prop.value.frameId) {
              console.log(prop.value.frameId)
            } else {
              prop.value = {
                frameId: 'base',
                position: {x: 0, y: 0, z: 0},
                orientation: {x: 0, y: 0, z: 0},
                eulerOrder: 'XYZ'
              }
            }
          } else if (prop.type === 'slider') {
            if (prop.value && prop.value.value) {
              console.log(prop.value.frameId)
            } else {
              prop.value = {
                value: 0,
                range: [0.0, 10.5]
              }
            }
          }
          return prop
        })

      // console.log('Properties')
      // console.log(this.properties)

      // console.log('Module')
      // console.log(this.module)
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
</style>
