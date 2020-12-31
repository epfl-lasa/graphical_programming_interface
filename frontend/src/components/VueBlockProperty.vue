<template>
<div class="property-panel">
  <h1> {{ module.title }} </h1>
  <h5> [ Module {{ module.name }} ] </h5>
  <div class="property" v-for="p in properties">
    <div class="property-box" v-if="p.type==='eulerPose'">
      <h3> Position </h3>
      <div>
        <label> Frame: </label>
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
      <br>
    </div>

    <div class="property-box" v-else>
      <label :for="p.name">{{p.label||p.name}}:</label>
      <input type="text" v-model="p.value">
    </div>

  </div>
  <button class="save-button" @click.prevent="save">Save</button>
  <!-- <button class="save-button" @click.prevent="check">Check</button> -->
</div>
</template>

<script>
export default {
  name: 'VueBlockProperty',
  // props: ['property'],
  mounted () {
    this.loadModule()
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
      properties: []
    }
  },
  methods: {
    // updateFrameId (prop, key) {
      // prop.value.frameId = key
    // },
    // Why is it currently needed twice? Can this be changed...
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

      Object.values(this.properties)
        .forEach(prop => {
          if (prop.type === 'eulerPose') {
            if (prop.value.frameId) {
              console.log(prop.value.frameId)
            } else {
              prop.value = {
                frameId: 'base',
                position: {x: 0, y: 0, z: 0},
                orientation: {x: 0, y: 0, z: 0},
                eulerOrder: 'XYZ'
              }
            }
          }
          return prop
        })

      console.log('Properties')
      console.log(this.properties)

      console.log('Module')
      console.log(this.module)
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
