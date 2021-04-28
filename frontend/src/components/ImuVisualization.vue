<template>
<div>
  <div class="property-panel property-field">
    <p>
      <a id="imu-graph"></a>
    </p>
  </div>
  <div>
    <div v-if="updatingData"  class="aica-button"
         @click="stopUpdating($event)" @touchstart="stopUpdating($event)" >
      <p> Stop Updating </p>
    </div>
    <div v-else  class="aica-button" id="button-closing"
         @click="startUpdating($event)" @touchstart="startUpdating($event)">
      <p> Visualize Data </p>
    </div>
  </div>
  <div>
   <tr>
     <th></th>
     <th>Acceleration [m/s2]</th>
     <th>Gyroscope [deg/s]</th>
   </tr>
   <tr>
     <td> x </td>
     <td>{{roundDec(this.motion.acceleration.x, 3)}}</td>
     <td>{{roundDec(this.motion.rotationRate.alpha, 3)}}</td>
   </tr>
   <tr>
     <td> y </td>
     <td>{{roundDec(this.motion.acceleration.y, 3)}}</td>
     <td>{{roundDec(this.motion.rotationRate.beta, 3)}}</td>
   </tr>
   <tr>
     <td> z </td>
     <td>{{roundDec(this.motion.acceleration.z, 3)}}</td>
     <td>{{roundDec(this.motion.rotationRate.gamma, 3)}}</td>
   </tr>
  </div>
</div>
</template>


<script>
// import axios from 'axios'
import Plotly from 'plotly.js-dist'

// Create sleep-function using promise & resolve
const sleep = milliseconds => new Promise(resolve => setTimeout(resolve, milliseconds))

export default {
  name: 'ImuVisualization',
  // components: {
  // DataVisualization
  // },
  mounted () {
    this.colorDark = getComputedStyle(document.documentElement)
      .getPropertyValue('--color-main-medium')
    this.fontColor = getComputedStyle(document.documentElement)
      .getPropertyValue('--fontcolor-main')
    // return document.getElementById(').style.getProperty('--fontcolor-main')
    this.colorMediumBright = getComputedStyle(document.documentElement)
      .getPropertyValue('--color-main-mediumbright')
    this.plotColor0 = getComputedStyle(document.documentElement)
      .getPropertyValue('--color-highlight3-bright')
    this.plotColor1 = getComputedStyle(document.documentElement)
      .getPropertyValue('--color-highlight1-bright')
    this.plotColor2 = getComputedStyle(document.documentElement)
      .getPropertyValue('--color-highlight2-bright')

    // this.$emit('update:title', this.title)
    this.$emit('updateTitle')

    this.preparePlot()
    // Always send it. Just to be sure...
    this.stopUpdating()

    window.addEventListener('deviceorientation', this.handleOrientation, true)
    window.addEventListener('devicemotion', this.handleMotion, true)
  },
  beforeUnmount () {
    console.log('Unmount')

    // Always send it. Just to be sure...
    this.stopUpdating()

    window.removeEventListener('deviceorientation', this.handleOrientation, true)
    window.removeEventListener('devicemotion', this.handleMotion, true)
  },
  props: {
  },
  data () {
    return {
      isActive: false,

      deltaTime: 10, // [s]

      forceGraph: null,

      updatingData: false,

      // Data
      imuData: [],

      timeData: [],
      orientation: {
        alpha: [],
        beta: [],
        gamma: []
      },

      //
      referenceValueY: 0,
      valueRange: [-1, 1],
      //
      // Define colors
      plotColor1: '',
      plotColor2: '',
      colorMediumBright: '',
      fontColor: '',
      colorDark: ''
    }
  },
  computed: {
    motion () {
      if (this.imuData.length) {
        return this.imuData[this.imuData.length - 1]
      } else {
        return {
          acceleration: {x: 0, y: 0, z: 0},
          rotationRate: {alpha: 0, beta: 0, gamma: 0}
        }
      }
    }
  },
  methods: {
    roundDec (x, dec) {
      return Math.round(x * Math.pow(10, dec)) / Math.pow(10, dec)
    },
    handleMotion (event) {
      // console.log(event)

      // Get Acceleromter
      this.imuData.push(event)

      let maxSavingTime = 5000
      while (this.imuData[this.imuData.length - 1].timeStamp - this.imuData[0].timeStamp > maxSavingTime) {
        // delete first element
        this.imuData.shift()
        // console.log('Remove element')
      }
    },
    handleOrientation (event) {
      // Device orientation
      this.timeData.push(event.timeStamp)

      this.orientation.alpha.push(event.alpha)
      this.orientation.beta.push(event.beta)
      this.orientation.gamma.push(event.gamma)

      // console.log('imu alpha', this.orientation.orientation.alpha)
      //
      let maxSavingTime = 5000 // [ms]
      while (this.timeData[this.timeData.length - 1] - this.timeData[0] > maxSavingTime) {
        // delete first element
        this.timeData.shift()
        this.orientation.alpha.shift()
        this.orientation.beta.shift()
        this.orientation.gamma.shift()
        // console.log('Remove element')
      }
    },
    startUpdating (e) {
      if (e !== null && e.type === 'touchstart') {
        e.preventDefault()
      }
      this.updatingData = true
      this.updateLoopOfData()
    },
    async updateLoopOfData (deltaTime = 333) {
      while (this.updatingData) {
        // Get new data

        // const timeAccelGyroArray = this.imuDatamap(value => {
        // return [
        // })
        const accX = this.imuData.map(value => {
          return value.acceleration.x
        })

        const accY = this.imuData.map(value => {
          return value.acceleration.y
        })

        const accZ = this.imuData.map(value => {
          return value.acceleration.z
        })

        const timeMove = this.imuData.map(value => {
          return value.timeStamp - this.imuData[0].timeStamp
        })

        const newTime = this.timeData.map(value => {
          // return (value - this.timeData[this.timeData.length - 1])
          return (value - this.timeData[0])
        })
        console.log('time', newTime)

        let update
        // update = {'x': [this.timeData], 'y': [this.orientation.alpha]}
        update = {'x': [timeMove], 'y': [accX]}
        Plotly.update(this.forceGraph, update, {}, [0])
        // update = {'x': [newTime], 'y': [this.orientation.beta]}
        update = {'x': [timeMove], 'y': [accY]}
        Plotly.update(this.forceGraph, update, {}, [1])
        // update = {'x': [newTime], 'y': [this.orientation.gamma]}
        update = {'x': [timeMove], 'y': [accZ]}
        Plotly.update(this.forceGraph, update, {}, [2])

        // Reset y-range
        // Plotly.relayout(this.forceGraph, 'yaxis.range', this.valueRange)
        await sleep(deltaTime)
      }
    },
    stopUpdating (e = null) {
      if (e !== null && e.type === 'touchstart') {
        e.preventDefault()
      }

      this.updatingData = false

      this.imuData = []

      this.timeData = []
      this.orientation.alpha = []
      this.orientation.beta = []
      this.orientation.gamma = []

      // axios.get(this.$localIP + `/stopforcerecording`)
      //   .then(response => {
      //     console.log('Force recording stopped')
      //   })
      //   .catch(error => {
      //     console.log(error)
      //   })
    },
    preparePlot () {
      this.forceGraph = document.getElementById('imu-graph')

      // let TESTER = document.getElementById('tester')
      let sensorData0 = {
        x: [],
        y: [],
        mode: 'lines',
        line: {
          color: this.plotColor0,
          width: 1
        }
      }

      let sensorData1 = {
        x: [],
        y: [],
        mode: 'lines',
        line: {
          color: this.plotColor1,
          width: 1
        }
      }

      let sensorData2 = {
        x: [],
        y: [],
        mode: 'lines',
        line: {
          color: this.plotColor2,
          width: 1
        }
      }

      let layout = {
        xaxis: {
          title: {
            text: ' Time [ms]'
          }
        },
        yaxis: {
          title: {
            text: ' Orienation [deg]',
            fixedrange: true,
            range: this.valueRange
          }
        },
        font: {
          // family:
          // size:
          color: this.fontColor
        },
        margin: { r: 10, t: 5, l: 40, b: 35 },
        paper_bgcolor: this.colorMediumBright,
        plot_bgcolor: this.colorDark,
        showlegend: false
      }

      Plotly.newPlot(this.forceGraph, [sensorData0, sensorData1, sensorData2],
                     layout, {displayModeBar: false})
      // console.log(this.forceGraph, [refLine, sensorData], layout, {displayModeBar: false})
    }
  }
}
</script>

<style lang="less" scoped>
@import './../assets/styles/main.less';

// .modebar{
// display: none !important;
// }
td {
    text-align: right;
}

#imu-graph {
    // margin-top: @screen-height*0;
    width: @sidebar-width * 0.85;
    height: $width * 0.9;
}

.aica-button {
    margin-top: @sidebar-width*0.1;
}
</style>
