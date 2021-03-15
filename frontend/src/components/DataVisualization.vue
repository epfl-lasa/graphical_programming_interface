<template>
<div>
  <div class="property-panel property-field">
    <p>
      <a id="force-graph"></a>
    </p>
  </div>
  <div>
    <div v-if="backendIsGeneratingData"  class="aica-button"
         @click="stopForceRecording($event)" @touchstart="stopForceRecording($event)" >
      <p> Stop Updating </p>
    </div>
    <div v-else  class="aica-button" id="button-closing"
         @click="startForceRecording($event)" @touchstart="startForceRecording($event)">
      <p> Visualize Force </p>
    </div>
  </div>
</div>
</template>


<script>
import axios from 'axios'
import Plotly from 'plotly.js-dist'

// Create sleep-function using promise & resolve
const sleep = milliseconds => new Promise(resolve => setTimeout(resolve, milliseconds))

export default {
  name: 'DataVisualization',
  // components: {
  // DataVisualization
  // },
  mounted () {
    this.preparePlot()
    // this.$emit('update:title', this.title)
    this.$emit('updateTitle')

    // Always send it. Just to be sure...
    this.stopForceRecording()
  },
  beforeUnmount () {
    console.log('Unmount')

    // Always send it. Just to be sure...
    this.stopForceRecording()
  },
  props: {
  },
  data () {
    return {
      isActive: false,

      deltaTime: 10, // [s]

      forceGraph: null,

      backendIsGeneratingData: false,

      // Data
      timeData: [],
      measurementData: [],

      //
      referenceValueY: 10,
      valueRange: [0, 50]
    }
  },
  computed: {
    colorDark () {
      return getComputedStyle(document.documentElement)
        .getPropertyValue('--color-main-medium')
      // return document.getElementById(').style.getProperty('--fontcolor-main')
    },
    fontColor () {
      return getComputedStyle(document.documentElement)
        .getPropertyValue('--fontcolor-main')
      // return document.getElementById(').style.getProperty('--fontcolor-main')
    },
    colorMediumBright () {
      return getComputedStyle(document.documentElement)
        .getPropertyValue('--color-main-mediumbright')
    },
    plotColor1 () {
      return getComputedStyle(document.documentElement)
        .getPropertyValue('--color-highlight1-bright')
    },
    plotColor2 () {
      return getComputedStyle(document.documentElement)
        .getPropertyValue('--color-highlight2-bright')
    }
  },
  methods: {
    startForceRecording (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      this.backendIsGeneratingData = true

      axios.get(this.$localIP + `/startforcerecording`)
        .then(response => {
          // Only get response after shutdown. TODO: don't wait for response
          console.log('Ended force reading')
        })
        .catch(error => {
          console.log(error)
        })
      console.log('Started force fetching')
      this.updateLoopOfData()
    },
    async updateLoopOfData (deltaTime = 333) {
      while (this.backendIsGeneratingData) {
        axios.get(this.$localIP + `/updateforcedata`)
          .then(response => {
            this.measurementData = response.data.data
            this.timeData = response.data.time

            if ('reference' in response.data) {
              this.referenceValueY = response.data.reference
            }

            let update
            update = {'x': [this.timeData], 'y': [this.measurementData]}
            Plotly.update(this.forceGraph, update, {}, [1])

            update = {
              'x': [[this.timeData[0], this.timeData[this.timeData.length - 1]]],
              'y': [[this.referenceValueY, this.referenceValueY]]
            }
            Plotly.update(this.forceGraph, update, {}, [0])

            // Reset y-range
            Plotly.relayout(this.forceGraph, 'yaxis.range', this.valueRange)
          })
          .catch(error => {
            console.log(error)
          })
        await sleep(deltaTime)
      }
    },
    stopForceRecording (e = null) {
      if (e !== null && e.type === 'touchstart') {
        e.preventDefault()
      }
      this.backendIsGeneratingData = false
      console.log('Stop force fetching.')

      axios.get(this.$localIP + `/stopforcerecording`)
        .then(response => {
          console.log('Force recording stopped')
        })
        .catch(error => {
          console.log(error)
        })
    },
    stopRobot (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
    },
    preparePlot () {
      this.forceGraph = document.getElementById('force-graph')

      // let TESTER = document.getElementById('tester')
      let refLine = {
        x: [0, 1],
        y: [this.referenceValueY, this.referenceValueY],
        mode: 'lines',
        line: {
          color: this.plotColor2,
          dash: 'dot',
          width: 1
        }
      }

      let sensorData = {
        x: [],
        y: [],
        mode: 'lines',
        line: {
          color: this.plotColor1,
          width: 1
        }
      }

      let layout = {
        xaxis: {
          title: {
            text: ' Time [s]'
          }
        },
        yaxis: {
          title: {
            text: ' Force [N]',
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

      Plotly.newPlot(this.forceGraph, [refLine, sensorData], layout, {displayModeBar: false})
    }
  }
}
</script>

<style lang="less" scoped>
@import './../assets/styles/main.less';

// .modebar{
// display: none !important;
// }

#force-graph {
    // margin-top: @screen-height*0;
    width: @sidebar-width * 0.85;
    height: $width * 0.9;
}

.aica-button {
    margin-top: @sidebar-width*0.1;
}
</style>
