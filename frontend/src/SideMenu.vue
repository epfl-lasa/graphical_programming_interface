<template>
<div class="side-menu">
  <div v-if="isActive" class="side-menu-header">
    <h1> Force Visualization </h1>
    <img id="buttonCloseLibraries" class="aica-icon-small"
         src='./../assets/icons/keyboard_arrow_left-white-18dp.svg'
         @click="hideMenu($event)"
         @touchstart="hideMenu($event)"
         >
    <!-- <div v-if="robotIsMoving" class="aica-button danger" id="run-module" -->
         <!-- @click="stopRobot($event)" @touchstart="stopRobot($event)" -->
         <!-- > -->
      <!-- <p> Stop Moving </p> -->
    <!-- </div> -->
  </div>
  <div v-else class="side-menu-header inactive">
      <img id="buttonCloseLibraries" class="aica-icon-small"
         src='./../assets/icons/keyboard_arrow_right-white-18dp.svg'
         @click="showMenu($event)"
         @touchstart="showMenu($event)"
         >
  </div>
  <div v-if="isActive" class="property-panel side-menu-body">
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
    // TODO: reload when new module is changed
    if (this.isActive) {
      this.preparePlot()
    }
    // let aa = this.colorDark
    // console.log(aa)
    // this.startDataFetching()
    // this.updateLoopOfData()
  },
  beforeUnmount () {
    console.log('Unmount')
    if (this.backendIsGeneratingData) {
      this.stopForceRecording()
    }
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
    stopForceRecording (e) {
      if (e.type === 'touchstart') {
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
          titloe: {
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
    },
    showMenu (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      this.isActive = true
      this.preparePlot()
    },
    hideMenu (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      this.isActive = false
    }
  }
  // watch: {
    // isActive (newValue) {
      // if (newValue) {
        // this.preparePlot()
      // }
    // }
  // }
}
</script>

<style lang="less" scoped>
@import './../assets/styles/main.less';

// .modebar{
// display: none !important;
// }
#button-closing {
    position: absolute;
    right: 0px;
}

.side-menu-header .inactive {
    background: transparent;
}

.side-menu{
    left: 0;
}

#force-graph {
    padding-top: @screen-height*0.10;
    width: @sidebar-width * 0.9;
    height: $width * 0.9;
}

.aica-button {
    margin-top: @sidebar-width*0.1;
}



#run-module {
    position: absolute;
    top: @header-height*0.3;
    right: @header-padding-sideways;
}

.reference-button-container{
    display: grid;
    grid-template-columns: auto auto;

    .reference-button{
    }
}

.property-panel {
    .property-field {
        border-bottom: 1px solid @color-border;
        padding-bottom: 10px;
    }
    .property {
        border-bottom: 1px solid @color-border;
        padding-top: 20px;
        padding-bottom: 20px;
    }
}
</style>
