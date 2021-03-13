<template>
<div class="root-container">
  <div v-if="isActive" class="side-menu">
    <div  class="side-menu-header">
      <h1> {{title}} </h1>
      <img id="buttonCloseLibraryLeft" class="aica-icon-small"
           src='./../assets/icons/keyboard_arrow_left-white-18dp.svg'
           @click="hideMenu($event)"
           @touchstart="hideMenu($event)"
           >
                 <!-- @touchstart="hideMenu($event)" -->


      <!-- <div v-if="robotIsMoving" class="aica-button danger" id="run-module" -->
           <!-- @click="stopRobot($event)" @touchstart="stopRobot($event)" -->
           <!-- > -->
        <!-- <p> Stop Moving </p> -->
      <!-- </div> -->
    </div>
    <div class="property-panel side-menu-body">
      <DataVisualization
        @updateTitle="title='Force Visualization'"
        />
    </div>
  </div>
  <!-- <div> -->
  <div v-else class="side-menu-button-container">
    <img class="aica-icon-small"
         src='./../assets/icons/keyboard_arrow_right-white-18dp.svg'
         @click="showMenu($event)"
         @touchstart="showMenu($event)"
         >
             <!-- @touchstart="showMenu($event)" -->

  </div>
</div>
</template>


<script>
import DataVisualization from './DataVisualization'

export default {
  name: 'SideMenuLeft',
  components: {
    DataVisualization
  },
  mounted () {
    console.log('@SideMnueLeft: Mounted')
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
      valueRange: [0, 50],

      title: ''
    }
  },
  computed: {
  },
  methods: {
    stopRobot (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
    },
    showMenu (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      this.isActive = true
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
// #button-closing {
// position: absolute;
// right: 0px;
// }


.side-menu-header {
    justify-content: left;
}

#inactiveHeader {
    // .side-menu-header .inactive {
    background: transparent;
}

.side-menu{
    left: 0;
}

#buttonCloseLibraryLeft{
    position: absolute;
    left: @sidebar-width - @header-padding-sideways;
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

.side-menu-button-container {
    position: absolute;
    // top: @header-height*0.2;
    bottom: 50vh; // TODO: exactly in the middle?
    left: @header-padding-sideways;
    z-index: 5;
}
</style>
