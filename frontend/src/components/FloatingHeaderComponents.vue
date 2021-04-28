<template>
<header>
  <div class="floating-header">
    <div id="home-logo"
         @click="mainMenuToggle($event)" @touchstart="mainMenuToggle($event)">
      <img src='./../assets/logotypes/aica_small_closed_white.svg'>
    </div>
    <div class="aica-dropdown-menu" id="drop-down">
      <ul class="dropdown-head aica-dropdown-menu">
        <img src='./../assets/icons/keyboard_arrow_down-white-18dp.svg'
             @click="clickDropdown($event)" @touchstart="clickDropdown($event)"
             class="aica-icon-small"
             >
      </ul>
      <!-- <dl> -->
        <!-- <img src='./../assets/icons/keyboard_arrow_down-white-18dp.svg'> -->
      <!-- </dl> -->
      <ul class="aica-dropdown-menu" v-if="dropdownActive"
          @click="saveBlocks($event)" @touchstart="saveBlocks($event)"> Save Blocks </ul>
      <ul class="aica-dropdown-menu" v-if="dropdownActive"
          @click="loadBlocks($event)" @touchstart="loadBlocks($event)"> Load Blocks </ul>
      <!-- TODO: implement later... -->
      <!-- <ul class="aica-dropdown-menu" v-if="dropdownActive" -->
          <!-- @click="createNew($event)" @touchstart="createNew($event)"> Create New </ul> -->
    </div>
    <template v-if="!robotIsCompliant">
    <div v-if="robotIsMoving"  class="aica-button danger" id="run-sequence"
         @click="stopRobot($event)" @touchstart="stopRobot($event)" >
      <p> Stop Moving </p>
    </div>
    <div v-else  class="aica-button" id="run-sequence"
         @click="executeSequence($event)" @touchstart="executeSequence($event)">
      <p> Run All </p>
    </div>
    </template>

    <div v-if="!robotIsMoving"
         class="aica-button" :class="{compliant: robotIsCompliant}" id="compliant-button"
         @click="compliantModeToggle($event)" @touchstart="compliantModeToggle($event)" >
      <p v-if="robotIsCompliant"> Stop Compliant </p>
      <p v-else> Start Compliant </p>
    </div>
  </div>

  <div v-if="mainMenuActive" class="main-menu-page">
    <div class="logo-container">
      <img  class="logo-large" src='./../assets/logotypes/aica_original_white.svg'>
      <!-- <img  class="logo-large" src='./../assets/logotypes/recomatic.png'> -->
      <h1 id='aica-title'> Artifical Intelligence to <br> Simplify Robot Programming </h1>
    </div>
    <div  class="aica-button"
          @click="mainMenuToggle($event)" @touchstart="mainMenuToggle($event)">
          <p> Start Programming </p>
    </div>
    <div v-if="robotIsMoving"  class="aica-button danger"
         @click="stopRobot($event)" @touchstart="stopRobot($event)" >
      <p> Stop Moving </p>
    </div>
    <div v-else class="aica-button reference-button" :class="{critical: isResetting}"
         @click="resetBackendAll"
         @touchstart="resetBackendAll"
         >
      Reset Backend
    </div>
    <div class="aica-button reference-button"
         @click="mainMenuToggle($event)" @touchstart="mainMenuToggle($event)"
         >
      Cancel
    </div>
    <div v-if="isResetting" id="restart-message">
      <p> Please wait for restart...</p>
    </div>
  </div>
</header>
</template>


<script>
export default {
  name: 'FloatingHeaderComponents',
  data: function () {
    return {
      // For testing
      defaultVar: [],
      dropdownActive: false,
      mainMenuActive: false,
      isResetting: false
    }
  },
  mounted () {
    // window.addEventListener("keypress", function(event) {
    // if (!(event.which == 115 && event.ctrlKey) && !(event.which == 19)) return true
    // alert("Ctrl-S pressed")
    // event.preventDefault()
    // return false
    // })
  },
  // beforeDestroy () {
  // },
  props: {
    // Define what lists should be shown
    // Make them connected / computed with showLeft/Right Menu
    showRightIcon: true,
    showLeftIcon: true,
    showLogoIcon: true,
    showDropDown: true,

    // Robot states
    robotIsMoving: false,
    robotIsCompliant: false,

    modules: {
      type: Object,
      default: {}
    },
    drawMode: false
  },
  computed: {
    module_libraries () {
      var libs = Object.keys(this.modules)
      libs = libs.map((lib) => {
        return {'name': lib, 'function_call': 'loadModuleLibrary(' + lib + ')'}
      })
      return libs
    },
    drawMessage () {
      if (this.drawMode) {
        return 'Draw Arrows'
      } else {
        return 'Move Move'
      }
    }
  },
  methods: {
    // Propagate to App
    showLeftMenu (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      console.log('@FloatingHeader TODO: emit sync.')
    },
    showRightMenu (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      console.log('@FloatingHeader TODO: emit sync.')

      // Not ready yet
      // this.$emit('showLibrary')
    },
    handleMouseDown (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      console.log('@FloatingHeader - event')
      console.log(e)
      if (this.dropdownActive) {
        if (!e.target.classList.contains('aica-dropdown-menu')) {
          this.dropdownActive = false
        }
      }
    },
    setRobotStateMoving () {
      this.$emit('setRobotStateMoving')
    },
    // Robot movement states
    clickDropdown (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      this.dropdownActive = true
    },
    compliantModeToggle (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      this.$emit('setRobotStateCompliant', !this.robotIsCompliant)
    },
    stopRobot (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      this.$emit('stopRobot')
    },
    executeSequence (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      this.$emit('runSequence')
      // this.setRobotStateMoving()
    },
    getDrawString (buttonType) {
      var typeString = 'button'
      if ((buttonType === 'draw' && !this.drawMode) || (buttonType === 'move' && this.drawMode)) {
        typeString = typeString + ' is-primary'
      }
      return typeString
    },
    drawModeToggle (newMode) {
      this.$parent.drawModeToggle(newMode)
      // this.drawMode = !(this.drawMode)
    },
    notImplemented () {
      console.log('Not implemented yet.')
    },
    createNew (filename = null) {
      this.$buefy.dialog.confirm({
        title: `Opening New File`,
        message: `You are about to create a new file without saving your changes.`,
        cancelText: `Cancel`,
        confirmText: `Confirm`,
        type: 'is-success',
        onConfirm: () => {
          this.$buefy.toast.open('User does want to continue!')
        }
      })
    },
    saveBlocks (e, filename = null) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      console.log('Sent <saving> to parent')
      this.$parent.saveScene(filename)
      this.dropdownActive = false
    },
    loadBlocks (e, filename = null) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      // console.log('Sent <loading> to parent')
      this.$parent.loadScene(filename)
      this.dropdownActive = false
    },
    loadModuleLibrary (library) {
      // console.log('Loading Module Librar: <<' + library + '>>')
      this.$parent.loadModuleLibrary(library)
      this.dropdownActive = false
    },
    mainMenuToggle (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      this.mainMenuActive = !(this.mainMenuActive)
      console.log('Showing Main Menu')
      console.log(this.mainMenuActive)
    },
    resetBackendAll (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      this.isResetting = true
      this.$emit('resetBackendAll')
    },
    goHome () {
      console.log('Implement Home.')
    },
    quit () {
      console.log('Implement quit.')
      // this.$parent.loadModuleLibrary(library)
    },
    closeWindow () {
      if (confirm('Close Window?')) {
        close()
      }
    }
  },
  watch: {
    dropdownActive (value) {
      if (value) {
        // initializeDropdownListener (e) {
        document.documentElement.addEventListener('mousedown', this.handleMouseDown, true)
        document.documentElement.addEventListener('touchstart', this.handleMouseDown, true)
        // },
      } else {
        // destroyDropdownListener (e) {
        document.documentElement.removeEventListener('mousedown', this.handleMouseDown, true)
        document.documentElement.removeEventListener('touchstart', this.handleMouseDown, true)
      // },
      }
    }
  }
}
</script>


<style lang="less" scoped>
@import './../assets/styles/main.less';

.floating-header{
    height: @header-height;
    width: 100%;

    background: transparent;

    position: absolute;
    top: 0;
    left: 0;

    padding-top: @header-height*0.2;
    padding-bottom: @header-height*0.1;
    padding-left: @header-padding-sideways;
    padding-right: @header-padding-sideways;

    > div {
        position: absolute;
    }

    #drop-down {
        z-index: 4;
        left: ~"calc(@{height-icon-logo} +  2.0*@{height-icon-small})";

        > ul {
            cursor: pointer;

            font-size: @fontsize-large;

            margin: 0;
            padding: $font-size*0.4 $font-size*1.0;
            width: @sidebar-width*0.6;

            background-color: @color-main-dark;

            border-color: @color-border;
            border-style: solid;
            border-color: @color-main-mediumbright;
            border-width: 1px;

            &.dropdown-head {
                border-style: none;
                background-color: transparent;
            }
        }
    }
}

#aica-title {
    font-size: @fontsize-large*2.0;

    position: relative;
    top: @fontsize-large*5.5;
}

#compliant-button {
    font-size: @fontsize-large;
    border-radius: $font-size;
    margin-top: $font-size*0.8;
    padding-top: $font-size*0.2;

    width: @sidebar-width*0.6;

    z-index: 4;
    left: ~"calc(50*@{vw} - @{sidebar-width}*1.0)";
    top: $font-size*0.3;
}

#run-sequence {
    font-size: @fontsize-large;
    border-radius: $font-size;
    margin-top: $font-size*0.8;
    padding-top: $font-size*0.2;

    width: @sidebar-width*0.6;

    z-index: 4;
    left: ~"calc(50*@{vw} - @{sidebar-width}*0.2)";
    top: $font-size*0.3;
}

#home-logo {
    z-index: 4;
    width: @height-icon-small;
    height: @height-icon-small;
    // left: ~"calc(50% - @{height-icon-small}/2.0)";
    left: @height-icon-logo * 1.5;
    top: @header-height*0.3;
    cursor: pointer;
    z-level: 5;
}

.main-menu-page{
    position: absolute;
    left: 0;

    width: 100vw;
    height: 100vw;

    z-index: 10;

    background-color: @color-main-dark;

    .aica-button {
        font-size: @fontsize-huge;
        margin-top: $font-size * 0.5;
        margin-bottom: $font-size * 0.5;

        padding-top: $font-size * 0.4;
        padding-bottom: $font-size * 0.4;
        width: @sidebar-width * 0.9;

        text-align: center;

        position: relative;
        // top: 10*@vw;
        top: 10*@vw;
    }

}
.logo-container{
    text-align: center;
}

.logo-large {
    height: @sidebar-width * 0.4;

    position: relative;
    top: 8*@vw;
}

#restart-message {
    text-align: center;
    position: relative;
    top: 10*@vw;
    font-size: @fontsize-small;
}
</style>
