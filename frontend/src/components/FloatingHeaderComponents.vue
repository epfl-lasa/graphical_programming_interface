<template>
<header>
  <div class="floating-header">
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
      <ul class="aica-dropdown-menu" v-if="dropdownActive"
          @click="createNew($event)" @touchstart="createNew($event)"> Create New </ul>
    </div>

    <div id="home-logo">
      <img src='./../assets/logotypes/aica_small_closed_white.svg'>
    </div>

    <div v-if="robotIsMoving"  class="aica-button danger" id="run-sequence"
         @click="stopRobot($event)" @touchstart="stopRobot($event)" >
      <p> Stop Moving </p>
    </div>
    <div v-else  class="aica-button" id="run-sequence"
         @click="executeSequence($event)" @touchstart="executeSequence($event)">
      <p> Run All </p>
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
      dropdownActive: false
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

    robotIsMoving: false,
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

    padding-top: @header-height*0.1;
    padding-bottom: @header-height*0.1;
    padding-left: @header-padding-sideways;
    padding-right: @header-padding-sideways;

    > div {
        position: absolute;
    }

    #drop-down {
        z-index: 4;
        left: ~"calc(50% - @{height-icon-small}/2.0 - 3.0*@{height-icon-small})";

        > ul {
            cursor: pointer;

            font-size: 15pt;
            padding-top: @header-height*0.1;
            padding-bottom: @header-height*0.05;
            padding-left:@header-height*0.3;
            padding-right:@header-height*0.7;
            border-color: @color-border;
            background-color: @color-main-dark;
            border-style: solid;
            border-color: @color-main-medium;
            border-width: 1px;


            &.dropdown-head {
                border-style: none;
                background-color: transparent;
            }
        }
    }

    #home-logo {
        z-index: 4;
        width: @height-icon-small;
        height: @height-icon-small;
        left: ~"calc(50% - @{height-icon-small}/2.0)";

        cursor: pointer;
        z-level: 5;

    }

    #run-sequence {
        z-index: 4;
        left: ~"calc(50% + @{height-icon-small}/2.0 + @{height-icon-small}*2)";
        top: @height-icon-small * 0.3;
    }
}
</style>
