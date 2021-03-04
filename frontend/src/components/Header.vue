x<template>
<header class="header">
  <b-navbar
    type="dark" variant="dark" class="is-dark navigation-bar-top">
    <!-- :mobile-burger="false"> -->

  <!-- <b-navbar type="dark" variant="dark"> -->
    <template #brand>
      <!-- Insert logo here -->
      <b-navbar-item tag="router-link" :to="{ path: '/' }">
        <!-- TODO: use local image! -->
        <img
          src="https://images.squarespace-cdn.com/content/5f884bf8a4c7577ab9f18610/1603379949940-QGFEOUC3Z5YA4W4ELK4W/AICA+Original+White.png?content-type=image%2Fpng"
          alt="AICA Programming Interface"
          v-on:click="goHome()"
          >
      </b-navbar-item>
    </template>

    <template #start>
      <!-- <b-navbar-item v-on:click="goHome()" href="#"> -->
        <!-- Home -->
      <!-- </b-navbar-item> -->
      <b-navbar-dropdown label="File">
        <b-navbar-item v-on:click="createNew()" href="#">
          Create New
        </b-navbar-item>
        <b-navbar-item v-on:click="saveBlocks()"xo href="#">
          Save to File
        </b-navbar-item>
        <b-navbar-item v-on:click="loadBlocks()" href="#">
          Load from File
        </b-navbar-item>
      </b-navbar-dropdown>
    </template>

    <template #end>
      <!-- <b-navbar-item v-on:click="close_window()" href="#"> -->
          <!-- Close All -->
      <!-- </b-navbar-item> -->
      <b-navbar-item tag="div" v-if="false">
        <label> <small> Mode:</small> </label>
        <div class="button toggle">
          <a :class="getDrawString('move')"  v-on:click="drawModeToggle(true)" href="#">
            Move
          </a>
          <a :class="getDrawString('draw')"  v-on:click="drawModeToggle(false)" href="#">
            Draw Arrows
          </a>
        </div>
      </b-navbar-item>

      <b-navbar-dropdown label="Library">
        <b-navbar-item
          v-for="lib in module_libraries"
          v-on:click="loadModuleLibrary(lib.name)"
          v-bind:key="lib.name"
          href="#">
          {{ lib.name }}
        </b-navbar-item>
      </b-navbar-dropdown>

      <b-navbar-item tag="div">
        <!-- <a class="button is-primary" :on-click="update_program"> -->
            <!-- <strong> Update </strong> -->
          <!-- </a> -->
          <b-button v-if="robotIsMoving" v-on:click="stopRobot" type="is-primary"> Stop Robot </b-button>
          <b-button v-else v-on:click="executeSequence" type="is-primary"> Run Sequence </b-button>

      </b-navbar-item>
    </template>
  </b-navbar>
</header>
</template>



<script>
import axios from 'axios' // Needed to pass. Only temporarily? -- Import global?

// import axios from 'axios' // Needed to pass. Only temporarily?
export default {
  name: 'Header',
  data: function () {
    return {
      // For testing
      defaultVar: []
    }
  },
  // created () {
  // this.$parent.update_libraries()
  // },
  props: {
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
    // Robot movement states
    setRobotStateMoving () {
      this.$emit('setRobotStateMoving')
    },
    stopRobot () {
      this.$emit('stopRobot')
    },
    executeSequence () {
      this.setRobotStateMoving()
      axios.get(this.$localIP + `/executesequence`)
        .then(response => {
          console.log(response.statusText)
        })
        .catch(error => {
          console.log(error)
        })
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
    saveBlocks (filename = null) {
      console.log('Sent <saving> to parent')
      this.$parent.saveScene(filename)
    },
    loadBlocks (filename = null) {
      // console.log('Sent <loading> to parent')
      this.$parent.loadScene(filename)
    },
    loadModuleLibrary (library) {
      // console.log('Loading Module Librar: <<' + library + '>>')
      this.$parent.loadModuleLibrary(library)
    },
    goHome () {
      console.log('Implement Home.')
    },
    quit () {
      console.log('Implement quit.')
      // this.$parent.loadModuleLibrary(library)
    },
    close_window () {
      if (confirm('Close Window?')) {
        close()
      }
    }
  }
}
</script>


<style lang="less" scoped>
@import './../assets/styles/main.less';

.header {
    font-size: 30px;
    font-color: white;
    height: @header-height;

}

.navigation-bar-top {
    padding-left: 10px;
    padding-right: 10px;
}

b-navbar-dropdown {
    font-size: large;

    b-navbar-item {
        font-size: 30px;
    }
}
</style>
<!--     background: #333; -->
<!--     color: #fff; -->
<!--     text-align: center; -->
<!--     padding: 10px; -->
<!--   } -->
<!--   .header a { -->
<!--     color: #fff; -->
<!--     padding-right: 5px; -->
<!--     text-decoration: none; -->
<!--   } -->
