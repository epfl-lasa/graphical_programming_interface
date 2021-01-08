<template>
  <header class="header">
    <b-navbar type="dark" variant="dark">
      <b-navbar-nav>
        <b-nav-item v-on:click="goHome()" href="#">Home</b-nav-item>

        <!-- Navbar dropdowns -->
        <b-nav-item-dropdown text="Library" right>
          <b-dropdown-item v-for="lib in module_libraries"
                           v-on:click="loadModuleLibrary(lib.name)"
                           v-bind:key="lib.name">{{ lib.name }}</b-dropdown-item>
        </b-nav-item-dropdown>

        <b-nav-item-dropdown text="File" left>
          <b-dropdown-item v-on:click="createNew()" href="#">Create New</b-dropdown-item>
          <b-dropdown-item v-on:click="saveBlocks()" href="#">Save to File</b-dropdown-item>
          <b-dropdown-item v-on:click="printClicked()" href="#">Load from File</b-dropdown-item>
        </b-nav-item-dropdown>

        <b-nav-item v-on:click="quit()">Quit</b-nav-item>

        <!-- TODO remove after debugging -->
        <b-nav-item v-on:click="saveBlocks('default')" href="#">Save(Default)</b-nav-item>
        <b-nav-item v-on:click="loadBlocks('default')" href="#">Load(Default)</b-nav-item>

      </b-navbar-nav>
    </b-navbar>
    <!-- <div id="nav"> -->
      <!-- <router-link to="/">Home</router-link> | -->
      <!-- <router-link to="/about">About</router-link> -->
    <!-- </div> -->
  </header>
</template>

<script>
// import axios from 'axios' // Needed to pass. Only temporarily?
export default {
  name: 'Header',
  data: function () {
    return {
      // For testing
      posts: [
        {id: 1, title: 'One'},
        {id: 2, title: 'Two'},
        {id: 3, title: 'Three'}
      ]
    }
  },
  // created () {
  // this.$parent.update_libraries()
  // },
  props: {
    modules: {
      type: Object,
      default: {}
    }
  },
  computed: {
    module_libraries () {
      var libs = Object.keys(this.modules)
      libs = libs.map((lib) => {
        return {'name': lib, 'function_call': 'loadModuleLibrary(' + lib + ')'}
      })
      return libs
    }
  },
  methods: {
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
      // console.log('Sent <saving> to parent')
      this.$parent.saveScene(filename)
    },
    loadBlocks (filename = null) {
      console.log('Sent <loading> to parent')
      this.$parent.loadScene(filename)
    },
    loadModuleLibrary (library) {
      console.log('Loading Module Librar: <<' + library + '>>')
      this.$parent.loadModuleLibrary(library)
    },
    goHome () {
      console.log('Implement Home.')
    },
    quit () {
      console.log('Implement quit.')
      // this.$parent.loadModuleLibrary(library)
    }
  }
}
</script>

<style scoped>
.header {
  font-size: 20px;
  font-color: white;
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
