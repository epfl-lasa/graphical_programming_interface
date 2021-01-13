<template>
  <!-- <div v-if="IS_DEBUGGING"> -->
  <!--   <LoadSave -->
  <!--     ref="loadsave" -->
  <!--     class="loadsave" -->
  <!--     /> -->
  <!--   <\!-- :localFiles="localFiles" -\-> -->
  <!-- </div> -->
  <div id="app">
    <Header
      ref="header"
      class="header"
      :modules="modules"/>

    <template v-if="true">
    <!-- <template v-if="(appMode==='main' || appMode==='programming')"> -->
      <!-- TODO: create drawing sub-app ? -->
      <VueBlocksContainer
        @contextmenu.native="showContextMenu"
        @click.native="closeContextMenu"
        ref="container"
        :blocksContent="blocks"
        :scene.sync="scene"
        @blockSelect="selectBlock"
        @blockDeselect="deselectBlock"
        class="container"/>

      <template v-if="selectedBlock">
        <VueBlockProperty
          :module="selectedModule"
          :property="selectedBlockProperty"
          ref="property"
          @save="saveProperty"/>

      </template>
      <template v-else-if="loadedLibrary">
        <!-- <template v-if="0"> -->
        <VueModuleLibrary ref="module-library"
                          class="module-library"
                          :modules="modules"
                          :loadedLibrary="loadedLibrary"/>
      </template>
    </template>

      <!-- <template v-if="false"> -->
    <!-- <template v-if="true"> -->
    <template v-if="loadSaveMode">
      <LoadSave
        ref="loadsave"
        class="loadsave"
        :localFiles="localFiles"
        :appMode="appMode"
        />
        <p> Load-Safe </p>
    </template>

  </div>
</template>

<script>
import merge from 'deepmerge'

import VueBlocksContainer from './components/VueBlocksContainer'
import VueBlockProperty from './components/VueBlockProperty'
import domHelper from './helpers/dom'

import Header from './components/Header'
import LoadSave from './components/LoadSave'
import VueModuleLibrary from './components/VueModuleLibrary'

import axios from 'axios' // Needed to pass. Only temporarily?

// import {blocks, links} from '../../backend/userdata/default_data.json'

export default {
  name: 'App',
  components: {
    Header,
    LoadSave,
    VueBlocksContainer,
    VueBlockProperty,
    VueModuleLibrary
  },
  mounted () {
    this.loadLibraries()

    // For Debugging & Development
    this.loadScene('default')
    this.loadedLibrary = 'basic'

    setTimeout(() => {
      this.$refs.container.blockSelect(this.$refs.container.scene.blocks[0])
    }, 300)
    // When loading finished, press default
  },
  data: function () {
    return {
      // IS_DEBUGGING: true,
      appMode: 'main',
      loadSaveMode: false,
      // appMode: 'load',
      // loadSaveMode: true,
      blocks: [],
      scene: {
        blocks: [],
        links: [],
        container: {
          centerX: 1042,
          centerY: 140,
          scale: 1
        }
      },
      selectedBlock: null,
      selectedType: 'delay',
      useContextMenu: true,
      contextMenu: {
        isShow: false,
        mouseX: 0,
        mouseY: 0,
        top: 0,
        left: 0
      },
      modules: {},
      // loadedLibrary: null
      loadedLibrary: null,
      // File list of the local library.
      localFiles: []
    }
  },
  computed: {
    selectedModule () {
      if (!this.selectedBlock || !this.selectedBlock.values || !this.selectedBlock.values.property) {
        return null
      }
      return this.selectedBlock
    },
    selectedBlockProperty () {
      if (!this.selectedBlock || !this.selectedBlock.values || !this.selectedBlock.values.property) {
        return null
      }
      return this.selectedBlock.values.property
    },
    selectBlocksType () {
      return this.blocks.map(b => {
        return b.family
      }).filter((value, index, array) => {
        return array.indexOf(value) === index
      })
    }
  },
  methods: {
    loadLibraries () {
      console.log('Loading libraries.')
      axios.get(this.$localIP + `/getlibrariesandmodules`, {'params': {}})
        .then(response => {
          this.modules = response.data.moduleLibraries
          this.blocks = response.data.blockContent
        })
        .catch(error => {
          console.log(error)
        })
    },
    saveScene (filename = null) {
      console.log('Saving to save')
      console.log(filename)
      if (!(filename)) {
        filename = 'default'
        axios.get(this.$localIP + `/getfilelist`, {'params': {}})
          .then(response => {
            this.localFiles = response.data.localfiles
            this.loadSaveMode = true
            this.appMode = 'save'
            console.log('OK file load')
          })
          .catch(error => {
            console.log(error)
          })
      } else {
        var goal = this.$localIP + `/savetofile/` + filename

        axios.get(goal,
                  {'params': {'scene': this.scene, 'blockContent': JSON.stringify(this.blocks)}})
          .then(response => {
            console.log(response.statusText)
          })
          .catch(error => {
            console.log(error)
          })
        console.log('Successfully saved blocks to <<' + filename + '>>')
      }
    },
    loadScene (filename = null) {
      console.log('Loading')
      console.log(filename)
      if (filename === null) {
        axios.get(this.$localIP + `/getfilelist`, {'params': {}})
          .then(response => {
            this.localFiles = response.data.localfiles
            this.loadSaveMode = true
            this.appMode = 'load'
            console.log('OK file load')
          })
          .catch(error => {
            console.log(error)
          })
      } else {
        axios.get(this.$localIP + `/loadfromfile/` + filename)
          .then(response => {
            console.log(response.statusText)
            this.scene = response.data.scene
            if (this.blocks.length === 0) {
              this.blocks = response.data.blockContent
              console.log('Load block library')
            } else {
              console.log('Use default block library')
            }
            console.log('Load succesfull from file <<' + filename + '>>')
          })
          .catch(error => {
            console.log(error)
          })
      }
    },
    newScene () {
      // Delete and create new scene
      this.blocks = []
      this.scene = {
        blocks: [],
        links: [],
        container: {centerX: 1042, centerY: 140, scale: 1}
      }
    },
    selectBlock (block) {
      this.selectedBlock = block
    },
    deselectBlock (block) {
      console.log('deselect', block)
      this.selectedBlock = null
    },
    filteredBlocks (type) {
      return this.blocks.filter(value => {
        return value.family === type
      })
    },
    addBlock () {
      console.log(this.selectedType)
      this.$refs.container.addNewBlock(this.selectedType)
    },
    addModule (module) {
      console.log('Add module <<' + module + '>>')
      this.$refs.container.addNewBlock(module)
    },
    loadModuleLibrary (library) {
      this.loadedLibrary = library
    },
    saveProperty (val) {
      console.log('Save', val)

      let scene = this.scene
      let block = scene.blocks.find(b => {
        return b.id === this.selectedBlock.id
      })
      block.values.property = val

      this.scene = merge({}, scene)
    },
    showContextMenu (e) {
      // TODO remove from main app since list not existant anymore
      if (!this.useContextMenu) return
      if (e.preventDefault) e.preventDefault()

      this.contextMenu.isShow = true
      this.contextMenu.mouseX = e.x
      this.contextMenu.mouseY = e.y

      this.$nextTick(function () {
        this.setMenu(e.y, e.x)
        this.$refs.contextMenu.focus()
      })
    },
    setMenu (top, left) {
      // TODO remove from main app since list not existant anymore
      let border = 5
      let contextMenuEl = this.$refs.contextMenu
      let containerElRect = this.$refs.container.$el.getBoundingClientRect()
      let largestWidth = containerElRect.right - contextMenuEl.offsetWidth - border
      let largestHeight = containerElRect.bottom - contextMenuEl.offsetHeight - border

      console.log(this.$refs.container)
      console.log(containerElRect)

      if (left > largestWidth) left = largestWidth
      if (top > largestHeight) top = largestHeight

      this.contextMenu.top = top
      this.contextMenu.left = left
    },
    addBlockContextMenu (name) {
      let offset = domHelper.getOffsetRect(this.$refs.container.$el)
      let x = this.contextMenu.mouseX - offset.left
      let y = this.contextMenu.mouseY - offset.top

      this.$refs.container.addNewBlock(name, x, y)
      this.closeContextMenu()
    },
    closeContextMenu () {
      this.contextMenu.isShow = false
    }
  },
  watch: {
    blocks (newValue) {
      console.log(`Update blocks`)
      // console.log(`Imported blocks succesfully.`)
      // console.log('blocks', JSON.stringify(newValue))
    },
    scene (newValue) {
      console.log('Update scene')
      // console.log('scene', JSON.stringify(newValue))
    }
  }
}
</script>

<style >
html, body {
  margin: 0;
  padding: 0;
}
</style>

<!-- <style lang="scss"> -->
<!-- #app { -->
<!--   font-family: "Avenir", Helvetica, Arial, sans-serif; -->
<!--   -webkit-font-smoothing: antialiased; -->
<!--   -moz-osx-font-smoothing: grayscale; -->
<!--   text-align: center; -->
<!--   //this is where we use the variable -->
<!--   color: $primary; -->
<!--   margin-top: 60px; -->
<!-- } -->
<!-- </style> -->
<!-- <style lang="scss"> -->
<!--   html, body { -->
<!--     margin: 0; -->
<!--     padding: 0; -->
<!--   } -->
  <!-- </style> -->

<style lang="less">
  html, body {
    margin: 0;
    padding: 0;
  }

  html {
    width: 100vw;
    height: 100vh;
  }

  body {
    width: 100%;
    height: 100%;
    background-color: #0e1624;
  }

  h1 {
      fontsize: 20px;
  }

  #app {
      width: 100%;
      height: 100%;
      // width: ~"calc(100% - 00px)";
      // height: ~"calc(100% - 60px)";
      padding: 0px 0 0 0px;
  }

  .container {
      width: 100%;
      height: 95%;
      // height: ~"calc(100% - 50px)";
  }

  #contextMenu {
    // TODO remove from main app
      position: absolute;
    z-index: 1000;
    background: white;
    border: 1px solid black;
    padding: 5px;
    margin: 0;

    li {
      &.label{ s
        color: gray;
        font-size: 90%;
      }
      list-style: none;
    }

    &:focus {
      outline: none;
    }
  }

</style>
