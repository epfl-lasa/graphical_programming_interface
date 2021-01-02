<template>
  <div id="app">
    <Header
      ref="header"
      class="header"
      :modules="modules"/>

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
  </div>
</template>

<script>
import merge from 'deepmerge'

import VueBlocksContainer from './components/VueBlocksContainer'
import VueBlockProperty from './components/VueBlockProperty'
import domHelper from './helpers/dom'

import Header from './components/Header'
import VueModuleLibrary from './components/VueModuleLibrary'

import axios from 'axios' // Needed to pass. Only temporarily?

// import {blocks, links} from '../../backend/userdata/default_data.json'

export default {
  name: 'App',
  components: {
    Header,
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
      // console.log('Wait to be sure to load the whole scene.')
      // console.log(this.$refs.container.scene.blocks[1])
      this.$refs.container.blockSelect(this.$refs.container.scene.blocks[0])
      // this.$refs.container.blockSelect(this.$refs.container.scene.blocks[0])
      // this.$refs.container.blockSelect(this.$refs.container.scene.blocks[0])
    }, 300)
    // When loading finished, press default
  },
  data: function () {
    return {
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
      loadedLibrary: null
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
      axios.get(`http://localhost:5000/getlibrariesandmodules`, {'params': {}})
        .then(response => {
          // console.log(response.data)
          // console.log('libaries')
          this.modules = response.data.moduleLibraries
          this.blocks = response.data.blockContent
          // console.log(Object.keys(this.modules))
          // console.log('SUCCESS')
        })
        .catch(error => {
          console.log(error)
          // console.log('ERORIUS')
        })
    },
    saveScene (filename = null) {
      if (!filename) {
        filename = 'default'
      }
      // console.log('Sending blocks to database')
      var goal = `http://localhost:5000/savetofile/` + filename

      // console.log('blocks')
      // console.log(this.scene)
      // console.log('blocks', JSON.stringify(this.blocks))
      axios.get(goal,
                {'params': {'scene': this.scene, 'blockContent': JSON.stringify(this.blocks)}})
        .then(response => {
          console.log(response.statusText)
        })
        .catch(error => {
          console.log(error)
        })
      console.log('Successfully saved blocks to <<' + filename + '>>')
    },
    loadScene (filename = null) {
      if (!filename) {
        filename = 'default'
      }
      // console.log('Loading blocks from file')
      axios.get(`http://localhost:5000/loadfromfile/` + filename)
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
      // console.log('select', block)
      this.selectedBlock = block
    },
    deselectBlock (block) {
      // Save Automatically at exit
      // console.log('Save properties')
      // this.$refs.property.save()

      // console.log('deselect', block.properties)
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
      console.log(`Imported blocks succesfully.`)
      // console.log('blocks', JSON.stringify(newValue))
    },
    scene (newValue) {
      // console.log('update scene')
      console.log('scene', JSON.stringify(newValue))
    }
  }
}
</script>

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

  #app {
    width: ~"calc(100% - 00px)";
    height: ~"calc(100% - 60px)";
    padding: 0px 0 0 0px;
  }

  .container {
    width: 100%;
    height: ~"calc(100% - 50px)";
    border: 1px solid black;
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
      &.label {
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
