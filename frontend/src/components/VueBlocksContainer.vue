<!--
Nomenclature Introduction

# Application

# Task
Task is

# Action
An action is the most basic module that can exist.

# Module
A module is

# Block
Block is the visual element created for a 'module'

# Link
Links two different modules

# Line
Line is the visual element which is created from a link between modules
  -->

<template>
<div class="vue-container" id="main-element-container">

  <!-- TODO: Iterate over links, too! -->
  <VueLink :lines="lines"
           :linkingMode = "linkingMode"
           @showDropdownkMenu="showDropdownkMenu"
           @disableDropdownMenu="disableDropdownkMenu"
           @updateSelectedLine="updateSelectedLine"
           @removeLink="removeLink"
           />

  <VueBlock v-for="block in blocks"
            :key="block.id"
            :linkingMode="linkingMode"
            v-bind.sync="block"
            :options="optionsForChild"
            @update="updateScene"
            @linkingStart="linkingStart(block, $event)"
            @linkingStop="linkingStop(block, $event)"
            @linkingBreak="linkingBreak(block, $event)"
            @showBlockMenu="showBlockMenu(block, $event)"
            @disableBlockMenu="disableBlockMenu()"
            @select="blockSelect(block)"
            @delete="blockDelete(block)"
            @linkingStopDrawing = "linkingStopDrawing"
            />

  <DropDown v-if="blockMenuVisible"
            :linkingMode="linkingMode"
            :posX="blockMenuX"
            :posY="blockMenuY"
            :selectionType="blockSelectionType"
            @menuContent="blockMenuContent"
            @disableBlockMenu="disableBlockMenu()"
            @removeBlock="blockDelete"
            @linkingStart="linkingStart()"
            @removeLink="removeLink"
            />
</div>
</template>

<script>
import merge from 'deepmerge'
import mouseHelper from '../helpers/mouse'

import VueBlock from './VueBlock'
import VueLink from './LinkCreator' // TODO: rename file to linkCreator
import DropDown from './DropDown'

export default {
  name: 'VueBlockContainer',
  components: {
    VueBlock,
    VueLink,
    DropDown
  },
  props: {
    blocksContent: {
      type: Array,
      default () {
        return []
      }
    },
    scene: {
      type: Object,
      default: {
        blocks: [],
        links: [],
        container: {}
      }
    },
    options: {
      type: Object
    }
  },
  created () {
    this.mouseX = 0
    this.mouseY = 0

    this.lastMouseX = 0
    this.lastMouseY = 0

    this.minScale = 0.2
    this.maxScale = 5

    // this.linking = false
    this.linkStartData = null

    this.inputSlotClassName = 'inputSlot'

    this.defaultScene = {
      blocks: [],
      links: [],
      container: {}
    }
  },
  mounted () {
    document.documentElement.addEventListener('mousemove', this.handleMove, true)
    document.documentElement.addEventListener('mousedown', this.handleDown, true)
    document.documentElement.addEventListener('mouseup', this.handleUp, true)
    document.documentElement.addEventListener('wheel', this.handleWheel, true)

    this.centerX = this.$el.clientWidth / 2
    this.centerY = this.$el.clientHeight / 2

    this.importBlocksContent()
    this.importScene()
  },
  beforeDestroy () {
    document.documentElement.removeEventListener('mousemove', this.handleMove, true)
    document.documentElement.removeEventListener('mousedown', this.handleDown, true)
    document.documentElement.removeEventListener('mouseup', this.handleUp, true)
    document.documentElement.removeEventListener('wheel', this.handleWheel, true)
  },
  data () {
    return {
      dragging: false,
      //
      centerX: 0,
      centerY: 0,
      scale: 1,
      //
      nodes: [],
      blocks: [],
      links: [],
      //
      tempLink: null,
      selectedLink: null,
      selectedBlock: null,
      hasDragged: false,
      //
      blockMenuVisible: false,
      blockMenuContent: [],
      blockMenuX: 0,
      blockMenuY: 0,
      blockSelectionType: null,
      // Check if everything is saved & updated
      isSavedToFile: true,
      // isUpdatedToProgram: true
      loopIsClosed: false
    }
  },
  computed: {
    linkingMode () {
      // console.log('Double valuation for debugging')
      return (this.tempLink !== null)
    },
    optionsForChild () {
      return {
        width: 110,
        height: 110,
        titleHeight: 20,
        scale: this.scale,
        inputSlotClassName: this.inputSlotClassName,
        center: {
          x: this.centerX,
          y: this.centerY
        }
      }
    },
    container () {
      return {
        centerX: this.centerX,
        centerY: this.centerY,
        scale: this.scale
      }
    },
    // Links calculate
    // TODO: what is difference between lines & links (?!?!)
    lines () {
      let lines = []

      for (let link of this.links) {
        let originBlock = this.blocks.find(block => {
          return block.id === link.originID
        })

        let targetBlock = this.blocks.find(block => {
          return block.id === link.targetID
        })

        if (!originBlock || !targetBlock) {
          console.log('@VueBlockContainer: Remove invalid link', link)
          this.removeLink(link.id)
          continue
        }

        if (originBlock.id === targetBlock.id) {
          console.log('Loop detected, remove link', link)
          this.removeLink(link.id)
          continue
        }

        let originLinkPos = this.getConnectionPos(originBlock, link.originSlot, false)
        let targetLinkPos = this.getConnectionPos(targetBlock, link.targetSlot, true)

        if (!originLinkPos || !targetLinkPos) {
          console.log('@VueBlockContainer: Remove invalid link (slot not exist)', link)
          this.removeLink(link.id)
          continue
        }

        let x1 = originLinkPos.x
        let y1 = originLinkPos.y

        let x2 = targetLinkPos.x
        let y2 = targetLinkPos.y

        lines.push({
          id: link.id,
          x1: x1,
          y1: y1,
          x2: x2,
          y2: y2,
          style: {
            stroke: '#32D9CB',
            // stroke: '#F85',
            strokeWidth: 6 * this.scale,
            fill: 'none'
          },
          outlineStyle: {
            stroke: '#666',
            strokeWidth: 6 * this.scale,
            strokeOpacity: 0.6,
            fill: 'none'
          }
        })
      }

      if (this.tempLink) {
        this.tempLink.style = {
          stroke: '#8f8f8f',
          strokeWidth: 4 * this.scale,
          fill: 'none'
        }

        // Temp Link Is Added
        lines.push(this.tempLink)
      }

      return lines
    }
  },
  methods: {
    // Events
    /** @param e {MouseEvent} */
    handleMove (e) {
      let mouse = mouseHelper.getMousePosition(this.$el, e)
      this.mouseX = mouse.x
      this.mouseY = mouse.y

      if (this.dragging) {
        let diffX = this.mouseX - this.lastMouseX
        let diffY = this.mouseY - this.lastMouseY

        this.lastMouseX = this.mouseX
        this.lastMouseY = this.mouseY

        this.centerX += diffX
        this.centerY += diffY

        this.hasDragged = true
      }

      if (this.linkingMode && this.linkStartData) {
        let linkStartPos = this.getConnectionPos(this.linkStartData.block, this.linkStartData.slotNumber, false)
        this.tempLink = {
          x1: linkStartPos.x,
          y1: linkStartPos.y,
          x2: this.mouseX,
          y2: this.mouseY
        }
      }
    },
    handleDown (e) {
      const target = e.target || e.srcElement
      if ((target === this.$el || target.matches('svg, svg *')) && e.which === 1) {
        this.dragging = true

        let mouse = mouseHelper.getMousePosition(this.$el, e)
        this.mouseX = mouse.x
        this.mouseY = mouse.y

        this.lastMouseX = this.mouseX
        this.lastMouseY = this.mouseY

        this.deselectAll()
        if (e.preventDefault) e.preventDefault()
      }

      // Check if any block is clicked
      let clickElementInd = this.$children.filter(item => item.$el.className === 'vue-block')
          .find(item => item.$el.contains(target))
      if (clickElementInd === undefined && this.linkingMode) {
        this.linkingStopDrawing()
      }
    },
    handleUp (e) {
      const target = e.target || e.srcElement

      if (this.dragging) {
        this.dragging = false

        if (this.hasDragged) {
          this.updateScene()
          this.hasDragged = false
        }
      }
      if (this.$el.contains(target) && (typeof target.className !== 'string' || target.className.indexOf(this.inputSlotClassName) === -1)) {
        // this.linking = false
        // this.tempLink = null
        // this.linkStartData = null
        console.log('@VueBlocksContainer: debugin without link adapting')
      }
    },
    handleWheel (e) {
      const target = e.target || e.srcElement
      if (this.$el.contains(target)) {
        if (e.preventDefault) e.preventDefault()

        let deltaScale = Math.pow(1.1, e.deltaY * -0.01)
        this.scale *= deltaScale

        if (this.scale < this.minScale) {
          this.scale = this.minScale
          return
        } else if (this.scale > this.maxScale) {
          this.scale = this.maxScale
          return
        }

        let zoomingCenter = {
          x: this.mouseX,
          y: this.mouseY
        }

        let deltaOffsetX = (zoomingCenter.x - this.centerX) * (deltaScale - 1)
        let deltaOffsetY = (zoomingCenter.y - this.centerY) * (deltaScale - 1)

        this.centerX -= deltaOffsetX
        this.centerY -= deltaOffsetY

        this.updateScene()
      }
    },
    // Processing
    getConnectionPos (block, slotNumber, isInput, fromCenter = true) {
      if (!block || slotNumber === -1) {
        return undefined
      }

      let x = 0
      let y = 0

      x += block.x
      y += block.y

      if (fromCenter) {
        // Draw the arrows starting from the center
        x += this.optionsForChild.width / 2.0
        y += this.optionsForChild.width / 2.0
      } else {
        y += this.optionsForChild.titleHeight

        if (isInput && block.inputs.length > slotNumber) {
        } else if (!isInput && block.outputs.length > slotNumber) {
          x += this.optionsForChild.width
        } else {
          console.error('slot ' + slotNumber + ' not found, is input: ' + isInput, block)
          return undefined
        }

        // (height / 2 + blockBorder + padding)
        y += (16 / 2 + 1 + 2)
        //  + (height * slotNumber)
        y += (16 * slotNumber)
      }

      x *= this.scale
      y *= this.scale

      x += this.centerX
      y += this.centerY

      return {x: x, y: y}
    },
    // Linking
    linkingStart (block = null, slotNumber = 0) {
      // Choose default block
      if (block === null) {
        block = this.selectedBlock
      }

      this.linkStartData = {block: block, slotNumber: slotNumber}
      let linkStartPos = this.getConnectionPos(this.linkStartData.block, this.linkStartData.slotNumber, false)
      this.tempLink = {
        x1: linkStartPos.x,
        y1: linkStartPos.y,
        x2: this.mouseX,
        y2: this.mouseY
      }

      console.log('@VueBlocksContainer')
      console.log(this.linkingMode)
    },
    linkingStop (targetBlock, slotNumber = 0) {
      // Successful linking - Make temp link to permanent link
      if (this.linkStartData && targetBlock && slotNumber > -1) {
        this.links = this.links.filter(value => {
          return !(value.targetID === targetBlock.id && value.targetSlot === slotNumber)
        })

        let maxID = Math.max(0, ...this.links.map(function (o) {
          return o.id
        }))

        // skip if looping
        if (this.linkStartData.block.id !== targetBlock.id) {
          this.links.push({
            id: maxID + 1,
            originID: this.linkStartData.block.id,
            originSlot: this.linkStartData.slotNumber,
            targetID: targetBlock.id,
            targetSlot: slotNumber
          })
          this.updateScene()
        }
      }
      this.linkingStopDrawing()
    },
    linkingStopDrawing () {
      // Stop linking Everywhere
      this.linkStartData = null
      this.tempLink = null
      // this.blocks.forEach(function (item, index) {
      // item.linking = false
      // })
    },
    linkingBreak (targetBlock, slotNumber) {
      if (targetBlock && slotNumber > -1) {
        let findLink = this.links.find(value => {
          return value.targetID === targetBlock.id && value.targetSlot === slotNumber
        })

        if (findLink) {
          let findBlock = this.blocks.find(value => {
            return value.id === findLink.originID
          })

          this.links = this.links.filter(value => {
            return !(value.targetID === targetBlock.id && value.targetSlot === slotNumber)
          })

          this.linkingStart(findBlock, findLink.originSlot)

          this.updateScene()
        }
      }
    },
    removeLink (linkID = null) {
      if (linkID === null) {
        // Default link becoomes choose selected link ID
        linkID = this.selectedLineID
        this.selectedLineID = null
      }

      this.links = this.links.filter(value => {
        return !(value.id === linkID)
      })
    },
    // Blocks
    addNewBlock (nodeType, x, y) {
      let maxID = Math.max(0, ...this.blocks.map(function (o) {
        return o.id
      }))

      let node = this.nodes.find(n => {
        return n.type === nodeType
      })

      if (!node) {
        console.log('@BlocksContainer: Node not found' + nodeType)
        return
      }
      let block = this.createBlock(node, maxID + 1)

      // if x or y not set, place block to center
      if (x === undefined || y === undefined) {
        x = (this.$el.clientWidth / 2 - this.centerX) / this.scale
        y = (this.$el.clientHeight / 2 - this.centerY) / this.scale
      } else {
        x = (x - this.centerX) / this.scale
        y = (y - this.centerY) / this.scale
      }

      console.log('@VueBlocksContainer: Add block')

      block.x = x
      block.y = y
      this.blocks.push(block)

      this.updateScene()
    },
    createBlock (node, id) {
      let inputs = []
      let outputs = []
      let values = {}

      node.fields.forEach(field => {
        if (field.attr === 'input') {
          inputs.push({
            name: field.name,
            label: field.label || field.name
          })
        } else if (field.attr === 'output') {
          outputs.push({
            name: field.name,
            label: field.label || field.name
          })
        } else {
          if (!values[field.attr]) {
            values[field.attr] = {}
          }

          let newField = merge({}, field)
          delete newField['name']
          delete newField['attr']

          if (!values[field.attr][field.name]) {
            values[field.attr][field.name] = {}
          }

          values[field.attr][field.name] = newField
        }
      })

      return {
        id: id,
        x: 0,
        y: 0,
        selected: false,
        name: node.name,
        type: node.type,
        library: node.library,
        title: node.title || node.name,
        inputs: inputs,
        outputs: outputs,
        iconpath: node.iconpath,
        values: values
      }
    },
    deselectAll (withoutID = null) {
      // TOOD: rather use props for this (?!)
      this.blocks.forEach((value) => {
        if (value.id !== withoutID && value.selected) {
          this.blockDeselect(value)
        }
      })
      console.log('@VueBlockContainer: deselectAll | TODO fix in special case.')
      console.log(this.linkingMode)

      this.disableBlockMenu()
    },
    // --------------------------------------------
    // DropDownMenue Setup
    // --------------------------------------------
    showDropdownkMenu (e, elementType) {
      this.blockMenuVisible = true

      var containerElem = document.getElementById('main-element-container')

      // Relative position within div
      this.blockMenuX = e.clientX - containerElem.offsetLeft
      this.blockMenuY = e.clientY - containerElem.offsetTop

      this.blockSelectionType = elementType
    },
    disableDropdownkMenu () {
      console.log('@VueBlockContainer: Deselct DropdownMenu')
      this.blockMenuVisible = false
    },
    updateSelectedLine (lineID) {
      this.selectedLineID = lineID
    },
    updateSelectedBlock (block) {
      this.selectedBlock = block
    },
    disableBlockMenu () {
      console.log('@VueBlockContainer: Deselect block')
      this.blockMenuContent = []
      this.blockMenuVisible = false
    },
    showBlockMenu (element, e, elementType = 'block') {
      element.selected = true

      if (elementType === 'block') {
        console.log('@VueBlockContainer: show block menu')
        this.selectedBlock = element
      } else if (elementType === 'line') {
        console.log('@VueBlockContainer: line is not really passed')
      } else {
        console.log('@VueBlockContainer:Unexpected elementType')
      }
      this.blockMenuContent = []
      this.blockMenuVisible = true

      var containerElem = document.getElementById('main-element-container')

      // Relative position within div
      this.blockMenuX = e.clientX - containerElem.offsetLeft
      this.blockMenuY = e.clientY - containerElem.offsetTop

      this.blockSelectionType = elementType
    },
    // Events
    linkSelect (linkId) {
      console.log('@Blockscontainer: link select line')
    },
    linkDeselect (linkId) {
      console.log('@Blockscontainer: linkDeselect line')
    },
    blockSelect (block) {
      block.selected = true
      this.selectedBlock = block
      this.deselectAll(block.id)
      this.$emit('blockSelect', block)
    },
    blockDeselect (block) {
      block.selected = false

      if (block &&
          this.selectedBlock &&
          this.selectedBlock.id === block.id
         ) {
        this.selectedBlock = null
      }

      this.$emit('blockDeselect', block)
    },
    blockDelete (block = null) {
      console.log('@VueBlocksContainer: delete block')
      if (block === null) {
        block = this.selectedBlock
        this.blockDeselect(block)
      } else if (block.selected) {
        this.blockDeselect(block)
      }

      this.links.forEach(l => {
        if (l.originID === block.id || l.targetID === block.id) {
          this.removeLink(l.id)
        }
      })
      this.blocks = this.blocks.filter(b => {
        return b.id !== block.id
      })
      this.updateScene()
    },
    //
    prepareBlocks (blocks) {
      return blocks.map(block => {
        let node = this.nodes.find(n => {
          return n.name === block.name
        })

        if (!node) {
          return null
        }

        let newBlock = this.createBlock(node, block.id)

        newBlock = merge(newBlock, block, {
          arrayMerge: (d, s) => {
            return s.length === 0 ? d : s
          }
        })

        return newBlock
      }).filter(b => {
        return !!b
      })
    },
    prepareBlocksLinking (blocks, links) {
      if (!blocks) {
        return []
      }

      let newBlocks = []

      blocks.forEach(block => {
        let inputs = links.filter(link => {
          return link.targetID === block.id
        })

        let outputs = links.filter(link => {
          return link.originID === block.id
        })

        block.inputs.forEach((s, index) => {
          // is linked
          block.inputs[index].active = inputs.some(i => i.targetSlot === index)
        })

        block.outputs.forEach((s, index) => {
          // is linked
          block.outputs[index].active = outputs.some(i => i.originSlot === index)
        })

        newBlocks.push(block)
      })

      return newBlocks
    },
    importBlocksContent () {
      // Get all data (also of unrecored bocks)
      if (this.blocksContent) {
        this.nodes = merge([], this.blocksContent)
      }
    },
    importScene () {
      let scene = merge(this.defaultScene, this.scene)

      let blocks = this.prepareBlocks(scene.blocks)
      blocks = this.prepareBlocksLinking(blocks, scene.links)

      // set last selected after update blocks from props
      if (this.selectedBlock) {
        let block = blocks.find(b => this.selectedBlock.id === b.id)
        if (block) {
          block.selected = true
        }
      }

      this.blocks = blocks
      this.links = merge([], scene.links)

      let container = scene.container
      if (container.centerX) {
        this.centerX = container.centerX
      }
      if (container.centerY) {
        this.centerY = container.centerY
      }
      if (container.scale) {
        this.scale = container.scale
      }
      // Send to backend if complete

      this.orderBlocklistAndCheckIfIsLoop()
      if (this.loopIsClosed) {
        this.$emit('updateBackendProgram')
      }
    },
    exportScene () {
      let clonedBlocks = merge([], this.blocks)
      let blocks = clonedBlocks.map(value => {
        delete value['inputs']
        delete value['outputs']
        delete value['selected']

        return value
      })

      return {
        blocks: blocks,
        links: this.links,
        container: this.container
      }
    },
    updateScene () {
      // Send to app-vue
      this.$emit('update:scene', this.exportScene())
      // Send scene to backend file
      // console.log('BlocksContainer: Updating Scene')

      this.orderBlocklistAndCheckIfIsLoop()

      if (this.loopIsClosed) {
        this.$emit('updateBackendProgram')
      }
    },
    orderBlocklistAndCheckIfIsLoop () {
      // Only automatically send block to background when there is a complete loop
      if (this.links == null || this.links.length === 0) {
        this.loopIsClosed = false
        return
      }

      // Find start bock
      let itBlockOrigin = 0
      while (itBlockOrigin < this.blocks.length) {
        let blockType = this.blocks[itBlockOrigin]['name']
        if (blockType === 'idle' || blockType === 'start') {
          // Swap blocks
          [this.blocks[0], this.blocks[itBlockOrigin]] = [this.blocks[itBlockOrigin], this.blocks[0]]
          break
        }
        itBlockOrigin = itBlockOrigin + 1
      }

      let itLinkPosition = 0
      let itLinkSearcher = itLinkPosition

      itBlockOrigin = 0
      let itBlockTarget = itBlockOrigin + 1

      // console.log('@VueBlockContainer: Start sorting')
      // let debugItCount = 0

      // Nonzero check
      while (itLinkSearcher < this.links.length && itBlockOrigin < this.blocks.length) {
        // Find link which fits to 'originID' block
        // debugItCount = debugItCount + 1
        // if (debugItCount > 100) { break }
        // console.log('Block ID (origin)', this.blocks[itBlockOrigin]['id'])
        // console.log('Link: ' + this.links[itLinkSearcher]['originID'] + ' -> ' + this.links[itLinkSearcher]['targetID'])
        if (this.links[itLinkSearcher]['originID'] === this.blocks[itBlockOrigin]['id']) {
          // Swap Links
          // console.log('Swap')
          if (itLinkPosition !== itLinkSearcher) {
            [this.links[itLinkPosition], this.links[itLinkSearcher]] = [this.links[itLinkSearcher], this.links[itLinkPosition]]
          }
          // Find block which has corregt 'targetID'
          itBlockTarget = itBlockOrigin + 1
          while (itBlockTarget < this.blocks.length) {
            // debugItCount = debugItCount + 1
            // if (debugItCount > 100) { break }
            if (this.blocks[itBlockTarget]['id'] === this.links[itLinkPosition]['targetID']) {
              // Swap blocks to order
              if (itBlockTarget !== itBlockOrigin + 1) {
                [this.blocks[itBlockOrigin + 1], this.blocks[itBlockTarget]] = [this.blocks[itBlockTarget], this.blocks[itBlockOrigin + 1]]
              }
              itBlockOrigin = itBlockOrigin + 1
              // Go back to getting new link
              break
            } else {
              itBlockTarget = itBlockTarget + 1
            }
          }
          // Increment
          itLinkPosition = itLinkPosition + 1
          itLinkSearcher = itLinkPosition
        } else {
          // Increment
          itLinkSearcher = itLinkSearcher + 1
        }
      }

      // Check if closed
      if (this.links.length > itLinkPosition) {
        console.log('@VueBlocksContainer: Not all links used')
        this.loopIsClosed = false
      } else if (this.links[this.links.length - 1]['targetID'] !== this.blocks[0]['id']) {
        console.log('@VueBlocksContainer: Loop not closed')
        this.loopIsClosed = false
      } else {
        this.loopIsClosed = true
      }
    }
  },
  watch: {
    loopIsClosed (newValue) {
      if (newValue) {
        console.log('@VueBlocksContinaer: Update scene arrangement.')
        this.$emit('updateBackendProgram')
      }
    },
    // blocks () {
      // console.log('@VueBlocksContainer: change in blocks')
    // },
    // links () {
      // console.log('@VueBlocksContainer: change in links')
    // },
    blocksContent () {
      this.importBlocksContent()
    },
    scene () {
      this.importScene()
    }
    // tempLink () {
      // console.log('@VueBlocksContainer Change temp link')
      // console.log(this.tempLink)
    // }
  }
}
</script>

<style lang="less" scoped>
.vue-container {
    position: relative;
    overflow: hidden;
    box-sizing: border-box;
}

.vue-container{
    background-color:#0b214a;
}
</style>
