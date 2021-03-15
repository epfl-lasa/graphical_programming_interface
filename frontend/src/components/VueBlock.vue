<!--
TODO:>
 Divide sensible into links, block, & container
-->

<template>
<div class="vue-block"  :style="style"
     :class="{selected: selected, currentlyPlaying: currentlyPlaying}"
     @touchstart="handleDown($event)"
     @mousedown="handleDown($event)"
     >
  <!-- <img v-bind:src="require('./' + iconPathTotal)" /> -->
  <div class='icon-container'
       :class="{selected: selected, currentlyPlaying: currentlyPlaying}"
       >
  <!-- <div :class="'icon-container ' + selected"> -->
    <img v-bind:src="require('./../' + 'assets/icons_library/' + iconpath)" />
  </div>
  <h2 class='icon-name'> {{title}} </h2>
  </div>
</template>

<script>
import blockloaderHelper from '../helpers/blockloader'

export default {
  name: 'VueBlock',
  props: {
    x: {
      type: Number,
      default: 0,
      validator: function (val) {
        return typeof val === 'number'
      }
    },
    y: {
      type: Number,
      default: 0,
      validator: function (val) {
        return typeof val === 'number'
      }
    },
    selected: Boolean,
    currentlyPlaying: Boolean,
    title: {
      type: String,
      default: 'Title'
    },
    blockData: Object,

    inputs: Array,
    outputs: Array,

    iconpath: {
      type: String,
      default: 'idle.jpeg'
    },
    iconFolderPath: {
      type: String,
      default: 'assets/images/'
    },
    options: {
      type: Object
    },
    // Linking mode for environment
    linkingMode: false
  },
  created () {
    this.mouseX = 0
    this.mouseY = 0

    this.lastMouseX = 0
    this.lastMouseY = 0

    this.firstMouseX = 0
    this.firstMouseY = 0

    // Linking & Dragging of specific block
    // this.linking = false
    this.dragging = false
  },
  async mounted () {
    // Prepare Block for later inspection & menu-modification.
    await this.setupPropertyAndUpdate()

    // TODO: check how to suppress the previous update [currently there are two.. at mounting..]
    this.$emit('update')
  },
  beforeUnmount () {
    this.removeHandles()
  },
  data () {
    return {
      width: this.options.width,
      hasDragged: false,
      hasOnlyMoveLittleSinceClick: false,

      // Different press states
      presstimer: null,
      longpress: false,

      connecting: false,
      justFinishedLinking: false
    }
  },
  methods: {
    setupPropertyAndUpdate () {
      this.blockData.values.property = blockloaderHelper.propertyLoader(this.blockData.values.property)
    },
    initializeHandles () {
      document.documentElement.addEventListener('mousemove', this.handleMove, true)
      document.documentElement.addEventListener('mouseup', this.handleUp, true)

      document.documentElement.addEventListener('touchmove', this.handleMove, true)
      document.documentElement.addEventListener('touchend', this.handleUp, true)
    },
    removeHandles () {
      document.documentElement.removeEventListener('mousemove', this.handleMove, true)
      document.documentElement.removeEventListener('mouseup', this.handleUp, true)

      document.documentElement.removeEventListener('touchmove', this.handleMove, true)
      document.documentElement.removeEventListener('touchend', this.handleUp, true)
    },
    handleMove (e) {
      if (e.type === 'touchmove') {
        e.preventDefault()
        this.mouseX = e.touches[0].clientX
        this.mouseY = e.touches[0].clientY
      } else {
        this.mouseX = e.pageX || e.clientX + document.documentElement.scrollLeft
        this.mouseY = e.pageY || e.clientY + document.documentElement.scrollTop
      }

      // Add a minimum movement margin for 'touch events'
      if (e.type === 'touchmove' && this.hasOnlyMoveLittleSinceClick) {
        let diffXTot = this.mouseX - this.firstMouseX
        let diffYTot = this.mouseY - this.firstMouseY
        let refDist = (e.srcElement.width / 4.0)
        if (diffXTot * diffXTot + diffYTot * diffYTot < refDist * refDist) {
          return
        }
        this.hasOnlyMoveLittleSinceClick = false
      }

      this.cancelLongPress()

      if (this.dragging && !this.linkingMode) {
        let diffX = this.mouseX - this.lastMouseX
        let diffY = this.mouseY - this.lastMouseY

        this.lastMouseX = this.mouseX
        this.lastMouseY = this.mouseY

        this.moveWithDiff(diffX, diffY)

        this.hasDragged = true

        this.$emit('disableBlockMenu')
      }
      // else {
      // Remove if ever true / or the whole condition after debugging
      // console.log('@VueBlock: Testing if condition ever false. Remove NOW.')
      // }
    },
    handleDown (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      if (e.type === 'touchstart') {
        this.mouseX = e.touches[0].clientX
        this.mouseY = e.touches[0].clientY
      } else { // Else mouse event
        this.mouseX = e.pageX || e.clientX + document.documentElement.scrollLeft
        this.mouseY = e.pageY || e.clientY + document.documentElement.scrollTop
      }
      this.$emit('deselectAll')

      this.initializeHandles()

      this.lastMouseX = this.mouseX
      this.lastMouseY = this.mouseY

      this.firstMouseX = this.mouseX
      this.firstMouseY = this.mouseY

      this.hasOnlyMoveLittleSinceClick = true

      const target = e.target || e.srcElement

      if (this.$el.contains(target)) {
        // It always contains target...
        if (this.linkingMode) {
          console.log('@VueBock: Want to link')
          this.$emit('linkingStop')
          this.justFinishedLinking = true
          return
        }
        this.dragging = false

        // Check for long-click
        this.presstimer = setTimeout(this.handleLongPress, 300, e)
        // this.$emit('select')

        if (e.preventDefault) e.preventDefault()
      } else if (this.linkingMode) {
        this.$emit('linkingStopDrawing')
      }
    },
    handleLongPress (e) {
      this.longpress = true
      this.dragging = true

      this.showAppDropwdown = true
      this.$emit('showBlockMenu', this.blockData, this.mouseX, this.mouseY)
      // this.$emit('showBlockMenu', this.blockData, this., this.absolutemouseY)
    },
    cancelLongPress () {
      if (this.presstimer !== null) {
        clearTimeout(this.presstimer)

        this.presstimer = null
        this.longpress = false
      }
    },
    handleUp (e) {
      if (e.type === 'touchend') {
        e.preventDefault()
      }

      // console.log('@VueBlock: handleUp')
      if (!this.longpress && !this.dragging && !this.justFinishedLinking) {
        this.$emit('select')
      }

      this.justFinishedLinking = false
      this.cancelLongPress()
      this.removeHandles()

      if (this.dragging) {
        this.dragging = false

        if (this.hasDragged) {
          this.save()
          this.hasDragged = false
        }
      }
    },
    //
    // Slots (DEPRECIATED ?!)
    //
    slotMouseDown (e, index) {
      this.$emit('linkingStart', index)
      if (e.preventDefault) e.preventDefault()
    },
    slotMouseUp (e, index) {
      this.$emit('linkingStop', index)
      if (e.preventDefault) e.preventDefault()
    },
    slotBreak (e, index) {
      this.$emit('linkingBreak', index)
      if (e.preventDefault) e.preventDefault()
    },
    // End Depreciated
    //
    save () {
      this.$emit('update')
    },
    deleteBlock () {
      this.$emit('delete')
    },
    moveWithDiff (diffX, diffY) {
      let left = this.x + diffX / this.options.scale
      let top = this.y + diffY / this.options.scale

      this.$emit('update:x', left)
      this.$emit('update:y', top)
    },
    testTouchStart (e) {
      console.log(e)
      console.log('@VueBlock: Touchsuccess')
    },
    testTouchMove (e) {
      console.log(e)
      console.log('@VueBlock: Touchsuccess')
    },
    testTouchUp (e) {
      console.log(e)
      console.log('@VueBlock: Touchsuccess')
    }
  },
  computed: {
    style () {
      return {
        top: this.options.center.y + this.y * this.options.scale + 'px',
        left: this.options.center.x + this.x * this.options.scale + 'px',
        transformOrigin: 'top left'
      }
    },
    iconPathTotal () {
      // TODO: how can the path be autogenerated(?!) / given as a variable...
      console.log('Path')
      return './../assets/images/idle.jpeg'
    }
    // headerStyle () {
    // return {
    // height: this.options.titleHeight + 'px'
    // }
    // }
  },
  watch: {
    iconpath () {
      console.log('iconpath updated')
    },
    linkingMode (newValue) {
      console.log('We are linking')
      console.log(newValue)
    }
  }
}
</script>

<style lang="less" scoped>
@import './../assets/styles/main.less';

.vue-block{
    cursor: move;

    background:@color-main-dark;
    position: absolute;

    width: var(--block-container-height);
    height: var(--block-container-height);
    z-index: 1;
    opacity: 1.0;
    border-radius: var(--margin-block);

    .icon-container {
        background:@color-main-medium;
        width:@block-width;
        height:@block-height;

        // Put at center of parent
        position: relative;
        left: var(--margin-block);
        top: var(--margin-block);

        &.selected {
            box-shadow: 0px 0px 20px 4px var(--color-main-bright);
            // box-shadow: 0px 0px 20px 4px var(--color-highlight1-bright);
        }

        &.currentlyPlaying {
            box-shadow: 0px 0px 20px 4px var(--color-highlight1-bright);
        }
    }

    .icon-name {
        font-size: @fontsize-medium;
        color: @fontcolor-main;
        // text-align: center;
        position: relative;
        top: var(--margin-block);
        left: var(--margin-block);
    }
}
</Style>
