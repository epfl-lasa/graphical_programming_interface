<!--
TODO:
> Divide sensible into links, block, & container

-->
<template>
<div class="vue-block" :class="{selected: selected}" :style="style"
     @touchstart="handleDown($event)"
     @mousedown="handleDown($event)"
     >
  <!-- <img v-bind:src="require('./' + iconPathTotal)" /> -->
  <div class='icon-container' :class="{selected: selected}">
  <!-- <div :class="'icon-container ' + selected"> -->
    <img v-bind:src="require('./../' + 'assets/icons_library/' + iconpath)" />
  </div>
  <h2 class='icon-description'> {{title}} </h2>
  </div>
</template>

<script>
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
    title: {
      type: String,
      default: 'Title'
    },
    // library: {
    // type: String,
    // default: ''
    // },
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

    // Linking & Dragging of specific block
    // this.linking = false
    this.dragging = false
  },
  mounted () {
    // TODO: an event listener is added for each block (?!) -- MAYBE move to 'drawing-window-class'
    document.documentElement.addEventListener('mousemove', this.handleMove, true)
    document.documentElement.addEventListener('mouseup', this.handleUp, true)
    // document.documentElement.addEventListener('mousedown', this.handleDown, true)

    document.documentElement.addEventListener('touchemove', this.handleMove, true)
    document.documentElement.addEventListener('touchend', this.handleUp, true)

    // document.documentElement.addEventListener('touchmove', this.testTouchUp, true)
    // document.documentElement.addEventListener('touchup', this.testTouchDown, true)
  },
  beforeDestroy () {
    document.documentElement.removeEventListener('mousemove', this.handleMove, true)
    document.documentElement.removeEventListener('mouseup', this.handleUp, true)
    // document.documentElement.removeEventListener('mousedown', this.handleDown, true)

    document.documentElement.removeEventListener('touchmove', this.handleMove, true)
    document.documentElement.removeEventListener('touchend', this.handleUp, true)

    // document.documentElement.removeEventListener('touchmove', this.testTouchUp, true)
    // document.documentElement.removeEventListener('touchup', this.testTouchDown, true)

    // document.documentElement.removeEventListener('touchmove', this.handleMove, true)
    // document.documentElement.removeEventListener('touchup', this.handleUp, true)
  },
  data () {
    return {
      width: this.options.width,
      hasDragged: false,

      // Different press states
      presstimer: null,
      longpress: false,

      connecting: false
    }
  },
  methods: {
    handleMove (e) {
      if (e.type === 'touchmove') {
        e.preventDefault()
      }

      this.cancelLongPress()

      this.mouseX = e.pageX || e.clientX + document.documentElement.scrollLeft
      this.mouseY = e.pageY || e.clientY + document.documentElement.scrollTop

      // console.log('Dragging')
      // console.log(this.dragging)
      if (this.dragging && !this.linkingMode) {
        let diffX = this.mouseX - this.lastMouseX
        let diffY = this.mouseY - this.lastMouseY

        this.lastMouseX = this.mouseX
        this.lastMouseY = this.mouseY

        this.moveWithDiff(diffX, diffY)

        this.hasDragged = true

        this.$emit('disableBlockMenu')
      }
    },
    handleDown (e) {
      if (e.type === 'touchstart') {
        // Which one is necessary? Which one is good...
        e.preventDefault()
        e.stopPropagation()
      }
      // console.log('VueBlock -- gogogo (-1)')
      // console.log(this.linkingMode)
      console.log('@VueBlock: Handle Down')

      // if (typeof e ) {
      if (e.type === 'touchstart') {
        // console.log('@VueBlock:', e)
        console.log(e)
        // TODO: make touches more precise / better fitting.
        this.mouseX = e.touches[0].clientX
        this.mouseY = e.touches[0].clientY
      } else { // Else mouse event
        this.mouseX = e.pageX || e.clientX + document.documentElement.scrollLeft
        this.mouseY = e.pageY || e.clientY + document.documentElement.scrollTop
        this.absoluteMouseX = e.pageX
        this.absoluteMouseY = e.pageY
      }

      this.lastMouseX = this.mouseX
      this.lastMouseY = this.mouseY

      const target = e.target || e.srcElement

      if (this.$el.contains(target)) {
        if (this.linkingMode) {
          console.log('@VueBock: Want to link')
          this.$emit('linkingStop')
          return
        }
        this.dragging = false

        // Check for long-click
        this.presstimer = setTimeout(this.handleLongPress, 300, e)
        this.$emit('select')

        if (e.preventDefault) e.preventDefault()
      } else if (this.linkingMode) {
        this.$emit('linkingStopDrawing')
      }
    },
    handleLongPress (e) {
      this.longpress = true
      this.dragging = true

      this.showAppDropwdown = true

      // this.$emit('showBlockMenu', this.blockData, this.mouseX, this.mouseY)
      this.$emit('showBlockMenu', this.blockData, this.absoluteMouseX, this.absoluteMouseY)
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
      this.cancelLongPress()

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
    }
  }
}
</script>

<style lang="less" scoped>
@import './../assets/styles/main.less';

@blockBorder: 0px;

@ioPaddingInner: 2px 0;
@ioHeight: 20px;
@ioFontSize: 15px;

@circleBorder: 2px;
@circleSize: 10px;
@circleMargin: 0px; // left/right

@circleNewColor: #00FF00;
@circleRemoveColor: #FF0000;
@circleConnectedColor: #FFFF00;

// New variables
@fontsizeDescription: 11pt;
@marginBlock: 20px;

.vue-block{
    background:@color-main-dark;
    // background:@fontcolor-main;
    position: absolute;
    box-sizing: border-box;
    width: (2*@marginBlock + @block-width);
    height: (2*@marginBlock + @block-height);
    z-index: 1;
    opacity: 1.0;
    cursor: move;
}

.icon-container {
    background:@color-main-medium;
    width:@block-width;
    height:@block-height;

    // Put at center of paretn
    position: relative;
    left: @marginBlock;
    top: @marginBlock;

    &.selected {
        // border: @blockBorder solid black;
        // z-index: 2;
        box-shadow: 0px 0px 20px 4px @color-main-bright;
    }
}

.icon-description {
    font-size: @fontsizeDescription;
    color: @fontcolor-main;
    text-align: center;
    position: relative;
    top: @marginBlock;
    // bottom: 1px;
}


.vue-block {
    // border: @blockBorder solid black;
    // background-image: url(./../assets/images/idle.jpeg);
    // block-size: 110px;
    // width: 92px;
    // height: 92px;

    // &.selected {
        // border: @blockBorder solid black;
        // z-index: 2;
        // box-shadow: 0px 0px 20px 4px @color-main-bright;
    // }
    > header {
        background: #bfbfbf;
        text-align: center;

        > .delete {
            color: red;
            cursor: pointer;
            float: right;
            position: absolute;
            right: 5px;
        }
    }

    .inputs, .outputs {
        padding: @ioPaddingInner;

        display: block;
        width: 50%;

        > * {
            width: 100%;
        }
    }

    .circle {
        box-sizing: border-box;
        margin-top: @ioHeight / 2 - @circleSize / 2;

        width: @circleSize;
        height: @circleSize;

        border: @circleBorder solid rgba(0, 0, 0, 0.5);
        border-radius: 100%;

        cursor: crosshair;
        &.active {
            background: @circleConnectedColor;
        }
    }

    .inputs {
        float: left;
        text-align: left;

        margin-left: -(@circleSize/2 + @blockBorder);
    }

    .input, .output {
        height: @ioHeight;
        overflow: hidden;
        font-size: @ioFontSize;

        &:last-child {
        }
    }

    .input {
        float: left;

        .circle {
            float: left;
            margin-right: @circleMargin;

            &:hover {
                background: @circleNewColor;

                &.active {
                    background: @circleRemoveColor;
                }
            }
        }
    }

    .outputs {
        float: right;
        text-align: right;

        margin-right: -(@circleSize/2 + @blockBorder);
    }

    .output {
      float: right;

      .circle {
        float: right;
        margin-left: @circleMargin;

        // &:hover {
          // background: @circleNewColor;
        // }
      }
    }
}

</style>
