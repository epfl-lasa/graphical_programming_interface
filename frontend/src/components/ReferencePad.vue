<template>
  <!-- TODO: arrow pad -check paretn somehow... -->
<div class="arrow-pad pad-class">
  <div class="pad-class increment-button"
       @click="updateValueIncrement($event, incrementValue)"
       @mousdown="updateValueIncrement($event, incrementValue)"
       >
    <img
      class="pad-class increment-image"
      src='./../assets/icons/keyboard_arrow_up-white-18dp.svg'>
  </div>
  <div class="pad-class increment-button"
       @click="updateValueIncrement($event, (-1)*incrementValue)"
       @mousdown="updateValueIncrement($event, (-1)*incrementValue)"
       >
    <img
      class="pad-class increment-image"
      src='./../assets/icons/keyboard_arrow_down-white-18dp.svg'>
  </div>
  <div id="increment-tag" class="pad-class">
    <p class="inline pad-class"> Increment </p>
    <select v-model.number="incrementValue" name="incrementValue" class="inline pad-class" id="Increment">
      <option class="pad-class" value="1">1</option>
      <option class="pad-class" value="5">5</option>
      <option class="pad-class" value="10">10</option>
      <option class="pad-class" value="25">25</option>
      <option class="pad-class" value="50">50</option>
      <option class="pad-class" value="100">100</option>
    </select>
  </div>
</div>
</template>


<script>
// TODO: move load-save here...
// import axios from 'axios' // Needed to pass. Only temporarily?
export default {
  name: 'ReferencePad',
  // For testing
  // data: function () {
  data () {
    return {
      incrementValue: 10
    }
  },
  mounted () {
    document.documentElement.addEventListener('mousedown', this.handleMouseDown, true)
    document.documentElement.addEventListener('touchstart', this.handleMouseDown, true)
  },
  beforeUnmount () {
    document.documentElement.removeEventListener('mousedown', this.handleMouseDown, true)
    document.documentElement.removeEventListener('touchstart', this.handleMouseDown, true)
  },
  props: {
    referencePadValue: 0.0
  },
  methods: {
    roundDec (x, dec) {
      return Math.round(x * Math.pow(10, dec)) / Math.pow(10, dec)
    },
    handleMouseDown (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      // console.log('@Arrowpad - event')
      // console.log(e.target.classList)
      if (!e.target.classList.contains('pad-class')) {
        this.$emit('update:poseInputFocused', false)
      }
    },
    updateValueIncrement (e, increment) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }

      let newValue = this.referencePadValue + increment

      newValue = Math.round(newValue * 10.0) / 10.0
      // newValue = this.roundDec(newValue)
      // newValue =

      this.$emit('updatePadReading', newValue)
    }
  }
}
</script>


<style scoped lang="less">
@import './../assets/styles/main.less';

.inline {
    display: inline;
}

option {
    padding: @fontsize-medium*0.1;
}

select {
    background: @color-main-medium;
    color: @fontcolor-main;
    padding: @fontsize-medium*0.2;
    margin-left: @sidebar-width*0.1;
}

.arrow-pad {
    padding: @fontsize-medium;
    background: @color-main-medium;

    position: fixed;
    right: @sidebar-width;
    top: @header-height*2;
    z-index: 6;

    // width: @sidebar-width*0.8;
    // height: @sidebar-width*0.3;

}

.increment-button {
    border: 1px solid var(--color-border);
    width: @sidebar-width*0.5;
    height: @sidebar-width*0.2;
}

.increment-image {
    width: 100%;
    height: 100%;
}

#increment-tag {
    margin-top: @fontsize-medium*0.2;
}
</style>
