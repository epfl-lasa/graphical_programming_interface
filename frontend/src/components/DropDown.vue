<template>
<div class="dropdown-list"
     :style="dropDownStyle">
  <!-- TODO: separate element -->
  <template v-if="selectionType==='block'">
    <div class="dropdown-element">
      <p @mousedown="newBlockConnection($event)"
         @touchstart="newBlockConnection($event)"
         > Connect to </p>
    </div>
    <div class="dropdown-element">
      <p @mousedown="generalSettings($event)" @touchstart="generalSettings($event)"> Settings </p>
    </div>
    <!-- <div class="dropdown-element"> -->
      <!-- <p v-on:mousedown="moveBlock"> Move Block </p> -->
    <!-- </div> -->
    <div class="dropdown-element">
      <p @mousedown="removeBlock($event)" @touchstart="removeBlock($event)"> Delete Block </p>
    </div>
    <div v-for="(element, key) in menuContent">
      <p @mousedown="defaultAction($event, element.action)"
         @touchstart="defaultAction($event,element.action)"> {{element.content}} </p>
    </div>
  </template>
  <template v-else-if="selectionType==='line'">
    <div class="dropdown-element">
      <p @mousedown="removeLink($event)"
         @touchstart="removeLink($event)"
         > Delete Link </p>
    </div>
    <!-- <div class="dropdown-element"> -->
      <!-- <p v-on:mousedown="showLineSettings"> Line Settings </p> -->
    <!-- </div> -->
  </template>
</div>
</template>


<script>
export default {
  name: 'BlockDropdown',
  props: {
    menuContent: [],
    posX: 0,
    posY: 0,
    selectionType: null
  },
  mounted () {
    window.navigator.vibrate(100)
  },
  // mounted () {
  // console.log('Mounted Dropdown')
  // console.log(this.$parent.linkingMode)
  // console.log(this.$parent.linking)
  // },
  // mounted () {
  // console.log('@Dropdon: It is up now')
  // console.log(this.posX, this.posY)
  // },
  data () {
    return {
    }
  },
  methods: {
    newBlockConnection (e) {
      console.log('@DropDown: wanna connect')
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      this.$emit('disableBlockMenu', false)
      // Connect
      // console.log('@BlockDropDown: New block connection.')
      this.$emit('linkingStart')
    },
    generalSettings (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      // General Settings
      this.$emit('generalSettings')
      this.$emit('disableBlockMenu', false)
    },
    moveBlock (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      // General Settings
      // console.log('@BlockDropDown: Move block.')
      this.$emit('disableBlockMenu')
    },
    removeBlock (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      this.$emit('disableBlockMenu')
      this.$emit('removeBlock')
    },
    removeLink (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      this.$emit('disableBlockMenu')
      this.$emit('removeLink')
    },
    showLineSettings (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      console.log('Not implemented yet')
    },
    defaultAction (e, arg) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      console.log('@BlockDropDown: Define define action for arg=')
      console.log(arg)
    }
  },
  computed: {
    dropDownStyle () {          //
      // console.log('@BlockDropDown')
      // console.log(this.posX)
      // console.log(this.posY)
      return {
        left: this.posX + 'px',
        top: this.posY + 'px'
      }
    }
  }
}
</script>


<style lang="less" scoped>
@import './../assets/styles/main.less';

.dropdown-list {
    position: fixed;
    z-index: 2;
}

.dropdown-element {
    margin: 5px;
    padding: 10px 15px;

    font-size: 20px;

    border-style: solid;
    border-color: @color-main-medium;
    border-width: 1px;
    // border-radius: 10px;

    background:@color-main-dark;
    color: @fontcolor-main;
}
</style>
