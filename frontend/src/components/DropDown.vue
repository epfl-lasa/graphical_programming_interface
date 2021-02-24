<template>
<div id="dropdown-list"
     :style="dropDownStyle">
  <!-- TODO: separate element -->
  <template v-if="selectionType==='block'">
    <div class="dropdown-element">
      <p v-on:click="newBlockConnection"> Connect to </p>
    </div>
    <div class="dropdown-element">
      <p v-on:click="generalSettings"> Settings </p>
    </div>
    <!-- <div class="dropdown-element"> -->
      <!-- <p v-on:click="moveBlock"> Move Block </p> -->
    <!-- </div> -->
    <div class="dropdown-element">
      <p v-on:click="removeBlock"> Delete Block </p>
    </div>
    <div v-for="(element, key) in menuContent">
      <p v-on:click="defaultAction(element.action)"> {{element.content}} </p>
    </div>
  </template>
  <template v-else-if="selectionType==='line'">
    <div class="dropdown-element">
      <p v-on:click="removeLink"> Delete Link </p>
    </div>
    <!-- <div class="dropdown-element"> -->
      <!-- <p v-on:click="showLineSettings"> Line Settings </p> -->
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
  // mounted () {
    // console.log('Mounted Dropdown')
    // console.log(this.$parent.linkingMode)
    // console.log(this.$parent.linking)
  // },
  data () {
    return {
    }
  },
  methods: {
    newBlockConnection () {
      this.$emit('disableBlockMenu', false)
      // Connect
      console.log('@BlockDropDown: New block connection.')
      this.$emit('linkingStart')
    },
    generalSettings () {
      // General Settings
      console.log('@BlockDropDown: Show general settings.')
      this.$emit('disableBlockMenu', false)
    },
    moveBlock () {
      // General Settings
      console.log('@BlockDropDown: Move block.')
      this.$emit('disableBlockMenu')
    },
    removeBlock () {
      this.$emit('disableBlockMenu')
      this.$emit('removeBlock')
    },
    removeLink () {
      this.$emit('disableBlockMenu')
      this.$emit('removeLink')
    },
    showLineSettings () {
      console.log('Not implemented yet')
    },
    defaultAction (arg) {
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

.dropdown-element {
    margin: 5px;
    padding: 10px 15px;

    border-radius: 10px;

    font-size: 20px;

    border-color: black;
    background:#303030;
    color: #cccccc;

}

#dropdown-list {
    // position: absolute;
    // position: static;
    // position: absolute;
    position: absolute;
    border: black;
    color: grey;

    z-index: 2;
}
</style>
