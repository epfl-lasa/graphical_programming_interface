<template>
<div id="dropdown-list"
     :style="dropDownStyle">
  <!-- TODO: separate element -->
  <div class="dropdown-element">
    <p v-on:click="newBlockConnection"> Connect to </p>
  </div>
  <div class="dropdown-element">
    <p v-on:click="generalSettings"> Settings </p>
  </div>
  <div class=dropdown-element>
    <p v-on:click="moveBlock"> Move Block </p>
  </div>
  <div v-for="(element, key) in menuContent">
    <p v-on:click="defaultAction(element.action)"> {{element.content}} </p>
  </div>
</div>
</template>


<script>
export default {
  name: 'BlockDropdown',
  props: {
    menuContent: [],
    posX: 0,
    posY: 0
  },
  data () {
    return {
    }
  },
  methods: {
    newBlockConnection () {
      // Connect
      console.log('@BlockDropDown: New block connection.')
      this.$emit('disableBlockMenu', false)
    },
    generalSettings () {
      // General Settings
      console.log('@BlockDropDown: Show general settings.')
      this.$emit('disableBlockMenu', false)
    },
    moveBlock () {
      // General Settings
      console.log('@BlockDropDown: Move block.')
      this.$emit('disableBlockMenu', false)
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
