<template>
<div >
  <p> This is a thing </p>
  <input>
</div>
</template>


<script>
// import axios from 'axios' // Needed to pass. Only temporarily?
export default {
  name: 'LoadSave',
  // For testing
  // data: function () {
  data () {
    return {
      selectedListItem: null
    }
  }
}
</script>


<style scoped>
div {
    position: absolute;
    left: 300px;
    top: 300px;
    z-index: 20;
}

p {
    color: #d0d0d0;
}
</style>
