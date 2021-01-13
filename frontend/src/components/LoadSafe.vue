<template>
  <div class="loadsafe-div">
    <p> Load-Safe This </p>
  </div>
</template>

<script>
// import axios from 'axios' // Needed to pass. Only temporarily?
export default {
  name: 'Header',
  data: function () {
    return {
      // For testing
      test: []
    }
  },
  props: {
    testProp: {
      type: Object,
      default: {}
    }
  },
  computed: {
    module_libraries () {
      return 0
    }
  },
  methods: {
    notImplemented () {
      console.log('Not implemented yet.')
    }
  }
}
</script>

<style scoped>
.header {
  font-size: 20px;
  font-color: white;
}
</style>
