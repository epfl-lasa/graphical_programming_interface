<template>
  <div class="loadsave-window" v-if="false">
    <h1 v-if="appMode==='load'"> Load Module </h1>
      <h1 v-else-if="appMode==='save'"> Save as </h1>
    <h1 v-else> List Directory </h1>

    <b-table
      :data="localFiles"
      :selected.sync="selectedListItem"
      focusable
      >
      <!-- :columns="columns"> -->
      <b-table-column field="name" label="File Name" width="40" v-slot="props">
        {{ props.row.name }}
      </b-table-column>

      <b-table-column field="type" label="Type" width="40" v-slot="props">
        {{ props.row.type }}
      </b-table-column>
    </b-table>
    <br>

    <!-- <template v-if="appMode==='save'"> -->
      <!-- <label> File Name </label> -->
      <!-- <input class="position-input" type="text" v-model="selectedListItem.name"> -->
    <!-- </template> -->

    <!-- <b-button v-on:click="saveScene(selectedListItem.name)"> Save </b-button> -->
    <!-- <b-button v-on:click="loadScene(selectedListItem.name)"> Load </b-button> -->

    <!-- <b-button v-on:click="cancelLoading()"> Cancel </b-button> -->
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
  },
  props: {
    localFiles: Array,
    appMode: 'load'
  },
  computed: {
    moduleLibraries () {
      return 0
    }
  },
  methods: {
    notImplemented () {
      console.log('Not implemented yet.')
    },
    cancelLoading () {
      this.$parent.loadSaveMode = false
    },
    loadScene (fileName) {
      console.log('Save')
      this.$parent.loadScene(fileName)
      this.cancelLoading()
    },
    saveScene (fileName) {
      this.$parent.saveScene(fileName)
      this.cancelLoading()
    }
  }
}
</script>


<style scoped>
.loadsave-window {
    position: absolute;
    top: 15%;
    right: 10%;
    width: 80%;
    height: 80%;
    padding: 20px;

    background-color: #d0d0d0;

    z-index: 3;
    border: 5px solid #0c0c0c;
    border-radius: 10px;
}
</style>
