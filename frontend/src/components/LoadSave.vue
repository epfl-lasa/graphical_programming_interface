<template>
  <div class="loadsave-window">
    <h1 v-if="appMode==='load'"> Load Module </h1>
      <h1 v-else-if="appMode==='save'"> Save as </h1>
    <h1 v-else> List Directory </h1>


    <div class="file-table-content">
      <b-table
        :data="localFiles"
        :selected.sync="selectedListItem"
        focusable>
        <!-- :columns="columns"> -->
        <b-table-column field="name" label="File Name" width="40" v-slot="props">

          {{ props.row.name }}
        </b-table-column>

        <b-table-column field="datemodified" label="Last Modified" width="40" v-slot="props">
          {{ props.row.datemodified }}
        </b-table-column>

        <b-table-column field="type" label="Type" width="40" v-slot="props">
          {{ props.row.type }}
        </b-table-column>
      </b-table>
    </div>
    <br>

    <template v-if="appMode==='save'">
      <!-- TODO: fix that this updates when clicking on b-table list -->
      <label> File Name </label>
      <input class="position-input" type="text" v-model="saveName">
    </template>

    <b-button v-if="appMode==='load'"
              v-on:click="loadScene(selectedListItem.name)"> Load </b-button>
    <b-button v-else-if="appMode==='save'"
              v-on:click="saveScene(saveName)"> Save </b-button>

    <b-button v-on:click="cancelLoading()"> Cancel </b-button>
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
      selectedListItem: {'name': ''},
      saveName: ''
    }
  },
  props: {
    localFiles: Array,
    appMode: {
      type: String,
      default: 'load'
    }
  },
  methods: {
    // selectItemSync (item) {
    // this.selectedListItemName = item.name
    // },
    notImplemented () {
      console.log('Not implemented yet.')
    },
    cancelLoading () {
      this.$parent.loadSaveMode = false
      this.$parent.appMode = 'main'
    },
    loadScene (fileName) {
      console.log('Load')
      this.$parent.loadScene(fileName)
      this.cancelLoading()
    },
    saveScene (fileName) {
      console.log('Save')
      this.$parent.saveScene(fileName)
      this.cancelLoading()
    }
  },
  watch: {
    selectedListItem (newValue) {
      if (this.appMode === 'save') {
        this.saveName = newValue.name
      }
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


.file-table-content {
    max-height: 80%;
    overflow-y: auto;
}

/* Fix header !? --- TODO */
/* .file-table-content { */
    /* max-height: 80%; */
    /* overflow-y: auto; */
/* } */

</style>
