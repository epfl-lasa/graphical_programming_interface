<template>
  <div class="loadsave-window">
    <h1 v-if="appMode==='load'" class="window-title"> Load Module </h1>
      <h1 v-else-if="appMode==='save'" class="window-title"> Save as </h1>
    <h1 v-else> List Directory </h1>

    <div class="file-table-content" :class="{load: appMode==='load', save: appMode==='save'}">
      <table id="load-save-table">
        <thead>
          <tr>
            <th> File Name </th>
            <th> Last Modified </th>
            <th> Type </th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="row in localFiles"
              :class="{selected: selectedListItem.name===row.name}"
              @click="selectNew($event, row)"
              @touchstart="selectNew($event, row)">
            <td> {{row.name}} </td>
            <td> {{row.datemodified}} </td>
            <td> {{row.type}} </td>
          </tr>
        </tbody>
      </table>
    </div>
    <br>

    <template v-if="appMode==='save'">
      <!-- TODO: fix that this updates when clicking on b-table list -->
      <label> File Name </label>
      <input class="position-input" type="text" v-model="saveName">
    </template>
    <div id="save-button-container">
      <div v-if="appMode==='load'" class="aica-button reference-button"
           @click="loadScene($event, selectedListItem.name)"
           @touchstart="loadScene($event, selectedListItem.name)"
              >
        Load
      </div>
      <div v-else-if="appMode==='save'" class="aica-button reference-button"
           @click="saveScene($event, saveName)"
           @touchstart="saveScene($event, saveName)"
           >
        Save
      </div>
      <div class="aica-button critical reference-button"
           @click="cancelLoading($event)"
           @touchstart="cancelLoading($event)">
        Cancel
      </div>
    </div>
  </div>
</template>


<script>
// TODO: move load-save here...
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
    selectNew (e, item) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      if (this.selectedListItem.name === item.name) {
        // Allow selection toggle
        this.selectedListItem = {'name': ''}
      } else {
        this.selectedListItem = item
      }
    },
    notImplemented () {
      console.log('Not implemented yet.')
    },
    closeLoading () {
      this.$parent.loadSaveMode = false
      this.$parent.appMode = 'main'
    },
    cancelLoading (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      this.closeLoading()
      console.log('Load/Saving canceled.')
    },
    loadScene (e, fileName) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      // console.log('Doing loadging.')
      this.$emit('loadScene', fileName)
      // this.$parent.loadScene(fileName)
      this.closeLoading()
    },
    saveScene (e, fileName) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      // console.log('Save')
      this.$emit('saveScene', fileName)
      this.closeLoading()
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


<style scoped lang="less">
@import './../assets/styles/main.less';

table {
    width: 100%;
}

thead {
    position: sticky;
    top: 0;
}


table, th, td {
    color: var(--fontcolor-main);
    // border: 1px solid var(--color-main-mediumbright);
    border-collapse: collapse;
}

table {
    th {
        size: @fontsize-large;
        background-color: var(--color-main-dark);
        border-bottom: 1px solid var(--color-main-mediumbright);

        padding-left: @fontsize-medium*0.5;
        padding-right: @fontsize-medium*0.8;
        padding-top: @fontsize-medium*0.3;
        padding-bottom: @fontsize-medium*0.3;
    }

    td {
        size: @fontsize-medium;
        border-top: 1px solid var(--color-main-mediumbright);
        border-bottom: 1px solid var(--color-main-mediumbright);

        padding-left: @fontsize-medium*0.5;
        padding-right: @fontsize-medium*0.8;

        padding-top: @fontsize-medium*0.8;
        padding-bottom: @fontsize-medium*0.4;
    }

    .selected {
        background-color: var(--color-main-mediumbright);
    }
}

.file-table-content{
    overflow-y: auto;

    &.save {
        max-height: 60%;
    }

    &.load {
        max-height: 70%;
    }
}

#save-button-container {
    align-self: center;
    // position: absolute;
    // right: @sidebar-width *0.08;
    // bottom: @header-height;
    // padding: 20px;
    position: absolute;
    bottom: @header-height*1.4;
    right: @sidebar-width*0.5;

    display: grid;
    grid-template-columns: auto auto;
    grid-column-gap: @sidebar-width * 0.3;

    background-color: var(--color-main-dark);
}

.aica-button {
    // size: @fontsize-large;
    size: @fontsize-large;
}

.window-title{
    size: @fontsize-huge;
    margin-bottom: @fontsize-huge*0.3;
    font-color: @fontcolor-main;
    margin-bottom: $size*1.0;
}

.loadsave-window {
    position: absolute;
    top: 0;
    left: 0;
    width: 100%;
    height: 100%;
    padding: @header-height;

    background-color: var(--color-main-dark);
    z-index: 10;
}


</style>
