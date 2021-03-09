<template>
  <div class="data-window">
    <h2> Recorded Data </h2>
    <!-- <p>Test id: {{module.id }} </p> -->
    <div class="file-table-content">
      <b-table
        class="data-table"
        :data="database"
        :selected.sync="selected"
        focusable
        >
        <b-table-column field="id" label="ID" width="20" v-slot="props">
          {{ props.row.id }}
        </b-table-column>
        <b-table-column field="datemodified" label="Created" width="40" v-slot="props">
          {{ props.row.datemodified }}
        </b-table-column>
        </b-table>
    </div>
    <br>
    <template v-if="true">
        <div v-if="isRecording" class="aica-button reference-button"
          @click="stopRecording($event)" @touchstart="stopRecording($event)">
           <p> Stop </p>
        </div>
        <div v-else class="aica-button reference-button"
          @click="startAndReplaceRecording($event)" @touchstart="startAndReplaceRecording($event)">
           <p> New Recording </p>
        </div>
    </template>
    <template v-else>
      <div id="table-button-container">
        <div v-if="isRecording" class="aica-button reference-button"
          @click="stopRecording($event)" @touchstart="stopRecording($event)">
           <p> Stop </p>
        </div>
        <div v-else class="aica-button reference-button"
          @click="startRecording($event)" @touchstart="startRecording($event)">
           <p> Record </p>
        </div>

        <div @click="deleteElement" class="aica-button critical reference-button">
          <p> Delete Element</p>
         </div>
      </div>
    </template>
  </div>
</template>


<script>
import axios from 'axios' // Needed to pass. Only temporarily?

export default {
  name: 'ModuleDataList',
  // data: function () {
  props: {
    module: Object,
    robotIsMoving: Boolean,
    // title,
    // localFiles: Array,
    appMode: {
      type: String,
      default: 'load'
    },
    settings: Array
  },
  mounted () {
    // console.log('@ModuleDataList: Get Database')
    // console.log(this.module.id)
    // console.log(this.module)
    this.getDatabase()
  },
  data () {
    return {
      isRecording: false,
      database: [],
      selected: null,
      multipleRecordings: true,
      title: 'Title',
      selectedItem: {
        'name': ''
      }
    }
  },
  computed: {
    onlyOneRecording () {
      if ('onlyOneRecording' in this.settings && this.settings.onlyOneRecording) {
        return true
      } else {
        return false
      }
    }
  },
  methods: {
    startAndReplaceRecording (e) {
      // New recording [TODO: make a bit smoother... / no glitching in between]
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      console.log(this.database)

      if (this.database.length) {
        let i
        for (i = 0; i < this.database.length; i++) {
          this.selected = this.database[i]
          this.deleteElement(e)
        }
      }

      this.startRecording(e)
    },
    startRecording (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      this.isRecording = true
      axios.get(this.$localIP + `/recordmoduledatabase/` + this.module.id,
                {'params': {}})
        .then(response => {
          this.database = response.data.moduledatabase
        })
        .catch(error => {
          console.log(error)
          console.log('@ModuleDataList: failure while updating backend.')
        })
    },
    stopRecording (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      this.isRecording = false
      axios.get(this.$localIP + `/stoprecording`,
                {'params': {}})
        .catch(error => {
          console.log(error)
        })
    },
    deleteElement (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      if (!('name' in this.selected)) {
        console.log(this.selected)
        console.log('Skipping delete')
        return
      }
      axios.get(this.$localIP + `/deletemdouledatabase/` + this.module.id + '/' + this.selected.name,
                {'params': {}})
        .then(response => {
          this.database = response.data.moduledatabase
          // console.log('@ModuleDataList: success')
          // console.log(this.database)
        })
        .catch(error => {
          console.log(error)
        })
    },
    getDatabase () {
      console.log('@ModuleDataList:')
      console.log(this.module.id)

      axios.get(this.$localIP + `/getdataofmodule/` + this.module.id,
                {'params': {}})
        .then(response => {
          this.database = response.data.moduledatabase
          // console.log('@ModuleDataList: success')
          // console.log(this.database)
        })
        .catch(error => {
          console.log(error)
          console.log('@ModuleDataList: failure while updating backend.')
        })
    }
    // watch: {
      // database (newValue) {
        // console.log('@ModuleDataList: database-watcher')
        // console.log(newValue)
      // }
    // }
  }
  // watch: {
  // }
}
</script>


<style scoped lang="less">
@import './../assets/styles/main.less';

#table-button-container {
    padding-left: ;
    padding-right: ;
    display: grid;
    grid-template-columns: auto auto;
}
/* .data-window { */
    /* position: absolute; */
    /* top: 15%; */
    /* right: 10%; */
    /* width: 80%; */
    /* height: 80%; */
    /* padding: 20px; */
    /* background-color: #d0d0d0; */

    /* z-index: 3; */
    /* border: 5px solid #0c0c0c; */
    /* border-radius: 10px; */
/* } */


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
