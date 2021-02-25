<template>
  <div class="data-window">
    <!-- <h1> {{title}} </h1> -->
    <!-- <p>Test id: {{module.id }} </p> -->
    <div class="file-table-content">
      <b-table
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
        <!-- <b-table-column field="datemodified" label="Data Points" width="40" v-slot="props"> -->
          <!-- {{ props.row.datemodified }} -->
        <!-- </b-table-column> -->
        <p> Table </p>
          <!-- <b-table-column field="type" label="Type" width="40" v-slot="props"> -->
          <!-- {{ props.row.type }} -->
        <!-- </b-table-column> -->
        </b-table>
    </div>
    <br>
    <template>
      <b-button v-if="isRecording"
                v-on:click="stopRecording"> Stop </b-button>
      <b-button v-else-if="multipleRecordings"
                v-on:click="startRecording"> Record </b-button>
      <b-button v-else
                v-on:click="startRecording"> Record New </b-button>

      <b-button v-if="multipleRecordings" v-on:click="deleteElement">
        Delete Element </b-button>
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
    }
  },
  mounted () {
    console.log('@ModuleDataList: Get Database')
    console.log(this.module.id)
    console.log(this.module)
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
  methods: {
    startRecording () {
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
    stopRecording () {
      this.isRecording = false
      axios.get(this.$localIP + `/stoprecording`,
                {'params': {}})
        .catch(error => {
          console.log(error)
        })
    },
    deleteElement () {
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


<style scoped>
.data-window {
    /* position: absolute; */
    /* top: 15%; */
    /* right: 10%; */
    /* width: 80%; */
    /* height: 80%; */
    /* padding: 20px; */
    background-color: #d0d0d0;

    /* z-index: 3; */
    /* border: 5px solid #0c0c0c; */
    /* border-radius: 10px; */
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
