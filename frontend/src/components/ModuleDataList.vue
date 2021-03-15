<template>
  <div class="data-window">
    <h2> Recorded Data </h2>
    <!-- <p>Test id: {{module.id }} </p> -->
    <br>
    <div v-if="true" class="reference-button-container">
    <!-- <template v-if="true"> -->
        <div v-if="isRecording" class="aica-button reference-button critical"
          @click="stopRecording($event)" @touchstart="stopRecording($event)">
           <p> Stop Recording </p>
        </div>
        <div v-else class="aica-button reference-button"
          @click="startAndReplaceRecording($event)" @touchstart="startAndReplaceRecording($event)">
           <p> New Recording </p>
        </div>
        <div v-if="robotIsMoving" class="aica-button danger" id="run-module"
               @click="stopRobot($event)" @touchstart="stopRobot($event)"
               >
          <p> Stop Moving </p>
        </div>
        <div v-else class="aica-button" id="run-module"
             @click="replayData($event)" @touchstart="replayData($event)">
          <p> Replay Data </p>
        </div>
    </div>
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
    <br>
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
    <div v-if="robotIsMoving">
      <div  class="aica-button danger" id="run-module"
            @click="stopRobot($event)" @touchstart="stopRobot($event)"
               >
        <p> Stop Moving </p>
      </div>
    </div>
    <div v-else class="reference-button-container">
      <div class="aica-button reference-button"
        @click="moveToStartPoint($event)" @touchstart="moveToStartPoint($event)">
        <p> Move To </br> Start-Point </p>
      </div>
      <div class="aica-button" id="run-module"
           @click="moveToEndPoint($event)" @touchstart="moveToEndPoint($event)">
        <p> Move To </br> End-Point </p>
      </div>
    </div>
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
    settings: Array,
    properties: Object
  },
  mounted () {
    this.getDatabase()
  },
  beforeUnmount () {
    if (this.isRecording) {
      this.stopRecording()
    }
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
      if ('onlyOneRecording' in this.learning_data.onlyOneRecording &&
          this.learning_data.onlyOneRecording) {
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
          // this.module.numberOfDataPoints = this.database.length
        })
        .catch(error => {
          console.log(error)
          console.log('@ModuleDataList: failure while updating backend.')
        })
    },
    stopRecording (e) {
      if (e !== null && e.type === 'touchstart') {
        e.preventDefault()
      }
      this.isRecording = false
      axios.get(this.$localIP + `/stoprecording`,
                {'params': {}})
        .catch(error => {
          console.log(error)
        })
    },
    stopRobot (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      this.$emit('stopRobot')
    },
    moveToStartPoint (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      if (this.selected === null) {
        this.selected = this.database[0]
      }

      this.$emit('setRobotStateMoving')
      axios.get(this.$localIP + `/movetofirstdatapoint/` + this.module.id + '/' + this.selected.name,
                {'params': {}})
        .then(response => {
          console.log(response.statusText)
          // Finished robot movement - reset to not moving.
          this.$emit('stopRobot')
        })
        .catch(error => {
          console.log(error)
        })
    },
    moveToEndPoint (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }
      if (this.selected === null) {
        this.selected = this.database[0]
      }

      this.$emit('setRobotStateMoving')
      axios.get(this.$localIP + `/movetolastdatapoint/` + this.module.id + '/' + this.selected.name,
                {'params': {}})
        .then(response => {
          console.log(response.statusText)
          // Finished robot movement - reset to not moving.
          this.$emit('stopRobot')
        })
        .catch(error => {
          console.log(error)
        })
    },
    replayData (e) {
      if (e.type === 'touchstart') {
        e.preventDefault()
      }

      // this.moveToStartPoint(e)   // TODO: don't pass event...
      if (this.selected === null) {
        this.selected = this.database[0]
      }
      console.log('Settings')
      let desiredForce
      if ('force' in this.properties) {
        desiredForce = this.properties.force.value
      } else {
        desiredForce = 0
      }

      console.log(this.properties.force.value)
      this.$emit('setRobotStateMoving')
      axios.get(this.$localIP + `/replaydata/` + this.module.id + '/' + this.selected.name,
                {'params': {'force': desiredForce}})
        .then(response => {
          console.log(response.statusText)
          // Finished robot movement - reset to not moving.
          this.$emit('stopRobot')
        })
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
      this.selected = null
    },
    getDatabase () {
      console.log('@ModuleDataList:')
      console.log(this.module.id)

      axios.get(this.$localIP + `/getdataofmodule/` + this.module.id,
                {'params': {}})
        .then(response => {
          this.database = response.data.moduledatabase
          // this.module.numberOfDataPoints = this.database.length
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
    // padding-left: ;
    // padding-right: ;
    display: grid;
    grid-template-columns: auto auto;
}

.reference-button-container{
    display: grid;
    grid-template-columns: auto auto;
    // grid-column-gap: $right;
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
