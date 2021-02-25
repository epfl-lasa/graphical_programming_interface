<template>
  <div id="vuemodulelibrary" class="module-library">
    <!-- <h1> Library : {{ loadedLibrary }} </h1> -->
    <div class="library" v-for="mod in module_list">
      <div class="icon-button" v-on:click="addModule(mod.type)">
      <!-- <div class="icon-button"> -->
        <img v-bind:src="require('./../assets/images/'+mod.iconpath)" v-bind:alt=mod.name>
		  </div>
      <p class="icon-name"> {{mod.title}} </p>
    </div>
  </div>
</template>

<script>
export default {
  name: 'VueModuleLibrary',
  props: {
    modules: {
      type: Object,
      default: {}
    },
    blockContent: Array,
    loadedLibrary: String
  },
  data: function () {
    return {
      foo: null
    }
  },
  computed: {
    module_list () {
      if (!(this.loadedLibrary in this.modules)) {
        // TODO: only load after python return
        console.log('@VueModuleLibrary: Library not found')
        return []
      }

      // console.log('Active library')
      // console.log(this.loadedLibrary)
      // console.log('blockContent')
      // console.log(this.blockContent)
      const blockContent = this.blockContent.filter(
        block => (block.library === this.loadedLibrary)
      )
      return blockContent
    }
  },
  methods: {
    addModule (module) {
      console.log('@VueModuleLibrary: Want to add a module <<' + module + '>>')
      this.$parent.addModule(module)
      // Why does emit not work; (how to make it work..)
      // this.$emit('addModule', module)
    },
    save () {
      this.$emit('save', this.properties)
    }
  }
  // watch: {
  // }
}
</script>

<style lang="less" scoped>
.icon-button {
    display: inline;
}

.icon-name {
    display: inline;
    font-size: 30px;
}

.module-library {
    position: absolute;
    right: 0;
    top: 60px;

    width: 300px;
    min-height: 700px;
    box-sizing: border-box;
    padding: 10px;

    background: #87adc4;
    border: 1px solid #000000;

    .property {

    }
  }
</style>
