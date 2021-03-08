<template>
<div id="vuemodulelibrary" class="module-library">
  <div class="grid-container body" >
    <div class="library" v-for="mod in module_list">
      <div class="icon-module"
           @click="addModule($event, mod.type)"
           @touchstart="addModule($event, mod.type)"
           >
        <div class="icon-container">
          <img v-bind:src="require('./../assets/icons_library/'+mod.iconpath)"
               v-bind:alt=mod.name>
        </div>
        <h2 class="icon-name"> {{mod.title}} </h2>
		  </div>
    </div>
  </div>
</div>
</template>

<script>
export default {
  name: 'VueModuleLibrary',
  // components: {
  // },
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
      isActive: false
    }
  },
  computed: {
    module_list () {
      if (!(this.loadedLibrary in this.modules)) {
        // TODO: only load after python return
        console.log('@VueModuleLibrary: Library not found')
        console.log(this.loadedLibrary)

        return []
      }

      // console.log('blockContent')
      // console.log(this.blockContent)
      const blockContent = this.blockContent.filter(
        block => (block.library === this.loadedLibrary)
      )
      return blockContent
    }
  },
  methods: {
    addModule (e, module) {
      if (e.type === 'touchend') {
        e.preventDefault()
      }
      this.$emit('addModule', module)
    },
    hideLibrary () {
      this.$emit('hideLibrary')
    },
    save () {
      this.$emit('save', this.properties)
    }
  }
}
</script>

<style lang="less" scoped>
@import './../assets/styles/main.less';

.module-library {
    text-align: center;

    .grid-container {
        // padding: 10px;
        display: inline-grid;
        grid-template-columns: auto auto;

        column-gap: 50px;
        row-gap: 30px;

        padding: 10px;
        // text-align: center;
    }
    .property {
    }
}

.icon-module {
    // text-align: center;
    cursor: pointer;
    background: @color-main-mediumbright;

    .icon-container{
        background: @color-main-medium;
        width: @block-width;
        height: @block-height;
    }

    .icon-name {
        // display: inline;
        font-size: @fontsize-small;
        margin-top: 1px;
    }
}
</style>
