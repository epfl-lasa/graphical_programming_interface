<template>
<div id="vuemodulelibrary" class="module-library side-menu">
  <SideMenuHeader
    ref="libraryHeader"
    class="side-menu-header"
    />
  <div class="side-menu-body">
    <div class="grid-container body" >
      <!-- <h1> Library : {{ loadedLibrary }} </h1> -->
      <div class="library" v-for="mod in module_list">
        <div class="icon-module" v-on:click="addModule(mod.type)">
          <div class="icon-container">
            <img v-bind:src="require('./../assets/icons_library/'+mod.iconpath)"
               v-bind:alt=mod.name>
          </div>
		    </div>
        <h2 class="icon-name"> {{mod.title}} </h2>
      </div>
    </div>
  </div>
</div>
</template>

<script>
import SideMenuHeader from './SideMenuHeader'

export default {
  name: 'VueModuleLibrary',
  components: {
    SideMenuHeader
  },
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
@import './../assets/styles/main.less';

// .grid-item {
// background-color: rgba(255, 255, 255, 0.8);
// border: 1px solid rgba(0, 0, 0, 0.8);
// padding: 20px;
// font-size: 30px;
// text-align: center;
// }

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
    .icon-module{
        background: @color-main-mediumbright;
    }

    .icon-container{
        background: @color-main-medium;
        width: @block-width;
        height: @block-height;
    }

    .icon-name {
        display: inline;
        font-size: 30px;
    }
}
</style>
