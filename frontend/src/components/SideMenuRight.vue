<template>
<div class="root-container">
  <div v-if="isActive" class="side-menu">
    <div  class="side-menu-header">
      <div class="title-box">
        <h1 > {{libraryName}}</h1>
      </div>
      <img class="aica-icon-small"
             src='./../assets/icons/keyboard_arrow_down-white-18dp.svg'
             @click="dropDownLibrary($event)"
             @touchstart="dropDownLibrary($event)"
             >
        <img id="buttonCloseLibrariesRight" class="aica-icon-small"
             src='./../assets/icons/keyboard_arrow_right-white-18dp.svg'
             @click="hideMenu($event)"
             @touchstart="hideMenu($event)"
             >
    </div>
    <div class="property-panel side-menu-body">
      <VueModuleLibrary
        :libraryName="libraryName"
        :loadedLibrary="loadedLibrary"
        :modules="modules"
        :blockContent="blockContent"
        @addModule="addModule"
        />
    </div>
  </div>

  <div v-else class="side-menu-button-container">
    <img class="aica-icon-small"
         src='./../assets/icons/keyboard_arrow_left-white-18dp.svg'
         @click="showMenu($event)"
         @touchstart="showMenu($event)"
         >
  </div>
</div>
</template>


<script>
import VueModuleLibrary from './VueModuleLibrary'

export default {
  name: 'SideMenuHeader',
  components: {
    VueModuleLibrary
  },
  props: {
    loadedLibrary: String,
    libraryList: [],
    modules: Object,
    blockContent: Array
  },
  data: function () {
    return {
      libraryName: 'Polishing',
      headerActiveMode: 'inactive',
      isActive: false
    }
  },
  methods: {
    toggleMenu (e) {
      if (e.type === 'touchend') {
        e.preventDefault()
      }
      if (this.isActive) {
        this.$emit('update:isActive', false)
      } else {

      }
    },
    dropDownLibrary (e) {
      if (e.type === 'touchend') {
        e.preventDefault()
      }
      console.log('@@SideMenuHeader: TODO hide')
    },
    hideMenu (e) {
      if (e.type === 'touchend') {
        e.preventDefault()
      }
      this.isActive = false
    },
    showMenu (e) {
      if (e.type === 'touchend') {
        e.preventDefault()
      }
      this.isActive = true
    },
    addModule (module) {
      this.$emit('addModule', module)
    }
  }
}
</script>


<style lang="less" scoped>
@import './../assets/styles/main.less';
#library-header-container{
    display: grid;
    grid-template-columns: auto auto auto;
    margin-top: 0;
}

.library-header {
    height: @height-icon-small;
    width: @height-icon-small*5;

    text-align: left;
}

.aica-icon-small {
    display: inline-block;
    position: relative;
    botton: @header-height*0.4;
}

.title-box {
    width: @sidebar-width*0.7;
}

#buttonCloseLibrariesRight {
    position: absolute;
    top: @header-height*0.2;
    right: @header-padding-sideways;
}

.side-menu-button-container {
    position: absolute;
    top: @header-height*0.2;
    right: @header-padding-sideways;
    z-index: 3;
}

</style>
