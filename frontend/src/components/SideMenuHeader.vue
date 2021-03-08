<template>
<div id='library-header-container' :class='headerActiveMode'>
  <template v-if="isActive">
    <div class="library-header">
      <h1> {{libraryName}}</h1>
    </div>
    <img class="aica-icon-small"
         src='./../assets/icons/keyboard_arrow_down-white-18dp.svg'
         @click="dropDownLibrary($event)"
         @touchstart="dropDownLibrary($event)"
         >
    <img id="buttonCloseLibrariesRight" class="aica-icon-small"
         src='./../assets/icons/keyboard_arrow_right-white-18dp.svg'
         @click="hideLibrarieMenu($event)"
         @touchstart="hideLibrarieMenu($event)"
         >
  </template>
  <template v-else>
  <img class="aica-icon-small" id="buttonOpenLibrariesRight"
       src='./../assets/icons/keyboard_arrow_left-white-18dp.svg'
       @click="showLibrarieMenu($event)"
       @touchstart="showLibrarieMenu($event)"
       >
  </template>
</div>
</template>


<script>
export default {
  name: 'SideMenuHeader',
  props: {
    // libraryName: 'Polishing',
    libraryList: [],
    isActive: false
  },
  data: function () {
    return {
      libraryName: 'Polishing',
      headerActiveMode: 'inactive'
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
    hideLibrarieMenu (e) {
      if (e.type === 'touchend') {
        e.preventDefault()
      }
      this.$emit('update:isActive', false)
      this.headerActiveMode = 'inactive'
    },
    showLibrarieMenu (e) {
      if (e.type === 'touchend') {
        e.preventDefault()
      }
      this.$emit('update:isActive', true)
      this.headerActiveMode = 'active'
    }
  }
}
</script>


<style lang="less" scoped>
@import './../assets/styles/main.less';
// div {
// padding-left: @padding-sideways;
// text-align: left;
// }

#library-header-container{
    display: grid;
    grid-template-columns: auto auto auto;
    margin-top: 0;
}



.library-header {
    // position: relative;
    // left: 20px;
    // left:
    // top:
    height: @height-icon-small;
    width: @height-icon-small*5;

    text-align: left;

    // top: 50%;
    // -ms-transform: translateY(-50%);
    // transform: translateY(-50%);
}

.aica-icon-small {
    display: inline-block;
    position: relative;
    botton: @header-height*0.4;
    // top: 50%;
    // -ms-transform: translateY(-50%);
    // transform: translateY(-50%);
}

#buttonCloseLibraries {
    position: absolute;
    right: @header-padding-sideways;
}

#buttonOpenLibrariesRight {
    position: absolute;
    right: @header-padding-sideways;
}

.inactive {
    // .side-menu-header .inactive {
    background: transparent;
}
</style>
