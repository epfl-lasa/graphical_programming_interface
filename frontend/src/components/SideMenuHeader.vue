<template>
<div id='library-header-container'>
  <div class="library-header">
    <h1> {{libraryName}}</h1>
  </div>
  <img class="aica-icon-small"
       src='./../assets/icons/keyboard_arrow_down-white-18dp.svg'
       @click="dropDownLibrary($event)"
       @touchstart="dropDownLibrary($event)"
       >
  <img id="buttonCloseLibraries" class="aica-icon-small"
       src='./../assets/icons/keyboard_arrow_right-white-18dp.svg'
       @click="hideLibrarieMenu($event)"
       @touchstart="hideLibrarieMenu($event)"
       >
</div>
</template>


<script>
export default {
  name: 'SideMenuHeader',
  props: {
    // libraryName: 'Polishing',
    libraryList: []
  },
  data: function () {
    return {
      libraryName: 'Polishing',
      foo: null
    }
  },
  methods: {
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

      console.log('@@SideMenuHeader: TODO hide')
      // this.$emit('hideLibrary')
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

</style>
