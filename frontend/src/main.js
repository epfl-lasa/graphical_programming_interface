import Vue from 'vue'
import BootstrapVue from 'bootstrap-vue'
import App from './App'
import router from './router'
// import { BootstrapVue, IconsPlugin } from 'bootstrap-vue'

import 'bootstrap/dist/css/bootstrap.css'
import 'bootstrap-vue/dist/bootstrap-vue.css'

Vue.config.productionTip = false

// Install BootstrapVue
Vue.use(BootstrapVue)

/* eslint-disable no-new */
new Vue({
  el: '#app',
  // (Potentially) change delimiters to make compatible with flask...
  // delimiters: ['[[', ']]'],
  router,
  render: h => h(App)
})
