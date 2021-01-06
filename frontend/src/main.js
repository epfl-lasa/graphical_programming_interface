import Vue from 'vue'
import BootstrapVue from 'bootstrap-vue'
import App from './App'
import router from './router'
// import { BootstrapVue, IconsPlugin } from 'bootstrap-vue'

import 'bootstrap/dist/css/bootstrap.css'
import 'bootstrap-vue/dist/bootstrap-vue.css'

// Vuetify
// import vuetify from '@/plugins/vuetify' // path to vuetify export

Vue.config.productionTip = false

// Install BootstrapVue
Vue.use(BootstrapVue)

// Global Variable
Vue.prototype.$localIP = `http://localhost:5000/`

/* eslint-disable no-new */
new Vue({
  el: '#app',
  // (Potentially) change delimiters to make compatible with flask...
  // delimiters: ['[[', ']]'],
  router,
  // vuetify,
  render: h => h(App)
})
