import Vue from 'vue'

import App from './App'
import router from './router'
// import { BootstrapVue, IconsPlugin } from 'bootstrap-vue'

import Buefy from 'buefy'
import 'buefy/dist/buefy.css'

import BootstrapVue from 'bootstrap-vue'
import 'bootstrap/dist/css/bootstrap.css'
import 'bootstrap-vue/dist/bootstrap-vue.css'

// import VueSlider from 'vue-slider-component'
// import 'vue-slider-component/theme/default.css'

// Vuetify
// import vuetify from '@/plugins/vuetify' // path to vuetify export
// import Vuetify from 'vuetify'
// import 'vuetify/dist/vuetify.min.css'

// Vue.component('VueSlider', VueSlider) // TODO: remove?
Vue.use(Buefy)
Vue.use(BootstrapVue) // TODO: remove bootstrap
// Vue.use(Vuetify)

Vue.config.productionTip = false

// Create IP as global Variable
Vue.prototype.$localIP = `http://localhost:5000/`

/* eslint-disable no-new */
new Vue({
  el: '#app',
  // (Potentially) change delimiters to make compatible with flask...
  // delimiters: ['[[', ']]'],
  router,
  // vuetify,  // Neded or not?
  render: h => h(App)
})
