import Vue from 'vue'

import App from './App'
import router from './router'
// import { BootstrapVue, IconsPlugin } from 'bootstrap-vue'

import Buefy from 'buefy'
// import 'buefy/dist/buefy.css'
import 'buefy/dist/buefy.css'

// import BootstrapVue from 'bootstrap-vue'
// import 'bootstrap/dist/css/bootstrap.css'
// import 'bootstrap-vue/dist/bootstrap-vue.css'

// import VueSlider from 'vue-slider-component'
// import 'vue-slider-component/theme/default.css'

Vue.use(Buefy)
// Vue.use(BootstrapVue) // TODO: remove bootstrap

// Create IP as global Variable
Vue.prototype.$localIP = `http://localhost:5000`

Vue.config.productionTip = false

/* eslint-disable no-new */
new Vue({
  el: '#app',
  // (Potentially) change delimiters to make compatible with flask...
  // delimiters: ['[[', ']]'],
  router,
  // vuetify,  // Neded or not?
  render: h => h(App)
})
