import Vue from 'vue'

import App from './App'
import router from './router'

import Buefy from 'buefy'
import 'buefy/dist/buefy.css'

Vue.use(Buefy)

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
