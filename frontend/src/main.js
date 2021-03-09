import Vue from 'vue'

// TODO: really needed (?!)
import Buefy from 'buefy'
import 'buefy/dist/buefy.css'

import App from './App'
import router from './router'

// import App from './../../src/frontend/App'
// import router from './../../src/frontend/router'

Vue.use(Buefy)

// Get relative direcotries (to seperate npm & source) // TODO in future
// Vue.prototype.$relativeSrcDir = `./../../src/frontend/`
// Vue.prototype.$relativeNodeDir = `./../../frontend/`

// Create IP as global Variable
// Vue.prototype.$localIP = `http://localhost:5000`
Vue.prototype.$localIP = `http://192.168.1.102:5000`

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
