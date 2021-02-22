<template>
  <svg width="100%" height="100%">
    <!-- <g v-for="p in renderedPathes" > -->
      <!-- <g v-for="(p, index) in straightPaths" -->
    <g v-for="p in straightPaths"
       @mousedown="lineMouseDown($event, p, p.id)"
       @mouseup="lineMouseUp($event, p)"
       >
      <path v-if="outline" :d="p.data" :style="p.outlineStyle"></path>
      <path :d="p.data" :style="p.style"></path>
    </g>
    <g>
      <path v-for="a in straightArrows"
            d="M -1 -1 L 0 1 L 1 -1 z"
            :style="a.style"
            :transform="a.transform"
            @mousedown="lineMouseDown($event, a, a.id)"
            @mouseup="lineMouseUp($event, a)"
            ></path>
    </g>
  </svg>
</template>


<script>
export default {
  props: {
    lines: {
      type: Array,
      default () {
        return []
      }
    },
    outline: {
      type: Boolean,
      default: false
    },
    // linking: false,
    tempLinkActive: true
  },
  beforeDestroy () {
    // The whole container is still kept.
    console.log('Destroy all')
  },
  data () {
    return {
      // Timer for longpress
      presstimer: null,
      longpress: false
    }
  },
  methods: {
    lineMouseDown (e, line, lineID) {
      if (lineID === undefined) {
        // Temp line
        console.log('Undefined line')
        return 1
      } else {
        let linePosition = this.lines.find(element => element.id === lineID)

        if (linePosition === undefined) {
          console.log('@LinkCreator: Line not found in list')
          return 2
        }
      }

      this.$emit('updateSelectedLine', lineID)
      this.presstimer = setTimeout(this.lineMouseDownLong, 300, e)
    },
    lineMouseDownLong (e) {
      console.log('@LinkCreator: hLong press YES')
      self.longpress = true
      this.showAppDropwdown = true
      this.$emit('showDropdownkMenu', e, 'line')
    },
    lineMouseUp (e) {
      // console.log('@LinkCreator: Mouse Up on line')
      this.cancelLongPress()
    },
    cancelLongPress () {
      if (this.presstimer !== null) {
        clearTimeout(this.presstimer)
        this.presstimer = null
        this.longpress = false
      }
    },
    updateLineEndPoints () {
      if (!this.lines) {
        return []
      }
      console.log('Not yet implemented') // TODO
    },
    getDistance (x1, y1, x2, y2) {
      return Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))
    },
    computeConnectionPoint (x1, y1, x2, y2, t) {
      let dist = this.getDistance(x1, y1, x2, y2)
      let p0 = {x: x1, y: y1}
      let p1 = {x: x1 + dist * 0.25, y: y1}
      let p2 = {x: x2 - dist * 0.25, y: y2}
      let p3 = {x: x2, y: y2}

      let c1 = (1 - t) * (1 - t) * (1 - t)
      let c2 = 3 * ((1 - t) * (1 - t)) * t
      let c3 = 3 * (1 - t) * (t * t)
      let c4 = t * t * t

      let x = c1 * p0.x + c2 * p1.x + c3 * p2.x + c4 * p3.x
      let y = c1 * p0.y + c2 * p1.y + c3 * p2.y + c4 * p3.y
      return {x: x, y: y}
    }
  },
  computed: {
    linkListID () {
      let listID = []
      let ii = 0
      this.lines.forEach(l => {
        listID.push(ii)
        ii = ii + 1
      })
      return listID
    },
    straightPaths () {
      if (!this.lines) {
        return []
      }

      let paths = []
      this.lines.forEach(l => {
        // let dist = this.getDistance(l.x1, l.y1, l.x2, l.y2) * 0.25
        paths.push({
          id: l.id,
          data: `M ${l.x1}, ${l.y1}, ${l.x2}, ${l.y2}`,
          style: l.style,
          outlineStyle: l.outlineStyle
        })
      })
      return paths
    },
    straightArrows () {
      // The arrows on straight link between a & b
      if (!this.lines) {
        return []
      }

      let arrows = []
      this.lines.forEach(l => {
        let pos = this.computeConnectionPoint(l.x1, l.y1, l.x2, l.y2, 0.5)

        let angle = -Math.atan2(l.x2 - l.x1, l.y2 - l.y1)
        let degrees = (angle >= 0 ? angle : (2 * Math.PI + angle)) * 180 / Math.PI

        arrows.push({
          id: l.id,
          transform: `translate(${pos.x}, ${pos.y}) rotate(${degrees})`,
          style: {
            stroke: l.style.stroke,
            strokeWidth: l.style.strokeWidth * 4,
            fill: l.style.stroke
          }
        })
      })
      return arrows
    },
    renderedPathes () {
      // Depreciated --- keep for idea how to generate nice paths
      if (!this.lines) {
        return []
      }

      let pathes = []
      this.lines.forEach(l => {
        let dist = this.distance(l.x1, l.y1, l.x2, l.y2) * 0.25
        pathes.push({
          data: `M ${l.x1}, ${l.y1} C ${(l.x1 + dist)}, ${l.y1}, ${(l.x2 - dist)}, ${l.y2}, ${l.x2}, ${l.y2}`,
          style: l.style,
          outlineStyle: l.outlineStyle
        })
      })

      return pathes
    },
    renderedArrows () {
      // Depreciated --- keep for idea how to generate nice paths
      if (!this.lines) {
        return []
      }

      let arrows = []
      this.lines.forEach(l => {
        // l.x1, l.y1, l.x2, l.y2
        let pos = this.computeConnectionPoint(l.x1, l.y1, l.x2, l.y2, 0.5)
        let pos2 = this.computeConnectionPoint(l.x1, l.y1, l.x2, l.y2, 0.51)

        let angle = -Math.atan2(pos2.x - pos.x, pos2.y - pos.y)
        let degrees = (angle >= 0 ? angle : (2 * Math.PI + angle)) * 180 / Math.PI

        arrows.push({
          transform: `translate(${pos.x}, ${pos.y}) rotate(${degrees})`,
          style: {
            stroke: l.style.stroke,
            strokeWidth: l.style.strokeWidth * 2,
            fill: l.style.stroke
          }
        })
      })

      return arrows
    }
  }
}
</script>

<style scoped>

</style>
