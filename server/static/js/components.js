Vue.component("control-panel", {
  props: ["show-plots", "map", "path-names"],
  data: function () {
    return {
      isDrawingPolygon: false,
      pathName: "",
    };
  },
  methods: {
    loadDensePath: function () {
      loadDensePath();
    },
    togglePolygon: function () {
      this.isDrawingPolygon = store.drawing_polygon = !store.drawing_polygon;
    },
    saveGPSPath: function () {
      if (!this.pathName) {
        return;
      }
      updateServerPath(this.pathName, store.gps_path);
      console.log(store.gps_path);
    },
    loadPathList: function () {
      loadPathList();
    },
    selectPath: function (name) {
      this.pathName = name;
      loadPath(name);
    },
    deletePath: function () {
      deletePath(this.pathName);
    },
    saveAs: function () {
      if (!this.pathName) {
        return;
      }
      updateServerPath(this.pathName, store.displayed_path);
      console.log(store.displayed_path);
    },
    incStart: function (n) {
      store.path_start += n;
      if (store.path_start < 0) {
        store.path_start = 0;
      }
      const max = store.displayed_path.length - 1;
      if (store.path_start > max) {
        store.path_start = max;
      }
    },
    incEnd: function (n) {
      store.path_end += n;
      if (store.path_end < 0) {
        store.path_end = 0;
      }
      const max = store.displayed_path.length - 1;
      if (store.path_end > max) {
        store.path_end = max;
      }
    },
    incRemove: function (n) {
      store.path_point_to_remove += n;
      if (store.path_point_to_remove < 0) {
        store.path_point_to_remove = 0;
      }
      const max = store.displayed_path.length - 1;
      if (store.path_point_to_remove > max) {
        store.path_point_to_remove = max;
      }
    },
    modifyDisplayedPath: function () {
      if (store.path_point_to_remove < store.displayed_path.length &&
          store.path_point_to_remove > 0) {
        store.displayed_path.splice(store.path_point_to_remove, 1);
        store.path_end -= 1;
      }

      if (store.path_start > 0 || store.path_end < store.displayed_path.length - 1) {
        length = store.path_end - store.path_start - 1;
        store.displayed_path = store.displayed_path.splice(
          store.path_start, length);
      }
      store.path_start = 0;
      store.path_end = store.displayed_path.length - 1;
      store.path_point_to_remove = store.displayed_path.length;
    },
  },
});

Vue.component("robot-cardview", {
  props: ["robot", "showPlots"],
  data: function () {
    return {
      velRange: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
      velocity: 0.2,
      gpsCommands: ["Record", "Pause", "Clear"],
      currentGPSCommand: "Cancel",
    };
  },
  methods: {
    loadPath: function (event) {
      console.log("You clicked the load button ", event.target.innerText);
      updateVehiclePath(store.displayed_path_name, this.robot.name);
    },
    toggleShowPlots: function () {
      store.showPlots = !store.showPlots;
    },
    setVelocity: function (event) {
      console.log("velocity is set to ", event.target.innerText);
      this.velocity = event.target.innerText;
    },
    toggleAutonomy: function (event) {
      if (this.robot.autonomy_hold) {
        console.log("clicked button but autonomy not allowed");
        return;
      }
      console.log(
        `${event.target.innerText} for ${this.robot.name} with velocity ${this.velocity}`
      );
      updateVehicleAutonomy(
        this.robot.name,
        this.velocity,
        !this.robot.activate_autonomy
      );
    },
    clearAutonomyHold: function () {
      modifyAutonomyHold(this.robot.name, true);
    },
    gpsAction: function (event) {
      const cmd = event.target.innerText;
      this.currentGPSCommand = cmd;
      updateGpsRecordCommand(this.robot.name, cmd);
    },
  },
});

Vue.component("plots", {
  props: ["robot"],
  components: { 'plotly': window['vue-plotly'].Plotly },
  data: function() {
    const distances = {
      y: this.robot.gps_distances,
      type: "scatter",
      mode: "lines",
      name: "Distance error m",
    };

    const angles = {
      y: this.robot.gps_angles,
      type: "scatter",
      mode: "lines",
      name: "Angle error deg",
    };

    const distance_rates = {
      y: this.robot.gps_distance_rates,
      type: "scatter",
      mode: "lines",
      name: "Distance error rate",
    };

    const angle_rates = {
      y: this.robot.gps_angle_rates,
      type: "scatter",
      mode: "lines",
      name: "Angle error rate",
    };

    const strafeD = {
      y: this.robot.strafeD,
      type: "scatter",
      mode: "lines",
      name: "strafeD",
    };

    const steerD = {
      y: this.robot.steerD,
      type: "scatter",
      mode: "lines",
      name: "steerD",
    };

    const steerP = {
      y: this.robot.steerP,
      type: "scatter",
      mode: "lines",
      name: "steerP",
    };

    const strafeP = {
      y: this.robot.strafeP,
      type: "scatter",
      mode: "lines",
      name: "strafeP",
    };

    for (let i = 0; i < this.robot.autonomy_steer_cmd.length; i++) {
      this.robot.autonomy_steer_cmd[i] = this.robot.autonomy_steer_cmd[i] * 30;
    }

    const autonomy_steer_cmd = {
      y: this.robot.autonomy_steer_cmd,
      type: "scatter",
      mode: "lines",
      name: "autonomy_steer_cmd",
    };

    const autonomy_strafe_cmd = {
      y: this.robot.autonomy_strafe_cmd,
      type: "scatter",
      mode: "lines",
      name: "autonomy_strafe_cmd",
    };

    const layout_lateral = {
      title: {
        text: "Lateral Error",
        font: {
          family: "Courier New, monospace",
          size: 24,
        },
        xref: "paper",
        x: 0.05,
      },
    };

    const layout_angle = {
      title: {
        text: "Angle Error",
        font: {
          family: "Courier New, monospace",
          size: 24,
        },
        xref: "paper",
        x: 0.05,
      },
    };

    const layout_lateral_rates = {
      title: {
        text: "Lateral Error Rates",
        font: {
          family: "Courier New, monospace",
          size: 24,
        },
        xref: "paper",
        x: 0.05,
      },
    };

    const layout_angle_rates = {
      title: {
        text: "Angular Error Rates",
        font: {
          family: "Courier New, monospace",
          size: 24,
        },
        xref: "paper",
        x: 0.05,
      },
    };

    return {plots: [
      {
        data: [distances, distance_rates, strafeP, strafeD, autonomy_strafe_cmd],
        layout: layout_lateral_rates,
        height: '300px',
      },
      {
        data: [angles, angle_rates, steerP, steerD, autonomy_steer_cmd],
        layout: layout_angle_rates,
        height: '300px',
      },
      {
        data: [distances],
        layout: layout_lateral,
        height: '500px',
      },
      {
        data: [angles],
        layout: layout_angle,
        height: '500px',
      },
    ]}
  },
});

Vue.component("map-canvas", {
  props: ["robots", "current_robot_index",
          "displayed_path", "displayed_dense_path", "drawing_polygon",
          "path_start", "path_end", "path_point_to_remove"],
  components: {
      'l-map': window.Vue2Leaflet.LMap,
      'l-tile-layer': window.Vue2Leaflet.LTileLayer,
      'l-marker': window.Vue2Leaflet.LMarker,
      'l-icon': window.Vue2Leaflet.LIcon,
      'l-polyline': window.Vue2Leaflet.LPolyline,
  },
  mounted() {
    // Without this the map would partially show up somehow.
    this.$nextTick(() => {
      this.$refs.map.mapObject.invalidateSize();
    })
  },
  updated() {
    // this ugly trick is required because the button to start/stop polygon
    // resides in control panel instead of this component.
    const map = this.$refs.map.mapObject;
    if (this.drawing_polygon) {
      if (!this.userPolygon) {
        this.userPolygon = map.editTools.startPolygon();
      }
    } else if (this.userPolygon) {
      map.editTools.stopDrawing();
      console.log(this.userPolygon.toGeoJSON(10));
      savePolygonToServer("default", this.userPolygon.toGeoJSON(10));
      this.userPolygon = null;
    }
  },
  methods: {
    selectRobot(index) {
      if (store.current_robot_index == index) {
        store.current_robot_index = null
      } else {
        store.current_robot_index = index
      }
    },
    robotIcon(index) {
      return  L.icon({
        iconUrl: "/static/images/robot.png",
        iconSize: [30, 40],
        iconAnchor: [15, 20],
        className: this.current_robot_index === index ? "selected-acorn": null,
      })
    },
  },
  data: function() {
    const mapbox_layer = {
      url: "https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token={accessToken}",
      options: {
        attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors, Imagery © <a href="https://www.mapbox.com/">Mapbox</a>',
        maxZoom: 22,
        id: "mapbox/satellite-v9",
        tileSize: 512,
        zoomOffset: -1,
        accessToken: "pk.eyJ1IjoidHdpc3RlZGZpZWxkcyIsImEiOiJja2ozbmtlOXkwM2ZmMzNueTEzcGxhMGR1In0.eAhUMfZ786vm7KOhbrJj2g",
      },
    };
    let layers = {Mapbox: mapbox_layer};
    if (store.access_token_data) {
      const open_drone_map_layer = {
        url: "http://192.168.1.170:8090/api/projects/3/tasks/3116cce4-4215-4de9-9e9a-0e9c93df87f6/orthophoto/tiles/{z}/{x}/{y}.png?jwt={accessToken}",
        options: {
          attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors, Imagery © <a href="https://www.twistedfields.com/">Twisted Fields</a>',
          maxZoom: 22,
          tileSize: 256,
          zoomOffset: 0,
          id: "",
          accessToken: store.access_token_data["token"],
          tms: false,
        },
      };
      layers["Drone Map"] = open_drone_map_layer;
    }
    return {
      selectedRobot: null,
      userPolygon: null,
      mapOptions: {
        zoom: 18,
        center: [37.353, -122.332],
        editable: true,
      },
      layers: layers,
      arrowIcon: L.icon({
        iconUrl: "/static/images/arrow.png",
        iconSize: [60, 40],
        iconAnchor: [30, 35],
      }),
    }
  },
});

Vue.component("path-point", {
  props: ["point", "index", "path_start", "path_end", "path_point_to_remove"],
  template: '<l-circle :lat-lng="point" :fill-color="fillColor" :options="options"></l-circle>',
  components: {
    'l-circle': window.Vue2Leaflet.LCircle,
  },
  computed: {
    fillColor: function() {
      let color = "#FF6600";
      if (this.index < this.path_start || this.index > this.path_end) {
        color = "#F006FF";
      }
      if (this.index == this.path_point_to_remove) {
        color = "#FF0000";
      }
      return color
    },
  },
  data() {
    return {
      options: {
        radius: 0.3, // in meters
        fillOpacity: 0.3,
        color: "#FFF",
        weight: 1,
        interactive: false,
      },
    }
  },
});


Vue.component("debug-path-point", {
  props: ["point", "index"],
  template: '<l-circle :lat-lng="point" :options="options"></l-circle>',
  components: {
    'l-circle': window.Vue2Leaflet.LCircle,
  },
  data: function() {
    let color = "#FFFFFF";
    if (this.index > 1) {
      color = "#00FFFF";
    }
    return {
      options: {
        radius: 0.2, // in meters
        fillColor: color,
        fillOpacity: 1.0,
        color: "#FFF",
        weight: 1,
        interactive: false,
      },
    }
  },
});


Vue.component("gps-point", {
  props: ["pt"],
  template: '<l-polygon :lat-lngs="coords" :options="options"></l-polygon>',
  components: {
    'l-polygon': window.Vue2Leaflet.LPolygon,
  },
  data: function() {
    const offset = 0.000025;
    return {
      options: {
        color: "#00FF00",
        opacity: 1.0,
        weight: 2,
        fillColor: "#FFA500",
        fillOpacity: 1.0,
      },
      coords: [
        [this.pt.lat + this.offset * 0.7, this.pt.lon - this.offset],
        [this.pt.lat - this.offset * 0.7, this.pt.lon],
        [this.pt.lat + this.offset * 0.7, this.pt.lon + this.offset],
        [this.pt.lat + this.offset * 0.7, this.pt.lon - this.offset],
      ],
    }
  },
});
