Vue.component("robot-cardview", {
  props: ["robot"],
  data: function () {
    return {
      velRange: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
      velocity: 0.2,
      gpsCommands: ["Record", "Pause", "Cancel"],
      currentGPSCommand: "Cancel",
    };
  },
  methods: {
    loadPath: function (event) {
      console.log("You clicked the load button ", event.target.innerText);
      updateVehiclePath(store.displayed_path_name, this.robot.name);
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

Vue.component("control-panel", {
  props: ["show-plots", "map", "path-names"],
  data: function () {
    return {
      isDrawingPolygon: false,
      userPolygon: null,
      pathName: "",
    };
  },
  methods: {
    toggleCollapsePlots: function () {
      store.showPlots = !store.showPlots;
    },
    loadDensePath: function () {
      loadDensePath();
    },
    togglePolygon: function () {
      this.isDrawingPolygon = !this.isDrawingPolygon;
      if (this.isDrawingPolygon) {
        this.userPolygon = this.map.editTools.startPolygon();
      } else {
        this.map.editTools.stopDrawing();
        console.log(this.userPolygon.toGeoJSON(10));
        savePolygonToServer("default", this.userPolygon.toGeoJSON(10));
      }
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
      renderPath(store.displayed_path);
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
      renderPath(store.displayed_path);
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
      renderPath(store.displayed_path);
    },
    modifyDisplayedPath: function () {
      modifyDisplayedPath();
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
  props: ["robots", "displayed_path", "displayed_dense_path"],
  components: {
      'l-map': window.Vue2Leaflet.LMap,
      'l-tile-layer': window.Vue2Leaflet.LTileLayer,
      'l-marker': window.Vue2Leaflet.LMarker,
      'l-icon': window.Vue2Leaflet.LIcon,
      'l-polyline': window.Vue2Leaflet.LPolyline,
  },
  updated() {
          // Without this the map would partially show up somehow.
          this.$refs.map.mapObject.invalidateSize();
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
      zoom: 18,
      center: [37.353, -122.332],
      layers: layers,
      arrowIcon: L.icon({
        iconUrl: "/static/images/arrow.png",
        iconSize: [60, 40],
        iconAnchor: [30, 35],
      }),
      robotIcon: L.icon({
        iconUrl: "/static/images/robot.png",
        iconSize: [30, 40],
        iconAnchor: [15, 20],
      }),
    }
  },
});

Vue.component("path-point", {
  props: ["point", "index"],
  template: '<l-circle :lat-lng="point" :options="options"></l-circle>',
  components: {
    'l-circle': window.Vue2Leaflet.LCircle,
  },
  data: function() {
    let color = "#FF6600";
    if (this.index < store.path_start || this.index > store.path_end) {
      color = "#F006FF";
    }
    if (this.index == store.path_point_to_remove) {
      color = "#FF0000";
    }
    return {
      options: {
        radius: 0.3, // in meters
        fillColor: color,
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
        opacity: 0.0,
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
