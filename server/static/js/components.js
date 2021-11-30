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
      store.livePathName = this.pathName = name;
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
  props: ["show-plots"],
});
