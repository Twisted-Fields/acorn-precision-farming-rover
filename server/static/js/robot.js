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
