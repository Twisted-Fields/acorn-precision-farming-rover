// actions are triggered by UI events. It mostly involves calling backend API, and as a result, updating the store.

function updateHerdData(herd) {
  //loop through herd data from server
  for (const robot of herd) {
    const date = Date.parse(robot.time_stamp);
    robot.data_age_sec = (new Date() - date) / 1000.0;
    Vue.set(store.robots, robot.name, robot);

    // clear autonomy once on page open?
    if (!store.have_cleared_autonomy[robot.name]) {
      Vue.set(store.have_cleared_autonomy,robot.name, true);
      modifyAutonomyHold(`${robot.name}`, false);
    }

    // if(robot.autonomy_hold)
    // {
    //   let speed = 0.0;
    //   let active = false;
    //   updateVehicleAutonomy(`${robot.name}`, speed, active);
    // }
    //console.log(robot.autonomy_hold)
    //console.log(robot.strafeD)

    if (robot.gps_path_data.length != store.gpsPathLength) {
      store.gps_path = robot.gps_path_data;
      store.gpsPathLength = robot.gps_path_data.length;
    }
  }
}

function loadPath(pathname) {
  //request circle data from server API
  api("get_path/" + pathname)
    .then((resp) => resp.json())
    .then(function (pathData) {
      console.log(pathData);
      store.displayed_path = pathData;
      store.path_start = 0;
      store.path_end = store.displayed_path.length - 1;
      store.path_point_to_remove = store.displayed_path.length;

      console.log("pathData Length: ", pathData.length);
      store.displayed_path_name = pathname;
    }).catch(function (error) {
      console.log(error);
    });
}

function loadDensePath() {
  //request circle data from server API
  api("get_dense_path")
    .then((resp) => resp.json())
    .then(function (densePathData) {
      console.log(densePathData);
      store.displayed_dense_path = densePathData;
      // store.path_start = 0;
      // store.path_end = store.displayed_path.length-1;
      // store.path_point_to_remove =  store.displayed_path.length;
      //
      // console.log("pathData Length: ", pathData.length)
      //store.displayed_path_name = pathname
    }).catch(function (error) {
      console.log(error);
    });
}

function updateServerPath(pathname, pathData) {
  api("save_path/" + pathname, {
    method: "POST", // or 'PUT'
    body: JSON.stringify(pathData), // data can be `string` or {object}!
    headers: {
      "Content-Type": "application/json",
    }})
    .then(function (reply) {
      console.log(reply);
    }).catch(function (error) {
      console.log(error);
    });
}

function savePolygonToServer(polygonName, polygonData) {
  api("save_polygon/" + polygonName, {
    method: "POST", // or 'PUT'
    body: JSON.stringify(polygonData), // data can be `string` or {object}!
    headers: {
      "Content-Type": "application/json",
    }})
    .then(function (reply) {
      console.log(reply);
    }).catch(function (error) {
      console.log(error);
    });
}

function deletePath(pathname) {
  api("delete_path/" + pathname)
    .then(function (reply) {
      console.log(reply);
    }).catch(function (error) {
      console.log(error);
    });
}

function updateVehiclePath(pathname, vehicle_name) {
  api("set_vehicle_path/" + pathname + "/" + vehicle_name)
    .then(function (reply) {
      console.log(reply);
    }).catch(function (error) {
      console.log(error);
    });
}

function modifyAutonomyHold(vehicle_name, clear_hold) {
  api("modify_autonomy_hold/" + vehicle_name + "/" + clear_hold)
    .then(function (reply) {
      console.log(reply);
    }).catch(function (error) {
      console.log(error);
    });
}

function updateVehicleAutonomy(vehicle_name, speed, enabled) {
  api("set_vehicle_autonomy/" + vehicle_name + "/" + speed + "/" + enabled)
    .then(function (reply) {
      //console.log(reply);
    }).catch(function (error) {
      console.log(error);
    });
}

function updateGpsRecordCommand(vehicle_name, record_gps_command) {
  api("set_gps_recording/" + vehicle_name + "/" + record_gps_command)
    .then(function (reply) {
      console.log(reply);
    }).catch(function (error) {
      console.log(error);
    });
}

function loadPathList() {
  api("get_path_names")
    .then((resp) => resp.json())
    .then(function (pathnames) {
      store.pathNames = pathnames
    }).catch(function (error) {
      console.log(error);
    });
}

const apiPrefix = `http://${location.hostname}/api/`;
console.log("all API calls will hit " + apiPrefix);
function api(path, init) {
  return fetch(apiPrefix + path, init);
}
