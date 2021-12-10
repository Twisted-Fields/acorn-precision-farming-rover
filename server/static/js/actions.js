// actions are triggered by UI events. It mostly involves calling backend API, and as a result, updating the store.

function getRobotData() {
  api("get_herd_data")
    .then((resp) => resp.json())
    .then(function (herd) {
      store.robots = herd;

      //loop through herd data from server
      for (const robot of herd) {
        const date = Date.parse(robot.time_stamp);
        robot.data_age_sec = (new Date() - date) / 1000.0;

        // clear autonomy once on page open?
        if (store.have_cleared_autonomy == false) {
          store.have_cleared_autonomy = true;
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

        plotStats(robot);

        // Set store.simulation value.
        store.simulation = robot.simulated_data;

        if (robot.live_path_data.length > 0) {
          if (robot.live_path_data[0].lat != store.first_path_point) {
            // console.log("UPDATING PATH ")
            // console.log(store.first_path_point)
            store.livePathName = robot.loaded_path_name;
            store.first_path_point = robot.live_path_data[0].lat;
          }
        }

        //console.log(robot.debug_points)

        if (robot.debug_points) {
          store.debugPointsLength = robot.debug_points.length;
        }

        if (store.displayed_dense_path.length > 0) {
        }

        //console.log(robot.gps_path_data.length)

        if (robot.gps_path_data.length != store.gpsPathLength) {
          store.gps_path = robot.gps_path_data;
          store.gpsPathLength = robot.gps_path_data.length;
        }

        var arrow_icon = L.icon({
          iconUrl: "/static/images/arrow.png",
          iconSize: [60, 40],
          iconAnchor: [30, 35],
        });

        var robot_icon = L.icon({
          iconUrl: "/static/images/robot.png",
          iconSize: [30, 40],
          iconAnchor: [15, 20],
        });

        //console.log("Turn intent degrees: ", robot.turn_intent_degrees)
        //check store.robotMarkerStore array if we have marker already
        //check store.robotMarkerStore array if we have marker already
        if (store.arrowMarkerStore.hasOwnProperty(robot.id)) {
          //if we do, set new position attribute to existing marker
          store.arrowMarkerStore[robot.id].setIcon(arrow_icon);
          store.arrowMarkerStore[robot.id].setLatLng(
            new L.latLng(robot.lat, robot.lon)
          );
          store.arrowMarkerStore[robot.id].setRotationAngle(
            robot.turn_intent_degrees + robot.heading
          );
          //console.log(goat_path);
        } else {
          //if we don't, create new marker and set attributes
          var arrow_marker = L.marker([robot.lat, robot.lon], {
            icon: arrow_icon,
          });
          //add new marker to store.robotMarkerStore array
          store.arrowMarkerStore[robot.id] = arrow_marker;
          store.arrowMarkerStore[robot.id].setRotationAngle(
            robot.turn_intent_degrees + robot.heading
          );
          // store.arrowMarkerStore[robot.id].addTo(store.map);
        }

        if (store.robotMarkerStore.hasOwnProperty(robot.id)) {
          //if we do, set new position attribute to existing marker
          store.robotMarkerStore[robot.id].setIcon(robot_icon);
          store.robotMarkerStore[robot.id].setLatLng(
            new L.latLng(robot.lat, robot.lon)
          );
          store.robotMarkerStore[robot.id].setRotationAngle(robot.heading);
          //console.log(goat_path);
        } else {
          //if we don't, create new marker and set attributes
          var marker = L.marker([robot.lat, robot.lon], {
            icon: robot_icon,
          });
          //add new marker to store.robotMarkerStore array
          store.robotMarkerStore[robot.id] = marker;
          store.robotMarkerStore[robot.id].setRotationAngle(robot.heading);
          // store.robotMarkerStore[robot.id].addTo(store.map).on('mouseover', function(e) {
          //   alert("mouseover!")
          // });
        }
      }
    }).catch(function (error) {
      console.log(error);
    });

  if (store.simulation) {
    window.setTimeout(getRobotData, SIMULATION_INTERVAL);
  } else {
    window.setTimeout(getRobotData, INTERVAL);
  }
}

// plotStats plots robot stats on several graphs. The graphs can only be shown
// by clicking "Expand Plots".
function plotStats(robot) {
  var distances = {
    y: robot.gps_distances,
    type: "scatter",
    mode: "lines",
    name: "Distance error m",
  };

  var angles = {
    y: robot.gps_angles,
    type: "scatter",
    mode: "lines",
    name: "Angle error deg",
  };

  var distance_rates = {
    y: robot.gps_distance_rates,
    type: "scatter",
    mode: "lines",
    name: "Distance error rate",
  };

  var angle_rates = {
    y: robot.gps_angle_rates,
    type: "scatter",
    mode: "lines",
    name: "Angle error rate",
  };

  var strafeD = {
    y: robot.strafeD,
    type: "scatter",
    mode: "lines",
    name: "strafeD",
  };

  var steerD = {
    y: robot.steerD,
    type: "scatter",
    mode: "lines",
    name: "steerD",
  };

  var steerP = {
    y: robot.steerP,
    type: "scatter",
    mode: "lines",
    name: "steerP",
  };

  var strafeP = {
    y: robot.strafeP,
    type: "scatter",
    mode: "lines",
    name: "strafeP",
  };

  for (let i = 0; i < robot.autonomy_steer_cmd.length; i++) {
    robot.autonomy_steer_cmd[i] = robot.autonomy_steer_cmd[i] * 30;
  }

  var autonomy_steer_cmd = {
    y: robot.autonomy_steer_cmd,
    type: "scatter",
    mode: "lines",
    name: "autonomy_steer_cmd",
  };

  var autonomy_strafe_cmd = {
    y: robot.autonomy_strafe_cmd,
    type: "scatter",
    mode: "lines",
    name: "autonomy_strafe_cmd",
  };

  var layout_lateral = {
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

  var layout_angle = {
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

  var layout_lateral_rates = {
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

  var layout_angle_rates = {
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

  Plotly.newPlot(
    "plot_div_distance_rates",
    [distances, distance_rates, strafeP, strafeD, autonomy_strafe_cmd],
    layout_lateral_rates
  );
  Plotly.newPlot(
    "plot_div_angle_rates",
    [angles, angle_rates, steerP, steerD, autonomy_steer_cmd],
    layout_angle_rates
  );

  Plotly.newPlot("plot_div_distance", [distances], layout_lateral);
  Plotly.newPlot("plot_div_angles", [angles], layout_angle);
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

function modifyDisplayedPath() {
  if (
    store.path_point_to_remove < store.displayed_path.length &&
    store.path_point_to_remove > 0
  ) {
    store.displayed_path.splice(store.path_point_to_remove, 1);
    store.path_end -= 1;
  }

  if (
    store.path_start > 0 ||
    store.path_end < store.displayed_path.length - 1
  ) {
    length = store.path_end - store.path_start - 1;
    store.displayed_path = store.displayed_path.splice(
      store.path_start,
      length
    );
  }
  store.path_start = 0;
  store.path_end = store.displayed_path.length - 1;
  store.path_point_to_remove = store.displayed_path.length;
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
