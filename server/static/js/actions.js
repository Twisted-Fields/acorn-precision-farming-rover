// actions are triggered by UI events. It mostly involves calling backend API, and as a result, updating the store.

// <------- THIS FUNCTION GETS ROBOT DATA FROM SERVER AND UPDATES MARKERS AT INTERVAL ------->
function getRobotData() {
  //request herd data from server API
  api("get_herd_data")
    .then((resp) => resp.json())
    .then(function (herd) {
      //loop through herd data from server
      for (const robot of herd) {
        const date = new Date(robot.time_stamp);
        const data_age_sec = (new Date() - date) / 1000.0;
        robot.data_age_sec = data_age_sec;

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

        plot(robot);

        // Set store.simulation value.
        store.simulation = robot.simulated_data;

        if (robot.live_path_data.length > 0) {
          if (robot.live_path_data[0].lat != store.first_path_point) {
            // console.log("UPDATING PATH ")
            // console.log(store.first_path_point)
            renderPathLive(robot.live_path_data);
            store.livePathName = robot.loaded_path_name;
            store.first_path_point = robot.live_path_data[0].lat;
          }
        }

        //console.log(robot.debug_points)

        if (robot.debug_points) {
          renderPathDebug(robot.debug_points);
          store.debugPointsLength = robot.debug_points.length;
        }

        if (store.displayed_dense_path.length > 0) {
          renderPathDebug(store.displayed_dense_path);
        }

        //console.log(robot.gps_path_data.length)

        if (robot.gps_path_data.length != store.gpsPathLength) {
          store.gps_path = robot.gps_path_data;
          renderPathGPS(robot.gps_path_data);
          store.gpsPathLength = robot.gps_path_data.length;
        }

        var arrow_icon = L.icon({
          iconUrl: "/static/images/arrow.png",
          iconSize: [20, 70],
          iconAnchor: [10, 70],
        });

        var robot_icon = L.icon({
          iconUrl: "/static/images/robot.png",
          iconSize: [30, 40],
          iconAnchor: [15, 20],
        });

        //console.log("Turn intent degrees: ", robot.turn_intent_degrees)
        //check store.robotMarkerStore array if we have marker already
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
          store.robotMarkerStore[robot.id].addTo(map);
        }
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
          store.arrowMarkerStore[robot.id].addTo(map);
        }
      }
      store.robots = herd;
    })
    //catch any errors
    .catch(function (error) {
      console.log(error);
    });

  if (store.simulation) {
    window.setTimeout(getRobotData, SIMULATION_INTERVAL);
  } else {
    window.setTimeout(getRobotData, INTERVAL);
  }
}

function plot(robot) {
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
      //check data from server in console
      console.log(pathData);
      store.displayed_path = pathData;
      store.path_start = 0;
      store.path_end = store.displayed_path.length - 1;
      store.path_point_to_remove = store.displayed_path.length;

      console.log("pathData Length: ", pathData.length);
      renderPath(store.displayed_path);
      store.displayed_path_name = pathname;
    })
    //catch any errors
    .catch(function (error) {
      console.log(error);
    });
}

function loadDensePath() {
  //request circle data from server API
  api("get_dense_path")
    .then((resp) => resp.json())
    .then(function (densePathData) {
      //check data from server in console
      console.log(densePathData);
      store.displayed_dense_path = densePathData;
      // store.path_start = 0;
      // store.path_end = store.displayed_path.length-1;
      // store.path_point_to_remove =  store.displayed_path.length;
      //
      // console.log("pathData Length: ", pathData.length)
      renderPathDebug(store.displayed_dense_path);
      //store.displayed_path_name = pathname
    })
    //catch any errors
    .catch(function (error) {
      console.log(error);
    });
}

function updateServerPath(pathname, pathData) {
  //push path to server with the specified name
  api("save_path/" + pathname, {
    method: "POST", // or 'PUT'
    body: JSON.stringify(pathData), // data can be `string` or {object}!
    headers: {
      "Content-Type": "application/json",
    },
  })
    .then(function (reply) {
      //check data from server in console
      console.log(reply);
    })
    //catch any errors
    .catch(function (error) {
      console.log(error);
    });
}

function savePolygonToServer(polygonName, polygonData) {
  //push path to server with the specified name
  api("save_polygon/" + polygonName, {
    method: "POST", // or 'PUT'
    body: JSON.stringify(polygonData), // data can be `string` or {object}!
    headers: {
      "Content-Type": "application/json",
    },
  })
    .then(function (reply) {
      //check data from server in console
      console.log(reply);
    })
    //catch any errors
    .catch(function (error) {
      console.log(error);
    });
}

function deletePath() {
  let pathname = $("#path-name").val();
  //push path to server with the specified name
  api("delete_path/" + pathname)
    .then(function (reply) {
      //check data from server in console
      console.log(reply);
    })
    //catch any errors
    .catch(function (error) {
      console.log(error);
    });
}

function updateVehiclePath(pathname, vehicle_name) {
  //push path to server with the specified name
  api("/set_vehicle_path/" + pathname + "/" + vehicle_name)
    .then(function (reply) {
      //check data from server in console
      console.log(reply);
    })
    //catch any errors
    .catch(function (error) {
      console.log(error);
    });
}

function modifyAutonomyHold(vehicle_name, clear_hold) {
  //push path to server with the specified name
  api("modify_autonomy_hold/" + vehicle_name + "/" + clear_hold)
    .then(function (reply) {
      //check data from server in console
      console.log(reply);
    })
    //catch any errors
    .catch(function (error) {
      console.log(error);
    });
}

function updateVehicleAutonomy(vehicle_name, speed, enabled) {
  //push path to server with the specified name
  api("set_vehicle_autonomy/" + vehicle_name + "/" + speed + "/" + enabled)
    .then(function (reply) {
      //check data from server in console
      //console.log(reply);
    })
    //catch any errors
    .catch(function (error) {
      console.log(error);
    });
}

function updateGpsRecordCommand(vehicle_name, record_gps_command) {
  //push path to server with the specified name
  api("set_gps_recording/" + vehicle_name + "/" + record_gps_command)
    .then(function (reply) {
      //check data from server in console
      console.log(reply);
    })
    //catch any errors
    .catch(function (error) {
      console.log(error);
    });
}

function renderPath(pathData) {
  //check data from server in console
  //console.log(pathData);
  //console.log("Markers Length: ", markers.length)
  for (let i = 0; i < store.savedPathMarkers.length; i++) {
    store.savedPathMarkers[i].remove();
  }
  store.savedPathMarkers = [];
  //loop through circle data
  for (var i = 0, len = pathData.length; i < len; i++) {
    //draw circles on map

    var color = "#FF6600";
    if (i < store.path_start || i > store.path_end) {
      color = "#F006FF";
    }
    if (i == store.path_point_to_remove) {
      color = "#FF0000";
    }

    var marker = new L.Circle(new L.LatLng(pathData[i].lat, pathData[i].lon), {
      //radius: 0.6, // in meters
      radius: 0.3, // in meters
      fillColor: color,
      fillOpacity: 0.3,
      color: "#FFF",
      weight: 1,
      interactive: false,
    });

    marker.addTo(map);
    store.savedPathMarkers.push(marker);
  }
}

function renderPathDebug(pathData) {
  //check data from server in console
  //  console.log(pathData);
  for (let i = 0; i < store.debugPointMarkers.length; i++) {
    store.debugPointMarkers[i].remove();
  }
  store.debugPointMarkers = [];
  //loop through circle data
  for (var i = 0, len = pathData.length; i < len; i++) {
    //draw circles on map

    var color = "#FFFFFF";
    if (i > 1) {
      color = "#00FFFF";
    }

    var marker = new L.Circle(new L.LatLng(pathData[i].lat, pathData[i].lon), {
      radius: 0.2, // in meters
      fillColor: color,
      fillOpacity: 1.0,
      color: "#FFF",
      weight: 1,
      interactive: false,
    });
    marker.addTo(map);
    store.debugPointMarkers.push(marker);
  }
}

function renderPathLive(pathData) {
  //return;
  //check data from server in console
  //  console.log(pathData);
  for (let i = 0; i < store.livePathMarkers.length; i++) {
    store.livePathMarkers[i].remove();
  }
  store.livePathMarkers = [];
  //loop through circle data
  for (var i = 0, len = pathData.length; i < len; i++) {
    //draw circles on map

    var color = "#FFFF00";

    var offset = 0.000001;
    //  var offset = 0.000005 ;

    var triangleCoords = [
      [pathData[i].lat - offset * 0.7, pathData[i].lon - offset],
      [pathData[i].lat + offset * 0.7, pathData[i].lon],
      [pathData[i].lat - offset * 0.7, pathData[i].lon + offset],
      [pathData[i].lat - offset * 0.7, pathData[i].lon - offset],
    ];

    var marker = new L.Polygon(triangleCoords, {
      color: "#FF00FF",
      opacity: 0.0,
      weight: 2,
      fillColor: "#00FF00",
      fillOpacity: 1.0,
    });

    marker.addTo(map);
    store.livePathMarkers.push(marker);
  }
}

function renderPathGPS(pathData) {
  //  return;
  //check data from server in console
  console.log("PATHDATA");
  console.log(pathData);
  for (let i = 0; i < store.gpsPathMarkers.length; i++) {
    store.gpsPathMarkers[i].remove();
  }
  store.gpsPathMarkers = [];
  //loop through circle data
  for (var i = 0, len = pathData.length; i < len; i++) {
    //draw circles on map

    var color = "#FFFF00";

    var offset = 0.000025;

    var triangleCoords = [
      [pathData[i].lat + offset * 0.7, pathData[i].lon - offset],
      [pathData[i].lat - offset * 0.7, pathData[i].lon],
      [pathData[i].lat + offset * 0.7, pathData[i].lon + offset],
      [pathData[i].lat + offset * 0.7, pathData[i].lon - offset],
    ];

    try {
      var marker = new L.Polygon(triangleCoords, {
        color: "#FF00FF",
        opacity: 0.0,
        weight: 2,
        fillColor: "#FFA500 ",
        fillOpacity: 1.0,
      });
      marker.addTo(map);
      store.gpsPathMarkers.push(marker);
    } catch (error) {
      console.error(error);
      console.log(pathData[i]);
      // expected output: ReferenceError: nonExistentFunction is not defined
      // Note - error messages will vary depending on browser
    }
  }
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
  renderPath(store.displayed_path);
}

function loadPathList() {
  //request circle data from server API
  api("get_path_names")
    .then((resp) => resp.json())
    .then(function (pathnames) {
      //check data from server in console
      console.log(pathnames);

      var $dropdown = $("#dropdown-menu");
      $dropdown.empty();

      for (var i = 0; i < pathnames.length; i++) {
        $dropdown.append(
          "<a class='dropdown-item' href='#'>" + pathnames[i] + "</a>"
        );
      }
      //Register handler for all items
      $(".dropdown-item").on("click", function () {
        //console.log("You clicked the drop downs ", event.target.innerText)
        loadPath(event.target.innerText);
        $("#dropdownMenuButton").html(event.target.innerText);
        $("#path-name").val(event.target.innerText);
      });
    })
    //catch any errors
    .catch(function (error) {
      console.log(error);
    });
}

const apiPrefix = `http://${location.hostname}/api/`;
console.log("all API calls will hit " + apiPrefix);
function api(path, init) {
  return fetch(apiPrefix + path, init);
}
