$(document).ready(function () {
  if (window.matchMedia("(max-width: 767px)").matches) {
    // The viewport is less than 768 pixels wide

    $("#robot_detail_mobile").prop("id", "robot_detail");
    $("#robot_detail_desktop").hide();
    $("#full_map_view").hide();
  } else {
    // The viewport is at least 768 pixels wide

    $("#robot_detail_desktop").prop("id", "robot_detail");
    $("#robot_detail_mobile").hide();
  }

  $.ajax({
    type: "POST",
    url: "http://192.168.1.170:8090/api/token-auth/",
    data: { username: "taylor", password: "taylor" },
    timeout: 500,
    success: setup_map_with_opendronemap,
    error: setup_map_only_mapbox,
    complete: getRobotData, // kick off the ball after the map gets setup.
  });
});

// http://192.168.1.170:8090/api/projects/3/tasks/3116cce4-4215-4de9-9e9a-0e9c93df87f6/orthophoto/tiles.json
//
// http://192.168.1.170:8090/api/projects/3/tasks/3116cce4-4215-4de9-9e9a-0e9c93df87f6/orthophoto/tiles/{Z}/{X}/{Y}.png

var map;

//time between api refreshes
var INTERVAL = 2000;
var SIMULATION_INTERVAL = 1000;

$("#dropdown-container").on("show.bs.dropdown", function () {
  loadPathList();
});

$("#save-as").on("click", function () {
  //console.log("You clicked the drop downs ", event.target.innerText)
  updateServerPath($("#path-name").val(), displayed_path);
  console.log(displayed_path);
});

$("#btn-load-dense-points").on("click", function () {
  console.log("CLICK");
  loadDensePath();
});

var userPolygon;
$("#btn-start-polygon").on("click", function () {
  console.log($("#btn-start-polygon"));
  if ($("#btn-start-polygon").html() === "Start Polygon") {
    userPolygon = map.editTools.startPolygon();
    $("#btn-start-polygon").html("Stop Polygon");
  } else {
    map.editTools.stopDrawing();
    $("#btn-start-polygon").html("Start Polygon");
    console.log(userPolygon.toGeoJSON(10));
    savePolygonToServer("default", userPolygon.toGeoJSON(10));
    // console.log(map.editTools)
  }
});

$("#save-gps-as").on("click", function () {
  //console.log("You clicked the drop downs ", event.target.innerText)
  updateServerPath($("#path-name").val(), gps_path);
  console.log(gps_path);
});
$("#start-minus").on("click", function () {
  if (path_start > 0) {
    path_start -= 1;
  }
  renderPath(displayed_path);
});
$("#start-plus").on("click", function () {
  if (path_start < displayed_path.length - 2) {
    path_start += 1;
  }
  renderPath(displayed_path);
});
$("#end-minus").on("click", function () {
  if (path_end > 0) {
    path_end -= 1;
  }
  renderPath(displayed_path);
});
$("#end-plus").on("click", function () {
  if (path_end < displayed_path.length - 1) {
    path_end += 1;
  }
  renderPath(displayed_path);
});
$("#remove-minus").on("click", function () {
  path_point_to_remove -= 1;
  if (path_point_to_remove < 0) {
    path_point_to_remove = displayed_path.length;
  }
  renderPath(displayed_path);
});

$("#remove-plus").on("click", function () {
  path_point_to_remove += 1;
  if (path_point_to_remove > displayed_path.length) {
    path_point_to_remove = 0;
  }
  renderPath(displayed_path);
});

$("#modify-displayed-path").on("click", modifyDisplayedPath);


function setup_map_with_opendronemap(access_token_data) {
  var token = access_token_data["token"];
  var open_drone_map_layer = L.tileLayer(
    "http://192.168.1.170:8090/api/projects/3/tasks/3116cce4-4215-4de9-9e9a-0e9c93df87f6/orthophoto/tiles/{z}/{x}/{y}.png?jwt={accessToken}",
    {
      attribution:
        'Map data &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors, Imagery © <a href="https://www.twistedfields.com/">Twisted Fields</a>',
      maxZoom: 22,
      tileSize: 256,
      zoomOffset: 0,
      id: "",
      accessToken: token,
      tms: false,
    }
  );

  var mapbox_layer = L.tileLayer(
    "https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token={accessToken}",
    {
      attribution:
        'Map data &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors, Imagery © <a href="https://www.mapbox.com/">Mapbox</a>',
      maxZoom: 22,
      id: "mapbox/satellite-v9",
      tileSize: 512,
      zoomOffset: -1,
      accessToken:
        "pk.eyJ1IjoidHdpc3RlZGZpZWxkcyIsImEiOiJja2ozbmtlOXkwM2ZmMzNueTEzcGxhMGR1In0.eAhUMfZ786vm7KOhbrJj2g",
    }
  );

  map = L.map("map_canvas", {
    center: [37.35372, -122.333377],
    zoom: 20,
    layers: [open_drone_map_layer],
    editable: true,
  });

  var baseLayers = {
    "Drone Map": open_drone_map_layer,
    Mapbox: mapbox_layer,
  };

  L.control.layers(baseLayers).addTo(map);
}

function setup_map_only_mapbox() {
  var mapbox_layer = L.tileLayer(
    "https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token={accessToken}",
    {
      attribution:
        'Map data &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors, Imagery © <a href="https://www.mapbox.com/">Mapbox</a>',
      maxZoom: 22,
      id: "mapbox/satellite-v9",
      tileSize: 512,
      zoomOffset: -1,
      accessToken:
        "pk.eyJ1IjoidHdpc3RlZGZpZWxkcyIsImEiOiJja2ozbmtlOXkwM2ZmMzNueTEzcGxhMGR1In0.eAhUMfZ786vm7KOhbrJj2g",
    }
  );

  map = L.map("map_canvas", {
    center: [37.353, -122.332],
    zoom: 18,
    layers: [mapbox_layer],
    editable: true,
  });

  var baseLayers = {
    Mapbox: mapbox_layer,
  };

  L.control.layers(baseLayers).addTo(map);
}
