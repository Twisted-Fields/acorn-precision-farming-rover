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
});

var zoom_scales = [
  0, 591657550.5, 295828775.3, 147914387.6, 73957193.82, 36978596.91,
  18489298.45, 9244649.227, 4622324.614, 2311162.307, 1155581.153, 577790.5767,
  288895.2884, 144447.6442, 72223.82209, 36111.91104, 18055.95552, 9027.977761,
  4513.98888, 2256.99444, 1128.49722,
];

//grab user local ip address
var ipAddress = location.hostname;
console.log("ip address is: " + ipAddress);

// http://192.168.1.170:8090/api/projects/3/tasks/3116cce4-4215-4de9-9e9a-0e9c93df87f6/orthophoto/tiles.json
//
// http://192.168.1.170:8090/api/projects/3/tasks/3116cce4-4215-4de9-9e9a-0e9c93df87f6/orthophoto/tiles/{Z}/{X}/{Y}.png

var data = { username: "taylor", password: "taylor" };

var userPolygon;

$.ajax({
  type: "POST",
  url: "http://192.168.1.170:8090/api/token-auth/",
  data: data,
  timeout: 500,
  success: setup_map_with_opendronemap,
  error: setup_map_only_mapbox,
});

var map;

//initialize empty marker and circle arrays
var robotMarkerStore = {};
var arrowMarkerStore = {};
var savedPathMarkers = [];
var livePathMarkers = [];
//var livePathLength = 0;
var livePathName = "";
var gpsPathMarkers = [];
var gpsPathLength = 0;
var debugPointMarkers = [];
var debugPointsLength = 0;
var gps_path = [];
var displayed_path = [];
var displayed_path_name = "";
var displayed_dense_path = [];
var path_start = -1;
var path_end = -1;
var path_point_to_remove = -1;
var have_cleared_autonomy = false;
var simulation = false;
var first_path_point;

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
