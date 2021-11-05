// http://192.168.1.170:8090/api/projects/3/tasks/3116cce4-4215-4de9-9e9a-0e9c93df87f6/orthophoto/tiles.json
//
// http://192.168.1.170:8090/api/projects/3/tasks/3116cce4-4215-4de9-9e9a-0e9c93df87f6/orthophoto/tiles/{Z}/{X}/{Y}.png

//time between api refreshes
var INTERVAL = 2000;
var SIMULATION_INTERVAL = 1000;

var app = new Vue({
  el: "#app",
  data: store,
  delimiters: ["${", "}"], // the default, "{{-}}", conflicts with Jinjia2
  created: function () {
    const controller = new AbortController();
    setTimeout(() => controller.abort(), 500);
    fetch("http://192.168.1.170:8090/api/token-auth/", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ username: "taylor", password: "taylor" }),
      signal: controller.signal,
    })
      .then((resp) => {
        if (resp.status == 200) {
          return response.json();
        } else {
          throw new Error("unexpected HTTP " + resp.status);
        }
      })
      .then(setup_map_with_opendronemap)
      .catch(setup_map_only_mapbox)
      .finally(getRobotData); // kick off the ball after the map gets setup.
  },
});

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

  store.map = L.map("map_canvas", {
    center: [37.35372, -122.333377],
    zoom: 20,
    layers: [open_drone_map_layer],
    editable: true,
  });

  var baseLayers = {
    "Drone Map": open_drone_map_layer,
    Mapbox: mapbox_layer,
  };

  L.control.layers(baseLayers).addTo(store.map);
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

  store.map = L.map("map_canvas", {
    center: [37.353, -122.332],
    zoom: 18,
    layers: [mapbox_layer],
    editable: true,
  });

  var baseLayers = {
    Mapbox: mapbox_layer,
  };

  L.control.layers(baseLayers).addTo(store.map);
}
