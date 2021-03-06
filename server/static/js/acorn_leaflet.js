$(document).ready(function(){

        if(window.matchMedia("(max-width: 767px)").matches){

            // The viewport is less than 768 pixels wide

            $('#robot_detail_mobile').prop('id', 'robot_detail');
            $('#robot_detail_desktop').hide()
            $('#full_map_view').hide()


        } else{

            // The viewport is at least 768 pixels wide

            $('#robot_detail_desktop').prop('id', 'robot_detail');
            $('#robot_detail_mobile').hide()

        }

    });



  var zoom_scales = [ 0,
                      591657550.500000,
                      295828775.300000,
                      147914387.600000,
                      73957193.820000,
                      36978596.910000,
                      18489298.450000,
                      9244649.227000,
                      4622324.614000,
                      2311162.307000,
                      1155581.153000,
                      577790.576700,
                      288895.288400,
                      144447.644200,
                      72223.822090,
                      36111.911040,
                      18055.955520,
                      9027.977761,
                      4513.988880,
                      2256.994440,
                      1128.497220
                    ];




    //grab user local ip address
    var ipAddress = location.hostname;

    console.log(ipAddress);







// http://192.168.1.170:8090/api/projects/3/tasks/3116cce4-4215-4de9-9e9a-0e9c93df87f6/orthophoto/tiles.json
//
// http://192.168.1.170:8090/api/projects/3/tasks/3116cce4-4215-4de9-9e9a-0e9c93df87f6/orthophoto/tiles/{Z}/{X}/{Y}.png


var data = {'username': 'taylor', 'password': 'taylor'};


$.ajax({
  type: "POST",
  url: 'http://192.168.1.170:8090/api/token-auth/',
  data: data,
  success: setup_map
});

var map;

function setup_map(access_token_data) {

  var token = access_token_data['token']
  var open_drone_map_layer = L.tileLayer('http://192.168.1.170:8090/api/projects/3/tasks/3116cce4-4215-4de9-9e9a-0e9c93df87f6/orthophoto/tiles/{z}/{x}/{y}.png?jwt={accessToken}', {
  attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors, Imagery © <a href="https://www.twistedfields.com/">Twisted Fields</a>',
  maxZoom: 22,
  tileSize: 256,
  zoomOffset: 0,
  id: '',
  accessToken: token,
  tms: false,
  });


  var mapbox_layer = L.tileLayer('https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token={accessToken}', {
      attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors, Imagery © <a href="https://www.mapbox.com/">Mapbox</a>',
      maxZoom: 22,
      id: 'mapbox/satellite-v9',
      tileSize: 512,
      zoomOffset: -1,
      accessToken: 'pk.eyJ1IjoidHdpc3RlZGZpZWxkcyIsImEiOiJja2ozbmtlOXkwM2ZmMzNueTEzcGxhMGR1In0.eAhUMfZ786vm7KOhbrJj2g'
  });


  map = L.map('map_canvas', {
      center: [37.353720, -122.333377],
      zoom: 20,
      layers: [open_drone_map_layer]
  });

  var baseLayers = {
      "Drone Map": open_drone_map_layer,
      "Mapbox": mapbox_layer
  };

  L.control.layers(baseLayers).addTo(map);

}




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
    var path_start = -1;
    var path_end = -1;
    var path_point_to_remove = -1;
    var have_cleared_autonomy = false

    //time between api refreshes
    var INTERVAL = 5000;

    getRobotData();
    //getCircles();
  //  getPath();
  //  loadPathList();

    // <------- THIS FUNCTION GETS ROBOT DATA FROM SERVER AND UPDATES MARKERS AT INTERVAL ------->
    function getRobotData() {


        //request herd data from server API
        fetch('http://' + ipAddress + '/api/get_herd_data')
            .then((resp) => resp.json())
            .then(function (data) {
                //console.log(data);

                var current_zoom = map.getZoom();
                var scale = 1000 / zoom_scales[current_zoom] * 0.5;

                //loop through herd data from server
                for (var i = 0, len = data.length; i < len; i++) {

                    let robot = data[i]
                    let datenow = new Date();
                    //console.log(datenow)
                    //console.log(JSON.parse(robot.time_stamp))
                    let date = new Date(JSON.parse(robot.time_stamp));
                    let data_age_sec = (datenow-date)/1000.0

                    if (have_cleared_autonomy == false)
                    {
                      have_cleared_autonomy = true
                      modifyAutonomyHold(`${robot.name}`, false);
                    }

                    // if(robot.autonomy_hold)
                    // {
                    //   let speed = 0.0;
                    //   let active = false;
                    //   updateVehicleAutonomy(`${robot.name}`, speed, active);
                    // }
                    //console.log(robot.autonomy_hold)

                    var autonomy_allowed = robot.autonomy_hold?"False":"True"

                    //console.log(robot.strafeD)

                    var distances = {
                        y: robot.gps_distances,
                        type: 'scatter',
                        mode: 'lines',
                        name: 'Distances m'
                      };

                      var angles = {
                          y: robot.gps_angles,
                          type: 'scatter',
                          mode: 'lines',
                          name: 'Angles deg'
                        };


                    var distance_rates = {
                        y: robot.gps_distance_rates,
                        type: 'scatter',
                        mode: 'lines',
                        name: 'Distance rates'
                      };

                      var angle_rates = {
                          y: robot.gps_angle_rates,
                          type: 'scatter',
                          mode: 'lines',
                          name: 'Angle rates'
                        };

                        var strafeD = {
                            y: robot.strafeD,
                            type: 'scatter',
                            mode: 'lines',
                            name: 'strafeD'
                          };

                        var steerD = {
                            y: robot.steerD,
                            type: 'scatter',
                            mode: 'lines',
                            name: 'steerD'
                          };

                        var rotation = {
                            y: robot.rotation,
                            type: 'scatter',
                            mode: 'lines',
                            name: 'rotation'
                          };

                        var strafe = {
                            y: robot.strafe,
                            type: 'scatter',
                            mode: 'lines',
                            name: 'strafe'
                          };


                            var layout_lateral = {
                              title: {
                                text:'Lateral Error',
                                font: {
                                  family: 'Courier New, monospace',
                                  size: 24
                                },
                                xref: 'paper',
                                x: 0.05,
                              },
                            };


                            var layout_angle = {
                              title: {
                                text:'Angle Error',
                                font: {
                                  family: 'Courier New, monospace',
                                  size: 24
                                },
                                xref: 'paper',
                                x: 0.05,
                              },
                            };


                            var layout_lateral_rates = {
                              title: {
                                text:'Lateral Error Rates',
                                font: {
                                  family: 'Courier New, monospace',
                                  size: 24
                                },
                                xref: 'paper',
                                x: 0.05,
                              },
                            };


                            var layout_angle_rates = {
                              title: {
                                text:'Angular Error Rates',
                                font: {
                                  family: 'Courier New, monospace',
                                  size: 24
                                },
                                xref: 'paper',
                                x: 0.05,
                              },
                            };

                    Plotly.newPlot('plot_div_distance_rates', [strafe, strafeD], layout_lateral_rates);
                    Plotly.newPlot('plot_div_angle_rates', [rotation, steerD], layout_angle_rates);

                    Plotly.newPlot('plot_div_distance', [distances], layout_lateral);
                    Plotly.newPlot('plot_div_angles', [angles], layout_angle);



                    const markup = `
                    <div class="card" style="width: 18rem;">
                      <div class="card-header">
                        ${robot.name}
                      </div>
                      <ul class="list-group list-group-flush" id="${robot.name}-detail-view">
                        <li class="list-group-item" id="${robot.name}-detail-data-age">Data Age: ${data_age_sec.toFixed(1)} sec</li>
                        <li class="list-group-item" id="${robot.name}-detail-voltage">Voltage: ${robot.voltage.toFixed(2)}</li>
                        <li class="list-group-item" id="${robot.name}-detail-speed">Speed: ${robot.speed}</li>
                        <li class="list-group-item" id="${robot.name}-detail-control-state">Control State: ${robot.control_state}</li>
                        <li class="list-group-item" id="${robot.name}-detail-motor-state">Motor State: ${robot.motor_state}</li>
                        <li class="list-group-item" id="${robot.name}-detail-path-name">Loaded Path Name: ${robot.loaded_path_name}</li>
                        <li class="list-group-item" id="${robot.name}-detail-autonomy-allowed">Autonomy Allowed: ${autonomy_allowed}</li>
                        <li class="list-group-item" id="${robot.name}-detail-autonomy-active">Autonomy Active: ${robot.activate_autonomy}</li>
                        <li class="list-group-item" id="${robot.name}-detail-access-point">Access Point: ${robot.access_point_name}</li>
                        <li class="list-group-item" id="${robot.name}-detail-wifi-signal">Wifi Signal: ${robot.wifi_signal} dBm</li>
                      <li class="list-group-item"><button type="button" class="btn btn-primary mr-1" id="${robot.name}-load-button">Load Path</button></li>
                      <li class="list-group-item">
                        <div class="btn-group" role="group" aria-label="Autonomy Velocity">
                          <button type="button" class="btn btn-secondary" id="${robot.name}-vel-1">0.1</button>
                          <button type="button" class="btn btn-success" id="${robot.name}-vel-2">0.2</button>
                          <button type="button" class="btn btn-secondary" id="${robot.name}-vel-3">0.3</button>
                          <!--button type="button" class="btn btn-secondary" id="${robot.name}-vel-4">0.4</button-->
                        </div>
                      </li>
                      <li class="list-group-item"><button type="button" class="btn btn-lg btn-secondary mr-1 disabled" id="${robot.name}-activate-button">Activate Autonomy</button></li>
                      <li class="list-group-item"><button type="button" class="btn btn-primary mr-1" id="${robot.name}-clear-hold">Clear Autonomy Hold</button></li>
                      <li class="list-group-item">
                        <div class="btn-group" role="group" aria-label="GPS Recording">
                          <button type="button" class="btn btn-secondary" id="${robot.name}-gps-record-button">Record</button>
                          <button type="button" class="btn btn-secondary" id="${robot.name}-gps-pause-button">Pause</button>
                          <button type="button" class="btn btn-secondary" id="${robot.name}-gps-clear-button">Clear</button>
                        </div>
                      </li>

                      </ul>
                    </div>
                    `;


                    if(robot.loaded_path_name != livePathName)
                    {
                      renderPathLive(robot.live_path_data);
                      livePathName = robot.loaded_path_name;
                    }

                    if(robot.debug_points)
                    {
                      renderPathDebug(robot.debug_points);
                      debugPointsLength = robot.debug_points.length;
                    }

                    if(robot.gps_path_data.length != gpsPathLength)
                    {
                      gps_path = robot.gps_path_data
                      renderPathGPS(robot.gps_path_data);
                      gpsPathLength = robot.gps_path_data.length;
                    }

                    let div_id = `${robot.name}-cardview`;
                    const row = `
                    <div id=${div_id} class="row">
                    ${markup}
                    </div>
                    `;

                    if ( $( `#${div_id}` ).length ) {
                        $(`#${robot.name}-detail-voltage`).text(`Voltage: ${robot.voltage.toFixed(2)}`);
                        $(`#${robot.name}-detail-speed`).html(`Speed: ${robot.speed}`);
                        $(`#${robot.name}-detail-data-age`).html(`Data Age: ${data_age_sec.toFixed(1)} sec`);
                        $(`#${robot.name}-detail-path-name`).html(`Loaded Path Name: ${robot.loaded_path_name}`);
                        $(`#${robot.name}-detail-control-state`).html(`Control State: ${robot.control_state}`);
                        $(`#${robot.name}-detail-motor-state`).html(`Motor State: ${robot.motor_state}`);
                        $(`#${robot.name}-detail-autonomy-allowed`).html(`Autonomy Allowed: ${autonomy_allowed}`);
                        $(`#${robot.name}-detail-autonomy-active`).html(`Autonomy Active: ${robot.activate_autonomy}`);
                        $(`#${robot.name}-detail-access-point`).html(`Access Point: ${robot.access_point_name}`);
                        $(`#${robot.name}-detail-wifi-signal`).html(`Wifi Signal: ${robot.wifi_signal}`);

                        // console.log(robot.activate_autonomy);

                        if(robot.autonomy_hold)
                        {
                          $(`#${robot.name}-activate-button`).removeClass('btn-success')
                          $(`#${robot.name}-activate-button`).addClass('btn-secondary')
                          $(`#${robot.name}-activate-button`).addClass('disabled')
                          // $(`#${robot.name}-activate-button`).html("Disabled")

                        //  console.log("autonomy false");

                        } else
                        {
                        //  console.log("autonomy true");
                          // $(`#${robot.name}-activate-button`).html("Activate Autonomy")
                          $(`#${robot.name}-activate-button`).removeClass('btn-secondary')
                          $(`#${robot.name}-activate-button`).addClass('btn-success')
                          $(`#${robot.name}-activate-button`).removeClass('disabled')
                        }


                        if(robot.activate_autonomy)
                        {

                          if ($(`#${robot.name}-activate-button`).html() === "Activate Autonomy")
                          {
                            $(`#${robot.name}-activate-button`).removeClass('btn-success')
                            $(`#${robot.name}-activate-button`).addClass('btn-danger')
                            $(`#${robot.name}-activate-button`).html("Deactivate Autonomy")
                          }
                        }

                    } else
                    {
                      $('#robot_detail').append(row)
                      $("[id*=-load-button]").on('click', function (event) {

                            console.log("You clicked the load button ", event.target.innerText)
                            updateVehiclePath(displayed_path_name, `${robot.name}`);

                          });


                    $(`[id^=${robot.name}-vel-]`).on('click', function (event) {

                          $( `[id^=${robot.name}-vel-]`).each(function() {
                            $( this ).removeClass('btn-success')
                            $( this ).addClass('btn-secondary')
                          });

                          let active = $(`[id^=${robot.name}-active-][class*=btn-success]`).text()

                          $(event.target).removeClass('btn-secondary')
                          $(event.target).addClass('btn-success')

                          console.log(active + " "+ event.target.innerText)
                          active = active === "Activate"

                          let speed = event.target.innerText

                          updateVehicleAutonomy(`${robot.name}`, speed, active);

                        });


                        $(`[id^=${robot.name}-activate-button]`).on('click', function (event) {

                          if($(event.target).hasClass('disabled'))
                          {
                            console.log("clicked button but autonomy not allowed");
                            return;
                          }
                            console.log("clicked button and autonomy is allowed");

                          // $( `[id^=${robot.name}-active-]`).each(function() {
                          //   $( this ).removeClass('btn-success')
                          //   $( this ).addClass('btn-secondary')
                          // });

                          // $(event.target).removeClass('btn-secondary')
                          // $(event.target).addClass('btn-success')

                          let speed = $(`[id^=${robot.name}-vel-][class*=btn-success]`).text()

                          console.log(event.target.innerText + " "+ speed)
                          let active = event.target.innerText === "Activate Autonomy"

                          if(active)
                          {
                            $( event.target ).removeClass('btn-success')
                            $( event.target ).addClass('btn-danger')
                            $(event.target).html("Deactivate Autonomy")

                          } else
                          {
                            $( event.target ).addClass('btn-success')
                            $( event.target ).removeClass('btn-danger')
                            $( event.target).html("Activate Autonomy")

                          }

                          updateVehicleAutonomy(`${robot.name}`, speed, active);

                      });

                      $(`[id^=${robot.name}-clear-hold]`).on('click', function (event) {

                      //  $(`#${robot.name}-activate-button`).html("Deactivate Autonomy")

                        modifyAutonomyHold(`${robot.name}`, true);

                    });


                      $(`[id^=${robot.name}-gps-]`).on('click', function (event) {
                        console.log(event.target.innerText)
                        updateGpsRecordCommand(`${robot.name}`, event.target.innerText);

                    });

                    }



                    var arrow_icon = L.icon({
                        iconUrl: '/static/images/arrow.png',
                        iconSize: [20, 70],
                        iconAnchor: [10, 70],
                    });

                    var robot_icon = L.icon({
                        iconUrl: '/static/images/robot.png',
                        iconSize: [30, 40],
                        iconAnchor: [15, 20],

                    });


                    //console.log("Turn intent degrees: ", data[i].turn_intent_degrees)
                    //check robotMarkerStore array if we have marker already
                    if (robotMarkerStore.hasOwnProperty(data[i].id)) {
                        //if we do, set new position attribute to existing marker
                        robotMarkerStore[data[i].id].setIcon(robot_icon);
                        robotMarkerStore[data[i].id].setLatLng(new L.latLng(data[i].lat, data[i].lon));
                        robotMarkerStore[data[i].id].setRotationAngle(data[i].heading)
                        //console.log(goat_path);
                    } else {
                        //if we don't, create new marker and set attributes
                        var marker = L.marker([data[i].lat, data[i].lon], {icon: robot_icon});
                        //add new marker to robotMarkerStore array
                        robotMarkerStore[data[i].id] = marker;
                        robotMarkerStore[data[i].id].setRotationAngle(data[i].heading)
                        robotMarkerStore[data[i].id].addTo(map)
                    }
                    //check robotMarkerStore array if we have marker already
                    if (arrowMarkerStore.hasOwnProperty(data[i].id)) {
                        //if we do, set new position attribute to existing marker
                        arrowMarkerStore[data[i].id].setIcon(arrow_icon);
                        arrowMarkerStore[data[i].id].setLatLng(new L.latLng(data[i].lat, data[i].lon));
                        arrowMarkerStore[data[i].id].setRotationAngle(data[i].turn_intent_degrees + data[i].heading)
                        //console.log(goat_path);
                    } else {
                        //if we don't, create new marker and set attributes
                        var arrow_marker = L.marker([data[i].lat, data[i].lon], {icon: arrow_icon});
                        //add new marker to robotMarkerStore array
                        arrowMarkerStore[data[i].id] = arrow_marker;
                        arrowMarkerStore[data[i].id].setRotationAngle(data[i].turn_intent_degrees + data[i].heading)
                        arrowMarkerStore[data[i].id].addTo(map)
                    }

                }
            })
            //catch any errors
            .catch(function (error) {
                console.log(error);

            });
        //timeout function and restart at interval
        window.setTimeout(getRobotData, INTERVAL);
    }


    function loadPath(pathname) {
        //request circle data from server API
        fetch('http://' + ipAddress + '/api/get_path/' + pathname)
            .then((resp) => resp.json())
            .then(function (pathData) {
                //check data from server in console
                console.log(pathData);
                displayed_path = pathData;
                path_start = 0;
                path_end = displayed_path.length-1;
                path_point_to_remove =  displayed_path.length;

                console.log("pathData Length: ", pathData.length)
                renderPath(displayed_path);
                displayed_path_name = pathname

            })
            //catch any errors
            .catch(function (error) {
                console.log(error);
            });

    }


    function updateServerPath(pathname, pathData) {
        //push path to server with the specified name
        fetch('http://' + ipAddress + '/api/save_path/' + pathname, {
          method: 'POST', // or 'PUT'
          body: JSON.stringify(pathData), // data can be `string` or {object}!
          headers: {
            'Content-Type': 'application/json'
          }
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
      let pathname = $("#path-name").val()
        //push path to server with the specified name
        fetch('http://' + ipAddress + '/api/delete_path/' + pathname)
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
          fetch('http://' + ipAddress + '/api/set_vehicle_path/' + pathname + '/' + vehicle_name )
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
          fetch('http://' + ipAddress + '/api/modify_autonomy_hold/' + vehicle_name + '/' + clear_hold)
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
            fetch('http://' + ipAddress + '/api/set_vehicle_autonomy/' + vehicle_name + '/' + speed + '/' + enabled )
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
          fetch('http://' + ipAddress + '/api/set_gps_recording/' + vehicle_name + '/' + record_gps_command)
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
                for (let i = 0; i < savedPathMarkers.length; i++) {
                    savedPathMarkers[i].remove();
                }
                savedPathMarkers = [];
                //loop through circle data
                for (var i = 0, len = pathData.length; i < len; i++) {
                    //draw circles on map

                    var color = '#FF6600';
                    if (i < path_start || i > path_end)
                    {
                      color = '#F006FF';
                    }
                    if (i == path_point_to_remove)
                    {
                      color = '#FF0000';
                    }

                    var marker = new L.Circle(new L.LatLng(pathData[i].lat, pathData[i].lon), {
                        //radius: 0.6, // in meters
                        radius: 0.3, // in meters
                        fillColor: color,
                        fillOpacity: 0.3,
                        color: "#FFF",
                        weight: 1,
                        interactive: false
                    });

                    marker.addTo(map);
                    savedPathMarkers.push(marker);
                }

    }


      function renderPathDebug(pathData) {
                  //check data from server in console
                //  console.log(pathData);
                  for (let i = 0; i < debugPointMarkers.length; i++) {
                      debugPointMarkers[i].remove();
                  }
                  debugPointMarkers = [];
                  //loop through circle data
                  for (var i = 0, len = pathData.length; i < len; i++) {
                      //draw circles on map

                      var color = '#FFFFFF';
                      if(i > 1)
                      {
                        color = '#00FFFF';
                      }

                      var marker = new L.Circle(new L.LatLng(pathData[i].lat, pathData[i].lon), {
                          radius: 0.5, // in meters
                          fillColor: color,
                          fillOpacity: 1.0,
                          color: "#FFF",
                          weight: 1,
                          interactive: false
                      });
                      marker.addTo(map);
                      debugPointMarkers.push(marker);
                  }

      }

        function renderPathLive(pathData) {
                    //return;
                    //check data from server in console
                  //  console.log(pathData);
                    for (let i = 0; i < livePathMarkers.length; i++) {
                        livePathMarkers[i].remove();
                    }
                    livePathMarkers = [];
                    //loop through circle data
                    for (var i = 0, len = pathData.length; i < len; i++) {
                        //draw circles on map

                        var color = '#FFFF00';

                        var offset = 0.000001 ;
                      //  var offset = 0.000005 ;

                        var triangleCoords = [
                           [pathData[i].lat - offset * 0.7, pathData[i].lon - offset],
                           [pathData[i].lat + offset * 0.7, pathData[i].lon],
                           [pathData[i].lat - offset * 0.7, pathData[i].lon + offset],
                           [pathData[i].lat - offset * 0.7, pathData[i].lon - offset]
                        ];

                        var marker = new L.Polygon(triangleCoords, {
                          color: '#FF00FF',
                          opacity: 0.0,
                          weight: 2,
                          fillColor: '#00FF00',
                          fillOpacity: 1.0
                        });

                        marker.addTo(map);
                        livePathMarkers.push(marker);
                    }

        }


    function renderPathGPS(pathData) {
              //  return;
                //check data from server in console
                console.log("PATHDATA")
                console.log(pathData);
                for (let i = 0; i < gpsPathMarkers.length; i++) {
                    gpsPathMarkers[i].remove();
                }
                gpsPathMarkers = [];
                //loop through circle data
                for (var i = 0, len = pathData.length; i < len; i++) {
                    //draw circles on map

                    var color = '#FFFF00';

                    var offset = 0.000005 ;

                    var triangleCoords = [
                      [pathData[i].lat + offset * 0.7, pathData[i].lon - offset],
                      [pathData[i].lat - offset * 0.7, pathData[i].lon],
                      [pathData[i].lat + offset * 0.7, pathData[i].lon + offset],
                      [pathData[i].lat + offset * 0.7, pathData[i].lon - offset]
                    ];

                    try {
                      var marker = new L.Polygon(triangleCoords, {
                        color: '#FF00FF',
                        opacity: 0.0,
                        weight: 2,
                        fillColor: '#FFA500 ',
                        fillOpacity: 1.0
                      });
                      marker.addTo(map);
                      gpsPathMarkers.push(marker);
                    } catch (error) {
                    console.error(error);
                    console.log(pathData[i])
                    // expected output: ReferenceError: nonExistentFunction is not defined
                    // Note - error messages will vary depending on browser
                    }
                }

    }

   $('#dropdown-container').on('show.bs.dropdown', function () {
     loadPathList();
   });

       $("#save-as").on('click', function () {
        //console.log("You clicked the drop downs ", event.target.innerText)
        updateServerPath($("#path-name").val(), displayed_path)
        console.log(displayed_path)

       });

     $("#save-gps-as").on('click', function () {
      //console.log("You clicked the drop downs ", event.target.innerText)
      updateServerPath($("#path-name").val(), gps_path)
      console.log(gps_path)

     });
    $("#start-minus").on('click', function () {
       if (path_start > 0)
       {
         path_start-=1;
       }
       renderPath(displayed_path);
    });
    $("#start-plus").on('click', function () {
        if (path_start < displayed_path.length-2)
        {
          path_start+=1;
        }
        renderPath(displayed_path);
    });
    $("#end-minus").on('click', function () {
      if (path_end > 0)
      {
        path_end-=1;
      }
      renderPath(displayed_path);
    });
    $("#end-plus").on('click', function () {
      if (path_end < displayed_path.length-1)
      {
        path_end+=1;
      }
      renderPath(displayed_path);
    });
    $("#remove-minus").on('click', function () {
      path_point_to_remove-=1;
      if (path_point_to_remove < 0)
      {
        path_point_to_remove = displayed_path.length
      }
      renderPath(displayed_path);
    });
    $("#remove-plus").on('click', function () {

      path_point_to_remove+=1;
      if (path_point_to_remove > displayed_path.length)
      {
        path_point_to_remove = 0;
      }
      renderPath(displayed_path);
    });
    $("#modify-displayed-path").on('click', function () {

      modifyDisplayedPath();

    });









    function modifyDisplayedPath()
    {

      if(path_point_to_remove < displayed_path.length && path_point_to_remove > 0)
      {
        displayed_path.splice(path_point_to_remove,1);
        path_end-=1;
      }

      if(path_start > 0 || path_end < displayed_path.length-1)
      {
        length = path_end - path_start-1;
        displayed_path = displayed_path.splice(path_start,length);
      }
      path_start = 0;
      path_end = displayed_path.length-1;
      path_point_to_remove =  displayed_path.length;
      renderPath(displayed_path);
    }



    function loadPathList() {
        //request circle data from server API
        fetch('http://' + ipAddress + '/api/get_path_names')
            .then((resp) => resp.json())
            .then(function (pathnames) {
                //check data from server in console
                console.log(pathnames);

                var $dropdown = $("#dropdown-menu");
                $dropdown.empty();

                for (var i = 0; i < pathnames.length; i++) {
                  $dropdown.append("<a class='dropdown-item' href='#'>" + pathnames[i] + "</a>");
                  }
                  //Register handler for all items
                  $('.dropdown-item').on('click', function () {
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




    function readdTo() {
      //  getPath();
        getRobotData();
    }
