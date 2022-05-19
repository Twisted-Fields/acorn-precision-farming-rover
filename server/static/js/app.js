
// http://192.168.1.170:8090/api/projects/3/tasks/3116cce4-4215-4de9-9e9a-0e9c93df87f6/orthophoto/tiles.json
//
// http://192.168.1.170:8090/api/projects/3/tasks/3116cce4-4215-4de9-9e9a-0e9c93df87f6/orthophoto/tiles/{Z}/{X}/{Y}.png

var app = new Vue({
  el: "#app",
  data: store,
  delimiters: ["${", "}"], // the default, "{{-}}", conflicts with Jinjia2
  mounted: getRobotData // kick off the ball.
});
