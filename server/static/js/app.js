
// http://192.168.1.170:8090/api/projects/3/tasks/3116cce4-4215-4de9-9e9a-0e9c93df87f6/orthophoto/tiles.json
//
// http://192.168.1.170:8090/api/projects/3/tasks/3116cce4-4215-4de9-9e9a-0e9c93df87f6/orthophoto/tiles/{Z}/{X}/{Y}.png

const app = new Vue({
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
      .then((access_token_data) => {store.access_token_data = access_token_data})
      .catch(()=>{})
  },
});

const socket = io();
socket.on("connect", () => {
  console.log("socket connected");
});

socket.on("disconnect", () => {
  console.log("socket diconnected");
});
socket.on("herd-data", (data) => {
  updateHerdData(data)
});
