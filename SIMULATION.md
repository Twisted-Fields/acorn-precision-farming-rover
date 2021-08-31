## Running the code in simulation
August 30, 2021: It is now possible to run a simulated robot in Docker on
your computer. Our launch scripts are designed to run in linux, but if
you examine run_docker_simulation.sh you will see the commands needed.

This is early stages alpha quality and at this time only rough notes follow.

Install docker-compose for your platform.
https://docs.docker.com/compose/install/

Clone this git repo and change in to the directory.

`git clone https://github.com/Twisted-Fields/acorn-precision-farming-rover.git`
`cd acorn-precision-farming-rover`
`./run_docker_simulation.sh`

The docker container will now (hopefully) build and launch. This will take
some time. If it fails, please post a github issue. Note that several ports
must be available on the host including port 80.

The command runs two docker containers: the normal server container, and a
simulated robot vehicle container. The simulated vehicle stubs out all
hardware and sensors return fake data. The main goal is to support integration
testing but this simulation may be expanded with virtualized motion for path
following in simulation.

Connect to the vehicle container:
`./attach_docker_vehicle.sh`
Multiple processes are running in separate tmux windows. The vehicle has a main
process and a separate motor process.
`/home/pi # tmux ls
main: 1 windows (created Mon Aug 30 21:07:27 2021)
motors: 1 windows (created Mon Aug 30 21:07:27 2021)
`
Connect to the main process:
`tmux a -t main`

You should see a lot of output scrolling by the screen. Look for example for
GPS values.

To exit tmux, do not press Ctrl-C as that will kill the process. Instead tmux
uses a special key combination followed by different keys as different commands.
To exit, press Ctrl+B and then press D (for detach)

You can connect to the motors with:
`tmux a -t motors`

Exit out of any tmux windows, and at the root prompt of the container press
Ctrl+D to exit the docker container without stopping it.

The server is configured the same way, but with more tmux windows operating.
Attach to the server docker container with:
`./attach_docker_server.sh`

And explore the tmux windows there.

Now open a browser and enter "localhost" in the URL bar. The web page should
load. This is an engineering interface and we expect to make improvements before
deploying to end users. But this interface allows us to recall GPS tracks from
the database, load them on to Acorn, enable and disable autonomy, and monitor
robot status.

If you have made it this far, you are really important to us! We want to know
what you think of our software. Please sign up or log in to
[community.twistedfields.com](https://community.twistedfields.com/) and let us
know if you got the code running. This process is still an alpha release and
all user reports are valuable.
