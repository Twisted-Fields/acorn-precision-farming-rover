"""
*********************************************************************
                     This file is part of:
                       The Acorn Project
             https://wwww.twistedfields.com/research
*********************************************************************
Copyright (c) 2019-2021 Taylor Alexander, Twisted Fields LLC
Copyright (c) 2021 The Acorn Project contributors (cf. AUTHORS.md).

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*********************************************************************
"""
# Modified from example file
# Lazy Pirate poll by  Daniel Lundin <dln(at)eintr(dot)org>

import utils
import socketio

_POLL_TIMEOUT_SEC = 2.5
REQUEST_TIMEOUT = 4500
REQUEST_RETRIES = 30

sio = socketio.Client()


class AcornServerComms:
    def __init__(self, server_endpoint, logging, debug):
        self.server_endpoint = server_endpoint
        self.logger = logging.getLogger('main.comms')
        utils.config_logging(self.logger, debug=debug)
        self.logger.info("Connecting to server at {} …".format(server_endpoint))
        sio.connect(server_endpoint)

    def send(self, data):
        sio.emit('update-robot', data)

    @sio.on('robot-command')
    def on_robot_command(data):
        print(f'I received a message!: {data}')

    @sio.event
    def connect():
        print("I'm connected!")

    @sio.event
    def connect_error(data):
        print("The connection failed!")

    @sio.event
    def disconnect():
        print("I'm disconnected!")
