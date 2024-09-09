# Copyright 2024 Blazej Fiderek (xfiderek)
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     https://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import socket
import threading
import time


class TrickVariableServerClient:
    def __init__(
        self,
        host,
        port,
        client_tag,
        trick_variable_names,
        trick_var_cycle_time=0.1,
        on_receive_callback=None,
    ):
        self._client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._client_socket.connect((host, port))

        self._insock = self._client_socket.makefile("r")

        self._variable_names = trick_variable_names
        self._latest_data = {}
        self._on_receive_callback = on_receive_callback
        self._trick_var_cycle_time = trick_var_cycle_time

        # send configuration info to trick and ask for variables
        self._client_socket.send(
            f'trick.var_set_client_tag("{client_tag}") \n'.encode()
        )
        self._client_socket.send(
            f"trick.var_cycle({str(trick_var_cycle_time)}) \n".encode()
        )

        self._receive_thread = threading.Thread(target=self._rcv_trick_data_from_socket)
        self._receive_thread.start()

    def freeze_sim(self):
        self._client_socket.send("trick.exec_freeze()\n".encode())
        self.stop_listening()

    def unfreeze_sim(self):
        self.start_listening()
        self._client_socket.send("trick.exec_run()\n".encode())

    def start_listening(self):
        command = b""
        for trick_var_name in self._variable_names:
            command += f'trick.var_add("{trick_var_name}") \n'.encode()
        self._client_socket.send("trick.var_ascii()\n".encode())
        self._client_socket.send(command)

    def stop_listening(self):
        self._client_socket.send("trick.var_clear()\n".encode())

    def send_data_to_sim(self, var_name: str, value: str) -> None:
        self._client_socket.send(f"{var_name} = {value} \n".encode())

    def _rcv_trick_data_from_socket(self):
        while True:
            if self._insock:
                line = self._insock.readline()
                if line:
                    variables = line.split("\t")
                    if variables[0] == "0":
                        for i, var_name in enumerate(self._variable_names):
                            self._latest_data[var_name] = variables[i + 1].strip()
                        if self._on_receive_callback:
                            self._on_receive_callback(self._latest_data.copy())
            else:
                time.sleep(self._trick_var_cycle_time)
