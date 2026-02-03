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


trick.real_time_enable()
trick.exec_set_software_frame(0.02)

trick.exec_set_enable_freeze(True)
trick.exec_set_freeze_command(True)


trick.var_set_copy_mode(2)
trick.var_server_set_port(49765)
trick_message.mtcout.init()
trick.message_subscribe(trick_message.mtcout)
trick_message.separate_thread_set_enabled(True)
simControlPanel = trick.SimControlPanel()
trick.add_external_application(simControlPanel)
trick_view = trick.TrickView()
trick.add_external_application(trick_view)
