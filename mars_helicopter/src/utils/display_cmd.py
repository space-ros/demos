import sys
import os
import shutil

class CommandDisplay():
    def __init__(self, limits, grid_size):
        self.size = grid_size
        self.lims = limits
        self.grid = [[' ' for i in range(grid_size)] for i in range(grid_size)]
        self.empty_box = "+----------+\n|          |\n|          |\n|          |\n|          |\n+----------+"
        self.half_box = "+----------+\n|          |\n|   ++++   |\n|   ++++   |\n|          |\n+----------+"
        self.full_box = "+----------+\n|++++++++++|\n|++++++++++|\n|++++++++++|\n|++++++++++|\n+----------+"
        self.yaw_left = "+---\n|   \nv   "
        self.yaw_right = "---+\n   |\n   v"

    def clear_grid(self):
        self.grid = [[' ' for _ in range(len(self.grid))] for _ in range(len(self.grid))]

        
    def bar(self, progress):
        max_lines = 14.0
        border = "+------+\n"
        empty = "|      |\n"
        full = "|++++++|\n"

        n_full = int(progress*max_lines)
        n_empty = int(max_lines - n_full)
        return border + n_empty*empty + n_full*full + border

    def put_in_grid(self, start_position, text):
        p = list(start_position)
        x_reset = p[1]
        for c in text:
            if (c == '\n'):
                p[0] += 1
                p[1] = x_reset
            else:
                y = p[0]
                x = p[1]
                self.grid[y][x] = c
                p[1] += 1

    def print_grid(self):
        sys.stdout.write('\033[H\033[J')
        for row in self.grid:
            sys.stdout.write(''.join(row))
            sys.stdout.write("\n\r")
        sys.stdout.flush()
        self.clear_grid()

    def box_type(self, val, limit):
        if (val == 0.0):
            return self.empty_box
        elif (val == limit):
            return self.full_box
        else:
            return self.half_box

    def draw_commands(self, commands):
        # Clear the screen to update in place
        
        rotor_on = commands.cmd_speed > 0.0
        is_yaw_right = commands.cmd_yaw_coll > 0.0
        is_yaw_left = commands.cmd_yaw_coll < 0.0
        long_cycl = commands.cmd_long_cycl
        if (long_cycl >= 0.0):
            self.put_in_grid((2,12),self.box_type(long_cycl, self.lims.lim_long_cycl[1]))
            self.put_in_grid((14,12),self.empty_box)
        else:
            self.put_in_grid((2,12),self.empty_box)
            self.put_in_grid((14,12),self.box_type(long_cycl, self.lims.lim_long_cycl[0]))

        lat_cycl = commands.cmd_lat_cycl
        if (lat_cycl >= 0.0):
            self.put_in_grid((8,24),self.box_type(lat_cycl, self.lims.lim_lat_cycl[1]))
            self.put_in_grid((8,0),self.empty_box)
        else:
            self.put_in_grid((8,24),self.empty_box)
            self.put_in_grid((8,0),self.box_type(lat_cycl, self.lims.lim_lat_cycl[0]))

        coll = commands.cmd_thrust_coll
        progress = coll/(self.lims.lim_thrust_coll[1]-self.lims.lim_thrust_coll[0])
        self.put_in_grid((3,37),self.bar(progress))

        if (is_yaw_right):
            self.put_in_grid((3,29),self.yaw_right)
        if (is_yaw_left):
            self.put_in_grid((3,3),self.yaw_left)
        
        self.print_grid()