#!/usr/bin/env python

"""
    Copyright (c) 2019-2020, Concord Robotics Inc
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

     * Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of Concord Robotics Inc nor the names of its
       contributors may be used to endorse or promote products derived from
       this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
"""

class RobotCommand:

    def __init__(self, command_name, table):
        self.command_table = table.table.getSubTable(command_name) # Connect it

    def start(self): # Starts the command running
        #if not(self.command_table.getBoolean("running", True)):
        self.command_table.putBoolean("running", True)

    def stop(self):
        #if not(self.command_table.getBoolean("running", False)):
        self.command_table.putBoolean("running", False)

    def run_till_done(self): # Big danger
        self.command_table.putBoolean("running", True)
        while (self.command_table.getBoolean("running", True)):
            pass

    def toggle(self):
        self.command_table.putBoolean("running", not self.command_table.getBoolean("running", True))

    def check(self):
        return (self.command_table.getBoolean("running", False))
