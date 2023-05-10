//
// Copyright 2011-2012 Ettus Research LLC
// Copyright 2019 The Regents of the University of California
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

// Improved critical path to meet timing

// 64 bits worth of ticks

module time_compare
  (input [63:0] time_now,
   input [63:0] trigger_time,
   output now,
   output early,
   output reg late,
   output too_early);

    wire MSB_equal   = time_now [63:32] == trigger_time [63:32];
    wire LSB_equal   = time_now [31:0]  == trigger_time [31:0] ;
    wire MSB_compare = time_now [63:32] >  trigger_time [63:32];
    wire LSB_compare = time_now [31:0]  >  trigger_time [31:0] ;

    always @ (*)
     if (MSB_compare)
      late = 1'b1;
     else if (~MSB_equal)
      late = 1'b0;
     else if (LSB_compare)
       late = 1'b1;
     else
       late = 1'b0;

    assign now = MSB_equal && LSB_equal;
    assign early = ~now & ~late;
    assign too_early = 0; //not implemented

endmodule // time_compare
