//
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

// Conidering a one-hot representation of required bands,
// as well as knowing the current band it would find the
// next band which should be scanned. It rolls over to first
// one after the last band.

// Each time an update signal is asserted, the next band is
// outputted the next cycle. There are 3 stages to calculate
// the next update: decode last selected band into a mask, encode
// the first 1 in the results, and do comparison for roll over.
// so update request issues should have at least 3 cycles distance
// between them (in chirp module the number of cycles among
// the update requests is much more).

// When there is latch_input request the first band is ready
// within 2 delay cycle, and next 3 cycles there cannot be an update
// request. In this mode the previous selected band is ignored.
// If there is no 1 in the search bands it would give out the
// default 0.

module chirp_band_selector # (parameter MAX_BANDS=64, BAND_WIDTH=6) (
  input                   clk,
  input                   reset,

  input                   latch_input,
  input                   update_band,

  input  [MAX_BANDS-1 :0] to_use_bands,
  output [BAND_WIDTH-1:0] band,

  output                  ready_for_update,
  output                  to_roll_over
);

  reg  [MAX_BANDS-1:0]  to_use_bands_r;
  wire [MAX_BANDS-1:0]  mask;
  reg  [MAX_BANDS-1:0]  search_bands;

  reg  [BAND_WIDTH-1:0] first_band;
  reg  [BAND_WIDTH-1:0] first_one;
  reg  [BAND_WIDTH-1:0] first_one_r;
  reg  [BAND_WIDTH-1:0] band_r;
  wire [BAND_WIDTH-1:0] next_band;

  integer i;

  reg latch_input_r, latch_input_rr;
  reg update_band_r, update_band_rr, update_band_rrr;

  // One cycle after latch we want to update the search_bands
  // register. Also 3 cycles after update or registering the
  // first band a new update command cannot be received.
  // We could use 2 bit counter, instead we used 3 bits to
  // avoid the comparison logic.
  always @ (posedge clk) begin
    latch_input_r   <= latch_input;
    latch_input_rr  <= latch_input_r;
    update_band_r   <= update_band | latch_input_rr;
    update_band_rr  <= update_band_r;
    update_band_rrr <= update_band_rr;
  end

  // Save the desired bands
  always @ (posedge clk)
    if (reset)
      to_use_bands_r <= 1;
    else if (latch_input)
      to_use_bands_r <= to_use_bands;

  // Decoding and masking Stage
  // A mask which is one for all the one-hot bits after current band value
  assign mask = {{(MAX_BANDS-1){1'b1}},1'b0} << band_r;

  always @ (posedge clk)
    if (reset) begin
      search_bands  <= 0;
    end else begin
      // If we wanna find the first band in the list all the values
      // should be checked, no matter what is the current band.
      // Otherwise we use the masked version of bands
      search_bands <= (latch_input) ? to_use_bands : (to_use_bands_r & mask);
    end

  // Encoding Stage
  // A priority encoder to find the next one hot value
  always @ (*) begin
    first_one = {BAND_WIDTH{1'b0}};
    for (i=MAX_BANDS-1; i>=0; i = i-1)
      if (search_bands [i]) first_one = i;
  end

  always @ (posedge clk)
    first_one_r <= first_one;

  // Selection (comparison) and update Stage

  // If there is no band left, encoder output would be 0, so
  // if the first band is not 0 use that as the output which
  // means we rolled over, and if it's zero then that's the
  // default band.
  assign next_band = (first_band > first_one_r) ? first_band : first_one_r;
  assign to_roll_over = ~(|first_one_r);

  // first_band is known one cycle after latch_input is asserted
  // (just going through encode stage to find first 1) and after
  // a pipeline register results are ready after 2 cycles. So
  // if there is latch_input_rr signal there is no need for comparison
  // and we can set the band_r value. Otherwise band_r is updated from
  // next_band when there is update_band request. It takes 3 cycles
  // to comute the next_band when band_r is updated.
  always @ (posedge clk)
    if (reset) begin
      first_band <= 0;
      band_r     <= 0;
    end else if (latch_input_rr) begin
      first_band <= first_one_r;
      band_r     <= first_one_r;
    end else if (update_band) begin
      band_r     <= next_band;
    end

  assign band = band_r;
  assign ready_for_update = ~(latch_input_r | latch_input_rr |
                            update_band_r | update_band_rr | update_band_rrr);

endmodule
