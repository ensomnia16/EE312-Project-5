//
// Copyright 2012 Ettus Research LLC
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

// This module is driven from simple_spi_core.

// Sends data over SPI interface. Chip select (sen) is active low.
// It only accepts data when in WAIT_TRIG state and latches it upon
// receiving start signal, which should be a single cycle strobe.

// Number of bits transmitted is set by num_bits (up to 32).
// There is no miso line and receiving communication.

// It gets device number in one hot manner to insert chip
// select for them. The advantage is in the module we can
// determine it one cycle earlier and this helps critical
// path by adding a FF.

// sending edge is for determining the CPHA of SPI transmission

// SPI clock would be 2*(sclk_divider+1) slower than system clock

module chirp_spi_core # (parameter DEVICE_COUNT=1)(
        input                     clock,
        input                     reset,
        input                     start,
	      input                     sending_edge,
	      input  [5:0]              num_bits,
	      input  [15:0]             sclk_divider,
	      input  [31:0]             set_data,
        input  [DEVICE_COUNT-1:0] device,

        output                    ready,
        output [DEVICE_COUNT-1:0] sen,
        output                    sclk,
        output                    mosi
    );

    reg [2:0] state;
    localparam WAIT_TRIG = 0;
    localparam PRE_IDLE = 1;
    localparam CLK_REG = 2;
    localparam CLK_INV = 3;
    localparam POST_IDLE = 4;
    localparam IDLE_SEN = 5;

    reg ready_reg;
    // ready signal is asserted after getting in WAIT_TRIG state and
    // not receiving a start strobe.
    assign ready = ready_reg && ~start;

    reg sclk_reg;
    assign sclk = sclk_reg;

    // Active low chip select
    reg [DEVICE_COUNT-1:0] sen_reg;
    wire sen_is_idle = (state == WAIT_TRIG) || (state == IDLE_SEN);

    always @ (posedge clock)
        sen_reg <= {DEVICE_COUNT{sen_is_idle}} | (~device);
    assign sen = sen_reg;

    // Shift register for output data
    reg [31:0] dataout_reg;
    wire [31:0] dataout_next = {dataout_reg[30:0], 1'b0};
    assign mosi = dataout_reg[31];

    // counter for clock division
    reg [15:0] sclk_counter;
    wire sclk_counter_done = (sclk_counter == sclk_divider);
    wire [15:0] sclk_counter_next = (sclk_counter_done)? 16'd0 : sclk_counter + 16'd1;

    //counter for latching bits miso/mosi
    reg [6:0] bit_counter;
    wire [6:0] bit_counter_next = bit_counter + 7'd1;
    wire bit_counter_done = (bit_counter_next == num_bits);

    always @(posedge clock) begin
	      if (!reset) begin
	          state <= WAIT_TRIG;
            ready_reg <= 1'b0;
	          sclk_reg <= 1'b0;
            dataout_reg <= 32'd0;
        end else begin
            case (state)
                WAIT_TRIG: begin
                    // only in WAIT_TRIG we accept start signal and latch the data
                    if (start) begin
                      state <= PRE_IDLE;
                      dataout_reg <= set_data;
                    end
                    ready_reg <= ~start;
                    sclk_counter <= 1'b0;
                    bit_counter <= 1'b0;
                    sclk_reg <= 1'b0;
                end

                PRE_IDLE: begin
                    if (sclk_counter_done) state <= CLK_REG;
                    sclk_counter <= sclk_counter_next;
                    sclk_reg <= 1'b0;
                end

                CLK_REG: begin
                    if (sclk_counter_done) begin
                        state <= CLK_INV;
                        if (sending_edge == 1'b0 && bit_counter != 7'd0) dataout_reg <= dataout_next;
                        sclk_reg <= 1'b1;
                    end
                    sclk_counter <= sclk_counter_next;
                end

                CLK_INV: begin
                    if (sclk_counter_done) begin
                        state <= (bit_counter_done)? POST_IDLE : CLK_REG;
                        bit_counter <= bit_counter_next;
                        if (sending_edge == 1'b1 && ~bit_counter_done) dataout_reg <= dataout_next;
                        sclk_reg <= 1'b0;
                    end
                    sclk_counter <= sclk_counter_next;
                end

                POST_IDLE: begin
                    if (sclk_counter_done) state <= IDLE_SEN;
                    sclk_counter <= sclk_counter_next;
                    sclk_reg <= 1'b0;
                end

                IDLE_SEN: begin
                    if (sclk_counter_done) state <= WAIT_TRIG;
                    sclk_counter <= sclk_counter_next;
                    sclk_reg <= 1'b0;
                end

                default: state <= WAIT_TRIG;
            endcase
	      end
    end

endmodule
