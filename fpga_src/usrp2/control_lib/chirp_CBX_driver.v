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

module chirp_CBX_driver(
        input clock,
        input reset,
        input set_stb_user,
        input [7:0] set_addr_user,
        input [31:0] set_data_user,
        input trigger,

        output [1:0] sen,
        output sclk,
        output mosi,
        output [7:0] debug,

        output counter_sen,
        output counter_mosi,
        output counter_sclk,

        output reg chirp_en
     );

  localparam BAND_COUNT = 64;
  localparam BAND_WIDTH = 6;

  // reset for state machine, SPI modules and counter, not to start sending
  // before initialization and calibration of the system are done.
  wire full_reset = trigger & ~reset;

  // ******************************************************** //
  // ********************* CYCLE COUNTER ******************** //
  // ******************************************************** //
  // A cycle counter for full state machine loop. It also latches
  // the value.
  reg  counter_reset;
  reg  counter_latch;
  reg  [19:0] time_counter;
  wire [19:0] n_time_counter;
  reg  [19:0] time_counter_latched;

  assign n_time_counter = time_counter + 20'd1;

  // Since counter_reset is a register, it enables one cycle later
  // after we started the loop, so we start from 1
  always @ (posedge clock)
    if (~full_reset | counter_reset)
      time_counter <= 20'd1;
    else
      time_counter <= n_time_counter;

  always @ (posedge clock)
    if (~full_reset)
      time_counter_latched <= 20'd0;
    else if (counter_latch)
      time_counter_latched <= time_counter;

  // ******************************************************** //
  // **************** CONFIGURATION REGISTERS *************** //
  // ******************************************************** //
  // receiving register configuration from user, as well as
  // setting default and fixed values.

  reg [11:0] jump;
  reg [31:0] freq_bit_array [0:7];
  reg [31:0] freq_bit_array2 [0:7];
  reg [11:0] start_ramp;
  reg [11:0] end_ramp;
  reg [15:0] sclk_divider_dac;
  reg [2:0]  rf_div_decode;
  reg [2:0]  select_rf_div;
  reg [7:0]  rf_div_bit_array;
  reg new_rf_divs_received;

  wire [7:0] DAC_prefix = 8'b00011000;

  // decoding first 5 bits of user data user for RF divider
  always @ (*)
    case(set_data_user[7:0])
      8'd1:    rf_div_decode = 3'h0;
      8'd2:    rf_div_decode = 3'h1;
      8'd4:    rf_div_decode = 3'h2;
      8'd8:    rf_div_decode = 3'h3;
      8'd16:   rf_div_decode = 3'h4;
      8'd32:   rf_div_decode = 3'h5;
      8'd64:   rf_div_decode = 3'h6;
      8'd128:  rf_div_decode = 3'h7;
      // If the user enters a wrong value, set default to divide by 2
      default: rf_div_decode = 3'h1;
    endcase

// integer i;

  always @ (posedge clock) begin
    if (reset) begin
        chirp_en         <= 1'b0;
        jump             <= 12'd1024;
        sclk_divider_dac <= 16'd4;
        select_rf_div    <= 3'h1;
        start_ramp       <= 12'h26D;
        end_ramp         <= 12'hC1D;
        rf_div_bit_array <= 8'd2;

    end else if (set_stb_user == 1'b1) begin
      case (set_addr_user)
        8'd1: freq_bit_array[select_rf_div]  <= set_data_user;
        8'd2: freq_bit_array2[select_rf_div] <= set_data_user;
        8'd3: chirp_en                       <= set_data_user[0];
        8'd4: jump                           <= set_data_user[11:0];
        8'd5: sclk_divider_dac               <= set_data_user[15:0];
        8'd6: select_rf_div                  <= rf_div_decode;
        8'd7: start_ramp                     <= set_data_user[11:0];
        8'd8: end_ramp                       <= set_data_user[11:0];
        8'd9: rf_div_bit_array               <= set_data_user[7:0];
      endcase
    end
  end

  always @ (posedge clock)
    if (reset)
        new_rf_divs_received <= 1'b0;
    else if ((set_stb_user == 1'b1) &&
             (set_addr_user==8'd9))
        new_rf_divs_received <= 1'b1;
    else
        new_rf_divs_received <= 1'b0;


  // ******************************************************** //
  // ********************* RF DIVIDER SELECTOR ******************** //
  // ******************************************************** //
  // Based on the one-hot selected bands finds the next rf_div to
  // be used, and latches the appropriate register value for it
  reg         rf_div_updated, rf_div_updated_r;
  wire [2:0]  rf_div_r;
  reg         update_rf_div;
  reg         find_first_band;

  always @ (posedge clock)
    if (reset) begin
      rf_div_updated   <= 1'b0;
      rf_div_updated_r <= 1'b0;
    end else begin
      rf_div_updated   <= update_rf_div;
      rf_div_updated_r <= rf_div_updated;
    end

  // priority encoder for band selector based on one-hot selected bands
  // There is more than three cycles delay between update requests, so
  // there is not need to check the ready_for_update signal
  chirp_band_selector #(.MAX_BANDS(8), .BAND_WIDTH(3)) rf_div_sel (
    .clk(clock), .reset(reset),
    .latch_input(find_first_band | new_rf_divs_received), .update_band(update_rf_div),
    .to_use_bands(rf_div_bit_array),
    .band(rf_div_r), .ready_for_update(), .to_roll_over()
  );

  // ******************************************************** //
  // ********************* BAND SELECTOR ******************** //
  // ******************************************************** //
  // Based on the one-hot selected bands finds the next band to
  // be sampled, and latches the appropriate register value for it
  wire [5:0]  band_r;
  reg         update_band;
  wire        to_roll_over;
  reg [31:0] selected_freq_bit_array, selected_freq_bit_array2;

  always @ (posedge clock)
    if (reset) begin
      selected_freq_bit_array  <= 32'd1;
      selected_freq_bit_array2 <= 32'd0;
    end else if (find_first_band | rf_div_updated) begin
      selected_freq_bit_array  <= freq_bit_array[rf_div_r];
      selected_freq_bit_array2 <= freq_bit_array2[rf_div_r];
    end

  // priority encoder for band selector based on one-hot selected bands
  // There is more than three cycles delay between update requests, so
  // there is not need to check the ready_for_update signal
  chirp_band_selector #(.MAX_BANDS(BAND_COUNT), .BAND_WIDTH(BAND_WIDTH)) band_sel (
    .clk(clock), .reset(reset),
    .latch_input(find_first_band | rf_div_updated_r), .update_band(update_band),
    .to_use_bands({selected_freq_bit_array2[BAND_COUNT-33:0],selected_freq_bit_array}),
    .band(band_r), .ready_for_update(), .to_roll_over(to_roll_over)
  );

  // ******************************************************** //
  // ********************* STATE MACHINE ******************** //
  // ******************************************************** //
  localparam START_CHIRP       = 0;
  localparam SET_FREQUENCY     = 1;
  localparam WAIT_READY_VCO    = 2;
  // localparam WAIT_FOR_VCO_LOCK = 3;
  localparam GENERATE_CHIRP    = 4;
  localparam WAIT_READY_DAC    = 5;
  localparam WAIT_CYCLES       = 6;

  reg [2:0]  state;
  reg [2:0]  counter;       // sending 7 words to the VCO module
  // reg [13:0] wait_counter;  // wait for VCO lock time out

  reg [31:0] load_data;
  reg        start_tr; // start strobe for SPI send
  reg        started_chirp;
  reg        ended_chirp;
  wire       ready;
  reg [15:0] sclk_divider;

  reg        device;   // There is only 2 devices
  reg [5:0]  num_bits; // 24 or 32 bits SPI messages
  reg [12:0] DAC_data; // DAC data is 13 bits instead of 12,
                       // so we know when we passed the end_ramp

  always @ (posedge clock) begin
    if (~full_reset) begin
      start_tr        <= 0;
      state           <= START_CHIRP;
      counter_reset   <= 1'b1;
      counter_latch   <= 1'b0;
      update_band     <= 1'b0;
      update_rf_div   <= 1'b0;
      find_first_band <= 1'b1;
      started_chirp   <= 1'b0;
      ended_chirp     <= 1'b0;
    end else begin

      case(state)
        // We start sending first register value to VCO here (which is reg2)
        // Also first band to scan is latched in this state.
        START_CHIRP: begin
          counter         <= 3'd0;
          num_bits        <= 6'd32;
          device          <= 1'b1;
          sclk_divider    <= 16'd4;
          state           <= SET_FREQUENCY;
          // After reset and before full reset the first band is found
          // It needs two cycles, one is START_CHIRP and one is first cycle of
          // SET_FREQUENCY where the band_r register is not used.
          find_first_band <= 1'b0;
        end

        SET_FREQUENCY: begin
          // load data selection, first config word is sent from
          // the previous state and the next 6 config words are sent here
          // 0 is not used for the counter value
          case (counter)
            3'd0:    load_data <= 32'h01400005;                // Register 5
            3'd1:    load_data <= {9'h0C3,rf_div_r,20'hF423C}; // Register 4
            3'd2:    load_data <= {band_r, 26'h300001B};       // Register 3
            3'd3:    load_data <= 32'h6500AE52;                // Register 2
            3'd4:    load_data <= 32'h2000FFF9;                // Register 1
            3'd5:    load_data <= 32'h00607AD8;                // Register 0
            default: load_data <= 32'd0;
          endcase

          // update the current band after sending the request
          if (counter == 3'd2) begin
            update_band     <= 1'b1;
            if (to_roll_over)
              update_rf_div <= 1'b1;
          end

          if (counter == 3'd6) begin // All config words have been sent
            // wait_counter <= 14'd0;
            start_tr     <= 1'b0;
            state        <= GENERATE_CHIRP;
          end else begin
            start_tr     <= 1'b1;
            state        <= WAIT_READY_VCO;
          end

          if (counter == 3'd0) begin
            counter_reset <= 1'b0;
            counter_latch <= 1'b0;
          end

          counter <= counter + 3'd1;

        end

        WAIT_READY_VCO: begin
          if (ready == 1)
            state <= SET_FREQUENCY;
          else
            state <= WAIT_READY_VCO;

          update_band   <= 1'b0;
          update_rf_div <= 1'b0;
          start_tr      <= 1'b0;
        end

        // WAIT_FOR_VCO_LOCK: begin
        //   if(wait_counter == 14'd12000) begin // initializing for DAC
        //     wait_counter <= 14'd0;
        //     state        <= GENERATE_CHIRP;
        //   end else begin // Looping here until time out
        //     wait_counter <= wait_counter + 14'd1;
        //     state        <= WAIT_FOR_VCO_LOCK;
        //   end
        // end

         GENERATE_CHIRP: begin
          if (~started_chirp) begin
            num_bits      <= 6'd24;
            device        <= 1'b0;
            sclk_divider  <= sclk_divider_dac;
            DAC_data      <= start_ramp + jump;
            load_data     <= {DAC_prefix, start_ramp, 12'h000};
            state         <= WAIT_READY_DAC;
            start_tr      <= 1'b1;
            started_chirp <= 1'b1;
          end else if (ended_chirp) begin
            state     <= WAIT_CYCLES;
            started_chirp <= 1'b0;
            ended_chirp   <= 1'b0;
          end else if (DAC_data >= end_ramp) begin
            // set the
            load_data    <= {DAC_prefix, start_ramp, 12'h000};
            state        <= WAIT_READY_DAC;
            start_tr     <= 1'b1;
            ended_chirp  <= 1'b1;
          end else begin
            DAC_data  <= DAC_data + jump;
            // SPI sends from MSB, the messages are 24 bits:
            // 8 bit prefix, 12 bit data, 4 bits zero
            load_data <= {DAC_prefix, DAC_data[11:0], 12'h000};
            state     <= WAIT_READY_DAC;
            start_tr  <= 1'b1;
          end
        end

        WAIT_READY_DAC: begin
          if (ready == 1'b1)
            state <= GENERATE_CHIRP;
          else
            state <= WAIT_READY_DAC;

          start_tr <= 1'b0;
        end

        // Similar to START_CHIRP, used for padding wait time to make loop time
        // a round number.
        // We start sending first register value to VCO here (which is reg2)
        WAIT_CYCLES:
          if (time_counter[1:0] == 2'b11) begin
            state         <= SET_FREQUENCY;
            start_tr      <= 1'b0;
            counter       <= 3'd0;
            num_bits      <= 6'd32;
            device        <= 1'b1;
            sclk_divider  <= 16'd4;
            counter_reset <= 1'b1;
            counter_latch <= 1'b1;
          end else begin
            state         <= WAIT_CYCLES;
          end

        default:
          state <= START_CHIRP;
      endcase
    end
  end

  // Main SPI module for sending out data to data-collector and DAC
  chirp_spi_core # (.DEVICE_COUNT(2)) chirp (
          .clock(clock),
          .reset(full_reset),
          .sending_edge(device), //two devices have different sending edges
          .device({device, ~device}), //1 hot device select, device 1 has MSB bit
          .start(start_tr),
          .num_bits(num_bits),
          .sclk_divider(sclk_divider),
          .set_data(load_data),
          .ready(ready),
          .sen(sen),
          .sclk(sclk),
          .mosi(mosi)
      );

  // ******************************************************** //
  // ********************* DEBUG SIGNALS ******************** //
  // ******************************************************** //

  // Second SPI module for sending the counter value out on the debug port
  wire ready2;
  reg start2;

  // This SPI module is constantly sending out the latched timer value
  // for debugging. When ever it's ready we assert the start signal again.
  always@ (posedge clock)
    if (~full_reset)
      start2 <= 1'b0;
    else if (ready2 & ~start2)
      start2 <= 1'b1;
    else
      start2 <= 1'b0;

  chirp_spi_core counter_out (
          .clock(clock),
          .reset(full_reset),
          .sending_edge(1'b1),
          .device(1'b1),
          .start(~ready2),
          .num_bits(6'd20),
          .sclk_divider(16'd4),
          .set_data({time_counter_latched,12'd0}),
          .ready(ready2),
          .sen(counter_sen),
          .sclk(counter_sclk),
          .mosi(counter_mosi)
      );

  assign debug = {5'b0, state};

endmodule
