//
// zx8302.v
//
// ZX8302 for Sinclair QL for the MiST
// https://github.com/mist-devel
// 
// Copyright (c) 2015 Till Harbaum <till@harbaum.org> 
// Copyright (c) 2021 Daniele Terdina
// 
// This source file is free software: you can redistribute it and/or modify 
// it under the terms of the GNU General Public License as published 
// by the Free Software Foundation, either version 3 of the License, or 
// (at your option) any later version. 
// 
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of 
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

//
// QL4M65 port done by Jose Daniel Fernandez Santos (dfsantos) in 2026 and
// licensed under GPL v3
//
// Removed the embedded "ipc" instance (the emulated Intel 8049 IPC
// microcontroller + its keyboard.v/T48 dependents) and exposed its
// comdata/comctrl/audio/ipl signals as top-level ports instead, so that
// MiSTer2MEGA65's own keyboard.vhd (which speaks the comdata/comctrl
// protocol directly, without emulating the 8049) can sit as a sibling
// instance in main.vhd rather than a child of this module. js0/js1/ps2_key
// (only used by the removed ipc's internal keyboard.v for PS/2 and
// joystick-as-keys input) are removed for the same reason - M2M's keyboard
// interface is wired through the new ipc_* ports instead. See
// doc/m2m/exceptions.md for the full list of what changed and why.
//

module zx8302
(
		input          clk,
		input          ce_11m,    	// 11 MHz ipc
      input          reset,
      input          reset_mdv,
		
		// interrupts
		output [1:0]   ipl,
		input          xint,
		
		// interface to watch MDV cartridge upload
		input [16:0]   mdv_dl_addr,
		input [15:0]   mdv_dl_data,
		input          mdv_download,
		input          mdv_dl_wr,

		input          mdv_reverse,
		output         led,
		output         audio,
		
		// vertical synv 
		input          vs,

		// IPC link (QL4M65: was the embedded "ipc" instance, now expects an
		// external IPC - see MiSTer2MEGA65's keyboard.vhd, instantiated as a
		// sibling in main.vhd)
		input          ipc_comctrl_i,   // strobe FROM the external IPC
		output         ipc_comdata_o,   // this chip's own outgoing bit, TO the external IPC
		input          ipc_comdata_i,   // the external IPC's outgoing bit, FROM the external IPC
		input  [1:0]   ipc_ipl_i,       // interrupt-priority lines FROM the external IPC
		input          ipc_audio_i,     // audio bit FROM the external IPC

      // bus interface
		input				cep,
		input				cen,

		input          ce_131k,
		input  [32:0]  rtc_data,		// Seconds since 1970-01-01 00:00:00

		input				cpu_sel,
		input				cpu_wr,
		input [1:0] 	cpu_addr,      // a[5,1]
		input			 	cpu_uds,
		input			 	cpu_lds,
		input [15:0]   cpu_din,
		output [15:0]  cpu_dout
		
);


// comdata shift register
wire ipc_comdata_in = comdata_reg[0];
reg [3:0] comdata_reg /* synthesis noprune */;
reg [1:0] ipc_busy;
reg comdata_to_cpu;
reg prev_ipc_comctrl;


// ---------------------------------------------------------------------------------
// ----------------------------- CPU register write --------------------------------
// ---------------------------------------------------------------------------------

reg [7:0] mctrl;


// Handles:
// 1. CPU is writing to registers
// 2. reset
// 3. comctrl from IPC

always @(posedge clk) begin
	if (reset) begin
		comdata_reg <= 4'b0000;
		ipc_busy <= 2'b11;
	end
	else if(cen) begin
		irq_ack <= 5'd0;


		// cpu writes to 0x18XXX area
		if(cpu_sel && cpu_wr) begin
			// even addresses have uds asserted and use the upper 8 data bus bits
			if (cpu_uds) begin
				// cpu writes microdrive control register
				if(cpu_addr == 2'b10)
					mctrl <= cpu_din[15:8];
			end

			// odd addresses have lds asserted and use the lower 8 data bus bits
			if (cpu_lds) begin
				// 18003 - IPCWR
				// (host sends a single bit to ipc)
				if(cpu_addr == 2'b01) begin
					// data is ----XEDS
					// S = start bit (should be 0)
					// D = data bit (0/1)
					// E = stop bit (should be 1)
					// X = extra stopbit (should be 1)
					comdata_reg <= cpu_din[3:0];
					ipc_busy <= 2'b11;		// Show IPC BUSY until the IPC asserts COMCTL twice
				end

				// cpu writes interrupt register
				if(cpu_addr == 2'b10) begin
					irq_mask <= cpu_din[7:5];
					irq_ack <= cpu_din[4:0];
				end
			end
		end
	end
	if (!ipc_comctrl_i && prev_ipc_comctrl) begin
		comdata_to_cpu <= zx8302_comdata_in;	// Latch COMDATA since the IPC will quickly reset it to 1 when sending data
		comdata_reg <= { 1'b1, comdata_reg[3:1] };
		ipc_busy <= { 1'b0, ipc_busy[1] };
	end
	prev_ipc_comctrl <= ipc_comctrl_i;
end

// ---------------------------------------------------------------------------------
// ----------------------------- CPU register read ---------------------------------
// ---------------------------------------------------------------------------------

// status register read
// bit 0       Network port
// bit 1       Transmit buffer full
// bit 2       Receive buffer full
// bit 3       Microdrive GAP
// bit 4       SER1 DTR
// bit 5       SER2 CTS
// bit 6       IPC busy
// bit 7       COMDATA

wire [7:0] io_status = { comdata_to_cpu, ipc_busy[0], 2'b00,
		mdv_gap, mdv_rx_ready, mdv_tx_empty, 1'b0 };

assign cpu_dout =
	// 18000/18001 and 18002/18003
	(cpu_addr == 2'b00)?rtc[31:16]:
	(cpu_addr == 2'b01)?rtc[15:0]:

	// 18020/18021 and 18022/18023
	(cpu_addr == 2'b10)?{io_status, irq_pending}:
	(cpu_addr == 2'b11)?{mdv_byte, mdv_byte}:

	16'h0000;	

// ---------------------------------------------------------------------------------
// ------------------------------ IPC (external) ------------------------------------
// ---------------------------------------------------------------------------------
//
// QL4M65: the 8049 IPC used to be instantiated right here (see header note).
// This chip's own outgoing comdata bit is exposed on ipc_comdata_o; the
// external IPC's outgoing bit, strobe and interrupt-priority lines come in
// on ipc_comdata_i/ipc_comctrl_i/ipc_ipl_i; audio is now an input too.

assign ipc_comdata_o = ipc_comdata_in;

// 8302 sees its own comdata as well as the one from the external IPC
wire zx8302_comdata_in = ipc_comdata_in && ipc_comdata_i;

assign audio = ipc_audio_i;

// ---------------------------------------------------------------------------------
// -------------------------------------- IRQs -------------------------------------
// ---------------------------------------------------------------------------------

wire [7:0] irq_pending = {1'b0, (mdv_sel == 0), rtc[0], xint_irq, vsync_irq, 1'b0, 1'b0, gap_irq };
reg [2:0] irq_mask;
reg [4:0] irq_ack;

// any pending irq raises ipl to 2 and the ipc can control both ipl lines
//
// QL4M65: as written, this ANDs ipc_ipl_i[1] with "no irq pending" - the
// exact opposite of the comment above it. With the real embedded ipc (now
// removed, see file header), ipc_ipl_i[1] apparently defaulted high whenever
// the ipc itself had nothing else to report, so this line acted as a
// defensive clamp. Our external stand-in (keyboard.vhd) permanently drives
// ipc_ipl_i to "00" (it never implements the ipc's real serial poll-and-relay
// protocol for zx8302's own interrupts - only its own keyboard commands 8/9)
// - so with the AND, this line silenced ipl[1] unconditionally, including
// zx8302's own vsync_irq (the ~50Hz frame interrupt Minerva/QDOS's scheduler
// depends on) - the leading suspect for the reproducible post-RAM-test hang
// seen in M1001-M1005 hardware tests. Changed to OR + not-equal so any
// zx8302-internal pending irq (xint/vsync/gap) can raise ipl[1] on its own,
// independent of the external ipc - matching the comment's literal intent.
assign ipl = { ipc_ipl_i[1] || (irq_pending[4:0] != 0), ipc_ipl_i[0] };

// vsync irq is set whenever vsync rises
reg vsync_irq;
wire vsync_irq_reset = reset || irq_ack[3];
always @(posedge clk) begin
	reg old_vs;
	
	old_vs <= vs;
	if(vsync_irq_reset)   vsync_irq <= 1'b0;
	else if(~old_vs & vs) vsync_irq <= 1'b1;
end

// toggling the mask will also trigger irqs ...
wire gap_irq_in = mdv_gap && irq_mask[0];
reg gap_irq;
wire gap_irq_reset = reset || irq_ack[0];
always @(posedge clk) begin
	reg old_irq;
	
	old_irq <= gap_irq_in;
	if(gap_irq_reset)              gap_irq <= 1'b0;
	else if(~old_irq & gap_irq_in) gap_irq <= 1'b1;
end

// toggling the mask will also trigger irqs ...
wire xint_irq_in = xint && irq_mask[2];
reg xint_irq;
wire xint_irq_reset = reset || irq_ack[4];
always @(posedge clk) begin
	reg old_irq;
	
	old_irq <= xint_irq_in;
	if(xint_irq_reset)              xint_irq <= 1'b0;
	else if(~old_irq & xint_irq_in) xint_irq <= 1'b1;
end


// ---------------------------------------------------------------------------------
// ----------------------------------- microdrive ----------------------------------
// ---------------------------------------------------------------------------------
//
// QL4M65: removed the "mdv mdv (...)" instance for milestone 1 (microdrive
// is milestone 3, not implemented yet). mdv.v itself instantiates "dpram"
// (the Quartus-specific altsyncram wrapper - see doc/m2m/exceptions.md),
// which would otherwise get pulled into the Vivado build transitively even
// though microdrive support isn't used. Tied to a "no drive present" state
// instead - re-instantiate mdv (and give it a Vivado-clean dpram, same
// treatment ql_rom/vram already got) when milestone 3 is implemented.

wire mdv_gap      = 1'b0;
wire mdv_tx_empty = 1'b1;
wire mdv_rx_ready = 1'b0;
wire [7:0] mdv_byte = 8'h00;

assign led = mdv_sel[0];

// the microdrive control register mctrl generates the drive selection
reg [7:0] mdv_sel;
always @(posedge clk) begin
	reg old_mctrl;
	
	old_mctrl <= mctrl[1];
	if(old_mctrl & ~mctrl[1]) mdv_sel <= { mdv_sel[6:0], mctrl[0] };
end

// ---------------------------------------------------------------------------------
// -------------------------------------- RTC --------------------------------------
// ---------------------------------------------------------------------------------

reg [31:0] rtc;
reg [17:0] divClk;
always @(posedge clk) begin
	reg old_stb;

	if (ce_131k)
	begin
		divClk <= divClk + 18'd1;
		if (divClk == 18'd131249) divClk <= 0;
		if (!divClk) rtc <= rtc + 1'd1;
	end

	// QL base is 1961-01-01 00:00:00
	// MiSTer base is 1970-01-01 00:00:00
	// Difference is 283996800 seconds (9 years + 2 leap days)
	
	// Bootstrap clock 
	old_stb <= rtc_data[32];
	if (old_stb != rtc_data[32]) rtc <= {32'd283996800 + rtc_data[31:0]};
end

endmodule
