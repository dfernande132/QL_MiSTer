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
// MiSTer2MEGA65's own IPC (CORE/vhdl/ipc.vhd - a straight structural port
// of rtl/ipc.v, instantiating the real T48 core as a sibling instance in
// main.vhd instead of a child of this module) can plug in the same way.
// js0/js1/ps2_key (only used by the removed ipc's internal keyboard.v for
// PS/2 and joystick-as-keys input) are removed for the same reason - M2M's
// keyboard interface (CORE/vhdl/keyboard.vhd) feeds ipc.vhd directly
// instead. See doc/m2m/exceptions.md for the full list of what changed and
// why.
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

		// QL4M65 (Milestone 2 phase A): external mdv1 sibling instance (see
		// main.vhd) - mdv_dl_addr/mdv_dl_data/mdv_download/mdv_dl_wr above are
		// unused leftovers from the original design (never referenced inside
		// this module even before our own port; the loader in main.vhd drives
		// the external mdv instance's own dl_* ports directly instead). This
		// chip's own mdv_sel register is exposed so main.vhd can select which
		// physical drive (if any) is backing the currently-selected number;
		// mdv1_*_i are drive 1's real outputs, muxed in below in place of the
		// "no drive present" placeholders whenever mdv_sel[0] is set. See
		// doc/m2m/exceptions.md.
		output [7:0]   mdv_sel_o,
		input          mdv1_gap_i,
		input          mdv1_tx_empty_i,
		input          mdv1_rx_ready_i,
		input  [7:0]   mdv1_byte_i,

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
		// QL4M65 (M1033): was 4'b0000 (comdata_reg[0]=0, i.e. the ipc_comdata_o
		// line reads DRIVEN/LOW immediately after reset) - changed to idle-high
		// (4'b1111), the electrically correct state for a wired-AND bus with
		// nobody actively driving it low. With the real emulated 8049 (M1031,
		// ipc.vhd), the real firmware's own "wait for line idle" receive loop
		// (rtl/ipc8049.hex disassembly, Anexo B.5 of DECISIONES.md: "espera
		// linea a 0" before each nibble receive) reads this line as "the CPU
		// wants to talk RIGHT NOW" the instant it starts polling - if it never
		// naturally settles to idle-high, the firmware can end up perpetually
		// re-triggering a phantom receive against reset garbage instead of
		// ever cleanly waiting for a REAL CPU-initiated transfer. See
		// DECISIONES.md for the full M1031/M1032/M1033 investigation.
		comdata_reg <= 4'b1111;
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

// QL4M65 (M1038): bit1 = "interface" (pc.intri on real hardware - "happens
// whenever data transfer goes on between CPU and IPC", per inc/pc in
// Minerva's own source). Never implemented before (hardcoded 0 since the
// very first zx8302.v port) - found missing while chasing why real
// SuperBASIC's own keyboard-read code never gets unblocked (M1036/M1037):
// it busy-waits on a flag that only an interrupt can clear, right after
// calling a routine that ends with the exact same "write to pc_intr to
// acknowledge talking to the IPC" pattern mt_ipcom uses everywhere else -
// but nothing was ever raising this specific interrupt for it to
// acknowledge. Approximated as: fires once a real comdata/comctrl exchange
// with the external IPC completes (ipc_busy transitions to idle). Unlike
// the external ipc_ipl_i (which can get stuck - see the ipl comment below),
// this is a zx8302-internal, self-contained flag cleared the normal way via
// irq_ack[1] (bit already reserved in the 5-bit ack register, unused until
// now) - Minerva's own mt_ipcom already does this unconditionally after
// every IPC exchange, so it can never get stuck the way M1031-M1034's raw
// ipl[1] did.
reg intri_irq;
wire intri_irq_reset = reset || irq_ack[1];
always @(posedge clk) begin
	reg old_ipc_busy;

	old_ipc_busy <= |ipc_busy;
	if(intri_irq_reset)                     intri_irq <= 1'b0;
	else if(old_ipc_busy && !(|ipc_busy))   intri_irq <= 1'b1;
end

wire [7:0] irq_pending = {1'b0, (mdv_sel == 0), rtc[0], xint_irq, vsync_irq, 1'b0, intri_irq, gap_irq };
reg [2:0] irq_mask;
reg [4:0] irq_ack;

// any pending irq raises ipl to 2 and the ipc can control both ipl lines
//
// QL4M65: OR, not AND - any zx8302-internal pending irq (xint/vsync/gap/
// intri) can raise ipl[1] on its own, independent of the external ipc.
// This matters because vsync_irq (the ~50Hz tick the whole QDOS scheduler
// depends on) MUST be able to reach the CPU regardless of what the
// external ipc's own ipl_i line is doing - AND was tried twice (this
// project's very first hang, M1001-M1005, and again in M1036 with the real
// 8049 wired in) and both times it silenced vsync_irq whenever ipc_ipl_i
// happened to be low (i.e. almost always), freezing the whole system.
// main.vhd currently ties ipc_ipl_i to "00" (see its own comment) because
// the real 8049 can assert ipl_i[1] without ever having it cleared again -
// with that permanently unconnected, this OR effectively just passes
// through zx8302's own internal irq_pending. See DECISIONES.md's M1006/
// M1031-M1037 sections for the full investigation.
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
// QL4M65 (Milestone 2 phase A): "mdv mdv (...)" was removed for milestone 1
// (microdrive wasn't implemented yet at the time) since mdv.v itself
// instantiates "dpram" (the Quartus-specific altsyncram wrapper - see
// doc/m2m/exceptions.md), which would otherwise get pulled into the Vivado
// build transitively even when unused. Now that a real mdv1 sibling exists
// (main.vhd, with a Vivado-clean dpram replacement), mdv_tx_empty/
// mdv_rx_ready/mdv_byte are muxed: mdv1's real outputs when mdv_sel[0] is
// set (drive 1 selected), the original "no drive present" placeholder
// otherwise (mdv_sel==0, or mdv_sel selecting drives 2-8 - phase D territory,
// not backed yet).

wire mdv_tx_empty = mdv_sel[0] ? mdv1_tx_empty_i : 1'b1;
wire mdv_rx_ready = mdv_sel[0] ? mdv1_rx_ready_i : 1'b0;
wire [7:0] mdv_byte = mdv_sel[0] ? mdv1_byte_i : 8'h00;

assign led = mdv_sel[0];
assign mdv_sel_o = mdv_sel;

// the microdrive control register mctrl generates the drive selection
reg [7:0] mdv_sel;
always @(posedge clk) begin
	reg old_mctrl;

	old_mctrl <= mctrl[1];
	if(old_mctrl & ~mctrl[1]) mdv_sel <= { mdv_sel[6:0], mctrl[0] };
end

// QL4M65 (M1040): mdv_gap was hardcoded 1'b0 ("no drive present") since the
// project began - milestone 3 (microdrive) was never implemented, so this
// seemed harmless. It is NOT harmless: found while chasing why real
// SuperBASIC (any ROM - Minerva, MGE, all tested) never responds to the
// keyboard after boot. Minerva's own sb/start.asm looks for a boot file on
// mdv1_ right after the F1-F4 screen resolves, via dd/mdvop.asm's
// "wait: tst.b md_estat(a2); bgt.s wait" - and md_estat can ONLY ever be
// set (to success or, in our no-medium case, error -1) by md_serve
// (md/serve.asm), which is ONLY ever invoked by a genuine gap interrupt
// (ss_int2.asm's gpint branch). With mdv_gap permanently low, that
// interrupt never fires even once, so md_serve/md_sectr never run, and the
// wait never ends - md_estat is simply never touched again after dd_mdvop
// sets it to 1. Confirmed byte-for-byte against a real disassembly of our
// own ROM: the busy-wait sits at offset $23 from its base register, exactly
// md_estat's real offset in inc/md's physical definition block.
//
// The fix does NOT need real microdrive hardware: QDOS's own downstream
// code (md/endgp.asm's two polling loops, md_sectr's "not a sector header"/
// "unreadable" paths) already has generous ~0.5s software timeouts of its
// own and converges cleanly to "no medium found" on its own once it's
// actually invoked - the only missing piece is a single gap interrupt to
// start that chain. Generate a slow, periodic gap pulse whenever any
// microdrive is selected (mdv_sel!=0) - approximates "motor spinning, no
// cartridge" without needing a real microdrive data channel; exact timing
// doesn't matter given QDOS's own generous timeouts on the far end.
reg [20:0] mdv_gap_cnt;
reg        mdv_gap_r;
always @(posedge clk) begin
	if (reset || mdv_sel == 0) begin
		mdv_gap_cnt <= 21'd0;
		mdv_gap_r   <= 1'b0;
	end
	else if (ce_131k) begin
		if (mdv_gap_cnt == 21'd16384) begin  // ~125ms @ 131.25kHz ce_131k
			mdv_gap_cnt <= 21'd0;
			mdv_gap_r   <= 1'b1;              // one-cycle (at ce_131k rate) pulse
		end
		else begin
			mdv_gap_cnt <= mdv_gap_cnt + 21'd1;
			mdv_gap_r   <= 1'b0;
		end
	end
	else
		mdv_gap_r <= 1'b0;
end
// QL4M65 (Milestone 2 phase A): real mdv1 gap timing when drive 1 is
// selected, the M1040 placeholder pulse otherwise (see its own comment
// above - still needed for mdv_sel==0 and for drives 2-8, not backed yet).
wire mdv_gap = mdv_sel[0] ? mdv1_gap_i : mdv_gap_r;

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
