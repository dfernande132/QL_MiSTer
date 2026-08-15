//
// mdv.v - Microdrive
//
// Sinclair QL for the MiST
// https://github.com/mist-devel
// 
// Copyright (c) 2015 Till Harbaum <till@harbaum.org> 
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

module mdv
(
   input        clk,   			// System clock (84mHz)
   input        ce,				// CPU clock
	input        reset,
	
	input        reverse,
	
	input        sel,

   // control bits	
	output       gap,
	output       tx_empty,
	output       rx_ready,
	output [7:0] dout,

	// ram interface to read image
	input        download,
	input [16:0] dl_addr,
	input [15:0] dl_data,
	input        dl_wr,
	output [15:0] dl_q,        // QL4M65 fase B: lectura del buffer para el volcado

	// QL4M65 fase B: canal de escritura desde la CPU
	input        wr_en,        // nivel: mctrl[2] (pc..writ) ya sincronizado al dominio del core
	input        wr_strobe,    // pulso de 1 ciclo de clk por cada byte que la CPU escribe en $18022
	input  [7:0] wr_data,      // el byte
	output [7:0] sector,       // indice del sector bajo el cabezal, 0..254
	output       wr_commit     // pulso de 1 ciclo de clk por cada PALABRA confirmada en la RAM
);

reg  [16:0] mem_addr;
reg  [16:0] region_base;    // QL4M65 fase B: base de la region que se esta reproduciendo (D1)
reg         region_state;   // copia de mdv_gap_state para la region que viene (0=cabecera, 1=datos)
reg  [7:0]  mdv_sector;     // QL4M65 fase B: sector bajo el cabezal, 0..254
wire [15:0] mdv_din;

// QL4M65 fase B: puerto A de la RAM, compartido ahora por el cargador/
// volcado de QNICE y por la confirmacion de escritura de la CPU. La
// confirmacion tiene prioridad (D3, microdrive-write-design.md S3.5): en la
// practica solo puede coincidir con un volcado, nunca con una carga (cargar
// y reproducir son excluyentes), y ahi es preferible perder un ciclo de
// volcado (QNICE reintenta, esta en espera) que un dato escrito por el QL.
wire [16:0] pa_addr = wr_do ? wr_addr : dl_addr;
wire [15:0] pa_data = wr_do ? wr_word : dl_data;
wire        pa_wren = wr_do | dl_wr;

dpram #(17, 88000) vram
(
	.wrclock(clk),
	.wraddress(pa_addr),
	.wren(pa_wren),
	.byteena_a(2'b11),
	.data(pa_data),
	.q_a(dl_q),

	.rdclock(clk),
	.rdaddress(mem_addr),
	.q(mdv_din)
);

// a gap is permanently present if no mdv is inserted or if
// there's a gap on the inserted one. This is the signal that triggers
// the irq and can be seen by the cpu
assign gap = (!mdv_present) || mdv_gap /* synthesis keep */;  

// the mdv_rx_ready flag must be quite short as the CPU never waist for it to end
wire mdv_valid = (mdv_bit_cnt[2:0] == 2);
assign rx_ready = mdv_present && mdv_data_valid && mdv_valid;
assign tx_empty = 1'b0;

// microdrive implementation works with images which are uploaded by the user into
// the BRAM. It is then continously replayed from there at 200kbit/s

// determine mdv image size after download
reg [16:0] mdv_end;
always @(posedge clk or posedge reset) begin
	if(reset) mdv_end <= 0;
	else begin
		if(dl_wr) mdv_end <= dl_addr;
	end
end

// the microdrive at 200kbit/s reads a bit every 8.3us and needs a new word
// every 80us.
// gaps are 2800/3400 us which is 35 words at 200kbit/s

assign dout = mdv_bit_cnt[3]?mdv_data[7:0]:mdv_data[15:8];

// a microdrive image is present if at least one word is in the buffer
wire mdv_present = sel && (mdv_end != 0);
reg [3:0] mdv_bit_cnt /* synthesis noprune */;

// also generate gap timing
reg [15:0] mdv_data;
reg mdv_data_valid;
reg mdv_gap;

// microdrive clock runs at 200khz
// -> new word required every 80us
localparam mdv_clk_scaler = 7500000/(200000)-1;

always @(posedge clk) begin
	reg [9:0] mdv_gap_cnt;
	reg mdv_gap_state;
	reg mdv_gap_active;
	reg [7:0] mdv_clk_cnt;

	if(download) begin
		mem_addr <= 0;
		
		// assume we start at the end of a post-sector/pre-header gap
		mdv_gap_cnt <= 10'd0;      // count bytes until gap
		mdv_gap_state <= 1'b1;      // toggle header + data gap
		mdv_gap_active <= 1'b1;     // gap atm
		mdv_gap <= 1'b1; 
	end

	if(ce) begin
		if(mdv_clk_cnt == mdv_clk_scaler) mdv_clk_cnt <= 0;
		else mdv_clk_cnt <= mdv_clk_cnt + 1'd1;

		if(!mdv_clk_cnt) begin
			mdv_bit_cnt <= mdv_bit_cnt + 4'd1;
			if(mdv_bit_cnt == 15) begin
				mdv_data <= mdv_din;
				mdv_data_valid <= !mdv_gap_active && (mdv_gap_cnt > 5) && !(mdv_gap_state && (mdv_gap_cnt > 7) && (mdv_gap_cnt < 12));

				// reset counters when address is out of range
				if(mem_addr > mdv_end) begin
					mem_addr <= 0;
					mdv_sector <= 8'd0;              // QL4M65 fase B: wrap de cinta

					// assume we start at the end of a post-sector/pre-header gap
					mdv_gap_cnt <= 10'd0;      // count bytes until gap
					mdv_gap_state <= 1'b1;      // toggle header + data gap
					mdv_gap_active <= 1'b1;     // gap atm
					mdv_gap <= 1'b1;
				end else begin
					mdv_gap_cnt <= mdv_gap_cnt + 10'd1;

					if(mdv_gap_active) begin

						// QL4M65 fase B (D1): mem_addr no avanza durante el
						// hueco, asi que ya es la direccion de la primera
						// palabra de la region entrante - capturarla aqui,
						// continuamente mientras dure el hueco, deja
						// region_base congelado y correcto en cuanto
						// termine. region_state es el complemento de
						// mdv_gap_state porque el toggle de abajo ocurre en
						// este mismo evento (gap_cnt==34).
						region_base  <= mem_addr;
						region_state <= !mdv_gap_state;

						// stop sending gap after 35 words = 70 bytes = 2800us
						if(mdv_gap_cnt == 34) begin
							mdv_gap_cnt <= 10'd0;            // restart counter until next gap
							mdv_gap_active <= 1'b0;          // no gap anymore
							mdv_gap_state <= !mdv_gap_state; // toggle gap/data
							mdv_gap <= 1'b0;
						end
					end else begin
						mem_addr <= mem_addr + 1'd1;

						if((!mdv_gap_state) && (mdv_gap_cnt == 13)) begin
							// done reading 14 words header data
							mdv_gap_cnt <= 10'd0;            // restart counter for gap
							mdv_gap_active <= 1'b1;          // now comes a gap
							mdv_gap <= 1'b1;
						end else if(mdv_gap_state && (mdv_gap_cnt == 328)) begin
							// done reading 330 words sector data
							mdv_gap_cnt <= 10'd0;            // restart counter for gap
							mdv_gap_active <= 1'b1;          // now comes a gap
							mdv_gap <= 1'b1;
							mdv_sector <= mdv_sector + 8'd1; // QL4M65 fase B: siguiente sector

							if(reverse) begin
								// The sectors on cartridges are written in descending order
								// Some images seem to contain them in ascending order. So we
								// have to replay them backwards for better performance

								if(mem_addr == 343 - 1)
									mem_addr <= mdv_end  - 17'd343 + 1'd1;
								else
									mem_addr <= mem_addr - 17'd686 + 1'd1;
							end
						end
					end
				end
			end
		end
	end
end

// ---------------------------------------------------------------------------------
// QL4M65 fase B: acumulador de bytes de escritura y confirmacion en RAM.
//
// Espejo posicional de la lectura (microdrive-write-design.md S3.4): cada
// palabra escrita se deposita en region_base + indice_de_palabra, sin
// importar cuando llega respecto al recorrido de mem_addr. Sin control de
// flujo (D2): tx_empty se queda en 1'b0 siempre, wr_in_range es la unica
// proteccion necesaria para no desbordar al sector siguiente.
// ---------------------------------------------------------------------------------

assign sector = mdv_sector;

wire        wr_session  = wr_en && mdv_present; // nunca escribir sin cartucho ni sin unidad seleccionada
wire [8:0]  wr_word_idx = wr_byte_cnt[8:1];
wire        wr_in_range = region_state
                        ? (wr_word_idx < 9'd329)  // region de datos
                        : (wr_word_idx < 9'd14);  // region de cabecera (solo FORMAT, no usado en el MVP)

reg  [8:0]  wr_byte_cnt; // 0..537 en una sesion completa; 9 bits sobran
reg  [7:0]  wr_byte_hi;  // primer byte del par (el alto, ver mdv.v:110 - dout sirve mdv_data[15:8] primero)
reg         wr_pending;  // hay medio par acumulado
reg         wr_do;       // pulso: hay una palabra que confirmar en este ciclo
reg  [16:0] wr_addr;
reg  [15:0] wr_word;

// QL4M65 fase B (fix post-M2022): wr_strobe llega desde zx8302.v generado en
// un bloque gateado por cen (~7.5MHz) - una vez a 1'b1, se queda retenido a
// ritmo de clk COMPLETO (84MHz) hasta el siguiente tick de cen, no es un
// pulso de 1 ciclo de clk. Este bloque muestrea a ritmo de clk (sin gatear
// por ce, a diferencia del resto de la maquina de estados de lectura), asi
// que sin deteccion de flanco aqui tambien, contaria cada byte real como
// ~11 bytes (uno por cada ciclo de clk que wr_strobe se mantiene en alto) -
// exactamente el riesgo R1 del diseno, colado un nivel mas adentro de lo
// que protegia la deteccion de flanco de zx8302.v.
reg wr_strobe_prev;

always @(posedge clk) begin
	wr_do <= 1'b0;
	wr_strobe_prev <= wr_strobe;

	if(!wr_session) begin
		wr_byte_cnt <= 9'd0;      // cada sesion empieza de cero
		wr_pending  <= 1'b0;
	end
	else if(wr_strobe && !wr_strobe_prev) begin
		wr_byte_cnt <= wr_byte_cnt + 9'd1;
		if(!wr_pending) begin
			wr_byte_hi <= wr_data;         // byte alto: el primero del par
			wr_pending <= 1'b1;
		end
		else begin
			wr_pending <= 1'b0;
			if(wr_in_range) begin
				wr_addr <= region_base + {8'd0, wr_word_idx};
				wr_word <= {wr_byte_hi, wr_data};
				wr_do   <= 1'b1;
			end
		end
	end
end

assign wr_commit = wr_do;

endmodule
