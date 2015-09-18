-----------------------------------------------------------------------------------------------------------------------------------------------
---- Name: Manoj Vishwanathan
---- 
---- 
---- email:mvn8c@mst.edu
---- 
---- Instructor : Dr. William Hanna
---- 
---- Course name: Computer Engineering 315: Digital Computer Design 
---- 
---- Final project: Design of 32 bit pipelined MIPS microprocessor
---- 
---- Missouri University of Science and Technology,Rolla 
---- 
---- Date of Submission: 12/10/2013
----
----------------------------------------------------------------------------------------------------------------------------------------------
library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.STD_LOGIC_ARITH.UNSIGNED;
use IEEE.STD_LOGIC_UNSIGNED.all;
use IEEE.NUMERIC_STD.UNSIGNED;
use STD.TEXTIO.all;
--
package ProcP is
	type REG32 			is Array (0 to 31) of std_logic_vector(31 downto 0);
	type MEM1K 			is Array(0 to 1000)of std_logic_vector(31 downto 0);
	type Popcode 		is (LW, SW, Jr, Jump, Beq, Bne, Slt, Add, Sub, Mpy, Not32, Comp, And32, Or32, Xor32, MSLL, MSRL, MSLA, MSRA);
	type Pstate 		is (Fetch, Decode, Execute, Retire);
	type Tinstruction 	is record
		Opcode: Popcode;
		Rs: std_logic_vector (0 to 4);
		Rt: std_logic_vector (0 to 4);
		Rd: std_logic_vector (0 to 4);
		SHAMT: std_logic_vector (0 to 4);
		Funct: std_logic_vector (0 to 5);
	end record;
	function to_std_logic(c: character) return std_logic;
	function to_std_logic_vector(s: string) return std_logic_vector;
	function Conv (temp: string (1 to 32)) return Tinstruction;
	function to_record (PW: std_logic_vector (31 downto 0)) return Tinstruction;
	function to_string (PW: std_logic_vector) return string;
end package ProcP;
--
package body ProcP is
--
	function to_std_logic(c: character) return std_logic is 
		variable sl: std_logic;
	begin
	case c is
		when 'U' => 
			sl := 'U'; 
		when 'X' =>
			sl := 'X';
		when '0' => 
			sl := '0';
		when '1' => 
			sl := '1';
		when 'Z' => 
			sl := 'Z';
		when 'W' => 
			sl := 'W';
		when 'L' => 
			sl := 'L';
		when 'H' => 
			sl := 'H';
		when '-' => 
			sl := '-';
		when others =>
			sl := 'X'; 
		end case;
		return sl;
	end to_std_logic;
--
	function to_std_logic_vector(s: string) return std_logic_vector is
		variable slv: 	std_logic_vector(s'high-s'low downto 0);
		variable k: 	integer;
	begin
		k := s'high-s'low;
		for i in s'range loop
			slv(k) := to_std_logic(s(i));
			k := k - 1;
		end loop;
		return slv;
	end to_std_logic_vector;
--
	function Conv (temp: string (1 to 32)) return Tinstruction is
		variable con: 	std_logic_vector (31 downto 0);
		variable T: 	Tinstruction;
	begin
		--convert std_logic_vector to record??	
		con := to_std_logic_vector(temp);
		T 	:= to_record(con);
		return T;
	end function Conv;
--
	function to_record (PW: std_logic_vector (31 downto 0)) return Tinstruction is
		variable T: Tinstruction;
	begin
	
		case PW(31 downto 26) is
			when "000000" =>
				case PW(5 downto 0) is
					when "001000" =>
						T.opcode := jr;
						--jr
					when "101010" =>
						T.opcode := slt;
						--slt
					when "100000" =>
						T.opcode := add;
						--add
					when "100010" =>
						T.opcode := sub;
						--sub
					when "011000" =>
						T.opcode := mpy;
						--mpy
					when "100100" =>
						T.opcode := and32;
						--and
					when "100101" =>
						T.opcode := or32;
						--S:/Music/Manoj_vishwanathan_final.vhdor
					when "100110" =>
						T.opcode := xor32;
						--xor
					when "000000" =>
						T.opcode := msll;
						--msll (sll)
					when "000010" =>
						T.opcode := msrl;
						--msrl (srl)
					when "000001" =>
						--not on green card
						T.opcode := msla;
						--msla (sla)
					when "000011" =>
						T.opcode := msra;
						--msra (sra)
					when "101100" =>
						--not on green card
						T.opcode := comp;
						--S:/Music/Manoj_vishwanathan_final.vhdcomp
					when "101000" =>
						--not on green card
						T.opcode := not32;
						--not
					when others =>
				end case;
			when "100011" =>
				T.opcode := lw;
				--lw
			when "101011" =>
				T.opcode := sw;
				--sw
			when "000010" =>
				T.opcode := jump;
				--jump
			when "000100" =>
				T.opcode := beq;
				--beq
			when "000101" =>
				T.opcode := bne;
				--bne
			when "010001" =>
				--(2)
			when others =>
		end case;
	
		-- T.Opcode	:= PW(31 downto 26);
		
		T.Rs 		:= PW(25 downto 21);
		T.Rt 		:= PW(20 downto 16);
		T.Rd 		:= PW(15 downto 11);
		T.SHAMT 	:= PW(10 downto 6);
		T.funct 	:= PW(5 downto 0);
		
		return T;
	end function to_record;
--
	function to_string(PW: std_logic_vector) return string is
		use std.TextIO.all;
		variable bv: bit_vector(PW'range) := to_bitvector(PW);
		variable lp: line;
	begin
		write(lp, bv);
		return lp.all;
	end;
--
end package body ProcP;

--
library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;
--
use work.ProcP.all;
use work.all;
--
entity ALU_32 is
	port (	A_bus, B_bus: in std_logic_vector(31 downto 0);
			Q_bus: inout std_logic_vector(63 downto 0);
			Opcode: in Popcode;
			Proc_ready: out std_logic;
			clk, reset: in std_logic);
end entity ALU_32;
--
architecture Behavior of ALU_32 is
	signal A, B: 		std_logic_vector(31 downto 0):=(others => '0');
	signal Q: 			std_logic_vector(63 downto 0):=(others => '0');
	signal Overflow: 	std_logic;
	signal Status_code: std_logic_vector (0 to 3);
--
	procedure action (Bus_A, Bus_B: in std_logic_vector (31 downto 0); signal Bus_Q: inout std_logic_vector (63 downto 0); B_Opcode: in Popcode) is
		variable temp_Q: 	std_logic_vector(31 downto 0);
		variable zeros: 	std_logic_vector(31 downto 0):=(others => '0');
		variable ones: 		std_logic_vector(31 downto 0):=(others => '1');
	begin
		case B_Opcode is
			when LW => Bus_Q <= zeros(31 downto 0) & std_logic_vector(signed(Bus_A) + signed(Bus_B));
				
			when SW => Bus_Q <= zeros(31 downto 0) & std_logic_vector(signed(Bus_A) + signed(Bus_B));
				
			when Jr => Bus_Q <= zeros(31 downto 1) & std_logic_vector(signed('0' & Bus_A) + signed('0' & Bus_B));
				
			when JUMP => Bus_Q <= zeros(31 downto 0) & Bus_A;
				
			when Bne =>
				if Bus_A /= Bus_B then
					-- Branch
					Bus_Q <= zeros(31 downto 0) & zeros(31 downto 1) & "1";
				else
					Bus_Q <= zeros(31 downto 0) & zeros(31 downto 0);
				end if;
				
			when Beq =>
				if Bus_A = Bus_B then
					Bus_Q <= zeros(31 downto 0) & zeros(31 downto 1) & "1";
				else
					Bus_Q <= zeros(31 downto 0) & zeros(31 downto 0);
				end if;
				
			when Slt =>
				if Bus_A < Bus_B then Bus_Q <= zeros(31 downto 0) & zeros(31 downto 1) & "1";
				else Bus_Q <= zeros(31 downto 0) & zeros(31 downto 1) & "0";
				end if;
			
			when MSLL =>
				Bus_Q <= zeros(31 downto 0) & std_logic_vector(shift_left( signed(Bus_A), to_integer(signed(Bus_B)) ));
			
			when MSRL =>
				-- shift_right replaces upper new bits with sign bit
				-- need to replace those with 0s for SRL
				temp_Q := std_logic_vector(shift_right( signed(Bus_A), to_integer(signed(Bus_B)) ));
				Bus_Q <= zeros(31 downto 0) & (temp_Q and (zeros(31 downto (31 - to_integer(signed(Bus_B)) + 1)) & ones((31 - to_integer(signed(Bus_B))) downto 0)));
			
			when MSLA =>
				Bus_Q <= zeros(31 downto 0) & std_logic_vector(shift_left( signed(Bus_A), to_integer(signed(Bus_B)) ));
			
			when MSRA =>
				Bus_Q <= zeros(31 downto 0) & std_logic_vector(shift_right( signed(Bus_A), to_integer(signed(Bus_B)) ));
			
			when others => 	Bus_Q <= Bus_Q; -- Execute a NOP
		end case;
	end procedure action;

begin
	A<= A_bus;
	B<= B_bus;
--
	ALU_Exec: process(clk, reset, Opcode)
		variable zeros: std_logic_vector(31 downto 0):=(others => '0');
	begin
		if reset='0' and clk'event and clk='1' then
			case Opcode is
				when Add=> Q <= zeros(31 downto 0) & std_logic_vector(signed(A) + signed(B));
					if unsigned(Q) > 16#FFFF# then
						overflow<='1';
						assert not(overflow='1') report "An overflow occurred - ADD" severity warning;
					end if;
				when Sub=> Q <= zeros(31 downto 0) & std_logic_vector(signed(A) - signed(B));
					if signed(B) > signed(A) then
						overflow<='1';
						assert not(overflow='1') report "An overflow occurred - SUB" severity warning;
					end if;
				when MPY=> Q <= std_logic_vector(signed(A) * signed(B));
					if unsigned(Q) > 16#FFFF# then
						overflow<='1';
						assert not(overflow='1') report "An overflow occurred - MPY" severity warning;
					end if;
			--	when Comp=>
					--if (A<B) then Status_code (0 to 1) <= "01"; 	--report "Less Than"
				--	elsif (A=B) then Status_code (0 to 1) <= "00"; 	--report "Equal"
					--else Status_code(0 to 1) <= "10"; 				--report "Greater Than"
					--end if;
					
					Q <= zeros(31 downto 0) & zeros(31 downto 2) & Status_code(0 to 1);
				when And32 => 	Q<= zeros(31 downto 0) & (A and B);
				when Or32 => 	Q<= zeros(31 downto 0) & (A or B);
				when Not32 => 	Q<= zeros(31 downto 0 ) & not A;
				when LW => 		action (A, B, Q, Opcode);
				when SW => 		action (A, B, Q, Opcode);
				when Jr => 		action (A, B, Q, Opcode);
				when JUMP => 	action (A, B, Q, Opcode);
				when Bne => 	action (A, B, Q, Opcode);
				when Beq => 	action (A, B, Q, Opcode);
				when Slt => 	action (A, B, Q, Opcode);
				when MSLL => 	action (A, B, Q, Opcode);
				when MSRL => 	action (A, B, Q, Opcode);
				when MSLA => 	action (A, B, Q, Opcode);
				when MSRA => 	action (A, B, Q, Opcode);
				when others => 	Q <= zeros(31 downto 0) & B; -- Execute a NOP
			end case;
		end if;
	Q_bus <= Q; -- Result deposited to Q_bus
	Proc_ready <= '1';
	end process;
end architecture behavior; 

library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;
--
use work.Procp.all;
use work.all;


--
entity MCProc is
	port (	PC, PW		: inout std_logic_vector (31 downto 0) := (others => '0');
			CLK, Reset	: in std_logic;
			Instruction_mem		: inout MEM1K );
end entity MCProc;
--
architecture First of MCProc is
	--
	component ALU_32
		port (	A_bus, B_bus	: in std_logic_vector(31 downto 0);
				Q_bus			: inout std_logic_vector(63 downto 0);
				Opcode			: in Popcode;
				Proc_ready		: out std_logic;
				clk, reset		: in std_logic);
	end component ALU_32;
	--
	signal A, B: std_logic_vector (31 downto 0):= (others => '0');
	signal Rout: std_logic_vector (4 downto 0);
	signal Q: std_logic_vector (63 downto 0):=(others => '0');
	Signal Reg_Input, Reg_output: REG32:= ((others=> (others=>'0')));
	signal Opcode: Popcode;
	signal Proc_ready: std_logic := '0';
	signal Instruction: Tinstruction;
	signal Jaddress: Std_logic_vector (25 downto 0);
	 signal Offset12: Std_logic_vector (31 downto 0);
	--signal STATE: PSTATE := Fetch;
		
	
	
	signal MemRead, MemWrite, RegWrite: Std_logic :='0';
  signal MemReadF, MemWriteF, RegWriteF: Std_logic:= '0';
  signal RegWriteF2, MemReadF2: Std_logic:= '0';
  signal Offset: Std_logic_vector (15 downto 0);
	--
	for ALU_32C: ALU_32 use entity work.ALU_32(Behavior);
	--
begin
  
	ALU_32C: ALU_32 port map (A, B, Q, Opcode, Proc_ready, CLK, Reset);
	    Reg_Input(0) <= "00000000000000001111111111111111";
			Reg_Input(1) <= "00000000000000000000000000000011";
			Reg_Input(2) <= "11111111111111111111111111111111";
			Reg_Input(3) <= "00011111111111111110000000000010";
			Reg_Input(4) <= "10101010101010101010101010101010";
			Reg_Input(5) <= "01010101010101010101010101010101";
			Reg_Input(6) <= "11110000111100001111000011110000";
			Reg_Input(7) <= "00000000000000000000000000000010"; 
	    Reg_Input(8) <= "00000000000000000000000000000000"; --0
	    Reg_Input(9) <= "00000000000000000000000000000001"; --1
			Reg_Input(10) <= "00000000000000000000000000000011"; --3
			Reg_Input(11) <= "00000000000000000000000000100010"; --
			Reg_Input(12) <= "00000000000000000000000001111110";
			Reg_Input(13) <= "00000000110000100100100101000010";
			Reg_Input(14) <= "00000000000011110000000000000010";
			Reg_Input(15) <= "11111111111111110000000000000000";
			
		
			
	PControl: Process (reset,clk)
		variable zeros: std_logic_vector(31 downto 0):=(others => '0');
		variable ones : std_logic_vector(31 downto 0):=(others => '1');
		variable temp : std_logic_vector(31 downto 0):=(others => '0');
	begin
		if(Reset = '1' and CLK'event and CLK='1') then
			-- Test instructions loaded on reset
			Instruction_mem(0) <= "00000000001000100000000000100000"; --Add R0=R1+R2 
      Instruction_mem(1) <= "00000000100000110000100000100010"; --Sub R1=R4-R3
      Instruction_mem(2) <= "00000000100001010001000000100101"; --Or R2=R4|R5
      Instruction_mem(3) <= "00000000110001110001100000100100"; --And R3=R6&R7
      Instruction_mem(4) <= "00000001000000010010000000000000"; --MSLL R8=R4
      Instruction_mem(5) <= "00000001100000010010100000000001"; --MSLA R12=R5
      Instruction_mem(6) <= "00000001101000010011000000000011"; --MSRA R13=R6
      Instruction_mem(7) <= "00000001011000010110000000000010"; --MSRL R11=R12
      Instruction_mem(8) <= "00010001001010100011100000100110"; --BEQ R1,R5 
      Instruction_mem(9) <= "00000001100011010100100000101010"; --SLT R9=R12<R13
      Instruction_mem(10) <= "00000001101011000101000000101010"; --SLT R10=R13<R12
      Instruction_mem(11) <= "00000000101011110101100000011000"; --MPY R11=R5*R15
      Instruction_mem(12) <= "00000001111000000100000000101000"; --NOT R8 = Not R15
      
      Instruction_mem(13) <= "00001001111000000100000000101000"; --JUMP $14 
      Instruction_mem(14) <= "00000000100000110000100000100010"; --Sub R1=R4-R3
      Instruction_mem(15) <= "00000000100001010001000000100101"; --Or R2=R4|R5
      Instruction_mem(16) <= "00000001000000010010000000000000"; --MSLL R8=R4
      Instruction_mem(17) <= "00000001000000010010000000000000"; --MSLL R8=R4
      Instruction_mem(18) <= "00000000110001110001100000100100"; --And R3=R6&R7
      Instruction_mem(19) <= "00000001101000010011000000000011"; --MSRA R13=R6
      Instruction_mem(20) <= "00000001011000010110000000000010"; --MSRL R11=R12
      Instruction_mem(21) <= "00010100001000100000000000100000"; --BNE R1,R4
      
      Instruction_mem(22) <= "00000001100011010100100000101010"; --SLT R9=R12<R13
      Instruction_mem(23) <= "00000001101011000101000000101010"; --SLT R10=R13<R12
      Instruction_mem(24) <= "00000000101011110101100000011000"; --MPY R11=R5*R15
      Instruction_mem(25) <= "00000001111000000100000000101000"; --NOT R8 = Not R15
      
      Instruction_mem(26) <= "00000000001000100000000000100000"; --Add R0=R1+R2 
      Instruction_mem(27) <= "00000000100000110000100000100010"; --Sub R1=R4-R3
      Instruction_mem(28) <= "00000000100001010001000000100101"; --Or R2=R4|R5
      Instruction_mem(29) <= "00000001100000010010100000000001"; --MSLA R12=R5
      Instruction_mem(30) <= "00000000110001110001100000100100"; --And R3=R6&R7
      Instruction_mem(31) <= "00000001100000010010100000000001"; --MSLA R12=R5
      Instruction_mem(32) <= "00000001101000010011000000000011"; --MSRA R13=R6
      Instruction_mem(33) <= "00000001011000010110000000000010"; --MSRL R11=R12
      Instruction_mem(34) <= "00000001001010100011100000100110"; --XOR R10=R9 xor R7 
      Instruction_mem(35) <= "00000001100011010100100000101010"; --SLT R9=R12<R13
      Instruction_mem(36) <= "00000001101011000101000000101010"; --SLT R10=R13<R12
      Instruction_mem(37) <= "00000001001010100011100000100110"; --XOR R10=R9 xor R7 
	      Instruction_mem(38) <= "00010000001000100000000000100000"; --Add R0=R1+R2
   		
		
			
			
					
		
		end if;
	end process;
	
	

Fetch_P: process (clk)
  begin
    if(Reset = '0' and CLK'event and CLK='1') then
      PW <= Instruction_mem(to_integer(unsigned(PC)));
			--if (Opcode=) then PC <= Jaddress;
			  --else
			PC <= std_logic_vector(unsigned(PC) + 1); 
			--enf if;
			-- Please remember to replace PC with Branch PC --
-- if Branch success this will happen in the Execute Phase
  end if;
  end process;

Decode_P: process (reset,clk)
begin
  if(Reset = '0' and CLK'event and CLK='1') then

Instruction <= to_record(Instruction_mem(to_integer(unsigned(PC))));
Opcode <= Instruction.opcode;
A <= Reg_Input(to_integer(unsigned(Instruction.Rs)));
B <= Reg_Input(to_integer(unsigned(Instruction.Rt)));
Rout<= Instruction.Rd;
if (Opcode = LW) then MemRead <='1';offset <= PW(15 downto 0);
 elsif (Opcode= SW) then MemWrite <= '1';offset <= PW(15 downto 0);
   elsif ((Opcode = bne) or (opcode= beq))then offset <= PW(15 downto 0);
   elsif (Opcode = Jump) then Jaddress <= PW (25 downto 0);
     else  RegWrite <= '1';  
end if;
end if;
end process;

Execute_P: Process (clk, reset)
begin
  if(Reset = '0' and CLK'event and CLK='1') then
  
  MemReadF <= MemRead;
  MemWriteF <= MemWrite;
  RegWriteF <= RegWrite;
 end if; 
  
  case Opcode is
        when Jr => --Jaddress <= "00000000000000000000000000000111" 		;
				when JUMP => 	 --Jaddress <= "00000000000000000000000000000111" ;
				when beq => 	
				if (Q = "00000000000000000000000000000000000000000000000000000000000000000000000000000000001") then 
				Jaddress <= "00000000000000000000000000000111" ;
				  end if;
				when bne => --Jaddress <= "00000000000000000000000000000111" 	;
      when others => offset12 <= "00000000000000000000000000000111";
  -- ALU Operations including Branch Success address change
  -- if Offet (15) = '1' then Extendedoffset <= X#FFFF# & Offset 
  --   else ExtendedOffset <= X#0000# & Offset; end if; 
  -- PC <= PC + Extendedoffset   
	
  --  when -----
end case;
  --case Opcode is
  -- ALU Operations including Branch Success address change
  -- if Offet (15) = '1' then Extendedoffset <= X#FFFF# & Offset 
  --   else ExtendedOffset <= X#0000# & Offset; end if; 
  -- PC <= PC + Extendedoffset   

  --  when -----
end process;

Write_P: Process (clk,reset)
begin
  if(Reset = '0' and CLK'event and CLK='1') then
  if Proc_ready = '1' then
    MemReadF2 <= MemReadF; -- Forwarding
    RegWriteF2 <= RegWriteF; -- Forwarding
  -- if MemWriteF = '1' then Instruction_mem(to_integer(unsigned(Reg_output(to_integer(unsigned(Instruction.Rd))))+to_integer(unsigned(offset))))<= Q(31 downto 0); 
    --end if;
  end if;
end if;
end process;
--
Writeback_P: Process (clk,reset)
begin 
  if(Reset = '0' and CLK'event and CLK='1') then
 -- if MemReadF2 = '1' then Reg_output(to_integer(unsigned(Instruction.Rd))) <= Instruction_mem(to_integer(unsigned(offset)));
if RegWriteF2 ='1' then Reg_output(to_integer(unsigned(Rout))) <= Q(31 downto 0);
----     else
   -- end if;
end if;
end if;
end process;


end architecture First;


-- Test Bench for MIPS-2 Processor
library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;
--
use work.ProcP.all;
use STD.TEXTIO.all;
use work.all;
--
entity MCPROC_TB is
end entity MCPROC_TB;
--
architecture TEST of MCPROC_TB is
	signal PC, PW: std_logic_Vector (31 downto 0) := (others => '0');
	Signal reset, clk: std_logic;
	signal Instruction_mem: MEM1K := ((others => (others=>'0')));
--	signal proc_reset: std_logic := '1';
  signal Instruction: Tinstruction;
--
	component MCProc
		port (PC, PW			: inout std_logic_vector (31 downto 0):= (others => '0');
			  clk, reset		: std_logic;
			  Instruction_mem			: inout MEM1K);
	end component MCProc;
--
	for MY_PROC: MCProc use entity work.MCPROC(First);
begin
	Reset <= '1','0' after 100 ps;
	--
	CLK_P: process
	begin
		CLK <= '0';
		wait for 5 ps;
		CLK <= '1';
		wait for 5 ps;
	end process;

	MY_PROC: MCPROC port map (PC, PW, clk, reset, Instruction_mem);

	Inst_Stimulate: process(clk, reset)
	begin
	if (Reset='0' and CLK'event and CLK= '1') then
	
    Instruction <= to_record(PW);
--		proc_reset <= '0';

		end if;
end process;
end architecture TEST;

