library verilog;
use verilog.vl_types.all;
entity jrdec is
    port(
        aluop           : in     vl_logic_vector(1 downto 0);
        funct           : in     vl_logic_vector(5 downto 0);
        jr              : out    vl_logic
    );
end jrdec;
