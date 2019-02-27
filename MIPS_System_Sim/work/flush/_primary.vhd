library verilog;
use verilog.vl_types.all;
entity flush is
    port(
        jump            : in     vl_logic;
        jr              : in     vl_logic;
        jal             : in     vl_logic;
        pcsrc           : in     vl_logic;
        flush           : out    vl_logic
    );
end flush;
