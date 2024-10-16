if {![info exists VSIM_PATH ]} {
    return -code error -errorinfo "[ERRORINFO] You must set the \"VSIM_PATH\" variable before sourcing the start script."
    set VSIM_PATH ""
}

if {![info exists APP]} {
    set APP "./build/test/test"
}

vsim +permissive -suppress 3053 -suppress 8885 -lib $VSIM_PATH/work +APP=$APP +notimingchecks +nospecify  -t 1ps -sv_lib $VSIM_PATH/work-dpi/cl_dpi  pulp_cluster_tb_optimized +permissive-off ++$APP

add log -r /*

proc run_and_exit {} {
    run -all
    quit -code [examine -radix decimal sim:/pulp_cluster_tb/ret_val(30:0)]
}

run_and_exit
