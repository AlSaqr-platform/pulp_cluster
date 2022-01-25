GIT ?= git
BENDER ?= bender
VSIM ?= vsim

# Ensure half-built targets are purged
.DELETE_ON_ERROR:

# --------------
# SYNTHESIS
# --------------

gf22/cockpit.log:
	cd gf22 && icdesign gf22 -update all -nogui

gf22/synopsys/scripts/analyze.tcl:
	echo 'set ROOT [file normalize [file dirname [info script]]/../../../]' > gf22/synopsys/scripts/analyze.tcl
	bender script synopsys -t rtl -t synthesis -t asic | grep -v "set ROOT" >> gf22/synopsys/scripts/analyze.tcl

