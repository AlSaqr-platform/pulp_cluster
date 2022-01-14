GIT ?= git
BENDER ?= bender
VSIM ?= vsim

# Ensure half-built targets are purged
.DELETE_ON_ERROR:

# --------------
# SYNTHESIS
# --------------

define generate_synopsys
	echo 'set ROOT [file normalize [file dirname [info script]]/../../..]' > $1
	bender script synopsys $2 | grep -v "set ROOT" >> $1
	echo >> $1
endef

gf22/cockpit.log:
	cd gf22 && icdesign gf22 -update all -nogui

gf22/synopsys/scripts/analyze.tcl: Bender.yml | gf22/cockpit.log
	$(call generate_synopsys, $@, -t rtl -t default -t synthesis -t gf22,..)

