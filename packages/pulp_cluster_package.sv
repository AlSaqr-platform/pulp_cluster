// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

/*
 * pulp_cluster_package.sv
 * Davide Rossi <davide.rossi@unibo.it>
 * Michael Gautschi <gautschi@iis.ee.ethz.ch>
 */

package pulp_cluster_package;

  typedef struct packed {
      logic [63:0] idx;
      logic [63:0] start_addr;
      logic [63:0] end_addr;
  } addr_map_rule_t;
  
  parameter NB_SPERIPH_PLUGS_EU  = 2;

  // number of master and slave cluster periphs
  parameter NB_CLUSTER_MPERIPHS          =  1;
  parameter NB_CLUSTER_SPERIPHS          = 10;
  
  // position of peripherals on slave port of periph interconnect
  parameter SPER_EOC_ID      = 0;                                                                // 1020_0000 - 1020_0400
  parameter SPER_TIMER_ID    = 1;                                                                // 1020_0400 - 1020_0800
  parameter SPER_EVENT_U_ID  = 2;                                                                // 1020_0800 - 1020_1000
                             // 3 also used for Event Unit                                       
  parameter SPER_HWPE_ID     = 4;                                                                // 1020_1000 - 1020_1400
  parameter SPER_ICACHE_CTRL = 5;                                                                // 1020_1400 - 1020_1800
  parameter SPER_DMA_CL_ID   = 6;                                                                // 1020_1800 - 1020_1C00
  parameter SPER_DMA_FC_ID   = 7;                                                                // 1020_1C00 - 1020_2000
  parameter SPER_DECOMP_ID   = 8; // Currently unused / grounded, available for specific designs // 1020_2000 - 1020_2400
  parameter SPER_EXT_ID      = 9;                                                                // 1020_2400 - 1020_2800
  parameter SPER_ERROR_ID    = 10;                                                               // 1020_2800 - 1020_2C00
  
  // if set to 1, then instantiate APU in the cluster
  // parameter APU_CLUSTER = 0;
  
  // // if set to 1, the 0x0000_0000 to 0x0040_0000 is the alias of the current cluster address space (eg cluster 0 is from  0x1000_0000 to 0x1040_0000)
  // parameter CLUSTER_ALIAS = 1;
  
  // // if set to 1, the DEMUX peripherals (EU, MCHAN) are placed right before the test and set region.
  // // This will steal 16KB from the 1MB TCDM reegion.
  // // EU is mapped           from 0x10100000 - 0x400
  // // MCHAN regs are mapped  from 0x10100000 - 0x800
  // // remember to change the defines in the pulp.h as well to be coherent with this approach
  // parameter DEM_PER_BEFORE_TCDM_TS = 0;
  
endpackage
