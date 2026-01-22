#!/usr/bin/env bash
set -e

export IFACE="eno1"                 # modify here
export PC_IP="192.168.123.222/24"   # modify here

# Export DDS and Network Interface
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="eno1" priority="default" multicast="default" />
      </Interfaces>
    </General>
  </Domain>
  <DDSI2E>
    <Internal>
      <MaxSampleSize>2MB</MaxSampleSize>
    </Internal>
  </DDSI2E>
</CycloneDDS>'