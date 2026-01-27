#!/usr/bin/env bash

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
      <MaxSampleSize>50MB</MaxSampleSize>
    </Internal>
  </DDSI2E>
</CycloneDDS>'

