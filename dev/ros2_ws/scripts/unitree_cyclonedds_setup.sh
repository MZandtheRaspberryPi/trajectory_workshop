#!/usr/bin/env bash
set -u # exit if variable is undefined

NET_INTERFACE=$1

# Export DDS and Network Interface
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="'$NET_INTERFACE'" priority="default" multicast="default" />
      </Interfaces>
    </General>
  </Domain>
  <DDSI2E>
    <Internal>
      <MaxSampleSize>50MB</MaxSampleSize>
    </Internal>
  </DDSI2E>
</CycloneDDS>'

echo "Setup net interface for ${NET_INTERFACE}"

set +u