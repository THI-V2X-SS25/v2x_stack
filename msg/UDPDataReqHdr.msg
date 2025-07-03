# BTP Type
uint8 btp_type

# GN Packet Transport type
uint8 pkt_transport

# GN Traffic class
uint8 traffic_class

# GN Maximum Packet Lifetime (seconds)
uint8 max_pkt_life_time

# BTP Destination port (BTP-A & BTP-B BTPTypes)
uint16 dest_port

# BTP Additional Port Information
## Source Port (BTP-A)
uint16 src_port
## Destination Port Info (BTP-B)
uint16 dest_info

# Destination Location (GeoUnicast)
uint64 location_gn_addr_as_64
uint64 location_unused

# Area for GBC or GAC
Area area

# Communication and Security
uint8 comm_profile
uint8 repeat_interval
uint8 sec_profile
uint8 ssp_len
uint32 aid

# SSP Bits/Data
uint8[32] ssp_bits  # Assuming UDPRX_SEC_SSP_BITS_LENGTH = 32

# The length in octets of the 'on-the-air' frame data
uint16 length
