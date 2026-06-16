# Packs numbers into two numbers of length 1 byte for sending to arduino
def num_packer(number: float):
    # Pack degrees as centi-degrees into two bytes (big-endian)
    centi = int(round(number * 100)) & 0xFFFF
    high = (centi >> 8) & 0xFF
    low  = centi & 0xFF
    return high, low