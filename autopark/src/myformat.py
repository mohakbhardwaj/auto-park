from construct import *


message_crc = Struct('message_crc', UBInt64('crc'))


message_format = Struct('message_format',
    ULInt16('vcl_id'),
    Enum(Byte('message_type'),
    HELLO = 0x40,
    INTRO = 0x3f,
    UPDATE = 0x30,
    GOODBYE = 0x20,
    PARKED = 0x31,
	 
        _default_ = Pass
    ),
    Byte('datalen'),
    Array(lambda ctx: ctx['datalen'], Byte('data')),
    Embed(message_crc)
)


if __name__ == "__main__":
    raw = message_format.build(Container(
	vcl_id=0x1,        
        message_type='HELLO',
        datalen=4,
        data=[0x1, 0xff, 0xff, 0xdd],
        crc=0x12345678))

    print raw
    mymsg=raw.encode('hex')
    print mymsg
    x=message_format.parse(raw)
    print x
