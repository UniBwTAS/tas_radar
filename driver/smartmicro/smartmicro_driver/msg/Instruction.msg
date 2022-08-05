uint8 request
uint8 response
uint16 section
uint16 id
uint8 datatype
uint8 dim_count
float64 value   # use for both integer and float32 values. float64's mantissa is precise enough to store an [u]int32; any integer will be accurately represented.

uint8 REQUEST_PARAMETER_SET = 1
uint8 REQUEST_PARAMETER_GET = 2
uint8 REQUEST_STATUS_GET = 3
uint8 REQUEST_COMMAND = 4

uint8 RESPONSE_UNSET = 0
uint8 RESPONSE_SUCCESS = 1
# ...

uint8 DATATYPE_I8 = 1
uint8 DATATYPE_U8 = 2
uint8 DATATYPE_I16 = 3
uint8 DATATYPE_U16 = 4
uint8 DATATYPE_I32 = 5
uint8 DATATYPE_U32 = 6
uint8 DATATYPE_F32 = 7
