import fcntl,struct,os,sys
fd=os.open("/dev/v4l/by-id/usb-BC-250219-ZW_USB_Camera_01.00.00-video-index0",os.O_RDWR|os.O_NONBLOCK)
N={"auto":0x9a0901,"exp":0x9a0902,"gamma":0x980910,"bright":0x980900,"contrast":0x980901,"backlight":0x98091c,"sharp":0x98091b}
def g(c):
    b=bytearray(8);struct.pack_into("I",b,0,c);fcntl.ioctl(fd,0xC008561B,b);return struct.unpack_from("i",b,4)[0]
for a in sys.argv[1:]:
    k,v=a.split("=");b=bytearray(8);struct.pack_into("Ii",b,0,N[k],int(v));fcntl.ioctl(fd,0xC008561C,b)
print({k:g(c) for k,c in N.items()})
