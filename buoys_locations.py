#from main2 import mapping
import matplotlib.image as mpimg
import numpy as np
buoys_locations_gps=np.array([[48.210168,-3.023417],
                [48.210168,-3.022237],
                [48.210116,-3.020348],
                [48.210326,-3.018223],
                [48.210273,-3.015428]])
def mapping(xGPS,yGPS,xlGPS,ylGPS,xuGPS,yuGPS):
    im =mpimg.imread("map.png")
    # <class 'numpy.ndarray'>
    xl=0
    yl=0
    xu=im.shape[1]
    yu=im.shape[0]
    #xGPS,yGPS = convert_to_meter_space(xGPS,yGPS)
    #xlGPS,ylGPS = convert_to_meter_space(xlGPS,ylGPS)
    #xuGPS,yuGPS = convert_to_meter_space(xuGPS,yuGPS)

    xMapped=((xGPS-xlGPS)*(xu-xl))/(xuGPS-xlGPS)
    yMapped=((yGPS-ylGPS)*(yu-yl))/(yuGPS-ylGPS)
    
    #xMapped = xGPS-xlGPS
    #yMapped = yGPS-ylGPS
    return xMapped,yMapped
def buoys_locations():
    buoys_locations_pixel= np.zeros((buoys_locations_gps.shape[0],buoys_locations_gps.shape[1]))
    #print(buoys_locations_pixel.shape)
    xlGPS,ylGPS=-3.026018,48.207114
    xuGPS,yuGPS=-3.009049,48.213114
    #print(buoys_locations_gps.shape)
    for i in range(len(buoys_locations_gps)):
        lon,lat=buoys_locations_gps[i][1],buoys_locations_gps[i][0]
        xMapped_start,yMapped_start=mapping(lon,lat,xlGPS,ylGPS,xuGPS,yuGPS)
        
        buoys_locations_pixel[i][0]=xMapped_start
        buoys_locations_pixel[i][1]=yMapped_start
    return buoys_locations_pixel