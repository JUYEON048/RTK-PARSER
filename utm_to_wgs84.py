# utm_to_wgs84_fromCSVfile

import csv
from pyproj import Proj, transform


if __name__ == '__main__':
    utm_x = []
    utm_y = []
    proj_WGS84 = Proj("+proj=latlong +datum=WGS84 +ellps=WGS84") 
    proj_UTM =  Proj("+proj=utm +zone=52 +ellps=WGS84 +datum=WGS84 +units=m +no_defs")


    with open('/home/miserver13/Desktop/220901_magock/09-01-23-14-25-rtk.csv') as fr:
    
        rdr = csv.DictReader(fr)
        for i in rdr:
            utm_x.append(i['field.utm_x'])
            utm_y.append(i['field.utm_y'])


    with open('/home/miserver13/Desktop/220901_magock/09-01-23-14-25-wgs.csv', 'a', newline='') as fw:
        wr = csv.writer(fw)
        wr.writerow(['lon', 'lat', 'utm_x', 'utm_y'])
        for i in range(len(utm_x)):
            lon, lat = transform(proj_UTM, proj_WGS84, utm_x[i], utm_y[i])
            wr.writerow([lon, lat, utm_x[i], utm_y[i]])











